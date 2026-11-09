#!/usr/bin/env python3
"""L5 closed loop (run in WSL): camera -> YOLO -> standoff control (pymavlink)
   -> adversarial patch appears -> bbox shrinks -> drone pulled in -> collision,
   with an independent-anchor consistency defense that alarms.

Frame source: Gazebo camera disk-save dir (/tmp/fpv_frames, newest file).
Vehicle:      ArduCopter SITL (gazebo-iris), MAVLink tcp:127.0.0.1:5770.
Target:       bus billboard at world +x (north), 10 m ahead, centre z=3.
Patch:        patch_panel model moved in front of the bus at t_attack via `gz`.

This is a defensive-research reproduction (NDSS'26 FlyTrap distance-pulling) in
simulation only. Outputs a CSV log and a plot.
"""
import glob
import math
import os
import subprocess
import sys
import time

import cv2
import numpy as np
from pymavlink import mavutil
from ultralytics import YOLO

FRAMES = "/tmp/fpv_frames"
# Default ArduCopter SITL (-I0) MAVLink is tcp:5760 and WORKS when that port is
# free. This machine's concurrent RESS experiment (e0_harvest -n 4) continuously
# occupies 5760-5793, and this ArduPilot build does NOT honour `-A "--serial0
# tcp:<other>"` (the override opens no MAVLink socket). So run L5-live only when
# RESS is not occupying 5760 (or in a separate WSL): launch the SITL plainly with
#   sim_vehicle.py -v ArduCopter -f gazebo-iris -I0 --no-mavproxy --no-rebuild
# and keep MAV = tcp:127.0.0.1:5760 below.
MAV = "tcp:127.0.0.1:5760"
OUT = "/mnt/c/Users/default.DESKTOP-5PUUIEJ/Desktop/adversarial_uav_demo/outputs/sim"
TARGET_CLASS = "bus"
TARGET_H_M = 4.0          # billboard panel height (m) -> real target size S
TARGET_WORLD_X = 10.0     # bus north position (m)
HOVER_Z = 3.0
D0 = None                 # standoff setpoint; auto-calibrated at first lock
KP = 0.25
VMAX = 1.2
DT = 0.30
T_ATTACK = 14.0
T_MAX = 70.0
COLLISION_M = 0.6
TAU = 0.18
ANCHOR_NOISE = 0.15


def newest_frame():
    fs = glob.glob(os.path.join(FRAMES, "*.jpg"))
    if not fs:
        return None
    f = max(fs, key=os.path.getmtime)
    for _ in range(3):
        img = cv2.imread(f)
        if img is not None and img.size:
            return img
        time.sleep(0.02)
    return None


def target_box_h(model, img):
    """Return bounding-box height (px) of the highest-confidence target, else None."""
    r = model(img, verbose=False)[0]
    best, best_conf = None, 0.0
    for b in r.boxes:
        if r.names[int(b.cls[0])] == TARGET_CLASS:
            conf = float(b.conf[0])
            if conf > best_conf:
                x1, y1, x2, y2 = b.xyxy[0].tolist()
                best, best_conf = (y2 - y1), conf
    return best, best_conf


def move_patch_in_front():
    """Move the patch panel between the drone and the bus.
    NOTE (2026-11-07): tested 3 ways live -- adversarial patch full-cover,
    adversarial top-cover, and a plain gray top-band occluder -- ALL produce
    SUPPRESSION (YOLO drops the bus to conf 0), never a shrunk box. Occlusion is
    all-or-nothing for the detector, so live distance-pulling needs a trained
    scale-manipulation patch, not geometry. The distance-pulling + defense claim
    stays validated by the numerical 20-seed study (src/sim/defense_study.py)."""
    try:
        subprocess.run(["gz", "model", "-m", "patch", "-x", "9.0",
                        "-y", "0.0", "-z", "3.0"], timeout=5,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception as e:
        print("patch move failed:", e)


def main():
    os.makedirs(OUT, exist_ok=True)
    print("loading YOLO..."); model = YOLO("yolov8n.pt")

    m = None
    for attempt in range(20):
        try:
            m = mavutil.mavlink_connection(MAV, autoreconnect=True)
            print("waiting heartbeat (try %d)..." % (attempt + 1))
            if m.wait_heartbeat(timeout=4) is not None:
                print("heartbeat ok sys=%d" % m.target_system)
                break
        except Exception as e:
            print("connect retry:", e)
        try:
            if m:
                m.close()
        except Exception:
            pass
        m = None
        time.sleep(2)
    if m is None or m.target_system == 0:
        print("RESULT: could not connect to SITL on %s" % MAV); sys.exit(1)

    # arm + takeoff (GUIDED)
    m.set_mode_apm("GUIDED"); time.sleep(1)
    for _ in range(20):
        m.mav.command_long_send(m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            break
    print("armed; takeoff")
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, HOVER_Z)

    # wait to reach hover altitude
    t0 = time.time()
    while time.time() - t0 < 25:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=3)
        if msg and msg.relative_alt / 1000.0 > HOVER_Z - 0.6:
            break
    print("at hover; begin tracking")

    # record start north for distance travelled
    start_n = None
    log = []
    global D0
    f_px = None
    attacked = False
    alarm_t = None
    collided = False
    t_start = time.time()

    def north():
        msg = m.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        return msg.x if msg else None

    cur_n = 0.0
    while time.time() - t_start < T_MAX:
        t = time.time() - t_start
        # update north position
        n = north()
        if n is not None:
            if start_n is None:
                start_n = n
            cur_n = n - start_n
        d_true = max(TARGET_WORLD_X - cur_n, 0.0)

        img = newest_frame()
        box_h, conf = (None, 0.0) if img is None else target_box_h(model, img)

        # attack trigger
        if (not attacked) and t >= T_ATTACK:
            move_patch_in_front(); attacked = True
            print("[%.1f] PATCH DEPLOYED" % t)

        v = 0.0
        d_box = None
        mism = None
        if box_h is not None and box_h > 2:
            if f_px is None and t < T_ATTACK:
                # calibrate so that clean perceived distance == true distance
                f_px = d_true * box_h / TARGET_H_M
                D0 = d_true
                print("calibrated f_px=%.1f D0=%.2f (box_h=%.1f)" % (f_px, D0, box_h))
            if f_px is not None:
                d_box = f_px * TARGET_H_M / box_h
                v = float(np.clip(KP * (d_box - D0), -VMAX, VMAX))
                # independent-anchor defense
                d_anchor = d_true + np.random.normal(0, ANCHOR_NOISE)
                mism = abs(d_box - d_anchor) / max(d_anchor, 1e-3)
                if mism > TAU and alarm_t is None:
                    alarm_t = t
                    print("[%.1f] DEFENSE ALARM  d_box=%.2f d_anchor=%.2f mism=%.2f"
                          % (t, d_box, d_anchor, mism))

        # send forward body velocity setpoint (vx>0 = forward/toward target)
        if f_px is not None:
            m.mav.set_position_target_local_ned_send(
                0, m.target_system, m.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0b0000111111000111,  # use vx,vy,vz
                0, 0, 0, v, 0, 0, 0, 0, 0, 0, 0)

        log.append((t, d_true, d_box if d_box else float("nan"),
                    box_h if box_h else float("nan"),
                    mism if mism is not None else float("nan"),
                    v, conf))
        print("[%.1f] d_true=%.2f d_box=%s box_h=%s v=%.2f conf=%.2f%s"
              % (t, d_true,
                 ("%.2f" % d_box) if d_box else "--",
                 ("%d" % box_h) if box_h else "--", v, conf,
                 "  <ATTACK>" if attacked else ""))

        if d_true <= COLLISION_M:
            collided = True
            print("[%.1f] COLLISION (d_true=%.2f)" % (t, d_true))
            break
        time.sleep(DT)

    # stop + land
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    m.set_mode_apm("LAND")

    # write log + plot
    arr = np.array([(r[0], r[1], r[2], r[3], r[4], r[5]) for r in log], dtype=float)
    np.savetxt(os.path.join(OUT, "l5_log.csv"), arr, delimiter=",",
               header="t,d_true,d_box,box_h,mismatch,v", comments="")
    print("RESULT collided=%s alarm_t=%s attack_t=%.1f" % (collided, alarm_t, T_ATTACK))
    if collided and alarm_t is not None:
        print("  defense lead before collision = %.1f s" % (arr[-1, 0] - alarm_t))

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(13, 5))
        ax[0].plot(arr[:, 0], arr[:, 1], label="true distance", lw=2, color="crimson")
        ax[0].plot(arr[:, 0], arr[:, 2], label="perceived d_box (deceived)", lw=2, ls="--", color="orange")
        ax[0].axhline(COLLISION_M, ls="--", color="k", alpha=0.5, label="collision")
        ax[0].axvline(T_ATTACK, ls=":", color="gray", label="patch appears")
        if alarm_t is not None:
            ax[0].axvline(alarm_t, ls="-.", color="green", label="defense alarm")
        ax[0].set_xlabel("time (s)"); ax[0].set_ylabel("distance (m)")
        ax[0].set_title("L5 Gazebo closed loop: distance-pulling"); ax[0].legend(); ax[0].grid(alpha=0.3)
        ax[1].plot(arr[:, 0], arr[:, 4], color="purple", lw=2)
        ax[1].axhline(TAU, ls="-.", color="green", label="defense tau")
        ax[1].axvline(T_ATTACK, ls=":", color="gray")
        ax[1].set_xlabel("time (s)"); ax[1].set_ylabel("box/anchor mismatch")
        ax[1].set_title("Defense signal"); ax[1].legend(); ax[1].grid(alpha=0.3)
        fig.tight_layout(); fig.savefig(os.path.join(OUT, "l5_result.png"), dpi=120)
        print("FIGURE l5_result.png")
    except Exception as e:
        print("plot skipped:", e)


if __name__ == "__main__":
    main()
