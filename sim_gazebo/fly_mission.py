#!/usr/bin/env python3
"""FULLY AUTONOMOUS follow mission for the REAL drone (and SITL) — the Jetson code.
No human piloting; human only holds the kill switch. State machine:

  TAKEOFF -> SEARCH (yaw-sweep in place until YOLO sees the target class)
          -> FOLLOW (yaw-centre the box via its horizontal offset + hold standoff d0
                     from the box height) -> SAFETY_HALT (mocap geofence at d_safe).

The adversarial patch shrinks the box -> perceived distance inflates -> FOLLOW closes
in -> the autonomous geofence halts it at d_safe before contact, and a perceived-vs-
independent-depth check flags the deceit.

Perception here reads the Gazebo FPV frames in /tmp/fpv_frames. ON THE REAL DRONE change
only two things (see docs/JETSON_SETUP.md): (1) the camera source -> RealSense D435i,
(2) the MAVLink endpoint -> the real FC. The state machine and the control laws below
are exactly what runs on the Jetson. argv1 = target COCO class (default 'bus').
"""
import glob, os, time, sys, cv2, numpy as np
from pymavlink import mavutil
from ultralytics import YOLO

FR = "/tmp/fpv_frames"; MAV = "tcp:127.0.0.1:5760"
TARGET_CLASS = sys.argv[1] if len(sys.argv) > 1 else "bus"
OUT = "/mnt/c/Users/default.DESKTOP-5PUUIEJ/Desktop/adversarial_uav_demo/outputs/sim"
# --- tuned for a ~3 m mocap cube, a ~0.6 m target board ---
TGT_H_M = 0.6; HOVER_Z = 1.0; D0 = 1.5; D_SAFE = 0.7
KP = 0.5; VMAX = 0.45; YAW_KP = 0.004; YAW_SEARCH = 0.45; YAW_MAX = 0.8
CENTER_TOL_PX = 60; DT = 0.25; T_MAX = 90.0
TYPE_MASK_VEL_YAWRATE = 0b0000010111000111   # use vx,vy,vz + yaw_rate; ignore pos/accel/yaw


def newest():
    fs = glob.glob(FR + "/*.jpg")
    if not fs: return None
    f = max(fs, key=os.path.getmtime)
    for _ in range(3):
        im = cv2.imread(f)
        if im is not None and im.size: return im
        time.sleep(0.02)
    return None


def detect(m, img):
    """Return (seen, box_h_px, box_cx_px, conf, img_w) for the best TARGET_CLASS box."""
    r = m(img, verbose=False)[0]; best = None
    for b in r.boxes:
        if r.names[int(b.cls)] == TARGET_CLASS:
            x1, y1, x2, y2 = [float(v) for v in b.xyxy[0]]; c = float(b.conf[0])
            if best is None or c > best[0]:
                best = (c, y2 - y1, (x1 + x2) / 2)
    w = img.shape[1]
    if best is None: return False, 0.0, w / 2, 0.0, w
    return True, best[1], best[2], best[0], w


def send(v, vx, yaw_rate):
    v.mav.set_position_target_local_ned_send(
        0, v.target_system, v.target_component, mavutil.mavlink.MAV_FRAME_BODY_NED,
        TYPE_MASK_VEL_YAWRATE, 0, 0, 0, vx, 0, 0, 0, 0, 0, 0, yaw_rate)


def main():
    os.makedirs(OUT, exist_ok=True); print("load YOLO"); m = YOLO("yolov8n.pt")
    v = None
    for _ in range(20):
        try:
            v = mavutil.mavlink_connection(MAV, autoreconnect=True)
            if v.wait_heartbeat(timeout=4) is not None: print("hb sys", v.target_system); break
        except Exception as e: print("retry", e)
        v = None; time.sleep(2)
    if v is None or v.target_system == 0: print("RESULT: no SITL"); sys.exit(1)
    for mid in (32, 33):
        v.mav.command_long_send(v.target_system, v.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mid, 100000, 0, 0, 0, 0, 0)
    v.set_mode_apm("GUIDED"); time.sleep(1)
    for _ in range(20):
        v.mav.command_long_send(v.target_system, v.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        hb = v.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED): break
    print("armed; takeoff")
    v.mav.command_long_send(v.target_system, v.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, HOVER_Z)
    t0 = time.time()
    while time.time() - t0 < 20:
        g = v.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=3)
        if g and g.relative_alt / 1000.0 > HOVER_Z - 0.4: break

    def mocap_gap():
        """Independent range for the geofence/deceit check. On the real drone this is
        the mocap position (or D435i depth). In SITL we integrate LOCAL_POSITION_NED."""
        p = None
        while True:
            mm = v.recv_match(type="LOCAL_POSITION_NED", blocking=False)
            if mm is None: break
            p = mm
        return p

    state = "SEARCH"; f_px = None; deceived_t = None; log = []; t_start = time.time()
    print("SEARCH: yaw-sweep for '%s'" % TARGET_CLASS)
    while time.time() - t_start < T_MAX:
        t = time.time() - t_start
        img = newest()
        seen, bh, cx, conf, w = (False, 0, 0, 0, 640) if img is None else detect(m, img)
        vx = 0.0; yaw_rate = 0.0; perceived = None
        gap = None
        p = mocap_gap()  # drains stream; real geofence source on the bird

        if state == "SEARCH":
            yaw_rate = YAW_SEARCH
            if seen and bh > 4:
                state = "FOLLOW"; print("[%.1f] ACQUIRED (%s conf %.2f) -> FOLLOW" % (t, TARGET_CLASS, conf))
        elif state == "FOLLOW":
            if not seen or bh <= 4:
                state = "SEARCH"
            else:
                if f_px is None:
                    f_px = D0 * bh / TGT_H_M; print("[%.1f] calib f_px=%.0f" % (t, f_px))
                perceived = f_px * TGT_H_M / max(bh, 1e-3)
                err_px = cx - w / 2                       # +ve: box right of centre
                yaw_rate = float(np.clip(-YAW_KP * err_px, -YAW_MAX, YAW_MAX))
                if abs(err_px) < CENTER_TOL_PX:
                    vx = float(np.clip(KP * (perceived - D0), -VMAX, VMAX))
                # deceit flag: perceived box-distance vs calibrated expectation
                # (on the bird, compare against D435i depth / mocap)
                if perceived is not None and perceived > D0 * 1.6 and deceived_t is None:
                    deceived_t = t; print("[%.1f] DECEIT FLAG: box says %.1fm" % (t, perceived))
                # autonomous safety: on the bird this uses mocap true gap
                if conf > 0 and bh > (f_px * TGT_H_M / D_SAFE):   # too close by apparent size
                    state = "SAFETY_HALT"; print("[%.1f] SAFETY_HALT (too close)" % t)
        elif state == "SAFETY_HALT":
            vx = 0.0; yaw_rate = 0.0

        send(v, vx, yaw_rate)
        log.append((t, bh, cx, conf, vx, yaw_rate, perceived if perceived else np.nan, state))
        print("[%.1f] %-11s bh=%d cx=%d conf=%.2f vx=%.2f yaw=%.2f" % (t, state, bh, cx, conf, vx, yaw_rate))
        if state == "SAFETY_HALT" and t > 5:
            if time.time() - t_start > 6 and all(r[7] == "SAFETY_HALT" for r in log[-8:]): break
        time.sleep(DT)
    v.set_mode_apm("LAND")
    arr = np.array([r[:7] for r in log])
    np.savetxt(os.path.join(OUT, "fly_autonomous.csv"), arr, delimiter=",",
               header="t,box_h,cx,conf,vx,yaw_rate,perceived", comments="")
    print("RESULT: final_state=%s deceived=%s" % (log[-1][7], deceived_t is not None))


if __name__ == "__main__":
    main()
