"""Visualise the FULLY AUTONOMOUS mission (src/mission/follow_mission.py): takeoff ->
yaw-sweep SEARCH -> ACQUIRE an off-axis target -> FOLLOW (centre + hold d0) -> fooled by
the patch -> SAFETY_HALT. Produces a top-down animated GIF and a distance profile that
compares clean vs patched. Numbers are the 3 m-cube recipe (d0=1.5, d_safe=0.7, r=0.46).
"""
import os, sys, numpy as np
ROOT = r"C:\Users\default.DESKTOP-5PUUIEJ\Desktop\adversarial_uav_demo"; sys.path.insert(0, ROOT)
from src.mission.follow_mission import Mission, Params, SAFETY_HOLD
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, Wedge
from matplotlib.animation import FuncAnimation, PillowWriter
OUT = os.path.join(ROOT, "outputs")
TGT = (1.5, 2.0)        # target board location (UNKNOWN to the controller; off-axis)


def rollout(patch_on):
    p = Params(); m = Mission(p=p); log = []; t = 0.0
    while t < 70:
        vx, yr, vz, info = m.step(t, TGT, patch_on)
        log.append((t, m.x, m.y, m.z, m.yaw, info["true_gap"],
                    info["perceived_d"] if info["perceived_d"] else np.nan,
                    info["state"], int(info["deceived"])))
        if info["state"] == SAFETY_HOLD and all(r[7] == SAFETY_HOLD for r in log[-25:]) and len(log) > 25:
            break
        t += p.dt
    return log, p


# ---------------- distance profile: clean vs patched ----------------
lg_c, p = rollout(False); lg_p, _ = rollout(True)
fig, ax = plt.subplots(figsize=(8, 4.2))
for lg, c, lab in [(lg_c, "#1f77b4", "clean target"), (lg_p, "#d62728", "patched target")]:
    tt = [r[0] for r in lg]; gap = [r[5] for r in lg]
    ax.plot(tt, gap, color=c, lw=2, label="%s: true gap" % lab)
ax.axhline(p.d0, ls=":", c="green"); ax.text(0.2, p.d0 + 0.05, "intended standoff d0=%.1fm" % p.d0, color="green", fontsize=8)
ax.axhline(p.d_safe, ls="--", c="red"); ax.text(0.2, p.d_safe + 0.05, "safety geofence d_safe=%.1fm" % p.d_safe, color="red", fontsize=8)
ax.set_xlabel("time (s)"); ax.set_ylabel("TRUE gap to target (m)"); ax.set_ylim(0, 3.2)
ax.set_title("Autonomous mission: clean holds d0, patched is pulled to the geofence")
ax.legend(fontsize=8, loc="upper right")
plt.tight_layout(); plt.savefig(os.path.join(OUT, "autonomous_profile.png"), dpi=130)
print("saved autonomous_profile.png")
fc = lg_c[-1][5]; fp = lg_p[-1][5]
print("clean final gap=%.2fm (%s)  patched final gap=%.2fm (%s, deceived=%s)"
      % (fc, lg_c[-1][7], fp, lg_p[-1][7], any(r[8] for r in lg_p)))

# ---------------- top-down animation (patched = the attack) ----------------
lg = lg_p; L = np.array([r[:7] for r in lg], dtype=float); states = [r[7] for r in lg]
stride = max(1, len(lg) // 110); idx = list(range(0, len(lg), stride))
fig, ax = plt.subplots(figsize=(6.2, 6.2)); ax.set_aspect("equal")
ax.add_patch(Rectangle((-0.6, -0.6), 3.2, 3.2, fill=False, ec="#444", lw=1.2, ls="-"))
ax.text(-0.55, 2.68, "3 m mocap cube (top view)", fontsize=8, color="#444")
ax.add_patch(Circle(TGT, p.d0, fill=False, ec="green", ls=":", lw=1.3))
ax.add_patch(Circle(TGT, p.d_safe, fill=False, ec="red", ls="--", lw=1.3))
ax.plot(*TGT, "s", ms=13, color="#b22", mec="k"); ax.text(TGT[0] + 0.08, TGT[1] + 0.08, "patched\ntarget", fontsize=8, color="#b22")
ax.set_xlim(-0.8, 2.9); ax.set_ylim(-0.8, 2.9); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
trail, = ax.plot([], [], "-", color="#1f77b4", lw=1, alpha=0.6)
dron, = ax.plot([], [], "o", ms=9, color="#1f77b4", mec="k")
fov = ax.add_patch(Wedge((0, 0), p.d_detect, 0, 0, color="#1f77b4", alpha=0.10))
hdg, = ax.plot([], [], "-", color="#1f77b4", lw=2)
txt = ax.text(-0.75, -0.75, "", fontsize=9, family="monospace",
              bbox=dict(boxstyle="round", fc="white", ec="#aaa", alpha=0.9))


def frame(k):
    i = idx[k]; x, y, yaw, gap = L[i, 1], L[i, 2], L[i, 4], L[i, 5]
    st = states[i]; deg = np.degrees(yaw)
    trail.set_data(L[:i + 1, 1], L[:i + 1, 2])
    dron.set_data([x], [y])
    hdg.set_data([x, x + 0.35 * np.cos(yaw)], [y, y + 0.35 * np.sin(yaw)])
    fov.set_center((x, y)); half = np.degrees(p.fov / 2)
    fov.set_theta1(deg - half); fov.set_theta2(deg + half)
    fov.set_color("#f0a000" if st == "SEARCH" else "#1f77b4")
    col = {"TAKEOFF": "#888", "SEARCH": "#f0a000", "FOLLOW": "#1f77b4", "SAFETY_HOLD": "#d62728"}.get(st, "#333")
    dron.set_color(col)
    txt.set_text("state : %-11s\ntrue gap: %.2f m\n%s" % (
        st, gap, "DECEIVED -> pulled past d0" if (st in ("FOLLOW", "SAFETY_HOLD") and gap < p.d0 - 0.05) else ""))
    return trail, dron, hdg, fov, txt


anim = FuncAnimation(fig, frame, frames=len(idx), interval=90, blit=False)
plt.tight_layout()
anim.save(os.path.join(OUT, "autonomous_mission.gif"), writer=PillowWriter(fps=11))
print("saved autonomous_mission.gif", len(idx), "frames")
