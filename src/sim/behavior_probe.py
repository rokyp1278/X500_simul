"""EMPIRICAL: what does the follow-drone actually DO when the adversarial patch
appears? We don't assert — we run the closed loop driven by the LIVE-MEASURED
per-distance shrink ratio r(box_h) (outputs/eot_rows.json, measured in Gazebo) and
OBSERVE the trajectory, plus sweep the one real unknown (how small a box YOLO still
detects) to map the regimes: B = still detected but shrunk -> closes to a new, too-
close equilibrium or collides;  A = shrunk below detection -> target lost -> stops.

Kinematic flight model = drone moves at the commanded GUIDED velocity (faithful to a
real FC in velocity mode; the SITL horizontal-control flakiness is a SITL artifact,
not physics). Perception effect = real Gazebo measurement, not an assumed parameter.
"""
from __future__ import annotations
import json, os, numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ROWS = json.load(open(os.path.join(ROOT, "outputs", "eot_rows.json")))
OUT = os.path.join(ROOT, "outputs")

# measured: rows = [scale, clean_h_px, patched_h_px, conf, ratio]
CH = np.array([r[1] for r in ROWS]); RT = np.array([r[4] for r in ROWS])
o = np.argsort(CH); CH, RT = CH[o], RT[o]
def r_of(box_h):               # measured shrink ratio vs clean box height
    return float(np.interp(box_h, CH, RT))   # clamps to measured endpoints outside range

F_PX, S = 600.0, 4.0           # camera focal(px) and target height(m): true_h = F_PX*S/d
def true_h(d): return F_PX * S / max(d, 1e-3)

def sim(d0=8.0, d_start=11.0, h_min_detect=0.0, t_attack=3.0, kp=0.5, vmax=1.2,
        dt=0.1, t_max=80.0, lost_policy="stop", collision=0.6):
    """One run. h_min_detect: smallest patched box (px) YOLO still detects (0 = always).
    Returns trajectory + outcome label."""
    d = d_start; t = 0.0; log = []; outcome = "following"; lost = False
    while t < t_max:
        th = true_h(d)
        attacking = t >= t_attack
        ph = th * (r_of(th) if attacking else 1.0)      # patched (or clean) box height
        detected = ph >= h_min_detect
        if detected:
            d_box = F_PX * S / max(ph, 1e-3)            # distance the drone BELIEVES
            v = float(np.clip(kp * (d_box - d0), -vmax, vmax))
            lost = False
        else:                                            # target lost -> regime A
            lost = True
            v = 0.0 if lost_policy == "stop" else v     # stop (sane) or coast
            d_box = np.nan
        d = max(d - v * dt, 0.0)
        log.append((t, d, d_box, th, ph, v, int(detected)))
        if d <= collision:
            outcome = "COLLIDE"; break
        t += dt
    arr = np.array(log)
    if outcome != "COLLIDE":
        if lost: outcome = "LOST->stop"
        else:
            dmin = np.nanmin(arr[:, 1]); dfin = arr[-1, 1]
            outcome = "hold" if abs(dfin - d0) < 0.3 else "new-standoff %.1fm" % dfin
    return arr, outcome

def main():
    # --- 1) three representative runs: no patch / patch while following / strong-detect-loss ---
    runs = {
        "no patch (baseline)":        dict(t_attack=1e9),
        "patch, YOLO keeps detecting":dict(t_attack=3.0, h_min_detect=0.0),
        "patch strong, box < detect": dict(t_attack=3.0, h_min_detect=130.0),
    }
    fig, ax = plt.subplots(1, 2, figsize=(13, 4.8))
    for name, kw in runs.items():
        arr, oc = sim(d0=8.0, **kw)
        ax[0].plot(arr[:, 0], arr[:, 1], lw=2, label="%s -> %s" % (name, oc))
    ax[0].axhline(8.0, ls=":", c="gray"); ax[0].axhline(0.6, ls="--", c="r", alpha=.6)
    ax[0].text(0.5, 8.2, "standoff d0=8m", color="gray", fontsize=8)
    ax[0].text(0.5, 0.9, "collision", color="r", fontsize=8)
    ax[0].set_xlabel("time (s)"); ax[0].set_ylabel("TRUE distance to target (m)")
    ax[0].set_title("What the drone actually does (measured r)"); ax[0].legend(fontsize=8); ax[0].set_ylim(0, 12)

    # --- 2) regime map: sweep standoff d0 vs detection floor h_min_detect ---
    D0 = np.linspace(2, 12, 26); HM = np.linspace(0, 220, 26)
    M = np.zeros((len(HM), len(D0)))   # 0 hold/new, 1 collide, 2 lost-stop
    for i, hm in enumerate(HM):
        for j, d0 in enumerate(D0):
            arr, oc = sim(d0=d0, h_min_detect=hm)
            M[i, j] = 1 if oc == "COLLIDE" else (2 if oc.startswith("LOST") else 0)
    from matplotlib.colors import ListedColormap
    cm = ListedColormap(["#2c7fb8", "#d7301f", "#999999"])  # closes/holds, collide, lost
    ax[1].imshow(M, origin="lower", aspect="auto", cmap=cm, vmin=0, vmax=2,
                 extent=[D0[0], D0[-1], HM[0], HM[-1]])
    ax[1].set_xlabel("commanded standoff d0 (m)")
    ax[1].set_ylabel("YOLO detection floor (min box px)")
    ax[1].set_title("Outcome regimes")
    from matplotlib.patches import Patch
    ax[1].legend(handles=[Patch(color="#2c7fb8", label="B: closes to new (too-close) standoff / collide-margin"),
                           Patch(color="#d7301f", label="B: COLLIDE"),
                           Patch(color="#999999", label="A: loses target -> stops")],
                 fontsize=7, loc="upper right")
    plt.tight_layout(); p = os.path.join(OUT, "behavior_probe.png"); plt.savefig(p, dpi=130)
    print("saved", p)

    # --- 3) print the empirical verdict table ---
    print("\n=== EMPIRICAL OUTCOMES (measured r, d0=8m) ===")
    for name, kw in runs.items():
        arr, oc = sim(d0=8.0, **kw)
        dmin = np.nanmin(arr[:, 1])
        print("  %-30s -> %-18s  min_true=%.2fm  final=%.2fm" % (name, oc, dmin, arr[-1, 1]))
    # equilibrium check across patch strengths (constant-r ideal)
    print("\n=== where does it settle? (B regime, detected) ===")
    for d0 in (3, 5, 8):
        arr, oc = sim(d0=d0, h_min_detect=0.0)
        print("  d0=%2dm -> %-20s  settles/min=%.2fm" % (d0, oc, np.nanmin(arr[:, 1])))

if __name__ == "__main__":
    main()


def scenario_chooser():
    """Decide the demo: for a CONSTANT patch strength r and standoff d0, where does the
    drone settle, and when does that cross the collision envelope? Overlay the measured
    patch band and the 3m-cube operating point."""
    import numpy as np, os
    import matplotlib.pyplot as plt
    d0s = np.linspace(1.0, 10.0, 60); rs = np.linspace(0.25, 0.9, 60)
    FIN = np.zeros((len(rs), len(d0s)))
    for i, r in enumerate(rs):
        for j, d0 in enumerate(d0s):
            FIN[i, j] = r * d0            # stable equilibrium of v=kp*(d_true/r - d0)
    fig, ax = plt.subplots(figsize=(7.2, 5.2))
    im = ax.imshow(FIN, origin="lower", aspect="auto", cmap="viridis",
                   extent=[d0s[0], d0s[-1], rs[0], rs[-1]], vmin=0, vmax=6)
    cb = plt.colorbar(im, ax=ax); cb.set_label("final TRUE standoff (m)")
    # collision contour (object+drone envelope ~0.6 m)
    for env, c, ls in [(0.6, "red", "-"), (1.0, "orange", "--")]:
        ax.contour(d0s, rs, FIN, levels=[env], colors=c, linestyles=ls, linewidths=2)
        ax.plot([], [], c=c, ls=ls, lw=2, label="collides if final < %.1fm" % env)
    # measured patch band (r ~ 0.41..0.66)
    ax.axhspan(0.41, 0.66, color="white", alpha=0.18)
    ax.axhline(0.46, color="white", lw=1.5, ls=":")
    ax.text(9.6, 0.47, "our measured patch (r~0.46)", color="white", ha="right", fontsize=8)
    # operating points
    ax.scatter([8.0], [0.46], c="cyan", s=70, ec="k", zorder=5)
    ax.annotate("lab follow d0=8m\n-> parks 3.7m (near miss)", (8.0, 0.46),
                (5.2, 0.80), color="cyan", fontsize=8, arrowprops=dict(arrowstyle="->", color="cyan"))
    ax.scatter([1.5], [0.46], c="magenta", s=70, ec="k", zorder=5)
    ax.annotate("3m-cube d0=1.5m\n-> parks 0.7m (close!)", (1.5, 0.46),
                (2.0, 0.30), color="magenta", fontsize=8, arrowprops=dict(arrowstyle="->", color="magenta"))
    ax.set_xlabel("commanded standoff d0 (m)"); ax.set_ylabel("patch strength r (smaller = stronger)")
    ax.set_title("Scenario chooser: where the fooled drone PARKS\n(final standoff = r x d0)")
    ax.legend(loc="lower right", fontsize=8)
    p = os.path.join(OUT, "scenario_chooser.png"); plt.tight_layout(); plt.savefig(p, dpi=130)
    print("saved", p)
    print("\n=== decision numbers (final standoff = r*d0) ===")
    for d0 in (8.0, 3.0, 1.5):
        for r in (0.46, 0.35, 0.30):
            fin = r * d0
            tag = "COLLIDE" if fin < 0.6 else ("close" if fin < 1.0 else "near-miss")
            print("  d0=%.1fm  r=%.2f -> parks %.2fm  [%s]" % (d0, r, fin, tag))

if __name__ == "__main__":
    scenario_chooser()
