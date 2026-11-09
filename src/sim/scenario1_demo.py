"""SCENARIO 1 (confirmed): safety-standoff VIOLATION + defense catch.
Driven by the live-measured per-distance shrink r (outputs/eot_rows.json). The drone
holds standoff d0; the patch appears; it is fooled into closing to the new equilibrium
r*d0 (~46% of d0) and PARKS there -- it does NOT crash and does NOT land. The
independent depth anchor flags the box/anchor mismatch the instant the patch appears,
well before the drone reaches its (dangerously close) parking distance.
"""
from __future__ import annotations
import json, os, numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ROWS = json.load(open(os.path.join(ROOT, "outputs", "eot_rows.json")))
OUT = os.path.join(ROOT, "outputs")
CH = np.array([r[1] for r in ROWS]); RT = np.array([r[4] for r in ROWS])
o = np.argsort(CH); CH, RT = CH[o], RT[o]
r_of = lambda h: float(np.interp(h, CH, RT))
F_PX, S = 600.0, 4.0

def run(d0=8.0, d_start=8.0, t_attack=4.0, kp=0.5, vmax=1.2, dt=0.1, t_max=40.0,
        tau=0.15, anchor_noise=0.08, seed=0):
    rng = np.random.default_rng(seed); d = d_start; t = 0.0
    log = []; alarm_t = None
    while t < t_max:
        th = F_PX * S / max(d, 1e-3)
        atk = t >= t_attack
        ph = th * (r_of(th) if atk else 1.0)
        d_box = F_PX * S / max(ph, 1e-3)                 # believed distance
        v = float(np.clip(kp * (d_box - d0), -vmax, vmax))
        d_anchor = d + rng.normal(0, anchor_noise)       # independent depth sensor (D435i)
        mism = abs(d_box - d_anchor) / max(d_anchor, 1e-3)
        if atk and mism > tau and alarm_t is None: alarm_t = t
        d = max(d - v * dt, 0.0)
        log.append((t, d, d_box, mism, v)); t += dt
    return np.array(log), alarm_t

def main():
    d0 = 8.0
    arr, alarm_t = run(d0=d0)
    park = arr[-1, 1]
    fig, ax = plt.subplots(1, 2, figsize=(13, 4.6))
    # panel 1: distance over time
    ax[0].plot(arr[:, 0], arr[:, 1], lw=2.2, color="#1f77b4", label="TRUE distance")
    ax[0].plot(arr[:, 0], arr[:, 2], lw=1.5, ls="--", color="#ff7f0e", label="distance drone BELIEVES")
    ax[0].axhline(d0, ls=":", c="gray"); ax[0].text(0.3, d0 + 0.15, "commanded standoff %.0fm" % d0, color="gray", fontsize=8)
    ax[0].axhspan(0, d0, color="red", alpha=0.05)
    ax[0].axvline(4.0, color="purple", lw=1, ls="-", alpha=.7); ax[0].text(4.1, 9.2, "patch appears", color="purple", fontsize=8)
    ax[0].axhline(park, ls="-", c="red", alpha=.5); ax[0].text(0.3, park - 0.5, "parks at %.1fm (%.0f%% of standoff)" % (park, 100*park/d0), color="red", fontsize=8)
    if alarm_t is not None:
        ax[0].axvline(alarm_t, color="green", lw=1.5); ax[0].text(alarm_t + 0.1, 1.0, "DEFENSE alarm\n(t=%.1fs)" % alarm_t, color="green", fontsize=8)
    ax[0].set_xlabel("time (s)"); ax[0].set_ylabel("distance (m)"); ax[0].set_ylim(0, 10)
    ax[0].set_title("Scenario 1: patch -> drone PARKS too close (no crash, no land)")
    ax[0].legend(fontsize=8, loc="center right")
    # panel 2: defense mismatch
    ax[1].plot(arr[:, 0], arr[:, 3], lw=2, color="#2ca02c")
    ax[1].axhline(0.15, ls="--", c="k"); ax[1].text(0.3, 0.17, "alarm threshold tau=0.15", fontsize=8)
    ax[1].axvline(4.0, color="purple", lw=1, ls="-", alpha=.7)
    ax[1].set_xlabel("time (s)"); ax[1].set_ylabel("box vs independent-anchor mismatch")
    ax[1].set_title("Defense: mismatch spikes the instant the patch appears")
    plt.tight_layout(); p = os.path.join(OUT, "scenario1_demo.png"); plt.savefig(p, dpi=130)
    print("saved", p)
    print("d0=%.1fm  parks=%.2fm (%.0f%% violation)  defense_alarm_t=%.1fs  patch_t=4.0s"
          % (d0, park, 100*(1-park/d0), alarm_t if alarm_t else -1))

if __name__ == "__main__":
    main()
