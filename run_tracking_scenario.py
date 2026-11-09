"""distance-pulling 시나리오 실행: clean vs attack, 궤적/거리 그림 + 방어 경보.

사용: python run_tracking_scenario.py --config configs/tracking_scenario.yaml
"""
from __future__ import annotations
import argparse
import os
import sys
import copy

_ROOT = os.path.dirname(os.path.abspath(__file__))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import yaml  # noqa: E402
from src.sim.kinematic_tracker import ScenarioConfig, simulate  # noqa: E402


def load_cfg(path: str) -> ScenarioConfig:
    with open(path, encoding="utf-8") as f:
        y = yaml.safe_load(f)
    return ScenarioConfig(
        f_px=y["camera"]["f_px"], target_size_m=y["camera"]["target_size_m"],
        d0_m=y["scenario"]["d0_m"], kp=y["scenario"]["kp"], dt=y["scenario"]["dt"],
        steps=y["scenario"]["steps"], target_velocity_m_s=y["scenario"]["target_velocity_m_s"],
        t_attack_s=y["attack"]["t_attack_s"], shrink_mode=y["attack"]["shrink_mode"],
        r_constant=y["attack"]["r_constant"], r_rate_per_s=y["attack"]["r_rate_per_s"],
        r_min=y["attack"]["r_min"], collision_m=y["collision_m"],
        defense_enabled=y["defense"]["enabled"], anchor_noise_m=y["defense"]["anchor_noise_m"],
        tau=y["defense"]["tau"], seed=y.get("seed", 0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="configs/tracking_scenario.yaml")
    ap.add_argument("--out", default="outputs")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    cfg_atk = load_cfg(args.config)
    cfg_clean = copy.replace(cfg_atk, shrink_mode="none") if hasattr(copy, "replace") else _clone_none(cfg_atk)

    clean = simulate(cfg_clean)
    atk = simulate(cfg_atk)

    def summarize(name, res):
        lg = res["logs"]
        print(f"[{name}] 최종거리={lg['d'][-1]:.2f}m  충돌={res['collided']}"
              + (f" @t={res['collision_t']:.1f}s" if res["collided"] else "")
              + (f"  방어경보@t={res['alarm_t']:.1f}s" if res["alarm_t"] is not None else "  방어경보=없음"))

    summarize("clean ", clean)
    summarize("attack", atk)
    if atk["collided"] and atk["alarm_t"] is not None:
        margin = atk["collision_t"] - atk["alarm_t"]
        print(f"=> 방어가 충돌 {margin:.1f}s 전에 경보 (lead time)")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(13, 5))

    ax[0].plot(clean["logs"]["t"], clean["logs"]["d"], label="clean (holds standoff)", lw=2)
    ax[0].plot(atk["logs"]["t"], atk["logs"]["d"], label="attack (pulled in)", lw=2, color="crimson")
    ax[0].axhline(cfg_atk.collision_m, ls="--", color="k", alpha=0.5, label=f"collision {cfg_atk.collision_m}m")
    ax[0].axvline(cfg_atk.t_attack_s, ls=":", color="gray", label="patch appears")
    if atk["alarm_t"] is not None:
        ax[0].axvline(atk["alarm_t"], ls="-.", color="green", label=f"defense alarm {atk['alarm_t']:.1f}s")
    if atk["collided"]:
        ax[0].scatter([atk["collision_t"]], [cfg_atk.collision_m], color="crimson", zorder=5, s=60)
    ax[0].set_xlabel("time (s)"); ax[0].set_ylabel("drone-target distance (m)")
    ax[0].set_title("Distance-pulling: distance vs time"); ax[0].legend(); ax[0].grid(alpha=0.3)

    ax[1].plot(atk["logs"]["t"], atk["logs"]["d"], label="true distance d", lw=2, color="crimson")
    ax[1].plot(atk["logs"]["t"], atk["logs"]["d_box"], label="believed d_box (deceived)", lw=2, ls="--", color="orange")
    ax[1].plot(atk["logs"]["t"], atk["logs"]["d_anchor"], label="independent anchor (depth)", lw=1, alpha=0.6, color="blue")
    ax[1].axvline(cfg_atk.t_attack_s, ls=":", color="gray")
    ax[1].set_xlabel("time (s)"); ax[1].set_ylabel("distance (m)")
    ax[1].set_title("Defense: believed vs true/anchor (mismatch = attack)"); ax[1].legend(); ax[1].grid(alpha=0.3)

    out_png = os.path.join(args.out, "distance_pulling.png")
    fig.tight_layout(); fig.savefig(out_png, dpi=120)
    print(f"FIGURE={out_png}")


def _clone_none(cfg):
    import dataclasses
    return dataclasses.replace(cfg, shrink_mode="none")


if __name__ == "__main__":
    main()
