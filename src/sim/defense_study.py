"""방어 연구: distance-pulling 공격 vs 앵커-일관성 방어 정량화 (numerical, 시뮬 전용).

세 가지:
  S1. 공격 강도(축소율) × 20-seed -> 충돌률 / 방어 탐지율 / lead time
  S2. 적응형(은밀) 공격 tradeoff -> 끌어온 최종거리 vs 최대 불일치(mismatch).
      "충돌시키려면 r 을 작게 -> box/anchor 불일치가 커짐 -> 반드시 탐지" 를 보임.
  S3. 양성(benign) 거동 오경보율(FAR) vs tau, 그리고 ROC(탐지율 vs FAR).

핵심 방어 원리: 드론이 박스로 믿는 거리 d_box 와 독립 거리앵커(depth) 의 상대 불일치
mismatch=(1-r)/r. 충돌 조건 r<=collision/d0 이면 mismatch 가 매우 커져 어떤 실용적 tau
로도 탐지됨 -> 기하-속이기 공격은 독립 앵커와 근본적으로 모순.
"""
from __future__ import annotations
import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_PROJ = os.path.dirname(_ROOT)
if _PROJ not in sys.path:
    sys.path.insert(0, _PROJ)

import numpy as np  # noqa: E402
from src.sim.kinematic_tracker import ScenarioConfig, simulate  # noqa: E402

BASE = dict(f_px=600.0, target_size_m=1.7, d0_m=8.0, kp=0.6, dt=0.05, steps=6000,
            target_velocity_m_s=0.0, t_attack_s=5.0, shrink_mode="progressive",
            r_constant=0.5, r_rate_per_s=0.05, r_min=0.02, collision_m=0.5,
            defense_enabled=True, anchor_noise_m=0.1, tau=0.15)


def cfg(**kw):
    d = dict(BASE); d.update(kw)
    return ScenarioConfig(**d)


def run(c):
    res = simulate(c)
    lg = res["logs"]
    mism = np.abs(lg["d_box"] - lg["d_anchor"]) / np.maximum(lg["d_anchor"], 1e-3)
    return dict(t=lg["t"], d=lg["d"], mism=mism, collided=res["collided"],
                collision_t=res["collision_t"], t_attack=c.t_attack_s)


def alarm_time(r, tau):
    """공격 시작 이후 처음으로 mismatch>tau 가 된 시각 (없으면 None)."""
    idx = np.where((r["t"] >= r["t_attack"]) & (r["mism"] > tau))[0]
    return r["t"][idx[0]] if len(idx) else None


def main():
    out = "outputs"; os.makedirs(out, exist_ok=True)
    seeds = range(20)
    tau = BASE["tau"]

    # ---- S1: 공격 강도 스윕 ----
    rates = [0.01, 0.02, 0.03, 0.05, 0.08, 0.12]
    s1 = []
    for rate in rates:
        coll, det, leads = [], [], []
        for s in seeds:
            r = run(cfg(r_rate_per_s=rate, seed=s))
            coll.append(r["collided"])
            at = alarm_time(r, tau)
            if r["collided"] and at is not None:
                det.append(at < r["collision_t"]); leads.append(r["collision_t"] - at)
            elif at is not None:
                det.append(True)
            else:
                det.append(False)
        s1.append(dict(rate=rate, collision_rate=np.mean(coll),
                       detection_rate=np.mean(det),
                       lead=(np.mean(leads) if leads else float("nan"))))

    # ---- S2: 적응형(은밀) 공격 tradeoff: 상수 축소율 r ----
    rs = np.linspace(0.05, 0.98, 40)
    final_d, peak_m = [], []
    for rr in rs:
        res = run(cfg(shrink_mode="constant", r_constant=float(rr), seed=0))
        final_d.append(res["d"][-1] if not res["collided"] else BASE["collision_m"])
        peak_m.append(np.nanmax(res["mism"][res["t"] >= BASE["t_attack_s"]]))
    final_d, peak_m = np.array(final_d), np.array(peak_m)

    # ---- S3: benign FAR + ROC over tau ----
    taus = np.linspace(0.02, 1.0, 50)
    benign = [run(cfg(shrink_mode="none", target_velocity_m_s=v, seed=s))
              for v in (0.0, 0.5, 1.0) for s in seeds]
    attacks = [run(cfg(r_rate_per_s=0.05, seed=s)) for s in seeds]
    far, tpr = [], []
    for t in taus:
        fa = np.mean([alarm_time(b, t) is not None for b in benign])
        dt = np.mean([(alarm_time(a, t) is not None) and
                      (a["collision_t"] is None or alarm_time(a, t) < a["collision_t"])
                      for a in attacks])
        far.append(fa); tpr.append(dt)
    far, tpr = np.array(far), np.array(tpr)

    # ---- 출력 ----
    print("=== S1: attacker strength (20-seed) ===")
    for row in s1:
        print(f" r_rate={row['rate']:.2f}  collision={row['collision_rate']*100:5.0f}%  "
              f"detect={row['detection_rate']*100:5.0f}%  lead={row['lead']:.1f}s")
    r_eq = BASE["collision_m"] / BASE["d0_m"]
    print(f"\n=== S2: 충돌에 필요한 r <= {r_eq:.3f} -> mismatch >= {(1-r_eq)/r_eq:.1f} (거대) ===")
    print(f"=== S3: tau={tau}: FAR={far[np.argmin(np.abs(taus-tau))]*100:.1f}%  "
          f"TPR={tpr[np.argmin(np.abs(taus-tau))]*100:.1f}% ===")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(1, 3, figsize=(17, 5))
    ax[0].plot([r["rate"] for r in s1], [r["collision_rate"]*100 for r in s1], "o-", label="collision rate")
    ax[0].plot([r["rate"] for r in s1], [r["detection_rate"]*100 for r in s1], "s-", color="green", label="defense detection")
    ax[0].set_xlabel("attacker shrink rate (1/s)"); ax[0].set_ylabel("%")
    ax[0].set_title("S1: attack success vs defense (20-seed)"); ax[0].legend(); ax[0].grid(alpha=0.3)

    ax[1].plot(peak_m, final_d, "o-", color="purple")
    ax[1].axhline(BASE["collision_m"], ls="--", color="crimson", label="collision 0.5m")
    ax[1].axvline(tau, ls="-.", color="green", label=f"defense tau={tau}")
    ax[1].set_xlabel("peak box/anchor mismatch"); ax[1].set_ylabel("final distance (m)")
    ax[1].set_title("S2: stealth tradeoff (collide => huge mismatch)"); ax[1].legend(); ax[1].grid(alpha=0.3)

    ax[2].plot(far*100, tpr*100, "o-", color="navy")
    ax[2].set_xlabel("false alarm rate (benign) %"); ax[2].set_ylabel("detection rate %")
    ax[2].set_title("S3: defense ROC (over tau)"); ax[2].grid(alpha=0.3)
    ax[2].set_xlim(-2, 102); ax[2].set_ylim(-2, 102)

    png = os.path.join(out, "defense_study.png")
    fig.tight_layout(); fig.savefig(png, dpi=120)
    print(f"FIGURE={png}")

    # markdown 요약
    with open(os.path.join(out, "defense_study.md"), "w", encoding="utf-8") as f:
        f.write("# 방어 연구 결과 (numerical, 20-seed)\n\n")
        f.write("## S1 공격강도 vs 방어\n\n| shrink rate(1/s) | 충돌률 | 방어 탐지율 | 평균 lead(s) |\n|---|---|---|---|\n")
        for r in s1:
            f.write(f"| {r['rate']:.2f} | {r['collision_rate']*100:.0f}% | {r['detection_rate']*100:.0f}% | {r['lead']:.1f} |\n")
        f.write(f"\n## S2 은밀성 한계\n충돌하려면 r ≤ {r_eq:.3f} → 박스/앵커 mismatch ≥ {(1-r_eq)/r_eq:.1f}. "
                f"어떤 실용적 tau(≤1)로도 탐지됨 → 기하-속이기는 독립 앵커와 근본적으로 모순.\n")
        i = np.argmin(np.abs(taus - tau))
        f.write(f"\n## S3 tau={tau}: FAR={far[i]*100:.1f}%, TPR={tpr[i]*100:.1f}%\n")
    print("MD=outputs/defense_study.md")


if __name__ == "__main__":
    main()
