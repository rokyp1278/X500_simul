"""닫힌 루프 거리추종(ATT) 드론의 distance-pulling 시뮬 (NDSS'26 FlyTrap 재현, 시뮬 전용).

핀홀 모델: box_px(d) = f * S / d. 드론은 박스로 거리를 추정(d_box = f*S/box)하고
그 추정을 d0 로 유지하려 전후진. 패치는 박스를 r(t)배 축소 -> 드론은 d_box = d/r 로
"멀다"고 오판 -> 전진 -> d -> r*d0 (Theorem 1) -> progressive r 이면 충돌까지.

range 축(접근 방향) 1D 동역학. 3D 확장 = 측면/고도 yaw-centering 추가(후속).
비행제어가 아니라 현상 재현·방어 평가용 kinematic 시뮬.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np


@dataclass
class ScenarioConfig:
    f_px: float
    target_size_m: float
    d0_m: float
    kp: float
    dt: float
    steps: int
    target_velocity_m_s: float
    t_attack_s: float
    shrink_mode: str
    r_constant: float
    r_rate_per_s: float
    r_min: float
    collision_m: float
    defense_enabled: bool
    anchor_noise_m: float
    tau: float
    seed: int = 0


def box_px(d: float, cfg: ScenarioConfig) -> float:
    return cfg.f_px * cfg.target_size_m / max(d, 1e-3)


def shrink_factor(t: float, cfg: ScenarioConfig) -> float:
    if cfg.shrink_mode == "none" or t < cfg.t_attack_s:
        return 1.0
    if cfg.shrink_mode == "constant":
        return cfg.r_constant
    # progressive (PDP): 축소율이 시간에 따라 더 강해짐 -> 지속적 pulling
    return max(cfg.r_min, 1.0 - cfg.r_rate_per_s * (t - cfg.t_attack_s))


def simulate(cfg: ScenarioConfig) -> dict:
    rng = np.random.default_rng(cfg.seed)
    x_t = cfg.d0_m        # 타깃 위치(접근축)
    x_d = 0.0             # 드론 위치
    d0 = cfg.d0_m
    logs = {k: [] for k in ["t", "d", "d_box", "d_anchor", "box", "box_clean", "v", "alarm"]}
    collided, collision_t, alarm_t = False, None, None

    for i in range(cfg.steps):
        t = i * cfg.dt
        x_t += cfg.target_velocity_m_s * cfg.dt
        d = x_t - x_d

        r = shrink_factor(t, cfg)
        box_clean = box_px(d, cfg)
        box = r * box_clean
        d_box = cfg.f_px * cfg.target_size_m / max(box, 1e-3)   # 드론이 믿는 거리 (= d/r)

        # 거리제어: d_box 를 d0 로 유지 -> 멀다고 믿으면 전진
        v = cfg.kp * (d_box - d0)

        # 방어: 박스-추정거리 vs 독립 거리앵커(depth) 일관성
        alarm = False
        d_anchor = d + rng.normal(0, cfg.anchor_noise_m)
        if cfg.defense_enabled:
            mismatch = abs(d_box - d_anchor) / max(d_anchor, 1e-3)
            alarm = mismatch > cfg.tau
            if alarm and alarm_t is None and t >= cfg.t_attack_s:
                alarm_t = t

        for k, val in [("t", t), ("d", d), ("d_box", d_box), ("d_anchor", d_anchor),
                       ("box", box), ("box_clean", box_clean), ("v", v), ("alarm", alarm)]:
            logs[k].append(val)

        x_d += v * cfg.dt           # 전진 = 거리 감소
        if (x_t - x_d) <= cfg.collision_m and not collided:
            collided, collision_t = True, t
            break

    return {"logs": {k: np.array(v) for k, v in logs.items()},
            "collided": collided, "collision_t": collision_t,
            "alarm_t": alarm_t, "d0": d0}
