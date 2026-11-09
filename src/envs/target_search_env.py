"""Gymnasium env: backend + perception(+attack) -> obs/reward.

backend 를 교체하면 PyBullet/Isaac/SITL 로 시뮬이 바뀐다(코드 동일).
obs = observation.include, action = velocity_setpoint, reward = configs/mission.yaml.
TODO(Phase 1): gymnasium.Env 구현, reset/step, geofence/action-limit 안전경계.
"""
from __future__ import annotations


class TargetSearchEnv:
    def __init__(self, backend, perception, config, attack=None):
        self.backend = backend
        self.perception = perception
        self.config = config
        self.attack = attack  # None=clean, 객체=patched

    def reset(self, seed: int = 0):
        raise NotImplementedError  # Phase 1

    def step(self, action):
        raise NotImplementedError  # Phase 1
