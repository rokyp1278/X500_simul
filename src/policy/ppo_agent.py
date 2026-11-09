"""SB3 PPO 정책: 저차원 상태 -> 속도 setpoint.

clean perception 으로 학습(기본). patch 하에서의 거동은 "학습된 반응"이지 하드코딩 아님
(= 창발적 판단). 후속: adversarial training 으로 robust 정책(ADV-0 류).
TODO(Phase 1): from stable_baselines3 import PPO ; env=TargetSearchEnv ; train/save/load.
"""
from __future__ import annotations


class PPOAgent:
    def __init__(self, config: dict):
        self.config = config
        # TODO: self.model = PPO("MlpPolicy", env, ...)

    def train(self, total_steps: int):
        raise NotImplementedError  # Phase 1

    def act(self, obs):
        raise NotImplementedError  # Phase 1
