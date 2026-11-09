"""트랙1 백엔드: PyBullet(gym-pybullet-drones). Blackwell 무관(CPU 물리).

X500 근사 쿼드로터 + 타깃 사물 + RGB/depth 카메라 렌더.
TODO(Phase 1): gym-pybullet-drones 초기화, 카메라(getCameraImage: rgb+depth),
속도 setpoint -> 기체 제어, 타깃 객체 배치/텍스처(이후 patch 합성 지점).
"""
from __future__ import annotations
import numpy as np
from src.sim_backend.base import SimBackend, DroneState


class PyBulletBackend(SimBackend):
    def __init__(self, config: dict, gui: bool = False):
        self.config = config
        self.gui = gui
        # TODO: import pybullet / gym_pybullet_drones ; 환경 생성

    def reset(self, seed: int) -> None: raise NotImplementedError
    def get_camera(self): raise NotImplementedError
    def get_state(self) -> DroneState: raise NotImplementedError
    def send_setpoint(self, vx, vy, vz, yaw_rate) -> None: raise NotImplementedError
    def step(self) -> None: raise NotImplementedError
    def close(self) -> None: raise NotImplementedError
