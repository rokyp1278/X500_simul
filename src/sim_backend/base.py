"""백엔드 독립 인터페이스.

PyBullet(빠른 Blackwell-safe), Isaac Sim 5.1(포토리얼+Jetson), ArduPilot SITL(실기동일)이
모두 이 계약을 구현한다. perception/policy/attack 는 이 인터페이스에만 의존하므로
백엔드를 바꿔도 그대로 재사용된다.
"""
from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass
import numpy as np


@dataclass
class DroneState:
    pos: np.ndarray      # (3,) 좌표계는 백엔드에서 일관화(ENU/NED 택1)
    vel: np.ndarray      # (3,)
    att: np.ndarray      # (3,) roll, pitch, yaw


class SimBackend(ABC):
    @abstractmethod
    def reset(self, seed: int) -> None: ...

    @abstractmethod
    def get_camera(self) -> "tuple[np.ndarray, np.ndarray]":
        """(rgb[H,W,3] uint8, depth[H,W] float_m) — adversarial patch 는 이 rgb 에서 효과를 낸다."""

    @abstractmethod
    def get_state(self) -> DroneState: ...

    @abstractmethod
    def send_setpoint(self, vx: float, vy: float, vz: float, yaw_rate: float) -> None:
        """고수준 속도 setpoint. SITL/실기에서는 MAVLink 로 매핑."""

    @abstractmethod
    def step(self) -> None: ...

    @abstractmethod
    def close(self) -> None: ...
