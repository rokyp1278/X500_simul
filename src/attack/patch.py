"""탐지기 강건성 평가용 adversarial patch (순수 이미지).

baseline(method="occlusion"): 타깃 영역에 고주파 텍스처 patch 를 합성 → 탐지 교란 정도 측정.
upgrade(Phase 3b, method="dpatch"): ART RobustDPatch 로 detector gradient 최적화 patch.
  (ART estimator 로 ultralytics YOLO 래핑 필요 — 다음 단계에서 연결.)
목적은 "patch 가 인지를 얼마나 무너뜨리나"의 정량 측정(방어연구).
"""
from __future__ import annotations
import numpy as np


class PatchAttack:
    def __init__(self, method: str = "occlusion", size_frac: float = 0.35, seed: int = 0):
        self.method = method
        self.size_frac = size_frac
        self.rng = np.random.default_rng(seed)

    def _make_patch(self, h: int, w: int) -> np.ndarray:
        # 고주파 랜덤 텍스처(uint8 RGB) — 탐지기 교란 baseline
        return self.rng.integers(0, 256, size=(h, w, 3), dtype=np.uint8)

    def apply(self, image_rgb: np.ndarray, bbox=None) -> np.ndarray:
        """clean rgb -> patched rgb. bbox 주어지면 그 영역 중앙, 아니면 이미지 중앙."""
        if self.method == "dpatch":
            raise NotImplementedError("Phase 3b: ART RobustDPatch 연결 예정")
        img = image_rgb.copy()
        H, W = img.shape[:2]
        if bbox is None:
            bw, bh = int(W * self.size_frac), int(H * self.size_frac)
            x1, y1 = (W - bw) // 2, (H - bh) // 2
        else:
            bx1, by1, bx2, by2 = [int(v) for v in bbox]
            bw = max(1, int((bx2 - bx1) * self.size_frac))
            bh = max(1, int((by2 - by1) * self.size_frac))
            x1 = bx1 + ((bx2 - bx1) - bw) // 2
            y1 = by1 + ((by2 - by1) - bh) // 2
        x2, y2 = min(W, x1 + bw), min(H, y1 + bh)
        x1, y1 = max(0, x1), max(0, y1)
        img[y1:y2, x1:x2] = self._make_patch(y2 - y1, x2 - x1)
        return img
