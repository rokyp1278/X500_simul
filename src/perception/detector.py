"""YOLO 객체 탐지 래퍼 (지각-강건성 평가용, 순수 이미지 분석).

adversarial patch 는 이 detector 의 입력(rgb)에서 confidence 를 떨어뜨린다.
비행/자율제어와 무관 — 정지 이미지에서 탐지 신뢰도를 측정하는 모듈.
"""
from __future__ import annotations
from dataclasses import dataclass


@dataclass
class Detection:
    cls_name: str
    confidence: float
    xyxy: tuple  # (x1, y1, x2, y2)


class TargetPerception:
    def __init__(self, model: str = "yolov8n.pt", target_class: str = "bus", conf: float = 0.25):
        from ultralytics import YOLO
        self.model = YOLO(model)
        self.target_class = target_class
        self.conf = conf

    def detect(self, image) -> "list[Detection]":
        """image: 경로 또는 HxWx3 ndarray(RGB). 모든 탐지 반환."""
        res = self.model.predict(image, conf=self.conf, verbose=False)[0]
        names = res.names
        out = []
        for b in res.boxes:
            out.append(Detection(
                cls_name=names[int(b.cls)],
                confidence=float(b.conf),
                xyxy=tuple(float(v) for v in b.xyxy[0]),
            ))
        return out

    def target_confidence(self, image) -> "tuple[float, Detection | None]":
        """지정 사물(target_class)의 최고 신뢰도와 그 탐지. 없으면 (0.0, None)."""
        tgt = [d for d in self.detect(image) if d.cls_name == self.target_class]
        if not tgt:
            return 0.0, None
        best = max(tgt, key=lambda d: d.confidence)
        return best.confidence, best
