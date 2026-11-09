"""객체 탐지기의 adversarial patch 강건성 평가 (clean vs patched).

드론/비행 제어 없음 — 정지 이미지 위 순수 지각 분석. 데모/논문 Figure 생성.
사용: python src/eval/robustness_eval.py --image <path> --target bus --out outputs
"""
from __future__ import annotations
import argparse
import os
import sys

# 프로젝트 루트를 path 에 (스크립트 직접 실행 지원)
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import numpy as np  # noqa: E402
import cv2  # noqa: E402
from src.perception.detector import TargetPerception  # noqa: E402
from src.attack.patch import PatchAttack  # noqa: E402


def load_rgb(path: str) -> np.ndarray:
    bgr = cv2.imread(path)
    if bgr is None:
        raise FileNotFoundError(path)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", required=True)
    ap.add_argument("--target", default="bus")
    ap.add_argument("--model", default="yolov8n.pt")
    ap.add_argument("--out", default="outputs")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    det = TargetPerception(args.model, args.target)
    rgb = load_rgb(args.image)

    clean_conf, clean_det = det.target_confidence(rgb)
    bbox = clean_det.xyxy if clean_det else None

    patched = PatchAttack(method="occlusion").apply(rgb, bbox)
    patched_conf, _ = det.target_confidence(patched)

    drop = clean_conf - patched_conf
    print(f"TARGET={args.target}")
    print(f"CLEAN_CONF={clean_conf:.3f}  DETECTED={clean_conf > 0}")
    print(f"PATCHED_CONF={patched_conf:.3f}  DETECTED={patched_conf > 0}")
    print(f"CONF_DROP={drop:.3f}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(11, 5))
    ax[0].imshow(rgb); ax[0].set_title(f"clean: {args.target} conf={clean_conf:.2f}"); ax[0].axis("off")
    ax[1].imshow(patched); ax[1].set_title(f"patched: {args.target} conf={patched_conf:.2f}"); ax[1].axis("off")
    out_png = os.path.join(args.out, f"robustness_{args.target}.png")
    fig.tight_layout(); fig.savefig(out_png, dpi=120)
    print(f"FIGURE={out_png}")


if __name__ == "__main__":
    main()
