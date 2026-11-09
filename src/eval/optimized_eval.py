"""최적화 adversarial patch 강건성 평가 (clean vs optimized-patch).

정지 이미지 위 detector class-score(입력공간) 비교 + Figure.
사용: python src/eval/optimized_eval.py --image <path> --target bus --steps 150
"""
from __future__ import annotations
import argparse
import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import cv2  # noqa: E402
from src.perception.detector import TargetPerception  # noqa: E402
from src.attack.optimized_patch import optimize_patch  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", required=True)
    ap.add_argument("--target", default="bus")
    ap.add_argument("--model", default="yolov8n.pt")
    ap.add_argument("--patch-frac", type=float, default=0.2)
    ap.add_argument("--steps", type=int, default=150)
    ap.add_argument("--lr", type=float, default=0.03)
    ap.add_argument("--out", default="outputs")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    det = TargetPerception(args.model, args.target)
    names_inv = {v: k for k, v in det.model.names.items()}
    target_idx = names_inv[args.target]

    bgr = cv2.imread(args.image)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    _, clean_det = det.target_confidence(rgb)
    bbox = clean_det.xyxy if clean_det else None

    vis_clean, vis_patched, clean_score, patched_score = optimize_patch(
        det.model, rgb, target_idx, patch_frac=args.patch_frac, steps=args.steps, lr=args.lr,
        bbox_xyxy=bbox)

    drop = clean_score - patched_score
    print(f"TARGET={args.target} (idx {target_idx})")
    print(f"PATCH_FRAC={args.patch_frac}  STEPS={args.steps}")
    print(f"CLEAN_SCORE={clean_score:.3f}")
    print(f"PATCHED_SCORE={patched_score:.3f}")
    print(f"SCORE_DROP={drop:.3f}  ({100*drop/max(clean_score,1e-6):.1f}% 감소)")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(11, 5))
    ax[0].imshow(vis_clean); ax[0].set_title(f"clean: {args.target} score={clean_score:.2f}"); ax[0].axis("off")
    ax[1].imshow(vis_patched); ax[1].set_title(f"optimized patch: score={patched_score:.2f}"); ax[1].axis("off")
    out_png = os.path.join(args.out, f"optimized_{args.target}.png")
    fig.tight_layout(); fig.savefig(out_png, dpi=120)
    print(f"FIGURE={out_png}")


if __name__ == "__main__":
    main()
