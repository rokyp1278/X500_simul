"""최적화된 adversarial patch (detector gradient 기반).

입력공간(640x640)에서 타깃 클래스의 최고 탐지 점수를 최소화하도록 patch 픽셀을 최적화한다.
정지 이미지 위 순수 지각-강건성 측정(방어연구) — 비행/자율제어와 무관.
occlusion baseline(patch.py)과 달리, 작은 patch로도 신뢰도를 크게 떨어뜨린다.

metric: 모델 raw head 출력 out[0]=[1,84,8400] 의 class 점수(이미 sigmoid)에서
        타깃 클래스 최대값 = "탐지기가 그 사물을 얼마나 확신하는가".
"""
from __future__ import annotations
import numpy as np
import cv2
import torch


def _to_input(image_rgb: np.ndarray, size: int = 640) -> torch.Tensor:
    img = cv2.resize(image_rgb, (size, size)).astype("float32") / 255.0
    return torch.from_numpy(img).permute(2, 0, 1).unsqueeze(0)  # [1,3,S,S]


def optimize_patch(yolo, image_rgb, target_idx, patch_frac=0.2, steps=150, lr=0.03,
                   bbox_xyxy=None, device=None):
    device = device or ("cuda" if torch.cuda.is_available() else "cpu")
    model = yolo.model.to(device).eval()
    for p in model.parameters():
        p.requires_grad_(False)

    size = 640
    X0 = _to_input(image_rgb, size).to(device)

    ph = pw = int(size * patch_frac)
    if bbox_xyxy is not None:
        H, W = image_rgb.shape[:2]
        sx, sy = size / W, size / H
        cx = int((bbox_xyxy[0] + bbox_xyxy[2]) / 2 * sx)
        cy = int((bbox_xyxy[1] + bbox_xyxy[3]) / 2 * sy)
    else:
        cx = cy = size // 2
    x1 = max(0, cx - pw // 2); y1 = max(0, cy - ph // 2)
    x2 = min(size, x1 + pw); y2 = min(size, y1 + ph)
    mask = torch.zeros(1, 3, size, size, device=device)
    mask[:, :, y1:y2, x1:x2] = 1.0

    patch = X0.clone().detach().requires_grad_(True)
    opt = torch.optim.Adam([patch], lr=lr)

    def target_score(X):
        out = model(X)[0]              # [1,84,8400]
        cls = out[:, 4:, :]            # [1,80,8400] (sigmoid 적용됨)
        return cls[:, target_idx, :].max()

    with torch.no_grad():
        clean_score = float(target_score(X0))

    for _ in range(steps):
        X = X0 * (1 - mask) + patch.clamp(0, 1) * mask
        s = target_score(X)
        opt.zero_grad(); s.backward(); opt.step()
        with torch.no_grad():
            patch.clamp_(0, 1)

    with torch.no_grad():
        Xp = X0 * (1 - mask) + patch.clamp(0, 1) * mask
        patched_score = float(target_score(Xp))
        vis_clean = (X0[0].permute(1, 2, 0).cpu().numpy() * 255).astype("uint8")
        vis_patched = (Xp[0].permute(1, 2, 0).cpu().numpy() * 255).astype("uint8")

    return vis_clean, vis_patched, clean_score, patched_score
