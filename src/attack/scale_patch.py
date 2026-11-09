"""Scale-manipulation adversarial patch: keep the target DETECTED but SHRINK its box
to a SPECIFIC smaller size (not collapse it -> that is just suppression).

Perception basis for FlyTrap distance-pulling: a box shrunk to r x clean reads as a
~1/r x farther object, so a standoff tracker closes in. The objective drives the box
HEIGHT of the OBJECT'S anchors toward a TARGET height while holding their class score
above a margin; a real-detector eval each few steps keeps the best patch that still
yields a detection. Static-image proof (perception only).
"""
from __future__ import annotations
import numpy as np
import cv2
import torch
import torch.nn.functional as F


def _to_input(rgb, size=640):
    img = cv2.resize(rgb, (size, size)).astype("float32") / 255.0
    return torch.from_numpy(img).permute(2, 0, 1).unsqueeze(0)


def optimize_scale_patch(yolo, rgb, target_idx, bbox_xyxy, cover=0.55,
                         target_ratio=0.55, steps=400, lr=0.02, lam=6.0,
                         margin=0.55, device=None, eval_every=20):
    device = device or ("cuda" if torch.cuda.is_available() else "cpu")
    model = yolo.model.to(device).eval()
    for p in model.parameters():
        p.requires_grad_(False)
    size = 640
    X0 = _to_input(rgb, size).to(device)
    H, W = rgb.shape[:2]; sx, sy = size / W, size / H
    bx1, by1, bx2, by2 = bbox_xyxy[0]*sx, bbox_xyxy[1]*sy, bbox_xyxy[2]*sx, bbox_xyxy[3]*sy
    cx, cy = (bx1+bx2)/2, (by1+by2)/2
    pw, ph = (bx2-bx1)*cover, (by2-by1)*cover
    x1 = int(max(0, cx-pw/2)); y1 = int(max(0, cy-ph/2))
    x2 = int(min(size, cx+pw/2)); y2 = int(min(size, cy+ph/2))
    mask = torch.zeros(1, 3, size, size, device=device); mask[:, :, y1:y2, x1:x2] = 1.0

    def heads(X):
        out = model(X)[0]                 # [1,84,8400]
        return out[0, :4, :], out[0, 4+target_idx, :]

    with torch.no_grad():
        box0, c0 = heads(X0)
        # anchors whose decoded centre lies inside the clean bbox AND are confident
        inside = (box0[0] >= bx1) & (box0[0] <= bx2) & (box0[1] >= by1) & (box0[1] <= by2)
        sel = inside & (c0 > 0.25)
        if sel.sum() < 3:
            sel = c0 > max(0.1, float(c0.max()) * 0.5)
        clean_h = float(box0[3][sel].median())
        clean_c = float(c0.max())
    target_h = target_ratio * clean_h
    sel_idx = torch.where(sel)[0]

    patch = X0.clone().detach().requires_grad_(True)
    opt = torch.optim.Adam([patch], lr=lr)

    # real-detector eval (post-NMS) to keep the BEST detected+shrunk patch
    def real_eval(np_img):
        res = yolo.predict(np_img, conf=0.25, verbose=False)[0]
        best = None
        for b in res.boxes:
            if res.names[int(b.cls)] == yolo.names[target_idx]:
                x1b, y1b, x2b, y2b = [float(v) for v in b.xyxy[0]]; cc = float(b.conf)
                if best is None or cc > best[0]:
                    best = (cc, (y2b - y1b))
        return best  # (conf, box_h) or None

    best = {"h": 1e9, "img": None, "conf": 0.0}
    for it in range(steps):
        X = X0 * (1 - mask) + patch.clamp(0, 1) * mask
        box, c = heads(X)
        csel = c[sel_idx]; hsel = box[3][sel_idx]
        keep = F.softplus(margin - csel).mean()              # hold confidence up
        shrink = ((hsel - target_h) ** 2).mean() / (clean_h ** 2)   # hit TARGET height
        loss = keep + lam * shrink
        opt.zero_grad(); loss.backward(); opt.step()
        with torch.no_grad():
            patch.clamp_(0, 1)
        if it % eval_every == 0 or it == steps - 1:
            with torch.no_grad():
                Xi = X0 * (1 - mask) + patch.clamp(0, 1) * mask
                img = (Xi[0].permute(1, 2, 0).cpu().numpy() * 255).astype("uint8")
            r = real_eval(img)
            if r is not None and r[1] < best["h"]:        # still detected AND smaller
                best = {"h": r[1], "img": img.copy(), "conf": r[0]}
    return best, clean_h, clean_c, target_h
