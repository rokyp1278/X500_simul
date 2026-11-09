"""EOT scale-manipulation patch: ONE fixed patch texture that shrinks the detected
box across a RANGE of apparent sizes (= distances), so it holds as a tracking drone
approaches. Differentiable compositing: patch overlaid on the target crop, the crop
resized to many scales and pasted on a 640 canvas, YOLO head read, box-height of the
object's anchors regressed toward target_ratio x clean(scale). Pure perception.
"""
from __future__ import annotations
import numpy as np, cv2, torch
import torch.nn.functional as F


def _canvas_from(comp, s, size=640, fill=114/255.0):
    """Differentiably resize comp [1,3,Hc,Wc] by s and center-pad to size. Returns
    (canvas[1,3,size,size], (ox,oy,cw,ch)) giving the pasted region in px."""
    Hc, Wc = comp.shape[-2:]
    cw, ch = max(1, int(Wc*s)), max(1, int(Hc*s))
    r = F.interpolate(comp, size=(ch, cw), mode="bilinear", align_corners=False)
    ox, oy = (size-cw)//2, (size-ch)//2
    canvas = torch.full((1, 3, size, size), fill, device=comp.device)
    canvas[:, :, oy:oy+ch, ox:ox+cw] = r.clamp(0, 1)
    return canvas, (ox, oy, cw, ch)


def train_eot(yolo, crop_rgb, target_idx, scales=(0.3,0.4,0.5,0.6,0.8),
              cover=0.55, target_ratio=0.55, steps=250, lr=0.02, lam=6.0,
              margin=0.5, batch=3, device=None, eval_every=25, seed=0):
    device = device or ("cuda" if torch.cuda.is_available() else "cpu")
    model = yolo.model.to(device).eval()
    for p in model.parameters(): p.requires_grad_(False)
    torch.manual_seed(seed); np.random.seed(seed)
    Hc, Wc = crop_rgb.shape[:2]
    crop = torch.from_numpy(crop_rgb.astype("float32")/255.0).permute(2,0,1).unsqueeze(0).to(device)
    ph, pw = int(Hc*cover), int(Wc*cover)
    py, px = (Hc-ph)//2, (Wc-pw)//2
    mask = torch.zeros(1,3,Hc,Wc, device=device); mask[:,:,py:py+ph,px:px+pw]=1.0
    patch = crop.clone().detach().requires_grad_(True)
    opt = torch.optim.Adam([patch], lr=lr)

    def head(canvas):
        out = model(canvas)[0]
        return out[0,:4,:], out[0,4+target_idx,:]

    # clean box height per scale (no patch), + which anchors are "inside"
    cleanh = {}
    for s in scales:
        with torch.no_grad():
            cv, (ox,oy,cw,ch) = _canvas_from(crop, s)
            box, c = head(cv)
            inside = (box[0]>=ox)&(box[0]<=ox+cw)&(box[1]>=oy)&(box[1]<=oy+ch)&(c>0.2)
            cleanh[s] = (float(box[3][inside].median()) if inside.sum()>0 else float(box[3].max()))
    def real_h(np_img):
        res = yolo.predict(np_img, conf=0.25, verbose=False)[0]; best=None
        for b in res.boxes:
            if res.names[int(b.cls)]==yolo.names[target_idx]:
                x1,y1,x2,y2=[float(v) for v in b.xyxy[0]]; cc=float(b.conf)
                if best is None or cc>best[0]: best=(cc,(y2-y1))
        return best

    best = {"score": 1e9, "patch": None}
    for it in range(steps):
        comp = crop*(1-mask) + patch.clamp(0,1)*mask
        loss = 0.0
        for s in np.random.choice(scales, size=batch, replace=False):
            cv, (ox,oy,cw,ch) = _canvas_from(comp, float(s))
            box, c = head(cv)
            inside = (box[0]>=ox)&(box[0]<=ox+cw)&(box[1]>=oy)&(box[1]<=oy+ch)&(c>0.2)
            idx = torch.where(inside)[0]
            if idx.numel()==0: continue
            th = target_ratio*cleanh[float(s)]
            keep = F.softplus(margin - c[idx]).mean()
            shrink = ((box[3][idx]-th)**2).mean()/(cleanh[float(s)]**2)
            loss = loss + keep + lam*shrink
        opt.zero_grad(); loss.backward(); opt.step()
        with torch.no_grad(): patch.clamp_(0,1)
        if it % eval_every == 0 or it == steps-1:
            with torch.no_grad():
                comp = crop*(1-mask)+patch.clamp(0,1)*mask
            ratios=[]; alld=True
            for s in scales:
                with torch.no_grad():
                    cv,_ = _canvas_from(comp, s)
                    img=(cv[0].permute(1,2,0).cpu().numpy()*255).astype("uint8")
                r=real_h(img)
                if r is None: alld=False; ratios.append(1.0)
                else: ratios.append(r[1]/cleanh[s])
            sc = (0 if alld else 5) + float(np.mean(ratios))   # prefer all-detected, small
            if sc < best["score"]:
                with torch.no_grad():
                    best = {"score": sc, "patch": (patch.clamp(0,1)[0].permute(1,2,0).cpu().numpy()*255).astype("uint8"),
                            "comp": (comp[0].permute(1,2,0).cpu().numpy()*255).astype("uint8")}
    return best, cleanh, _canvas_from, mask, crop
