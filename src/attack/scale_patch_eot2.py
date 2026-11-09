"""Render-ROBUST EOT scale patch: trains through the transforms that a real 3D render
applies, so the box-shrink survives the Gazebo billboard (not just flat digital).
EOT per sample = patch on crop -> gaussian blur (mipmap/texture filter) -> brightness
/contrast/per-channel gain (lighting) -> mild perspective (viewing angle) -> resize to
a random scale (distance/resolution) -> paste on canvas -> YOLO. Objective = hold the
object anchors' class score up while regressing their box HEIGHT to target_ratio.
All ops differentiable (conv2d / arithmetic / grid_sample / interpolate).
"""
from __future__ import annotations
import numpy as np, cv2, torch
import torch.nn.functional as F


def _gauss(ksz, sigma, device):
    ax = torch.arange(ksz, device=device) - (ksz-1)/2
    g = torch.exp(-(ax**2)/(2*sigma**2)); g = (g/g.sum())
    k = torch.outer(g, g)
    return k.view(1,1,ksz,ksz).repeat(3,1,1,1)


def _blur(x, sigma, device):
    ksz = max(3, int(sigma*4)|1)
    k = _gauss(ksz, sigma, device)
    return F.conv2d(F.pad(x, [ksz//2]*4, mode="reflect"), k, groups=3)


def _perspective(x, mag, device):
    # coherent mild warp: random 4x4 displacement field upsampled smooth -> grid_sample
    B,C,H,W = x.shape
    base = F.affine_grid(torch.eye(2,3,device=device).unsqueeze(0), x.size(), align_corners=False)
    low = (torch.rand(1,2,4,4, device=device)-0.5)*2*mag
    disp = F.interpolate(low, size=(H,W), mode="bilinear", align_corners=False)
    grid = base + disp.permute(0,2,3,1)
    return F.grid_sample(x, grid, align_corners=False, padding_mode="border")


def train(yolo, crop_rgb, target_idx, scales=(0.35,0.45,0.55,0.7),
          cover=0.6, target_ratio=0.5, steps=400, lr=0.02, lam=8.0, margin=0.5,
          batch=3, device=None, eval_every=25, seed=0):
    device = device or ("cuda" if torch.cuda.is_available() else "cpu")
    model = yolo.model.to(device).eval()
    for p in model.parameters(): p.requires_grad_(False)
    torch.manual_seed(seed); np.random.seed(seed)
    Hc,Wc = crop_rgb.shape[:2]
    crop = torch.from_numpy(crop_rgb.astype("float32")/255.0).permute(2,0,1).unsqueeze(0).to(device)
    ph,pw = int(Hc*cover), int(Wc*cover); py,px=(Hc-ph)//2,(Wc-pw)//2
    mask = torch.zeros(1,3,Hc,Wc,device=device); mask[:,:,py:py+ph,px:px+pw]=1.0
    patch = crop.clone().detach().requires_grad_(True)
    opt = torch.optim.Adam([patch], lr=lr)

    def paste(comp, s):
        cw,ch = max(8,int(Wc*s)), max(8,int(Hc*s))
        r = F.interpolate(comp, size=(ch,cw), mode="bilinear", align_corners=False).clamp(0,1)
        oy,ox = (640-ch)//2,(640-cw)//2
        cv = torch.full((1,3,640,640), 114/255.0, device=device)
        cv[:,:,oy:oy+ch,ox:ox+cw] = r
        return cv, (ox,oy,cw,ch)
    def head(cv):
        out = model(cv)[0]; return out[0,:4,:], out[0,4+target_idx,:]

    # clean heights per scale (with blur+downsample realism, no patch)
    cleanh={}
    for s in scales:
        with torch.no_grad():
            cv,(ox,oy,cw,ch)=paste(_blur(crop,1.0,device),s); box,c=head(cv)
            ins=(box[0]>=ox)&(box[0]<=ox+cw)&(box[1]>=oy)&(box[1]<=oy+ch)&(c>0.2)
            cleanh[s]=float(box[3][ins].median()) if ins.sum()>0 else float(box[3].max())
    def real_h(img):
        res=yolo.predict(img,conf=0.25,verbose=False)[0]; best=None
        for b in res.boxes:
            if res.names[int(b.cls)]==yolo.names[target_idx]:
                x1,y1,x2,y2=[float(v) for v in b.xyxy[0]];cc=float(b.conf)
                if best is None or cc>best[0]: best=(cc,y2-y1)
        return best

    best={"score":1e9,"patch":None,"comp":None}
    for it in range(steps):
        comp0 = crop*(1-mask)+patch.clamp(0,1)*mask
        loss=0.0
        for s in np.random.choice(scales,size=batch,replace=False):
            # --- render-robustness transforms (EOT) ---
            x = _blur(comp0, float(np.random.uniform(0.6,1.6)), device)      # texture filter
            b = float(np.random.uniform(0.7,1.25)); x = (x*b).clamp(0,1)     # brightness
            gain = torch.tensor(np.random.uniform(0.85,1.15,3),dtype=torch.float32,device=device).view(1,3,1,1)
            x = (x*gain).clamp(0,1)                                          # per-channel (light colour)
            if np.random.rand()<0.7: x = _perspective(x, 0.03, device)       # viewing angle
            cv,(ox,oy,cw,ch)=paste(x,float(s))
            box,c=head(cv)
            ins=(box[0]>=ox)&(box[0]<=ox+cw)&(box[1]>=oy)&(box[1]<=oy+ch)&(c>0.2)
            idx=torch.where(ins)[0]
            if idx.numel()==0: continue
            th=target_ratio*cleanh[float(s)]
            loss = loss + F.softplus(margin-c[idx]).mean() + lam*((box[3][idx]-th)**2).mean()/(cleanh[float(s)]**2)
        if not torch.is_tensor(loss): continue
        opt.zero_grad(); loss.backward(); opt.step()
        with torch.no_grad(): patch.clamp_(0,1)
        if it%eval_every==0 or it==steps-1:
            with torch.no_grad(): comp=crop*(1-mask)+patch.clamp(0,1)*mask
            ratios=[];alld=True
            for s in scales:
                with torch.no_grad(): cv,_=paste(_blur(comp,1.0,device),s)
                img=(cv[0].permute(1,2,0).cpu().numpy()*255).astype("uint8")
                r=real_h(img)
                if r is None: alld=False;ratios.append(1.0)
                else: ratios.append(r[1]/cleanh[s])
            sc=(0 if alld else 5)+float(np.mean(ratios))
            if sc<best["score"]:
                with torch.no_grad():
                    best={"score":sc,
                          "patch":(patch.clamp(0,1)[0].permute(1,2,0).cpu().numpy()*255).astype("uint8"),
                          "comp":(comp[0].permute(1,2,0).cpu().numpy()*255).astype("uint8")}
            print("  it%3d score=%.3f ratios=%s"%(it,sc,[round(x,2) for x in ratios]))
    return best, cleanh
