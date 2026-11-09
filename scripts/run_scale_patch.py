import os, sys, numpy as np, cv2
ROOT = r"C:\Users\default.DESKTOP-5PUUIEJ\Desktop\adversarial_uav_demo"
sys.path.insert(0, ROOT)
from src.perception.detector import TargetPerception
from src.attack.scale_patch import optimize_scale_patch

det = TargetPerception("yolov8n.pt", "bus")
names_inv = {v:k for k,v in det.model.names.items()}; tidx = names_inv["bus"]
busimg = os.path.join(ROOT, ".venv","Lib","site-packages","ultralytics","assets","bus.jpg")
rgb = cv2.cvtColor(cv2.imread(busimg), cv2.COLOR_BGR2RGB)
clean640 = cv2.resize(rgb,(640,640))

def real_box(img):
    res = det.model.predict(img, conf=0.25, verbose=False)[0]
    best=None
    for b in res.boxes:
        if res.names[int(b.cls)]=="bus":
            x1,y1,x2,y2=[float(v) for v in b.xyxy[0]]; c=float(b.conf)
            if best is None or c>best[0]: best=(c,(y2-y1),(x1,y1,x2,y2))
    return best
cb = real_box(clean640)
print("CLEAN  conf=%.3f box_h=%.1f" % (cb[0], cb[1]))

_, cd = det.target_confidence(rgb); bbox = cd.xyxy
for tr in (0.6, 0.5, 0.4):
    best, ch, cc, th = optimize_scale_patch(det.model, rgb, tidx, bbox,
            cover=0.55, target_ratio=tr, steps=300, lr=0.02, lam=6.0)
    if best["img"] is None:
        print("target %.0f%%: NO detected patch (collapsed to suppression)" % (tr*100)); continue
    ratio = best["h"]/cb[1]
    print("target %.0f%%: PATCHED conf=%.3f box_h=%.1f (x%.2f clean) -> apparent dist x%.2f"
          % (tr*100, best["conf"], best["h"], ratio, 1.0/ratio))
    cv2.imwrite(os.path.join(ROOT,"outputs","scale_patch_%d.png"%int(tr*100)),
                cv2.cvtColor(best["img"], cv2.COLOR_RGB2BGR))
