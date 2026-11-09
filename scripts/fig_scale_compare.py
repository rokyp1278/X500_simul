import os, sys, cv2, numpy as np
ROOT=r"C:\Users\default.DESKTOP-5PUUIEJ\Desktop\adversarial_uav_demo"; sys.path.insert(0,ROOT)
from src.perception.detector import TargetPerception
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
det=TargetPerception("yolov8n.pt","bus")
busimg=os.path.join(ROOT,".venv","Lib","site-packages","ultralytics","assets","bus.jpg")
clean=cv2.resize(cv2.cvtColor(cv2.imread(busimg),cv2.COLOR_BGR2RGB),(640,640))
patched=cv2.cvtColor(cv2.imread(os.path.join(ROOT,"outputs","scale_patch_50.png")),cv2.COLOR_BGR2RGB)
def box(img):
    r=det.model.predict(img,conf=0.25,verbose=False)[0]; best=None
    for b in r.boxes:
        if r.names[int(b.cls)]=="bus":
            x1,y1,x2,y2=[float(v) for v in b.xyxy[0]];c=float(b.conf)
            if best is None or c>best[0]:best=(c,(x1,y1,x2,y2),y2-y1)
    return best
fig,ax=plt.subplots(1,2,figsize=(11,6))
for a,(img,tt) in zip(ax,[(clean,"CLEAN"),(patched,"SCALE-MANIPULATION PATCH")]):
    a.imshow(img); bb=box(img)
    if bb:
        x1,y1,x2,y2=bb[1]; a.add_patch(plt.Rectangle((x1,y1),x2-x1,y2-y1,fill=False,ec="lime",lw=2.5))
        a.set_title("%s\nbus conf=%.2f  box_h=%.0f px"%(tt,bb[0],bb[2]),fontsize=11)
    a.axis("off")
fig.suptitle("Trained patch: bus STILL detected (conf up) but box shrunk ~2x -> apparent distance ~2x\n"
             "(occlusion could only suppress -> this is the FlyTrap distance-pulling primitive)",fontsize=11)
out=os.path.join(ROOT,"outputs","scale_patch.png");fig.savefig(out,dpi=115,bbox_inches="tight");print("FIG",out)
