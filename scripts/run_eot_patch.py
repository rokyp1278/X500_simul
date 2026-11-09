import os,sys,cv2,numpy as np
ROOT=r"C:\Users\default.DESKTOP-5PUUIEJ\Desktop\adversarial_uav_demo";sys.path.insert(0,ROOT)
from src.perception.detector import TargetPerception
from src.attack.scale_patch_eot import train_eot, _canvas_from
import torch
det=TargetPerception("yolov8n.pt","bus")
tidx={v:k for k,v in det.model.names.items()}["bus"]
rgb=cv2.cvtColor(cv2.imread(os.path.join(ROOT,".venv","Lib","site-packages","ultralytics","assets","bus.jpg")),cv2.COLOR_BGR2RGB)
_,cd=det.target_confidence(rgb);x1,y1,x2,y2=[int(v) for v in cd.xyxy]
crop=rgb[y1:y2,x1:x2]
scales=(0.4,0.5,0.6,0.8)
best,cleanh,_,_,_=train_eot(det.model,crop,tidx,scales=scales,target_ratio=0.5,steps=350,lr=0.02,lam=8.0,cover=0.62)
print("trained. score=%.3f"%best["score"])
# evaluate the ONE trained patch across ALL scales (real NMS)
compt=torch.from_numpy(best["comp"].astype("float32")/255.0).permute(2,0,1).unsqueeze(0)
if torch.cuda.is_available(): compt=compt.cuda()
print("\n=== 거리(스케일)별 축소 유지 검증 (하나의 패치) ===")
print("scale | clean_h | patched_h | ratio | 겉보기거리")
rows=[]
for s in scales:
    cv,_=_canvas_from(compt,s)
    img=(cv[0].permute(1,2,0).cpu().numpy()*255).astype("uint8")
    res=det.model.predict(img,conf=0.25,verbose=False)[0]
    bh=0;bc=0
    for b in res.boxes:
        if res.names[int(b.cls)]=="bus":
            x1b,y1b,x2b,y2b=[float(v) for v in b.xyxy[0]];c=float(b.conf)
            if c>bc:bc=c;bh=y2b-y1b
    ratio=bh/cleanh[s] if bh>0 else 0
    rows.append((s,cleanh[s],bh,bc,ratio))
    print("%.2f  | %6.0f | %7.0f(conf%.2f) | %.2f | x%.2f"%(s,cleanh[s],bh,bc,ratio,(1/ratio if ratio>0 else 0)))
np.save(os.path.join(ROOT,"outputs","eot_patch.npy"),best["patch"])
cv2.imwrite(os.path.join(ROOT,"outputs","eot_patch.png"),cv2.cvtColor(best["patch"],cv2.COLOR_RGB2BGR))
import json
json.dump([[float(x) for x in r] for r in rows],open(os.path.join(ROOT,"outputs","eot_rows.json"),"w"))
print("saved eot_patch.png / eot_rows.json")
