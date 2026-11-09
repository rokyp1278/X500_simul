import os,sys,cv2,numpy as np
ROOT=r"C:\Users\default.DESKTOP-5PUUIEJ\Desktop\adversarial_uav_demo";sys.path.insert(0,ROOT)
from src.perception.detector import TargetPerception
from src.attack.scale_patch_eot2 import train
det=TargetPerception("yolov8n.pt","bus");tidx={v:k for k,v in det.model.names.items()}["bus"]
rgb=cv2.cvtColor(cv2.imread(os.path.join(ROOT,".venv","Lib","site-packages","ultralytics","assets","bus.jpg")),cv2.COLOR_BGR2RGB)
_,cd=det.target_confidence(rgb);x1,y1,x2,y2=[int(v) for v in cd.xyxy];crop=rgb[y1:y2,x1:x2]
best,cleanh=train(det.model,crop,tidx,steps=350,target_ratio=0.5,lam=9.0)
print("done score=%.3f"%best["score"])
cv2.imwrite(os.path.join(ROOT,"sim_gazebo","bus_patched.png"),cv2.cvtColor(best["comp"],cv2.COLOR_RGB2BGR))
cv2.imwrite(os.path.join(ROOT,"outputs","eot2_patch.png"),cv2.cvtColor(best["comp"],cv2.COLOR_RGB2BGR))
print("saved robust patch -> sim_gazebo/bus_patched.png")
