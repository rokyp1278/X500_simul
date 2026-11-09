#!/usr/bin/env python3
import glob,os,time,sys,cv2,numpy as np
from pymavlink import mavutil; from ultralytics import YOLO
FR="/tmp/fpv_frames";MAV="tcp:127.0.0.1:5760";TAG=sys.argv[1] if len(sys.argv)>1 else "run"
OUT="/mnt/c/Users/default.DESKTOP-5PUUIEJ/Desktop/adversarial_uav_demo/outputs/sim"
def newest():
    fs=glob.glob(FR+"/*.jpg")
    if not fs:return None
    f=max(fs,key=os.path.getmtime)
    for _ in range(3):
        im=cv2.imread(f)
        if im is not None and im.size:return im
        time.sleep(0.02)
    return None
m=YOLO("yolov8n.pt")
v=mavutil.mavlink_connection(MAV,autoreconnect=True)
v.wait_heartbeat(timeout=20); print("hb")
hs=[];cs=[];last=None;t0=time.time()
while time.time()-t0<22:
    img=newest()
    if img is not None:
        last=img;r=m(img,verbose=False)[0]
        best=None
        for b in r.boxes:
            if r.names[int(b.cls)]=="bus":
                y=float(b.xyxy[0][3])-float(b.xyxy[0][1]);c=float(b.conf[0])
                if best is None or c>best[1]:best=(y,c)
        if best:hs.append(best[0]);cs.append(best[1])
    time.sleep(0.3)
if hs:
    print("RESULT %s: box_h median=%.1f  conf median=%.2f  n=%d"%(TAG,np.median(hs),np.median(cs),len(hs)))
    if last is not None:
        r=m(last,verbose=False)[0]; vis=last.copy()
        for b in r.boxes:
            if r.names[int(b.cls)]=="bus":
                x1,y1,x2,y2=[int(z) for z in b.xyxy[0]];cv2.rectangle(vis,(x1,y1),(x2,y2),(0,255,0),2)
                cv2.putText(vis,"%s h%d c%.2f"%(TAG,y2-y1,float(b.conf[0])),(x1,max(12,y1-4)),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)
        cv2.imwrite(os.path.join(OUT,"single_%s.png"%TAG),vis)
    open(os.path.join(OUT,"single_%s.txt"%TAG),"w").write("%.1f %.2f %d\n"%(np.median(hs),np.median(cs),len(hs)))
else: print("RESULT %s: no bus"%TAG)
