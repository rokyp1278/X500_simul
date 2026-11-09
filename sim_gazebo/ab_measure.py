#!/usr/bin/env python3
"""NO-FLY A/B: the iris sits on the ground; SITL keeps lockstep-stepping the world so
the camera renders the two static billboards. Measure YOLO box height for the clean
(image one side) vs EOT-patched (other side) bus billboard from the live frames."""
import glob, os, time, sys, cv2, numpy as np
from pymavlink import mavutil
from ultralytics import YOLO
FR="/tmp/fpv_frames"; MAV="tcp:127.0.0.1:5760"
OUT="/mnt/c/Users/default.DESKTOP-5PUUIEJ/Desktop/adversarial_uav_demo/outputs/sim"
def newest():
    fs=glob.glob(FR+"/*.jpg")
    if not fs: return None
    f=max(fs,key=os.path.getmtime)
    for _ in range(3):
        im=cv2.imread(f)
        if im is not None and im.size: return im
        time.sleep(0.02)
    return None
def main():
    os.makedirs(OUT,exist_ok=True); print("load YOLO"); m=YOLO("yolov8n.pt")
    # confirm SITL alive (keeps world stepping); tolerate flaky connect
    v=None
    for a in range(15):
        try:
            v=mavutil.mavlink_connection(MAV,autoreconnect=True)
            if v.wait_heartbeat(timeout=4) is not None: print("hb ok sys",v.target_system); break
        except Exception as e: print("connect retry",e)
        try: v.close()
        except Exception: pass
        v=None; time.sleep(2)
    if v is None: print("WARN: no MAVLink; continuing on frames only")
    print("measuring 25s (no flight)")
    rows=[]; t0=time.time(); last=None
    while time.time()-t0<25:
        img=newest()
        if img is not None:
            last=img; r=m(img,verbose=False)[0]
            for b in r.boxes:
                if r.names[int(b.cls)]=="bus":
                    x1,y1,x2,y2=b.xyxy[0].tolist()
                    rows.append(((x1+x2)/2,y2-y1,float(b.conf[0])))
        time.sleep(0.3)
    if not rows or last is None: print("RESULT: no bus detected live"); return
    rows=np.array(rows); W=last.shape[1]
    # save an annotated sample frame
    r=m(last,verbose=False)[0]
    vis=cv2.cvtColor(last,cv2.COLOR_BGR2RGB).copy()
    for b in r.boxes:
        if r.names[int(b.cls)]=="bus":
            x1,y1,x2,y2=[int(v2) for v2 in b.xyxy[0]]
            cv2.rectangle(vis,(x1,y1),(x2,y2),(0,255,0),2)
            cv2.putText(vis,"%.2f h%d"%(float(b.conf[0]),y2-y1),(x1,max(12,y1-4)),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),1)
    cv2.imwrite(os.path.join(OUT,"ab_frame.png"),cv2.cvtColor(vis,cv2.COLOR_RGB2BGR))
    left=rows[rows[:,0]<W/2]; right=rows[rows[:,0]>=W/2]
    def rep(a,n):
        if len(a)==0: print("  %s: 0 det"%n); return None
        mh=np.median(a[:,1]); print("  %s: n=%d box_h=%.0f conf=%.2f"%(n,len(a),mh,np.median(a[:,2]))); return mh
    print("=== LIVE A/B (clean=+y side, patched=-y side) ==="); print("frame W=%d"%W)
    hl=rep(left,"image-LEFT "); hr=rep(right,"image-RIGHT")
    if hl and hr:
        lo,hi=min(hl,hr),max(hl,hr); print("smaller/larger box ratio = %.2f"%(lo/hi))
    np.savetxt(os.path.join(OUT,"ab_live.csv"),rows,delimiter=",",header="cx,box_h,conf",comments="")
    print("saved ab_live.csv + ab_frame.png")
if __name__=="__main__": main()
