#!/usr/bin/env python3
"""LIVE autonomous follow mission in Gazebo (adapted from the working l5 flight code).
Drone: takeoff -> follow the bus billboard keeping standoff d0 (distance from YOLO box
height) -> if the billboard is PATCHED the box is shrunk -> perceived distance inflates
-> drone closes in -> AUTONOMOUS safety geofence (true north from mocap/LOCAL_POSITION)
halts it at d_safe before contact. Logs perceived vs true. argv1=tag (clean|patched)."""
import glob,os,time,sys,cv2,numpy as np
from pymavlink import mavutil
from ultralytics import YOLO
FR="/tmp/fpv_frames"; MAV="tcp:127.0.0.1:5760"; TAG=sys.argv[1] if len(sys.argv)>1 else "run"
OUT="/mnt/c/Users/default.DESKTOP-5PUUIEJ/Desktop/adversarial_uav_demo/outputs/sim"
TARGET_X=10.0; TARGET_H_M=3.0; HOVER_Z=3.0; D0=6.0; D_SAFE=3.0
KP=0.5; VMAX=0.6; DT=0.3; T_MAX=45.0
def newest():
    fs=glob.glob(FR+"/*.jpg")
    if not fs:return None
    f=max(fs,key=os.path.getmtime)
    for _ in range(3):
        im=cv2.imread(f)
        if im is not None and im.size:return im
        time.sleep(0.02)
    return None
def box_h(m,img):
    r=m(img,verbose=False)[0];best=0;bc=0
    for b in r.boxes:
        if r.names[int(b.cls)]=="bus":
            h=float(b.xyxy[0][3])-float(b.xyxy[0][1]);c=float(b.conf[0])
            if c>bc:bc=c;best=h
    return best,bc
def main():
    os.makedirs(OUT,exist_ok=True);print("YOLO");m=YOLO("yolov8n.pt")
    v=None
    for a in range(20):
        try:
            v=mavutil.mavlink_connection(MAV,autoreconnect=True)
            if v.wait_heartbeat(timeout=4) is not None: print("hb sys",v.target_system);break
        except Exception as e:print("retry",e)
        try:v.close()
        except Exception:pass
        v=None;time.sleep(2)
    if v is None or v.target_system==0:print("RESULT %s: no SITL"%TAG);sys.exit(1)
    v.set_mode_apm("GUIDED");time.sleep(1)
    for _ in range(20):
        v.mav.command_long_send(v.target_system,v.target_component,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0)
        hb=v.recv_match(type="HEARTBEAT",blocking=True,timeout=2)
        if hb and (hb.base_mode&mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):break
    print("armed;takeoff")
    v.mav.command_long_send(v.target_system,v.target_component,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,HOVER_Z)
    t0=time.time()
    while time.time()-t0<25:
        g=v.recv_match(type="GLOBAL_POSITION_INT",blocking=True,timeout=3)
        if g and g.relative_alt/1000.0>HOVER_Z-0.6:break
    print("hover;follow")
    f_px=None;log=[];halted=False;t_start=time.time()
    def latest_gpi():
        g=None
        while True:
            mm=v.recv_match(type="GLOBAL_POSITION_INT",blocking=False)
            if mm is None:break
            g=mm
        return g
    cur_n=0.0; last_t=time.time()
    while time.time()-t_start<T_MAX:
        t=time.time()-t_start
        g=latest_gpi(); now=time.time(); ddt=now-last_t; last_t=now
        if g is not None: cur_n += (g.vx/100.0)*ddt   # integrate measured north vel (m/s)
        d_true=max(TARGET_X-cur_n,0.0)
        img=newest();bh,conf=(0,0) if img is None else box_h(m,img)
        vx=0.0;d_box=None
        if bh>2:
            if f_px is None:
                f_px=d_true*bh/TARGET_H_M   # calibrate at first sight (true distance)
                print("cal f_px=%.0f d=%.1f bh=%.0f"%(f_px,d_true,bh))
            d_box=f_px*TARGET_H_M/bh
            if not halted:
                vx=float(np.clip(KP*(d_box-D0),-VMAX,VMAX))
            if d_true<=D_SAFE and not halted:
                halted=True;print("[%.1f] SAFETY HALT true=%.2f"%(t,d_true))
        if halted:vx=0.0
        v.mav.set_position_target_local_ned_send(0,v.target_system,v.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,0b0000111111000111,0,0,0,vx,0,0,0,0,0,0,0,0)
        log.append((t,d_true,d_box if d_box else np.nan,bh,conf,vx))
        print("[%.1f] true=%.2f d_box=%s bh=%d v=%.2f conf=%.2f%s"%(t,d_true,("%.1f"%d_box) if d_box else "--",bh,vx,conf,"  HALT" if halted else ""))
        time.sleep(DT)
    v.set_mode_apm("LAND")
    arr=np.array(log)
    np.savetxt(os.path.join(OUT,"fly_%s.csv"%TAG),arr,delimiter=",",header="t,d_true,d_box,box_h,conf,v",comments="")
    print("RESULT %s: start_true~10 final_true=%.2f min_true=%.2f halted=%s"%(TAG,arr[-1,1],np.nanmin(arr[:,1]),halted))
if __name__=="__main__":main()
