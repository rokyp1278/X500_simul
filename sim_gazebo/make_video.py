#!/usr/bin/env python3
"""Stitch /tmp/obs_frames/*.jpg (Gazebo observer camera) -> outputs/sim/scenario1_gazebo.mp4
and drop downsampled RGB frames to outputs/sim/_g1frames/ for the Windows side to GIF."""
import glob, os, cv2
OUT="/mnt/c/Users/default.DESKTOP-5PUUIEJ/Desktop/adversarial_uav_demo/outputs/sim"
FR=os.path.join(OUT,"_g1frames"); os.makedirs(FR, exist_ok=True)
for f in glob.glob(os.path.join(FR,"*.png")): os.remove(f)
fs=sorted(glob.glob("/tmp/obs_frames/*.jpg"))
imgs=[cv2.imread(f) for f in fs]; imgs=[im for im in imgs if im is not None and im.size]
print("frames:", len(imgs))
if not imgs: raise SystemExit("NO FRAMES")
# skip black warmup frames (mean<8)
imgs=[im for im in imgs if im.mean()>8] or imgs
h,w=imgs[0].shape[:2]
mp4=os.path.join(OUT,"scenario1_gazebo.mp4")
vw=cv2.VideoWriter(mp4, cv2.VideoWriter_fourcc(*"mp4v"), 20, (w,h))
for im in imgs: vw.write(im)
vw.release(); print("saved", mp4, "(%dx%d, %d frames)"%(w,h,len(imgs)))
stride=max(1,len(imgs)//100)
for i,im in enumerate(imgs[::stride]):
    cv2.imwrite(os.path.join(FR,"f%03d.png"%i), cv2.resize(im,(w//2,h//2)))
print("downsampled frames ->", FR, len(range(0,len(imgs),stride)))
