"""Assemble outputs/sim/_g1frames/*.png -> outputs/sim/scenario1_gazebo.gif with a caption."""
import glob, os
from PIL import Image, ImageDraw, ImageFont
OUT=os.path.join("outputs","sim"); FR=os.path.join(OUT,"_g1frames")
fs=sorted(glob.glob(os.path.join(FR,"*.png")))
assert fs, "no frames"
try: font=ImageFont.truetype("arial.ttf", 18); small=ImageFont.truetype("arial.ttf",14)
except Exception: font=ImageFont.load_default(); small=font
frames=[]
n=len(fs)
for i,f in enumerate(fs):
    im=Image.open(f).convert("RGB"); d=ImageDraw.Draw(im)
    d.rectangle([0,0,im.width,26], fill=(20,20,28))
    d.text((6,4),"Adversarial patch vs vision-follow drone (3m mocap cube)", font=font, fill=(240,240,240))
    # phase caption by progress
    p=i/max(1,n-1)
    if p<0.33: cap="following: should hold at green safe line (1.5 m)"
    elif p<0.6: cap="PATCH fools range -> drone crosses the safe line"
    else: cap="parked 0.69 m from target  (safe standoff violated 54%)"
    d.rectangle([0,im.height-22,im.width,im.height], fill=(20,20,28))
    d.text((6,im.height-19), cap, font=small, fill=(255,210,120) if p>=0.33 else (170,230,170))
    frames.append(im)
gif=os.path.join(OUT,"scenario1_gazebo.gif")
frames[0].save(gif, save_all=True, append_images=frames[1:], duration=90, loop=0, optimize=True)
print("saved", gif, os.path.getsize(gif)//1024, "KB,", len(frames), "frames")
