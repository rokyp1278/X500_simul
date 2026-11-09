import os,sys,json,numpy as np
ROOT=r"C:\Users\default.DESKTOP-5PUUIEJ\Desktop\adversarial_uav_demo";sys.path.insert(0,ROOT)
from src.attack.closed_loop_patch import run
rows=json.load(open(os.path.join(ROOT,"outputs","eot_rows.json")))
print("measured r(clean_h):",[(round(r[1]),round(r[4],2)) for r in rows])
# defense OFF vs ON
off=run(rows,defense=False); on=run(rows,defense=True)
print("defense OFF: collided=%s at t=%.1f"%(off["collided"],off["log"][-1,0]))
print("defense ON : collided=%s alarm_t=%s"%(on["collided"],on["alarm_t"]))
if on["alarm_t"] is not None:
    print("  defense lead before OFF-collision = %.1f s"%(off["log"][-1,0]-on["alarm_t"]))
import matplotlib;matplotlib.use("Agg");import matplotlib.pyplot as plt
L=off["log"]
fig,ax=plt.subplots(1,2,figsize=(13,5))
ax[0].plot(L[:,0],L[:,1],lw=2,color="crimson",label="true distance (pulled in)")
ax[0].plot(L[:,0],L[:,2],lw=2,ls="--",color="orange",label="perceived d_box (deceived)")
ax[0].axhline(0.5,ls="--",color="k",alpha=.5,label="collision 0.5m")
ax[0].axvline(off["t_attack"],ls=":",color="gray",label="patch active")
if on["alarm_t"]:ax[0].axvline(on["alarm_t"],ls="-.",color="green",label="defense alarm")
ax[0].set_xlabel("t(s)");ax[0].set_ylabel("distance(m)");ax[0].legend();ax[0].grid(alpha=.3)
ax[0].set_title("Closed loop: EOT patch pulls the drone in (box shrunk ~0.5 all the way)")
ax[1].plot(L[:,0],L[:,5],color="purple",lw=2,label="box/anchor mismatch")
ax[1].axhline(0.15,ls="-.",color="green",label="defense tau")
ax[1].axvline(off["t_attack"],ls=":",color="gray")
ax[1].set_xlabel("t(s)");ax[1].set_ylabel("mismatch");ax[1].legend();ax[1].grid(alpha=.3)
ax[1].set_title("Defense signal (independent depth anchor)")
fig.tight_layout();out=os.path.join(ROOT,"outputs","closed_loop_patch.png");fig.savefig(out,dpi=115)
print("FIG",out)
