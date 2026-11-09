import os,sys,numpy as np
ROOT=r"C:\Users\default.DESKTOP-5PUUIEJ\Desktop\adversarial_uav_demo";sys.path.insert(0,ROOT)
from src.mission.follow_mission import Mission, Params, TAKEOFF, SEARCH, FOLLOW, SAFETY_HOLD
import matplotlib;matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation, PillowWriter

p=Params(); m=Mission(p=p)
T=22.0; dt=p.dt; n=int(T/dt)
t_patch=11.0
log=[]
tx=5.0
for i in range(n):
    t=i*dt
    # target motion: static, then walks 5->7 (t 4..9), then static
    if 4<=t<9: tx += 0.4*dt
    patch_on = (t>=t_patch) and (m.state in (FOLLOW,SAFETY_HOLD))
    gap = tx - m.x
    vx,vz,pd,info = m.step(t, tx, patch_on, gap)
    log.append((t, m.x, m.z, tx, gap, pd if pd else np.nan, info["state"], patch_on, info["deceived"]))
L=np.array([(r[0],r[1],r[2],r[3],r[4],r[5],r[7],r[8]) for r in log],dtype=float)
states=[r[6] for r in log]
# find key times
t_hold=next((r[0] for r in log if r[6]==SAFETY_HOLD), None)
t_dec=next((r[0] for r in log if r[8]), None)
print("patch t=%.1f  deceived flagged t=%s  safety-hold t=%s"%(t_patch,t_dec,t_hold))
print("min true gap=%.2f m (d_safe=%.1f) -> contact avoided=%s"%(L[:,4].min(),p.d_safe,L[:,4].min()>=p.d_safe-0.05))

# ---- animation (side view + distance plot) ----
fig,(ax,ax2)=plt.subplots(1,2,figsize=(13,5),gridspec_kw={"width_ratios":[1.4,1]})
def frame(k):
    k=k*3
    if k>=len(log): k=len(log)-1
    t,dx,dz,txx,gap,pd,st,patch,dec=log[k][0],*[L[k,i] for i in range(1,6)],states[k],L[k,6],L[k,7]
    ax.clear();ax2.clear()
    ax.axhline(0,color="k",lw=2)  # ground
    # target board
    ax.add_patch(Rectangle((txx-0.1,0),0.2,2.0,color="#8a6d3b"))
    ax.text(txx,2.15,"TARGET",ha="center",fontsize=8)
    if patch: ax.add_patch(Rectangle((txx-0.25,0.6),0.3,0.8,color="#d62728")); ax.text(txx-0.1,1.0,"PATCH",color="w",ha="center",va="center",fontsize=7,rotation=90)
    # standoff + safety lines
    ax.axvline(txx-p.d0,ls="--",color="gray",alpha=.6); ax.text(txx-p.d0,2.35,"desired 3m",color="gray",ha="center",fontsize=7)
    ax.axvline(txx-p.d_safe,ls="--",color="crimson"); ax.text(txx-p.d_safe,-0.3,"SAFETY 1.5m",color="crimson",ha="center",fontsize=7)
    # drone
    col={"TAKEOFF":"#1f77b4","SEARCH":"#1f77b4","FOLLOW":"#2ca02c","SAFETY_HOLD":"crimson"}[st]
    ax.plot(dx,dz,marker="v",ms=16,color=col)
    ax.text(dx,dz+0.25,"DRONE",ha="center",fontsize=8,color=col)
    ax.set_xlim(-1,9);ax.set_ylim(-0.6,3);ax.set_yticks([])
    banner={"TAKEOFF":"자율 이륙","SEARCH":"타깃 탐색","FOLLOW":"자율 추종 (3m 유지)","SAFETY_HOLD":"속아서 전진→안전선 자동정지"}[st]
    ax.set_title("t=%.1fs  [%s] %s"%(t,st,banner),fontsize=11,color=col)
    if dec and st!="SAFETY_HOLD": ax.text(4,2.7,"⚠ 속임 감지 (거리 불일치)",color="red",fontsize=10,ha="center")
    # distance plot
    ax2.plot(L[:k+1,0],L[:k+1,4],color="crimson",lw=2,label="진짜 거리(모캡)")
    ax2.plot(L[:k+1,0],L[:k+1,5],color="orange",lw=2,ls="--",label="드론이 믿는 거리")
    ax2.axhline(p.d0,color="gray",ls=":",alpha=.6); ax2.axhline(p.d_safe,color="crimson",ls="--",alpha=.6,label="안전선")
    ax2.axvline(t_patch,color="purple",ls=":",alpha=.7)
    ax2.set_xlim(0,T);ax2.set_ylim(0,8);ax2.set_xlabel("t(s)");ax2.set_ylabel("거리(m)")
    ax2.legend(loc="upper right",fontsize=8);ax2.grid(alpha=.3);ax2.set_title("거리: 믿는 값 vs 진짜 (벌어지면=속음)",fontsize=10)
try:
    plt.rcParams["font.family"]="Malgun Gothic"
except Exception: pass
import matplotlib.font_manager as fm,glob
for f in glob.glob(r"C:\Windows\Fonts\malgun*.ttf"): fm.fontManager.addfont(f)
plt.rcParams["font.family"]="Malgun Gothic"
ani=FuncAnimation(fig,frame,frames=len(log)//3,interval=80)
out=os.path.join(ROOT,"outputs","autonomous_mission.gif")
ani.save(out,writer=PillowWriter(fps=12));print("GIF",out)
frame(len(log)//3-1); fig.savefig(os.path.join(ROOT,"outputs","autonomous_mission_final.png"),dpi=110,bbox_inches="tight")
print("final PNG saved")
