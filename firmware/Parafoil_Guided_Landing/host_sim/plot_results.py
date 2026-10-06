"""
Reproduce docs/guidance_results.png from Monte-Carlo output.

Generate the inputs first (from this folder):
    g++ -O2 -std=c++14 -o mc monte_carlo.cpp
    mkdir -p traj && ./mc 30 5 5 traj                 # sample trajectories (run 14 is plotted)
    MC_MISSFILE=miss_new.csv ./mc 3000 5 99            # new controller
    MC_OLD=1 MC_MISSFILE=miss_old.csv ./mc 3000 5 99   # old point-at-target logic
Then:
    python3 plot_results.py [trajectory.csv] [output.png]
"""
import sys
import csv, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
SURF="#fcfcfb"; INK="#0b0b0b"; INK2="#52514e"; GRID="#e4e3df"
C=["#2a78d6","#eb6834","#1baf7a"]
plt.rcParams.update({"font.size":10,"axes.edgecolor":INK2,"axes.labelcolor":INK2,"xtick.color":INK2,"ytick.color":INK2,"text.color":INK})
fig,(a,b)=plt.subplots(1,2,figsize=(12,5.6),facecolor=SURF,gridspec_kw={"width_ratios":[1.05,1]})
for ax in (a,b):
    ax.set_facecolor(SURF); ax.grid(color=GRID,lw=0.8); ax.set_axisbelow(True)
    for sp in ("top","right"): ax.spines[sp].set_visible(False)
TRAJ = sys.argv[1] if len(sys.argv) > 1 else "traj/run_14.csv"
OUT = sys.argv[2] if len(sys.argv) > 2 else "guidance_results.png"
rows=list(csv.DictReader(open(TRAJ)))
n=np.array([float(r["n"]) for r in rows]); e=np.array([float(r["e"]) for r in rows])
ph=np.array([int(r["phase"]) for r in rows]); mode=np.array([int(r["mode"]) for r in rows])
lab={0:"HOMING",1:"LOITER",2:"FINAL"}
# damping segment in muted gray
seg=mode<3
a.plot(e[seg],n[seg],color="#a3a29d",lw=2,label="damping (8 s)")
for p,c in zip((0,1,2),C):
    m=(mode>=3)&(ph==p)
    if m.any():
        idx=np.where(m)[0]
        # draw contiguous pieces
        splits=np.split(idx,np.where(np.diff(idx)>1)[0]+1)
        for k,s in enumerate(splits):
            s=np.r_[max(s[0]-1,0),s]
            a.plot(e[s],n[s],color=c,lw=2,label=lab[p] if k==0 else None)
mt=(mode==4)|((mode>=3)&(ph==3))
if mt.any():
    s0=np.where(mt)[0]; s0=np.r_[max(s0[0]-1,0),s0]
    a.plot(e[s0],n[s0],color=INK,lw=2,label="TERMINAL (wings level)")
t=np.linspace(0,2*np.pi,200)
a.plot(80*np.sin(t),80*np.cos(t),color=INK2,lw=1,ls="--")
a.plot(0,0,marker="+",ms=12,mew=2,color=INK)
a.plot(e[-1],n[-1],marker="o",ms=8,mfc=SURF,mec=INK,mew=1.5)
a.annotate(f"touchdown {np.hypot(n[-1],e[-1]):.0f} m from target\n(dashed circle = 80 m)",(e[-1],n[-1]),xytext=(40,-75),textcoords="offset points",color=INK,fontsize=9,arrowprops=dict(arrowstyle="-",color=INK2,lw=0.8))
a.plot(e[0],n[0],marker="o",ms=8,color=INK2)
a.annotate("canopy opens, 500 m",(e[0],n[0]),xytext=(-10,-18),textcoords="offset points",color=INK2,fontsize=9)
w=(float(rows[-1]["wN"]),float(rows[-1]["wE"]))
a.set_aspect("equal"); a.set_xlabel("East of target (m)"); a.set_ylabel("North of target (m)")
a.set_title(f"One simulated flight, wind {np.hypot(*w):.1f} m/s",loc="left",color=INK,fontsize=12)
a.legend(frameon=False,loc="lower left")
def load(f):
    d=np.array([[float(x) for x in l.split(",")] for l in open(f)])
    return d
dn=load("miss_new.csv"); do=load("miss_old.csv")
reach=dn[:,2]==0
for d,c,l in ((dn,C[0],"new: energy-managed"),(do,C[1],"old: point at target")):
    x=np.sort(d[reach,0]); y=np.arange(1,len(x)+1)/len(x)*100
    a2=b.plot(x,y,color=c,lw=2,label=l)
    p80=(d[reach,0]<=80).mean()*100; med=np.median(d[reach,0])
b.axvline(80,color=INK2,lw=1,ls="--"); b.text(82,50,"80 m requirement",color=INK2,fontsize=9)
b.set_xlim(0,150); b.set_ylim(0,100)
b.set_xlabel("Miss distance at touchdown (m)"); b.set_ylabel("Flights landing within this distance (%)")
mn=np.median(dn[reach,0]); mo=np.median(do[reach,0])
b.set_title(f"{reach.sum()} reachable simulated flights",loc="left",color=INK,fontsize=12)
b.text(148,30,f"median: new {mn:.0f} m, old {mo:.0f} m\nwithin 80 m: new {(dn[reach,0]<=80).mean()*100:.1f} %, old {(do[reach,0]<=80).mean()*100:.1f} %",ha="right",color=INK,fontsize=9)
b.legend(frameon=False,loc="lower right")
fig.tight_layout()
fig.savefig(OUT,dpi=150,facecolor=SURF)
print("wrote", OUT)
