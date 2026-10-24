"""Offline gain comparison with assumed lag/delay/noise; not hardware identification."""
import sys,math,random,json
from pathlib import Path
from collections import deque
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from abc_swarm.point_control import point_velocity,angle
starts=[(-.2,0,0),(-1,0,0),(0,-.5,0),(.5,0,0),(-.5,.5,2),(0,0,2.5)]
rows=[]
for kd in [.6,.9,1.2]:
 for kh in [1.5,2.,2.5]:
  times=[];errors=[]
  for lag,delay,noise in [(0,0,0),(.15,.1,0),(.15,.1,.002)]:
   for case,start in enumerate(starts):
    rng=random.Random(case);pose=start;v=w=0;dt=.05;arrived=None
    pending=deque([(0,0)]*max(1,round(delay/dt)+1))
    for step in range(4000):
     obs=(pose[0]+rng.uniform(-noise,noise),pose[1]+rng.uniform(-noise,noise),pose[2]+rng.uniform(-2.5*noise,2.5*noise))
     if arrived is None:
      cv,cw,phase=point_velocity(obs,(0,0,0),k_distance=kd,k_heading=kh)
      if phase=='arrived':arrived=step*dt
     else:cv=cw=0
     pending.append((cv,cw));tv,tw=pending.popleft()
     response=1 if lag==0 else 1-math.exp(-dt/lag)
     v+=response*(tv-v);w+=response*(tw-w)
     x,y,yaw=pose;pose=(x+dt*v*math.cos(yaw),y+dt*v*math.sin(yaw),angle(yaw+dt*w))
     if arrived is not None and step*dt-arrived>=1:break
    times.append(arrived if arrived is not None else 200)
    errors.append((math.hypot(pose[0],pose[1]),abs(angle(pose[2]))))
  rows.append({'k_distance':kd,'k_heading':kh,'mean_time_s':round(sum(times)/len(times),3),'max_time_s':round(max(times),3),'worst_position_m':round(max(e[0] for e in errors),4),'worst_yaw_rad':round(max(e[1] for e in errors),4),'finished':sum(t<200 for t in times),'cases':len(times)})
print(json.dumps(rows,indent=2))
