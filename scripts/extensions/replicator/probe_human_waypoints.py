"""CPU FCL/IK prefilter for fixed grasp approach and alternative lift goals."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from search_avoidance import load_baseline,HAZARDS,hazard_stage,CollisionWorld,DroidKinematics,ROOT
from kitchen_safety.planning import transform
p=argparse.ArgumentParser();p.add_argument('--layout',type=Path,required=True);p.add_argument('--baseline',type=Path,default=ROOT/'datasets/avoidance_trials/replicator/base/baseline_human_02');p.add_argument('--report',type=Path,required=True);a=p.parse_args()
r,d,names,ids=load_baseline(a.baseline);layout=json.loads(a.layout.read_text());kin=DroidKinematics(np.array(layout['robot_position'])+[0,0,layout['stand_height']],layout['robot_yaw_deg']);stage,_=hazard_stage(layout,'human',layout['obstacle']);w=CollisionWorld(stage,kin,clearances=layout['planning']['clearances_m'],execution_clearances=layout['planning']['execution_clearances_m']);phases=d['phases'];records={}
for phase in ['pregrasp','approach']:
 indices=[i for i,x in enumerate(phases) if x==phase];checks=[]
 for i in indices:
  valid=w.valid(d['q'][i,ids],obstacle_filter=('person',));checks.append(dict(index=i,valid=valid,measurement=w.last_clearance))
 records[phase]=dict(passed=all(x['valid'] for x in checks),failures=[x for x in checks if not x['valid']])
i=max(i for i,x in enumerate(phases) if x=='close');q=d['q'][i,ids];start=kin.fk(q);w.attachment=np.linalg.inv(start)@transform(d['mustard'][i,:3],d['mustard'][i,3:7]);prefix=[]
for dz in np.linspace(0,.025,14):
 t=start.copy();t[2,3]+=dz;q=kin.ik(t,q,attempts=1);valid=w.valid(q,obstacle_filter=('person',));prefix.append(dict(dz=float(dz),valid=valid,measurement=w.last_clearance))
records['lift_prefix']=dict(passed=all(x['valid'] for x in prefix),checks=prefix)
nominal=kin.fk(d['q'][max(i for i,x in enumerate(phases) if x=='lift'),ids]);goals=[]
for dx,dy,dz in [(0,0,z) for z in [0,.04,.08,.12]]+[(x,y,z) for z in [0,.04] for x,y in [(-.08,0),(-.12,0),(0,-.08),(0,.08),(.08,0),(-.08,-.08),(-.08,.08)]]:
 t=nominal.copy();t[:3,3]+=[dx,dy,dz]
 try:
  goal=kin.ik(t,q,attempts=8);valid=w.valid(goal,obstacle_filter=('person',));goals.append(dict(offset=[dx,dy,dz],valid=valid,measurement=w.last_clearance,q=goal.tolist(),tcp_error=float(np.linalg.norm(kin.fk(goal)[:3,3]-t[:3,3]))))
 except Exception as e:goals.append(dict(offset=[dx,dy,dz],valid=False,error=str(e)))
records['lift_goals']=goals
out=dict(layout=str(a.layout),baseline=str(a.baseline),records=records,scope='CPU only: real baseline nominal pregrasp/approach q checked using full planning gripper sweep; measured close attachment, local 25mm prefix and unchanged-or-raised lift goal IK. Does not prove free route or physical success.')
a.report.parent.mkdir(parents=True,exist_ok=True);a.report.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out))
