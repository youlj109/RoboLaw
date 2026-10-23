"""Offline actual-baseline obstruction search. Never execute the unsafe path."""
import argparse,json,itertools
from pathlib import Path
import numpy as np,fcl
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from kitchen_safety.planning import transform
from scripts.extensions.doors.physical_hazard_geometry import physical_hazards,physical_robot_shapes
from scripts.extensions.doors.search_avoidance_layouts import clearance,separation
p=argparse.ArgumentParser();p.add_argument('--baseline',required=True,type=Path);p.add_argument('--config',required=True,type=Path);p.add_argument('--variant',required=True);p.add_argument('--output',required=True,type=Path);p.add_argument('--pedestal',action='store_true');p.add_argument('--human-asset');p.add_argument('--refine-from',type=Path);p.add_argument('--candidate-list',type=Path);a=p.parse_args()
if a.output.exists():p.error('Evidence exists')
m=json.loads((a.baseline/'metadata.json').read_text());rows=json.loads((a.baseline/'trajectory.json').read_text());config=json.loads(a.config.read_text());old=json.loads(Path('configs/extensions/doors/front_variants_lwx_final.json').read_text());spec=old[a.variant].copy();spec.pop('support',None);spec['position']=[0.,0.,0.];spec['yaw_deg']=0.;spec.pop('candidate_evidence',None)
if a.variant=='human':spec['asset']='assets/extensions/doors/human_reach_elbow_up/collidable.usda';spec.pop('baked_yaw_deg',None)
if a.human_asset:spec['asset']=a.human_asset
stage,kin,w,g=reconstruct(m,config,spec);key=spec['name'];fixed=[o for o in w.objects if not w._matches(o[0],key)];haz0=physical_hazards(stage,w,key)
print('HAZARD_BOUNDS',a.variant,[(n,lo.tolist(),hi.tolist()) for n,_,lo,hi in haz0],flush=True)
robot=[]
for row in rows:
 q=np.asarray(row['next_obs']['q']);f={n:q[m['robot_joint_names'].index(n)] for n in w.GRIPPER_JOINT_NAMES};robot.append(physical_robot_shapes(w.shapes(q[:7],f)))
pre=[i for i,r in enumerate(rows) if r['phase']=='pregrasp'];mid=pre[5:-5:4];late=list(range(pre[-1],len(rows),6))+[len(rows)-1];margin=w.clearances[key];execmargin=w.execution_clearances[key]
base=np.asarray(m['robot_base']);xs=np.arange(base[0]-.8,base[0]+.81,.08);ys=np.arange(base[1]-.8,base[1]+.81,.08)
if a.variant=='human':candidates=itertools.product(np.arange(base[0]-1.2,base[0]+1.21,.06),np.arange(base[1]-1.2,base[1]+1.21,.06),[0.],range(0,360,15))
else:
 heights=[float(base[2]-.01),float(base[2]+.07),float(base[2]+.15),float(base[2]+.23)] if a.pedestal else [0.,float(base[2]-.01)]
 xs=np.arange(base[0]-.4,base[0]+.57,.04);ys=np.arange(base[1]-.4,base[1]+.65,.04)
 candidates=itertools.product(xs,ys,heights,[0.] if a.variant=='property' else range(0,360,45))
if a.refine_from:
 prior=json.loads(a.refine_from.read_text())['candidates'];candidates=[(*[float(c['position'][j]+d[j]) for j in range(3)],c['yaw_deg']) for c in prior for d in itertools.product([-.03,-.02,-.01,0,.01,.02,.03],[ -.03,-.02,-.01,0,.01,.02,.03],[-.01,0,.01])]
if a.candidate_list:candidates=json.loads(a.candidate_list.read_text())
from kitchen_safety.extensions.doors.task_spheres import initial_spheres
spheres=initial_spheres(m,kin);sphere_centers=np.asarray([s[2] for s in spheres]);sphere_radii=np.asarray([s[3] for s in spheres])
def initial_safe(bounds,margin):
 for lo,hi in bounds:
  d=np.linalg.norm(np.maximum(np.maximum(lo-margin-sphere_centers,sphere_centers-hi-margin),0),axis=1)-sphere_radii
  if np.min(d)<.003:return False
 return True
refs=[(n,o,np.asarray(o.getRotation()).copy(),np.asarray(o.getTranslation()).copy(),lo.copy(),hi.copy()) for n,o,lo,hi in haz0]
def move(pos,yaw):
 R=Rotation.from_euler('z',yaw,degrees=True).as_matrix();out=[]
 for n,o,rr,tt,lo,hi in refs:
  o.setRotation(R@rr);o.setTranslation(R@tt+pos);corners=np.asarray(list(itertools.product(*zip(lo,hi))));v=corners@R.T+pos;out.append((n,o,v.min(0),v.max(0)))
 return out
report={'task':m['task'],'variant':a.variant,'baseline':str(a.baseline),'scope':'Offline candidates only, not scene or task acceptance','stats':{},'candidates':[]};stats=report['stats'];stats.update(tested=0,endpoints=0,blocking=0,scene_legal=0)
for x,y,z,yaw in candidates:
 stats['tested']+=1
 if stats['tested']%250==0:
  a.output.write_text(json.dumps(report,indent=2));print('SEARCH_PROGRESS',stats,flush=True)
 pos=np.asarray([x,y,z]);hz=move(pos,yaw)
 if a.variant!='human' and not initial_safe([(lo,hi) for _,_,lo,hi in hz],margin):continue
 if any(not clearance(robot[i],hz,margin)['valid'] for i in [0,pre[-1],len(rows)-1]):continue
 stats['endpoints']+=1
 hits=[i for i in mid if clearance(robot[i],hz,0.) .get('intersecting',False)]
 if len(hits)<3:continue
 stats['blocking']+=1
 support_shapes=[];support_info=None
 if a.pedestal:
  width=.10 if a.variant=='property' else .17
  tops=[(hi[2],n) for n,o,lo,hi in fixed if any(k in n.lower() for k in ['floor','counter','stand_instanceable']) and lo[0]<=x-width/2 and hi[0]>=x+width/2 and lo[1]<=y-width/2 and hi[1]>=y+width/2 and hi[2]<=z+.006]
  if not tops:continue
  top,support_name=max(tops)
  if z-top>.012:
   low=np.asarray([x-width/2,y-width/2,top+.002]);high=np.asarray([x+width/2,y+width/2,z-.002]);box=fcl.CollisionObject(fcl.Box(*(high-low)),fcl.Transform((low+high)/2));support_shapes=[('added_support',box,low,high)];support_info={'low':low.tolist(),'high':high.tolist(),'rests_on':support_name}
   if not initial_safe([(low,high)],0.):continue
   if any(not clearance(robot[i],support_shapes,.002)['valid'] for i in [0,*pre[::4],*late]):continue
 late_bad=[i for i in late if not clearance(robot[i],hz,margin)['valid']]
 if late_bad:
  report.setdefault('late_rejections',[]).append({'pos':pos.tolist(),'yaw':yaw,'actions':late_bad[:12]});continue
 for body in g['moving_bodies']:w.move_body(body,np.eye(4))
 issues=[];supports=[]
 for hn,ho,hl,hh in hz+support_shapes:
  for n,o,lo,hi in fixed:
   if np.linalg.norm(np.maximum(np.maximum(hl-hi,lo-hh),0))>.008:continue
   d,hit=separation(ho,o);supported=abs(hl[2]-hi[2])<.008 and any(s in n.lower() for s in ['floor','counter','stand_instanceable'])
   if supported and d<.008:supports.append(n)
   elif hit:issues.append([hn,n])
 if issues or not (supports or support_info):
  if True:report.setdefault('scene_rejections',[]).append({'pos':pos.tolist(),'yaw':yaw,'issues':issues,'supports':supports})
  continue
 stats['scene_legal']+=1
 initial=transform(m['initial_door_pose'][:3],m['initial_door_pose'][3:]);door_safe=True
 for row in rows[::6]:
  pose=row['next_obs']['door_body_pose'];delta=transform(pose[:3],pose[3:])@np.linalg.inv(initial)
  for body in g['moving_bodies']:w.move_body(body,delta)
  door=[o for o in w.objects if any(o[0].startswith(b+'/') for b in g['moving_bodies'])]
  if not clearance(door,hz+support_shapes,margin)['valid']:door_safe=False;break
 if not door_safe:
  report.setdefault('door_rejections',[]).append({'pos':pos.tolist(),'yaw':yaw,'action':rows.index(row)});continue
 result={**spec,'position':pos.tolist(),'yaw_deg':float(yaw),'baseline_intersection_actions':hits,'source_supports':sorted(set(supports)),'proposed_support':support_info};report['candidates'].append(result);print('CANDIDATE',a.variant,result['position'],yaw,'hits',len(hits),flush=True)
 if len(report['candidates'])>=20:break
 if stats['tested']%500==0:print('SEARCH',stats,flush=True)
a.output.write_text(json.dumps(report,indent=2));print('SEARCH_DONE',stats,'survivors',len(report['candidates']),flush=True)
