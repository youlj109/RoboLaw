"""Offline actual-baseline obstruction search. Never execute the unsafe path."""
import argparse,json,itertools
from pathlib import Path
import numpy as np,fcl
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from kitchen_safety.planning import transform
from scripts.extensions.doors.physical_hazard_geometry import physical_hazards,physical_robot_shapes
from scripts.extensions.doors.search_avoidance_layouts import clearance,separation
from kitchen_safety.extensions.doors.obstruction_broadphase_lwx import potential_intersection_rows
from kitchen_safety.extensions.doors.source_sphere_screen_lwx import source_spheres_safe
p=argparse.ArgumentParser();p.add_argument('--baseline',required=True,type=Path);p.add_argument('--config',required=True,type=Path);p.add_argument('--variant',required=True);p.add_argument('--output',required=True,type=Path);p.add_argument('--pedestal',action='store_true');p.add_argument('--heights');p.add_argument('--min-blocking-samples',type=int,default=3);p.add_argument('--mid-stride',type=int,default=4);p.add_argument('--human-asset');p.add_argument('--refine-from',type=Path);p.add_argument('--candidate-list',type=Path);p.add_argument('--paired-reference',type=Path);p.add_argument('--hazard-asset');p.add_argument('--support-width-m',type=float);p.add_argument('--yaws');p.add_argument('--xy-step-m',type=float,default=.04);p.add_argument('--screen-approach-proxies',action='store_true');p.add_argument('--support-offset-xy',nargs=2,type=float,default=[0.,0.]);a=p.parse_args()
if not .01<=a.xy_step_m<=.08:p.error('Discovery xy step must be between .01 and .08m')
if np.linalg.norm(a.support_offset_xy)>.4:p.error('Support offset must be at most .4m')
if a.output.exists():p.error('Evidence exists')
m=json.loads((a.baseline/'metadata.json').read_text());rows=json.loads((a.baseline/'trajectory.json').read_text());config=json.loads(a.config.read_text());old=json.loads(Path('configs/extensions/doors/front_variants_lwx_final.json').read_text());spec=old[a.variant].copy();spec.pop('support',None);spec['position']=[0.,0.,0.];spec['yaw_deg']=0.;spec.pop('candidate_evidence',None)
if a.variant=='human':spec['asset']='assets/extensions/doors/human_reach_elbow_up/collidable.usda';spec.pop('baked_yaw_deg',None)
if a.human_asset:spec['asset']=a.human_asset
if a.hazard_asset:spec['asset']=a.hazard_asset;spec['semantic_anchor']='Unscaled original library asset; placement searched against actual source meshes and all reference off paths. Physical stability and same-target small rerouting still require independent acceptance.'
stage,kin,w,g=reconstruct(m,config,spec);key=spec['name'];fixed=[o for o in w.objects if not w._matches(o[0],key)];haz0=physical_hazards(stage,w,key)
print('HAZARD_BOUNDS',a.variant,[(n,lo.tolist(),hi.tolist()) for n,_,lo,hi in haz0],flush=True)
robot=[]
for row in rows:
 q=np.asarray(row['next_obs']['q']);f={n:q[m['robot_joint_names'].index(n)] for n in w.GRIPPER_JOINT_NAMES};robot.append(physical_robot_shapes(w.shapes(q[:7],f)))
pre=[i for i,r in enumerate(rows) if r['phase']=='pregrasp'];mid=pre[5:-5:a.mid_stride];late=list(range(pre[-1],len(rows),6))+[len(rows)-1];margin=w.clearances[key];execmargin=w.execution_clearances[key]
mid_bounds=np.asarray([[[lo,hi] for _,_,lo,hi in robot[i]] for i in mid])
base=np.asarray(m['robot_base']);xs=np.arange(base[0]-.8,base[0]+.81,.08);ys=np.arange(base[1]-.8,base[1]+.81,.08)
if a.variant=='human':candidates=itertools.product(np.arange(base[0]-1.2,base[0]+1.21,.06),np.arange(base[1]-1.2,base[1]+1.21,.06),[0.],range(0,360,15))
else:
 heights=[float(base[2]-.01),float(base[2]+.07),float(base[2]+.15),float(base[2]+.23)] if a.pedestal else [0.,float(base[2]-.01)]
 if a.heights:heights=list(map(float,a.heights.split(',')))
 xs=np.arange(base[0]-.4,base[0]+.57,a.xy_step_m);ys=np.arange(base[1]-.4,base[1]+.65,a.xy_step_m)
 candidates=itertools.product(xs,ys,heights,list(map(float,a.yaws.split(','))) if a.yaws else ([0.] if a.variant=='property' else range(0,360,45)))
if a.refine_from:
 prior=json.loads(a.refine_from.read_text())['candidates'];candidates=[(*[float(c['position'][j]+d[j]) for j in range(3)],c['yaw_deg']) for c in prior for d in itertools.product([-.03,-.02,-.01,0,.01,.02,.03],[ -.03,-.02,-.01,0,.01,.02,.03],[-.01,0,.01])]
if a.candidate_list:candidates=json.loads(a.candidate_list.read_text())
from kitchen_safety.extensions.doors.task_spheres import initial_spheres
goal_metadata=dict(m);goal_metadata['initial_q']=rows[pre[-1]]['next_obs']['q'];goal_spheres=initial_spheres(goal_metadata,kin)
spheres=initial_spheres(m,kin)+goal_spheres
if a.screen_approach_proxies:
 for row in rows:
  if row['phase'] not in ['approach_handle','approach']:continue
  approach_metadata=dict(m);approach_metadata['initial_q']=row['next_obs']['q'];spheres+=initial_spheres(approach_metadata,kin)
print('SCREENING_SPHERES',len(spheres),'approach_proxies',a.screen_approach_proxies,flush=True)
sphere_centers=np.asarray([s[2] for s in spheres]);sphere_radii=np.asarray([s[3] for s in spheres])
def initial_safe(bounds,margin):
 for lo,hi in bounds:
  d=np.linalg.norm(np.maximum(np.maximum(lo-margin-sphere_centers,sphere_centers-hi-margin),0),axis=1)-sphere_radii
  if np.min(d)<.003:return False
 return True
paired_robot_paths=[]
if a.paired_reference:
 pairdoc=json.loads(a.paired_reference.read_text())
 for pair in pairdoc['pairs']:
  if not pair['off'].get('success'):raise RuntimeError('Reference off-plan failed')
  path=[]
  for joints in pair['off']['positions'][::2]:
   f={n:m['initial_q'][m['robot_joint_names'].index(n)] for n in w.GRIPPER_JOINT_NAMES}
   path.append(physical_robot_shapes(w.shapes(np.asarray(joints),f)))
  paired_robot_paths.append((pair['seed'],path))
refs=[(n,o,np.asarray(o.getRotation()).copy(),np.asarray(o.getTranslation()).copy(),lo.copy(),hi.copy()) for n,o,lo,hi in haz0]
def move(pos,yaw):
 R=Rotation.from_euler('z',yaw,degrees=True).as_matrix();out=[]
 for n,o,rr,tt,lo,hi in refs:
  o.setRotation(R@rr);o.setTranslation(R@tt+pos);corners=np.asarray(list(itertools.product(*zip(lo,hi))));v=corners@R.T+pos;out.append((n,o,v.min(0),v.max(0)))
 return out
report={'task':m['task'],'variant':a.variant,'baseline':str(a.baseline),'scope':'Offline candidates only, not scene or task acceptance','discovery_broadphase':'Vectorized conservative source AABB overlap; every counted intersection still requires exact FCL','endpoint_sphere_screen':'AABB lower bounds only prune safe pairs; close pairs use exact source FCL at unchanged margin+3mm','discovery_min_blocking_samples':a.min_blocking_samples,'discovery_mid_stride':a.mid_stride,'stats':{},'candidates':[]};report['search_parameters']={k:str(v) if isinstance(v,Path) else v for k,v in vars(a).items()};stats=report['stats'];stats.update(tested=0,endpoints=0,blocking=0,scene_legal=0)
for x,y,z,yaw in candidates:
 stats['tested']+=1
 if stats['tested']%250==0:
  a.output.write_text(json.dumps(report,indent=2));print('SEARCH_PROGRESS',stats,flush=True)
 pos=np.asarray([x,y,z]);hz=move(pos,yaw)
 if not source_spheres_safe(hz,sphere_centers,sphere_radii,margin+.003):continue
 if any(not clearance(robot[i],hz,margin)['valid'] for i in [0,pre[-1],len(rows)-1]):continue
 stats['endpoints']+=1
 potential=potential_intersection_rows(mid_bounds,[[lo,hi] for _,_,lo,hi in hz])
 if int(np.sum(potential))<a.min_blocking_samples:continue
 hits=[mid[j] for j in np.flatnonzero(potential) if clearance(robot[mid[j]],hz,0.).get('intersecting',False)]
 if len(hits)<a.min_blocking_samples:continue
 stats['blocking']+=1
 if paired_robot_paths:
  paired_hits={str(seed):[j for j,shapes in enumerate(path) if clearance(shapes,hz,0.).get('intersecting',False)] for seed,path in paired_robot_paths}
  if not all(paired_hits.values()):continue
 support_shapes=[];support_info=None
 if a.pedestal:
  width=a.support_width_m or (.10 if a.variant=='property' else .17)
  px,py=np.asarray([x,y])+np.asarray(a.support_offset_xy)
  tops=[(hi[2],n) for n,o,lo,hi in fixed if any(k in n.lower() for k in ['floor','counter','stand_instanceable']) and lo[0]<=px-width/2 and hi[0]>=px+width/2 and lo[1]<=py-width/2 and hi[1]>=py+width/2 and hi[2]<=z+.006]
  if not tops:continue
  top,support_name=max(tops)
  if z-top>.012:
   offset=bool(np.linalg.norm(a.support_offset_xy)>1e-9)
   low=np.asarray([px-width/2,py-width/2,top+.002]);high=np.asarray([px+width/2,py+width/2,z-(.022 if offset else .002)])
   boxes=[(low,high)]
   if offset:
    boxes.append((np.asarray([min(x,px)-width/2,min(y,py)-width/2,z-.022]),np.asarray([max(x,px)+width/2,max(y,py)+width/2,z-.002])))
   if any(np.any(hi<=lo) for lo,hi in boxes):continue
   support_shapes=[('added_support_'+str(i),fcl.CollisionObject(fcl.Box(*(hi-lo)),fcl.Transform((lo+hi)/2)),lo,hi) for i,(lo,hi) in enumerate(boxes)]
   support_info={'low':np.min([lo for lo,hi in boxes],axis=0).tolist(),'high':np.max([hi for lo,hi in boxes],axis=0).tolist(),'rests_on':support_name}
   if offset:support_info.update(parts=[{'low':lo.tolist(),'high':hi.tolist()} for lo,hi in boxes],design='Floor-anchored offset column with a short horizontal tray; full source collision checks, dynamic object stability and matched controls remain mandatory.')
   if not initial_safe(boxes,0.):continue
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
  source_check=clearance(door,hz,margin);support_check=clearance(door,support_shapes,.002) if support_shapes else {'valid':True}
  if not source_check['valid'] or not support_check['valid']:door_safe=False;break
 if not door_safe:
  report.setdefault('door_rejections',[]).append({'pos':pos.tolist(),'yaw':yaw,'action':rows.index(row),'source_check':source_check,'support_check':support_check});continue
 result={**spec,'position':pos.tolist(),'yaw_deg':float(yaw),'baseline_intersection_actions':hits,'source_supports':sorted(set(supports)),'proposed_support':support_info};
 if paired_robot_paths:result['source_intersection_in_reference_off_seeds']=paired_hits
 report['candidates'].append(result);print('CANDIDATE',a.variant,result['position'],yaw,'hits',len(hits),flush=True)
 if len(report['candidates'])>=20:break
 if stats['tested']%500==0:print('SEARCH',stats,flush=True)
report['search_complete']=True;a.output.write_text(json.dumps(report,indent=2));print('SEARCH_DONE',stats,'survivors',len(report['candidates']),flush=True)
