"""CPU search against a measured source-valid baseline; never executes counterfactuals."""
import argparse,copy,json,time
from pathlib import Path
import fcl,h5py,numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,Gf
from scripts.extensions.replicator.audit import hazard_stage,HAZARDS
from scripts.extensions.replicator.audit_fixed_base import fixed_colliders
from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld
from kitchen_safety.extensions.replicator.layout import resolve_layout
from kitchen_safety.planning import DroidKinematics
ROOT=Path(__file__).resolve().parents[3]
BASE=ROOT/'datasets/avoidance_trials/replicator/base/baseline_01'

def load_baseline(path):
 r=json.loads((path/'result.json').read_text());assert r['success'] and r['arena_success'] and r['deferred_safety']['passed']
 assert json.loads((path/'source_relation_audit.json').read_text())['passed']
 with h5py.File(path/'trajectory.h5') as f:
  names=json.loads(f.attrs['joint_names']);ids=[names.index(f'panda_joint{i}') for i in range(1,8)]
  d={k:np.concatenate([f['obs/'+k][:],f['next_obs/'+k][-1:]]) for k in ['q','mustard','bowl','background_rigid_poses','robot_body_pose_w']}
  d['phases']=f['phase'].asstr()[:].tolist();d['phases'].append(d['phases'][-1]);d['dt']=float(f.attrs['dt']);d['body_names']=json.loads(f.attrs['robot_body_names'])
 return r,d,names,ids

def settled_index(result):
 return int(result.get('effective_controls',{}).get('initial_settling_actions',30))

def kitchen_world(layout,kin,data,result):
 index=settled_index(result)
 stage=Usd.Stage.CreateInMemory();prefix='/World/envs/env_0'
 def ref(name,path,pose):
  root=UsdGeom.Xform.Define(stage,prefix+'/'+name);root.GetPrim().GetReferences().AddReference(str(ROOT/path));root.ClearXformOpOrder();root.AddTranslateOp().Set(Gf.Vec3d(*map(float,pose[:3])));q=pose[3:];root.AddOrientOp().Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,q[:3]))))
 ref('kitchen',layout['kitchen_usd'],[0,0,0,0,0,0,1]);ref('bowl','assets/robolab/objects/ycb/bowl.usd',data['bowl'][index,:7]);ref('mustard_bottle','assets/robolab/objects/hope/mustard_bottle.usd',data['mustard'][index,:7])
 w=CollisionWorld(stage,kin);w.sync_background(result['background_rigid_paths'],data['background_rigid_poses'][index]);return w

def initial_geometry(world,environment,fixed,variant):
 # Exact pair intersections plus conservative initial/goal motion-envelope checks elsewhere.
 failures=[];floor_support=False;island_support=False
 for name,obj,lo,hi in world.objects:
  for other,ob,low,high in environment.objects:
   if np.any(hi<low) or np.any(high<lo):continue
   support=(other.endswith('/CapSurface_03') and variant!='human' and lo[2]>=high[2]-.006) or ('floor' in other.lower() and variant=='human' and lo[2]>=high[2]-.006)
   if support:
    if variant=='human':floor_support=True
    else:island_support=True
    continue
   if fcl.collide(obj,ob,fcl.CollisionRequest(),fcl.CollisionResult()):
    failures.append([name,other]);return False,failures
  for ob,info in fixed:
   low,high=np.array(info['bounds'])
   if np.any(hi<low) or np.any(high<lo):continue
   if fcl.collide(obj,ob,fcl.CollisionRequest(),fcl.CollisionResult()):
    failures.append([name,info['path']]);return False,failures
 return True,failures

def main():
 p=argparse.ArgumentParser();p.add_argument('--variant',choices=HAZARDS,required=True);p.add_argument('--baseline',type=Path,default=BASE);p.add_argument('--stride',type=int,default=8);p.add_argument('--report',type=Path);p.add_argument('--candidate-json',type=Path);p.add_argument('--limit',type=int,default=0);p.add_argument('--refine-human',action='store_true');p.add_argument('--strict-waypoints',action='store_true');p.add_argument('--candidates-from-search',type=Path);p.add_argument('--west',action='store_true');p.add_argument('--retain-pregrasp',action='store_true');p.add_argument('--east-deep',action='store_true');a=p.parse_args();a.baseline=a.baseline.resolve()
 result,data,names,ids=load_baseline(a.baseline);variant=a.variant;hazard=HAZARDS[variant]
 layout=resolve_layout({'human':'human_default_mpfb','property':'property_vase','robot':'robot_projecting_crossbar'}[variant]);assert {k:v for k,v in layout.items() if k!='obstacle'}==result['layout'];entry=layout['obstacle'];kin=DroidKinematics(np.array(layout['robot_position'])+[0,0,layout['stand_height']],layout['robot_yaw_deg']);cfg=layout['planning']
 stage,_=hazard_stage(layout,variant,entry);world=CollisionWorld(stage,kin,clearances=cfg['clearances_m'],execution_clearances=cfg['execution_clearances_m']);environment=kitchen_world(layout,kin,data,result)
 fixed=fixed_colliders(data['robot_body_pose_w'][0,data['body_names'].index('panda_link0')]);n=len(data['q']);phases=data['phases'];endpoints=sorted(set([0,settled_index(result),n-1]+[max(i for i,p in enumerate(phases) if p==phase) for phase in ['approach','close','lower','release']]))
 if a.strict_waypoints:endpoints=sorted(set(endpoints)|{max(i for i,p in enumerate(phases) if p==phase) for phase in ['pregrasp','lift']})
 if a.retain_pregrasp:endpoints=sorted(set(endpoints)|{max(i for i,p in enumerate(phases) if p=='pregrasp')})
 indices=sorted(set(range(0,n,a.stride))|set(endpoints));original_shapes=world.shapes;shapes={}
 for i in indices:
  grip={name:float(data['q'][i,names.index(name)]) for name in world.GRIPPER_JOINT_NAMES};shapes[i]=original_shapes(data['q'][i,ids],grip,data['mustard'][i,:7])
 current=[0];world.shapes=lambda *args,**kwargs:shapes[current[0]]
 if a.candidates_from_search:
  candidates=[(*row['position'],row['yaw_deg']) for row in json.loads(a.candidates_from_search.read_text())['candidates']]
 elif a.candidate_json:
  e=json.loads(a.candidate_json.read_text())['obstacle'];candidates=[(*e['position'],e.get('yaw_deg',0))]
 elif variant=='human' and a.east_deep:
  candidates=[(float(x),float(y),0.,float(yaw)) for x in np.arange(1.05,1.651,.05) for y in np.arange(-.65,-.19,.025) for yaw in range(30,126,10)]
 elif variant=='human' and a.west:
  candidates=[(float(x),float(y),0.,float(yaw)) for x in np.arange(-.4,-.049,.025) for y in np.arange(-.175,.376,.025) for yaw in range(90,251,10)]
 elif variant=='human' and a.refine_human:
  candidates=[(float(x),float(y),0.,float(yaw)) for x in np.arange(1.07,1.331,.02) for y in np.arange(-.23,.111,.02) for yaw in range(65,116,5)]
 elif variant=='human':
  candidates=[(float(x),float(y),0.,float(yaw)) for x in np.arange(-.65,1.71,.1) for y in np.arange(-.15,1.61,.1) for yaw in range(0,360,30)]
 elif variant=='property':candidates=[(float(x),float(y),layout['counter_height']+.002,0.) for x in np.arange(.15,.91,.025) for y in np.arange(-1.15,-.375,.025)]
 else:candidates=[(float(x),float(y),layout['counter_height']+.002,float(yaw)) for x in np.arange(.15,.91,.05) for y in np.arange(-1.15,-.375,.05) for yaw in [0,45,90,135,180,225,270,315]]
 if a.limit:candidates=candidates[:a.limit]
 rows=[];outcomes=[];counts=dict(total=len(candidates),endpoint_rejected=0,geometry_rejected=0,no_obstruction=0);started=time.time()
 for num,(x,y,z,yaw) in enumerate(candidates):
  outcome=dict(candidate_index=num,position=[x,y,z],yaw_deg=yaw);outcomes.append(outcome)
  pose=np.r_[[x,y,z],Rotation.from_euler('z',yaw,degrees=True).as_quat()];world.set_obstacle_pose(hazard,pose);endpoint_bad=False
  for i in endpoints:
   current[0]=i
   if not world.valid(data['q'][i,ids],mode='planning',obstacle_filter=(hazard,)):
    endpoint_bad=True;break
  if endpoint_bad:
   counts['endpoint_rejected']+=1;outcome.update(status='endpoint_rejected',index=i,phase=phases[i],clearance=world.last_clearance);continue
  legal,failures=initial_geometry(world,environment,fixed,variant)
  if not legal:
   counts['geometry_rejected']+=1;outcome.update(status='geometry_rejected',intersections=failures);continue
  violations=[]
  for i in indices:
   if i in endpoints or phases[i] in ['settle','release','retreat']:continue
   current[0]=i
   if not world.valid(data['q'][i,ids],mode='planning',obstacle_filter=(hazard,)):
    violations.append(dict(index=i,phase=phases[i],time_s=i*data['dt'],**world.last_clearance))
  if not violations:
   counts['no_obstruction']+=1;outcome.update(status='no_sampled_obstruction');continue
  row=dict(position=[x,y,z],yaw_deg=yaw,baseline_violations=violations,minimum=min(violations,key=lambda v:v['slack_m']),tested_endpoints=endpoints,initial_geometry_clear=True)
  outcome.update(status='candidate_retained',violating_samples=len(violations),minimum=row['minimum']);rows.append(row);print(json.dumps(dict(candidate=num,position=row['position'],yaw_deg=yaw,violating_samples=len(violations),minimum=row['minimum'])),flush=True)
 rows.sort(key=lambda r:(not any(v['distance_m']<cfg['execution_clearances_m'][hazard] for v in r['baseline_violations']),-len(r['baseline_violations']),r['minimum']['slack_m']))
 report=dict(baseline=str(a.baseline.relative_to(ROOT)),variant=variant,sampling_stride=a.stride,elapsed_seconds=time.time()-started,counts=counts,candidate_outcomes=outcomes,candidates=rows,scope='Unexecuted geometric counterfactual of measured baseline q/fingers/bottle poses. Source layout unchanged. Endpoint/source/kitchen/fixed-solid collider prefilter; full-rate audit and live physical validation required before acceptance.')
 target=a.report or ROOT/'reports/avoidance_trials/replicator'/f'{variant}_search.json';target.parent.mkdir(parents=True,exist_ok=True);target.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(dict(report=str(target),valid_candidates=len(rows),counts=counts,elapsed_seconds=report['elapsed_seconds'])),flush=True)
if __name__=='__main__':main()
