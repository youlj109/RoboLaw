"""Seed-paired collision-world toggles for a fixed recorded free-transfer query.

No environment/physics starts and no proposed path is executed. All seed pairs
and rejected planner proposals remain in the JSON/NPZ outputs.
"""
import argparse,gc,hashlib,json,time
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom,Gf
from scipy.spatial.transform import Rotation
from scripts.extensions.replicator.audit import hazard_stage
from scripts.extensions.replicator.search_avoidance import load_baseline
from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld
from kitchen_safety.extensions.replicator.curobo_backend import CuroboPlanner
from kitchen_safety.extensions.replicator.source_geometry import SourceBottleGeometry,source_clearance
from kitchen_safety.geometry_audit import static_collision_shapes
from kitchen_safety.planning import DroidKinematics
ROOT=Path(__file__).resolve().parents[3]
def main():
 p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True);p.add_argument('--layout',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--seeds',type=int,nargs='+',default=[42,43,44]);a=p.parse_args()
 if a.output.exists():raise FileExistsError(a.output)
 a.output.mkdir(parents=True);import torch
 torch.set_num_threads(2)
 r,d,names,ids=load_baseline(a.baseline);layout=json.loads(a.layout.read_text());assert {k:v for k,v in layout.items() if k!='obstacle'}==r['layout']
 kitchen_path=Path(layout['kitchen_usd'])
 if r.get('background_physics'):
  from kitchen_safety.extensions.replicator.background_physics import mounted_kitchen
  kitchen_path,background_profile=mounted_kitchen(ROOT/kitchen_path,ROOT/'reports/replicator_physics/assets')
  if background_profile!=r['background_physics']:raise ValueError('Baseline background installation differs from current source/recipe')
 idx=next(i for i,s in enumerate(d['phases']) if s=='above_bowl');q=d['q'][idx,ids];grip={name:float(value) for name,value in zip(names,d['q'][idx])};attachment=np.asarray(r['attachment_after_lift']);target=np.asarray(next(x for x in r['plans'] if x['phase']=='above_bowl')['effective_goal_pose_world']);kin=DroidKinematics(np.asarray(layout['robot_position'])+[0,0,layout['stand_height']],layout['robot_yaw_deg']);settings=layout['planning']
 hazardstage,_=hazard_stage(layout,'human',layout['obstacle']);hazard=static_collision_shapes(hazardstage,'/World/Person');auditworld=CollisionWorld(hazardstage,kin);source=SourceBottleGeometry();hull=SourceBottleGeometry(True)
 def world(on):
  stage=hazard_stage(layout,'human',layout['obstacle'])[0] if on else Usd.Stage.CreateInMemory()
  for name,path,pose in [('kitchen',kitchen_path,[0,0,0,0,0,0,1]),('bowl','assets/robolab/objects/ycb/bowl.usd',d['bowl'][idx,:7]),('mustard_bottle','assets/robolab/objects/hope/mustard_bottle.usd',d['mustard'][idx,:7])]:
   x=UsdGeom.Xform.Define(stage,'/World/envs/env_0/'+name);x.GetPrim().GetReferences().AddReference(str(ROOT/path));x.ClearXformOpOrder();x.AddTranslateOp().Set(Gf.Vec3d(*map(float,pose[:3])));x.AddOrientOp().Set(Gf.Quatf(float(pose[6]),Gf.Vec3f(*map(float,pose[3:6]))))
  w=CollisionWorld(stage,kin,clearances=settings['clearances_m'],execution_clearances=settings['execution_clearances_m']);w.sync_background(r['background_rigid_paths'],d['background_rigid_poses'][idx]);w.attachment=attachment.copy();w.gripper_joint_position=float(grip['finger_joint']);return stage,w
 def metrics(path):
  xyz=np.array([kin.fk(v)[:3,3] for v in path]);straight=float(np.linalg.norm(xyz[-1]-xyz[0]));return dict(tcp_path_length_m=float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).sum()),tcp_endpoint_distance_m=straight,joint_path_length_rad=float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum()),duration_s=(len(path)-1)*d['dt'],samples=len(path))
 original_indices=[i for i,s in enumerate(d['phases']) if s=='above_bowl'];original=metrics(d['q'][original_indices][:,ids]);rows=[]
 report=dict(planning_kitchen_reference=str(kitchen_path),background_physics=r.get('background_physics',{'profile':'legacy_source'}),planner_source_sha256={str(p):hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in [Path('kitchen_safety/curobo_backend.py'),Path('kitchen_safety/extensions/replicator/curobo_backend.py'),Path('kitchen_safety/extensions/replicator/collision_planner.py')]},seeds=a.seeds,predeclared_seed_order=True,layout=str(a.layout),layout_sha256=hashlib.sha256(a.layout.read_bytes()).hexdigest(),baseline=str(a.baseline),baseline_result_sha256=hashlib.sha256((a.baseline/'result.json').read_bytes()).hexdigest(),recorded_start_action_index=idx,start_q_rad=q.tolist(),gripper=grip,target_tcp_pose=target.tolist(),attachment_tcp_to_bottle=attachment.tolist(),controls=dict(initial_settling_seconds=r.get('effective_controls',{}).get('initial_settling_seconds',2.),dt=d['dt'],max_joint_speed_rad_s=settings['max_joint_speed_rad_s'],transport_object_padding_m=settings['transport_object_padding_m_by_variant']['human'],ignore_names=['mustard_bottle'],same_source_layout=True),source=source.provenance,original_recorded_successful_transfer=original,rows=rows,scope='Fixed measured lift-end state and fixed above-bowl target; only collision-world human toggled per seed. No physical counterfactual execution. This isolates free transfer, not full-task multi-seed completion or varied layouts. Source mesh and convex hull checks do not reproduce cooked PhysX shapes.')
 def save(): (a.output/'paired_plans.json').write_text(json.dumps(report,indent=2)+'\n')
 save()
 for seed in a.seeds:
  for on in [False,True]:
   name=f'seed_{seed}_'+('on' if on else 'off');start=time.perf_counter();row=dict(seed=seed,human_in_planning_world=on,success=False);rows.append(row)
   planner=None;stage=None;w=None
   try:
    np.random.seed(seed);torch.manual_seed(seed);stage,w=world(on);planner=CuroboPlanner(w,kin,dt=d['dt'],seed=seed,max_joint_speed=settings['max_joint_speed_rad_s'],transport_object_padding_m=settings['transport_object_padding_m_by_variant']['human'])
    route=planner.plan_pose(q,target,('mustard_bottle',),1,phase='above_bowl');path=route['positions'];np.savez_compressed(a.output/(name+'.npz'),q=path,tcp=np.array([kin.fk(v) for v in path]),attachment=attachment)
    # Every commanded segment sampled at eight subdivisions (120Hz equivalent),
    # with rigid predicted attachment; physical tracking is checked separately.
    dense=np.concatenate([path[:1]]+[np.linspace(x,y,9)[1:] for x,y in zip(path[:-1],path[1:])]);meshes=[];hulls=[]
    for i,joints in enumerate(dense):
     obj=kin.fk(joints)@attachment;pose=np.r_[obj[:3,3],Rotation.from_matrix(obj[:3,:3]).as_quat()]
     meshes.append(dict(sample=i,**source_clearance(auditworld,source,joints,grip,pose,hazard,.02)))
     hulls.append(dict(sample=i,**source_clearance(auditworld,hull,joints,grip,pose,hazard,.02)))
    row.update(success=True,**metrics(path),metadata=route['metadata'],source_mesh_minimum=min(meshes,key=lambda x:x['distance_m']),source_mesh_violations=sum(not x['valid'] for x in meshes),source_hull_minimum=min(hulls,key=lambda x:x['distance_m']),source_hull_violations=sum(not x['valid'] for x in hulls),geometric_samples=len(dense),path_file=name+'.npz',path_sha256=hashlib.sha256((a.output/(name+'.npz')).read_bytes()).hexdigest())
   except Exception as error:row['error']=repr(error)
   row['wall_seconds']=time.perf_counter()-start;save();print(json.dumps(row),flush=True)
   del planner,w,stage;gc.collect();torch.cuda.empty_cache()
 report['pairs']=[dict(seed=seed,off_success=next(x for x in rows if x['seed']==seed and not x['human_in_planning_world'])['success'],on_success=next(x for x in rows if x['seed']==seed and x['human_in_planning_world'])['success']) for seed in a.seeds];save()
if __name__=='__main__':main()
