"""Per-object Lipschitz certificate for source settling, original margins retained."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
from kitchen_safety.extensions.lightwheel_pick_place.task_config import episode_spec
p=argparse.ArgumentParser();p.add_argument('--episode',type=Path,required=True);p.add_argument('--baseline',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
paths={'baseline':a.baseline,'augmented':a.episode};r={k:json.load(open(v/'result.json')) for k,v in paths.items()};old=json.load(open(a.episode/'source_background_audit.json'));bounds=json.load(open(a.episode/'avoidance_audit.json'))['source_background_motion_envelopes']['backgrounds'];spec=episode_spec(r['baseline']);b=r['baseline']['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);objects=tuple(bounds['baseline']['objects']);world=CollisionWorld(Usd.Stage.Open(str(a.baseline/'scene.usda')),kin,pick_object=spec.pick_object,destination_object=spec.destination_object,source_surface_suffix=spec.source_surface_suffix,movable_objects=objects)
traces={k:json.load(open(v/'full_world_audit.json'))['measurements'] for k,v in paths.items()};cases={}
for label,meta in r.items():
 for n in objects:world.set_obstacle_pose(n,meta['initial_object_states'][n][:7])
 for trajectory,states in traces.items():
  key=trajectory+'_path_in_'+label+'_background';bound=max(v['max_surface_displacement_bound_m'] for v in bounds[label]['objects'].values());minimum=float('inf');checked=0;failures=[];refinements=[]
  for state,row in zip(states,old['combinations'][key]['measurements']):
   if row['slack_m']>=bound:minimum=min(minimum,row['slack_m']-bound);continue
   active=state.get('pick_object',spec.pick_object)
   if world.pick_object!=active:world.bind_pick(active)
   world.allowed_phase=state['phase'];model=r[trajectory]['source_support_contact_model'] if active==spec.pick_object else r[trajectory]['extra_source_support_models'][active];world.source_support_mode=r[trajectory]['source_support_contact_model']['mode'];world.source_detached=state['source_detached_before'];world.source_resting_min_z=state.get('source_support_calibrated_min_z_m',model['source_resting_min_z_m']);world.source_resting_root=np.asarray(model['source_resting_root_m'])
   for n in objects:
    c=world.clearance(state['q_arm'],(active,),mode='execution',gripper_q=state['gripper_q'],object_pose=state['lime_pose'],obstacle_filter=(n,));slack=c['slack_m']-bounds[label]['objects'][n]['max_surface_displacement_bound_m'];minimum=min(minimum,slack);checked+=1
    if slack<0:
     poses=np.unique(np.array([t['rigid_poses'][n][:7] for t in traces[label]]),axis=0)
     exact=float('inf')
     for pose in poses:
      world.set_obstacle_pose(n,pose)
      test=world.clearance(state['q_arm'],(active,),mode='execution',gripper_q=state['gripper_q'],object_pose=state['lime_pose'],obstacle_filter=(n,))
      exact=min(exact,test['slack_m'])
      if exact<0:break
     refinements.append({'action':state['action'],'object':n,'poses_checked':len(poses),'exact_slack_m':exact})
     world.set_obstacle_pose(n,meta['initial_object_states'][n][:7])
     if exact<0:failures.append({'action':state['action'],'object':n,'slack_m':slack,'exact_observed_envelope_slack_m':exact})
     else:print('EXACT_ENVELOPE',key,n,state['action'],len(poses),exact,flush=True)

  cases[key]={'passed':not failures,'minimum_bound_slack_m':minimum,'refined_checks':checked,'exact_observed_envelope_checks':refinements,'failures':failures[:20]};print(key,cases[key],flush=True)
out={'passed':all(x['passed'] for x in cases.values()),'input_sha256':old['input_sha256'],'cases':cases,'scope':'Every non-destination source object gets its own full observed rigid-motion envelope; original per-pair margin retained. Rows certified by global bound are reused; all other rows get individual exact FCL distances. Where an isotropic bound fails, every unique measured 120Hz object pose is cross-checked against that measured robot/payload state. No continuous-time interpolation claim. Destination cross-replay remains independently required.'};a.output.write_text(json.dumps(out,indent=2))
