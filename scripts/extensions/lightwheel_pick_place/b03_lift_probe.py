"""Offline separation of held-banana proxy and robot source-plate rejection."""
import json
from pathlib import Path
import h5py,numpy as np
from pxr import Usd
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
from kitchen_safety.extensions.lightwheel_pick_place.planner import LimePlanner
from kitchen_safety.extensions.lightwheel_pick_place.task_config import resolve_task,BANANA_TASK
p=Path('datasets/skill_trials/lightwheel_pick_place/base_lgd_b03_review_20261025T124856Z');r=json.load(open(p/'result.json'));spec=resolve_task(BANANA_TASK);b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:]).as_euler('xyz',degrees=True)[2])
with h5py.File(p/'trajectory.h5') as h:state={k:v[-1] for k,v in h['next_obs'].items()}
q=state['q'][:7];world=CollisionWorld(Usd.Stage.Open(str(p/'scene.usda')),kin,pick_object=spec.pick_object,destination_object=spec.destination_object,source_surface_suffix=spec.source_surface_suffix,movable_objects=tuple(n for n in spec.rigid_objects if n!=spec.pick_object));world.gripper_joint_position=float(state['q'][r['robot_joint_names'].index('finger_joint')]);world.attachment=np.array(r['planning_attachment_tcp_to_lime']);attachment=world.attachment.copy();source=r['source_support_contact_model'];world.source_support_mode='native_rest_until_detached';world.source_resting_min_z=source['source_resting_min_z_m'];world.source_resting_root=np.array(source['source_resting_root_m']);world.source_resting_xy=np.array(source['source_resting_xy_m']);world.source_detached=False;world.allowed_phase='lift'
for n in spec.rigid_objects:
 if n!=spec.pick_object:world.set_obstacle_pose(n,state[n][:7])
T=kin.fk(q);path=[q.copy()]
for dz in np.linspace(0,.12,61)[1:]:
 target=T.copy();target[2,3]+=dz;q=kin.ik(target,q,attempts=12);path.append(q.copy())
path=np.array(path);planner=LimePlanner(world,kin,dt=r['dt'],seed=43,max_joint_speed=.4);report={};planner.query_start_q=path[0]
for label,attached in [('with_payload',True),('robot_only',False)]:
 world.attachment=attachment if attached else None;planner._prepare_path_query((spec.pick_object,),1.,'lift');valid=planner.motion_gen.ik_solver.check_valid(planner.args.to_device(path)).detach().cpu().numpy().astype(bool);report[label]={'passed':bool(valid.all()),'invalid_waypoints':np.flatnonzero(~valid).tolist()}
print('LIFT_PROBE',json.dumps(report),flush=True);Path('/root/autodl-tmp/B03-review/lift_proxy_probe.json').write_text(json.dumps(report,indent=2));np.savez('/root/autodl-tmp/B03-review/lift_proxy_probe.npz',positions=path)

world.attachment=attachment;world.allowed_phase='lift'
target=T.copy();target[2,3]+=.12
trajectory=planner.plan_pose(path[0],target,(spec.pick_object,),1.,phase='lift')
checked=planner.validate_path(trajectory['positions'],(spec.pick_object,),1.,phase='lift')
print('FRESH_FULL_LIFT',json.dumps(checked),flush=True)
Path('/root/autodl-tmp/B03-review/fresh_validated_lift_plan.json').write_text(json.dumps({'validation':checked,'metadata':trajectory.get('metadata'), 'positions':np.asarray(trajectory['positions']).tolist()},indent=2))
