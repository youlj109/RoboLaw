"""One orientation-aware transfer correction per frozen B04 scene; no physics."""
import json
from pathlib import Path
import h5py,numpy as np
from scipy.spatial.transform import Rotation
from paired_planning import build_world,load_variants
from kitchen_safety.planning import DroidKinematics,transform
from kitchen_safety.extensions.lightwheel_pick_place.task_config import episode_spec
from kitchen_safety.extensions.lightwheel_pick_place.planner import LimePlanner
root=Path('datasets/skill_trials/lightwheel_pick_place');ep=root/'base_lgd_b04_handle_20261026T112101Z';ref=root/'base_lgd_b04_handle_20261025T194846Z';out=Path('/root/autodl-tmp/B04-review/planning_round04_20261027.json')
if out.exists():raise FileExistsError(out)
r=json.load(open(ep/'result.json'));spec=episode_spec(r);b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);pick=spec.pick_object
with h5py.File(ref/'trajectory.h5') as h:
 i=int(np.flatnonzero(h['phase'].asstr()[:]=='release')[0]);reference_quat=h['obs/'+pick][i,3:7]
with h5py.File(ep/'trajectory.h5') as h:
 i=int(np.flatnonzero(h['phase'].asstr()[:]=='lift')[-1]);ids=[r['robot_joint_names'].index('panda_joint'+str(j)) for j in range(1,8)];q=h['next_obs/q'][i,ids];pose=h['next_obs/'+pick][i,:7];dest=h['next_obs/'+spec.destination_object][i,:7];finger=float(h['next_obs/q'][i,r['robot_joint_names'].index('finger_joint')]);poses={n:h['next_obs/'+n][i,:7] for n in spec.rigid_objects if n!=pick}
variants=load_variants(Path('configs/extensions/lightwheel_pick_place/variants_ladle_overnight_layout.json'));report={'scope':'One transfer body-orientation correction per fixed scene, native-success reference quaternion, root120mm above plate, no physics or fallback.','cases':{}}
for variant in ('base','robot','property','human'):
 row={'passed':False}
 try:
  world,stage=build_world(ep,kin,variant,variants,task_spec=spec)
  for n,v in poses.items():world.set_obstacle_pose(n,v)
  attachment=np.linalg.inv(kin.fk(q))@transform(pose[:3],pose[3:7]);world.attachment=attachment;world.source_detached=True;world.allowed_phase='transfer';world.gripper_joint_position=finger
  target=transform(dest[:3]+[-.03,0,.12],reference_quat)@np.linalg.inv(attachment);planner=LimePlanner(world,kin,dt=r['dt'],seed=43,max_joint_speed=.4);planner.single_candidate_round=True
  world.source_detached=False;world.allowed_phase='lift';up=kin.fk(q).copy();up[2,3]+=.06
  extraction=planner.plan_cartesian_line(q,up,(pick,),1.,phase='lift');lift_check=planner.validate_path(extraction['positions'],(pick,),1.,phase='lift')
  if not lift_check['valid']:raise RuntimeError('Complete source extraction: '+str(lift_check))
  lifted_q=np.asarray(extraction['positions'][-1]);world.source_detached=True;world.allowed_phase='transfer'
  path=planner.plan_pose(lifted_q,target,(pick,),1.,phase='transfer');checked=planner.validate_path(path['positions'],(pick,),1.,phase='transfer')
  row['extra_extraction_positions']=np.asarray(extraction['positions']).tolist()
  if not checked['valid']:raise RuntimeError(str(checked))
  row.update(passed=True,target=target.tolist(),body_quaternion=reference_quat.tolist(),positions=np.asarray(path['positions']).tolist())
 except Exception as exc:row['error']=str(exc)
 report['cases'][variant]=row;out.write_text(json.dumps(report,indent=2));print('B04_ROUND04',variant,row['passed'],row.get('error'),flush=True)
report['all_passed']=all(v['passed'] for v in report['cases'].values());out.write_text(json.dumps(report,indent=2))
