"""Complete the same full-world FCL audit from every recorded 120 Hz state, on CPU workers."""
import argparse,json,multiprocessing
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
from kitchen_safety.extensions.lightwheel_pick_place.task_config import episode_spec


def initialize(ep):
 global world,result,spec
 result=json.loads((Path(ep)/'result.json').read_text());spec=episode_spec(result);b=result['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);world=CollisionWorld(Usd.Stage.Open(str(Path(ep)/'scene.usda')),kin,pick_object=spec.pick_object,destination_object=spec.destination_object,source_surface_suffix=spec.source_surface_suffix,movable_objects=tuple(spec.rigid_objects)+('glass',))


def check_batch(rows):
 output=[]
 for row in rows:
  pick=row['pick_object']
  if world.pick_object!=pick:world.bind_pick(pick)
  if set(row.get('rigid_poses',{}))!=set(spec.rigid_objects):raise ValueError('Missing actual 120 Hz rigid poses')
  for name,pose in row['rigid_poses'].items():
   if name!=pick:world.set_obstacle_pose(name,pose)
  if 'hazard_pose' in row:world.set_obstacle_pose('glass',row['hazard_pose'])
  model=result['source_support_contact_model'] if pick==spec.pick_object else result['extra_source_support_models'][pick]
  world.source_support_mode='native_rest_until_detached';world.source_resting_min_z=model['source_resting_min_z_m'];world.source_resting_root=np.asarray(model['source_resting_root_m']);world.source_resting_xy=world.source_resting_root[:2];world.source_detached=row['source_detached_before'];world.allowed_phase=row['phase']
  ignore=(pick,)+((spec.destination_object,) if row['phase'] in ('lower','release','retreat','settle_final') else ())
  measured=world.clearance(row['q_arm'],ignore,mode='execution',gripper_q=row['gripper_q'],object_pose=row['lime_pose'])
  if row.get('valid') is not None and (row['valid']!=measured['valid'] or abs(row['slack_m']-measured['slack_m'])>1e-7):raise ValueError('Offline/live full-world checks disagree')
  output.append({**row,**measured,'audit_pending':False})
 return output


def main():
 p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('--workers',type=int,default=4);a=p.parse_args();source=a.episode/'full_world_audit.json';report=json.loads(source.read_text());rows=report['measurements']
 if not report.get('pending_samples'):raise SystemExit('No deferred samples; preserve the original audit')
 chunks=[rows[i:i+128] for i in range(0,len(rows),128)];completed=[]
 with ProcessPoolExecutor(max_workers=a.workers,mp_context=multiprocessing.get_context('spawn'),initializer=initialize,initargs=(str(a.episode),)) as pool:
  for batch in pool.map(check_batch,chunks):completed.extend(batch);print('AUDITED',len(completed),len(rows),flush=True)
 report.update(passed=bool(completed and all(x['valid'] for x in completed)),minimum=min(completed,key=lambda x:x['slack_m']),pending_samples=0,measurements=completed,sampling_frequency_hz=120,live_checkpoint_parity_verified=True,scope='Complete offline 120 Hz full-world FCL using every actual measured arm, finger, fruit, property and task-rigid pose. Same geometry and safety margins as online checking. Every available live 15 Hz full-world result must agree within 1e-7 m. Added hazard checks remained live at 120 Hz.')
 backup=a.episode/'full_world_audit.before_deferred_recheck.json'
 if backup.exists():raise FileExistsError(backup)
 temp=a.episode/'full_world_audit.completed.json'
 with temp.open('x') as f:json.dump(report,f,indent=2)
 source.rename(backup);temp.rename(source);print('FULL_WORLD_PASSED',report['passed']);return 0 if report['passed'] else 2

if __name__=='__main__':raise SystemExit(main())
