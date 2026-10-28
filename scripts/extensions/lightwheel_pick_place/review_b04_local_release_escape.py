"""Bounded saved-state local release line proof, no physics rollout."""
import json,hashlib
from pathlib import Path
import h5py,numpy as np
from scipy.spatial.transform import Rotation
from paired_planning import build_world
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.lightwheel_pick_place.task_config import episode_spec
from kitchen_safety.extensions.lightwheel_pick_place.planner import LimePlanner
p=Path('datasets/skill_trials/lightwheel_pick_place/base_lgd_b04_fresh_matched_20261028T100913Z');r=json.loads((p/'result.json').read_text());spec=episode_spec(r);b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);world,stage=build_world(p,kin,None,{},task_spec=spec)
with h5py.File(p/'trajectory.h5') as h:
 index=int(np.flatnonzero(h['phase'].asstr()[:]=='release')[0])-1;q=np.asarray(h['next_obs/q'][index,:7]);allq=h['next_obs/q'][index];names=r['robot_joint_names'];finger={n:float(allq[names.index(n)]) for n in names if 'finger' in n or 'knuckle' in n}
 for n in spec.rigid_objects:world.set_obstacle_pose(n,h['next_obs/'+n][index,:7])
world.source_detached=True;world.attachment=None;world.allowed_phase='release';world.gripper_joint_position=finger['finger_joint'];world.planning_gripper_q=finger
proposal=json.loads(Path('/root/autodl-tmp/B04-review/measured_release_plan36.json').read_text());target=kin.fk(q);target[:3,3]+=np.array(proposal['selected']['delta'])
planner=LimePlanner(world,kin,dt=r['dt'],seed=r['seed'],max_joint_speed=.4);planner.single_candidate_round=True;planner.certify_execution_margin=True;planner.query_start_q=q.copy()
report={'scope':'Saved measured supported release-start state. Same40mm lateral/15mm rising escape goal and orientation, local warm-start Cartesian line instead of long redundant MotionGen detour. Original cuRobo/FCL constraints; zero physics, not native acceptance.','episode':str(p),'action_index':index,'result_sha256':hashlib.sha256((p/'result.json').read_bytes()).hexdigest(),'passed':False}
try:
 path=planner.plan_cartesian_line(q,target,(spec.pick_object,spec.destination_object),1.,phase='release');check=planner.validate_path(path['positions'],(spec.pick_object,spec.destination_object),1.,phase='release')
 report.update(passed=bool(check['valid']),validation=check,metadata=path['metadata'],positions=np.asarray(path['positions']).tolist(),target_tcp=target.tolist())
except RuntimeError as exc:report['error']=str(exc)
Path('/root/autodl-tmp/B04-review/fresh_local_release_escape_proof_20261028.json').write_text(json.dumps(report,indent=2));print('LOCAL_RELEASE_PROOF',report['passed'],report.get('error'))
