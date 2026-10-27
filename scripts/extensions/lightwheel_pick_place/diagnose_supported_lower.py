"""Read-only diagnosis of the saved failed descent; no PhysX rollout."""
import json,h5py,numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
from paired_planning import build_world
from kitchen_safety.planning import DroidKinematics,transform
from kitchen_safety.extensions.lightwheel_pick_place.task_config import episode_spec
from kitchen_safety.extensions.lightwheel_pick_place.planner import LimePlanner
root=Path('datasets/skill_trials/lightwheel_pick_place');ep=sorted(root.glob('robot_lgd_b03_checkpoint_replay_*'))[-1];r=json.load(open(ep/'result.json'));b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);world,stage=build_world(ep,kin,None,{},task_spec=episode_spec(r));pick='banana_2';world.bind_pick(pick)
with h5py.File(ep/'trajectory.h5') as h:
 ids=[r['robot_joint_names'].index('panda_joint'+str(j)) for j in range(1,8)];q=h['next_obs/q'][-1,ids];pose=h['next_obs/banana_2'][-1,:7];gripper={n:float(h['next_obs/q'][-1,r['robot_joint_names'].index(n)]) for n in world.GRIPPER_JOINT_NAMES}
 for n in r['initial_object_states']:
  if n!=pick:world.set_obstacle_pose(n,h['next_obs/'+n][-1,:7])
world.attachment=np.linalg.inv(kin.fk(q))@transform(pose[:3],pose[3:7]);world.source_detached=True;world.allowed_phase='lower';world.placed_task_contents={'banana_1'};world.gripper_joint_position=gripper['finger_joint'];world.planning_gripper_q=gripper
planner=LimePlanner(world,kin,dt=r['dt'],seed=43,max_joint_speed=.4);tcp=kin.fk(q);previous=q.copy();path=[q]
for index in range(1,71):
 target=tcp.copy();target[2,3]-=.002*index;candidate=kin.ik(target,previous,attempts=1)
 if np.linalg.norm(kin.fk(candidate)[:3,3]-target[:3,3])>.001 or not world.valid(candidate,(pick,'wooden_bowl')):break
 for f in (1/3,2/3,1.):path.append(previous+(candidate-previous)*f)
 previous=candidate
path=np.asarray(path);planner._prepare_path_query((pick,'wooden_bowl'),1.,'lower');check=lambda values:planner.motion_gen.ik_solver.check_valid(planner.args.to_device(np.asarray(values))).detach().cpu().numpy().astype(bool).reshape(-1)
normal=check(path);bad=np.flatnonzero(~normal);report={'source':str(ep),'path_samples':len(path),'first_bad':int(bad[0]) if len(bad) else None,'placed_names':[n for n in planner.obstacle_names if world._matches(n,'banana_1')],'full_independent_validation':planner.validate_path(path,(pick,'wooden_bowl'),1.,phase='lower')}
if len(bad):
 goal=path[bad[0]];report['normal_single_valid']=bool(check(goal[None])[0]);report['world_contents']=list(getattr(world,'placed_task_contents',()));report['planner_world_same']=planner.world is world;report['prepared_mask']=getattr(planner,'last_prepared_contact_mask',[]);
 for name in report['placed_names']:planner.motion_gen.world_coll_checker.enable_obstacle(name,enable=False)
 report['explicit_disable_placed_valid']=bool(check(goal[None])[0]);report['ignored_names']=[n for n in planner.obstacle_names if planner._ignored(n,(pick,'wooden_bowl'))];report['prepare_source']=planner._prepare_path_query.__func__.__code__.co_filename;report['q_bad']=goal.tolist();report['joint_limit_slack']=float(np.minimum(goal-np.asarray(kin.limits)[:,0],np.asarray(kin.limits)[:,1]-goal).min())
 for name in planner.obstacle_names:planner.motion_gen.world_coll_checker.enable_obstacle(name,enable=False)
 report['empty_world_valid']=bool(check(goal[None])[0])
 if report['empty_world_valid']:
  active=[n for n in planner.obstacle_names if not planner._ignored(n,(pick,'wooden_bowl')) and not world._matches(n,'banana_1')]
  def valid_names(names):
   selected=set(names)
   for name in planner.obstacle_names:planner.motion_gen.world_coll_checker.enable_obstacle(name,enable=name in selected)
   return bool(check(goal[None])[0])
  offenders=[]
  def locate(names):
   if not names or valid_names(names):return
   if len(names)==1:offenders.extend(names);return
   split=len(names)//2;locate(names[:split]);locate(names[split:])
  locate(active);report['offending_backend_colliders']=offenders
  if not offenders:
   locate(planner.obstacle_names);report['all_names_offenders']=offenders
Path('/root/autodl-tmp/B03-review/supported_lower_backend_mask_trace.json').write_text(json.dumps(report,indent=2));print('LOWER_DIAGNOSIS',report,flush=True)
