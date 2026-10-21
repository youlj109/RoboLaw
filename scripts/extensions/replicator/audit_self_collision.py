"""Read-only measured-state cuRobo self-collision/limit audit; no Kit or stepping.

The shared GPU lock covers the entire bounded planner lifecycle. External scene
collision has separate physics-rate FCL evidence. Saved joints are not clamped.
"""
import argparse,fcntl,json
from pathlib import Path
from types import SimpleNamespace
import h5py,numpy as np
from kitchen_safety.planning import DroidKinematics
ROOT=Path(__file__).resolve().parents[3]
p=argparse.ArgumentParser();p.add_argument('episode');a=p.parse_args();episode=Path(a.episode);destination=episode/'self_collision_audit.json'
if destination.exists():raise FileExistsError(destination)
result=json.loads((episode/'result.json').read_text());layout=result['layout']
with h5py.File(episode/'trajectory.h5') as f:
 names=json.loads(f.attrs['joint_names']);states=np.r_[f['obs/q'][:],f['next_obs/q'][-1:]];phases=list(f['phase'].asstr()[:]);phases.append(phases[-1]);dt=float(f.attrs['dt'])
lock=(ROOT/'reports/skill_trials/gpu.lock').open('a')
fcntl.flock(lock,fcntl.LOCK_EX)
from kitchen_safety.curobo_backend import CuroboPlanner
from curobo.types.state import JointState
kin=DroidKinematics(np.array(layout['robot_position'])+[0,0,layout['stand_height']],layout['robot_yaw_deg']);world=SimpleNamespace(objects=[],meshes={},attachment=None,gripper_joint_position=0.);planner=CuroboPlanner(world,kin,dt=dt);solver=planner.motion_gen.ik_solver;rollout=solver.rollout_fn;limits=rollout.kinematics.get_joint_limits().position.detach().cpu().numpy();ids=[names.index(n) for n in solver.joint_names];finger=names.index('finger_joint');assert rollout.robot_self_collision_constraint.enabled
violations=[];exceptions=[];max_excess=0.;max_cost=0.
for i,q in enumerate(states):
 world.gripper_joint_position=float(q[finger]);planner._sync_gripper_and_attachment(1);arm=q[ids];excess=float(np.maximum(limits[0]-arm,arm-limits[1]).clip(min=0).max());state=rollout._get_augmented_state(JointState(position=planner.args.to_device(arm[None])));cost=float(rollout.robot_self_collision_constraint.forward(state.robot_spheres).max().item());max_excess=max(max_excess,excess);max_cost=max(max_cost,cost);entry=dict(index=i,phase=phases[i],self_collision_constraint=cost,joint_limit_excess_rad=excess,q=arm.tolist())
 if excess>0:exceptions.append(entry)
 if not np.isfinite(arm).all() or cost!=0. or excess>1e-5:violations.append(entry)
report=dict(passed=not violations,samples=len(states),violations=violations,exact_bound_exceptions=exceptions,maximum_joint_limit_excess_rad=max_excess,maximum_self_collision_constraint=max_cost,position_tolerance_rad=1e-5,joint_names=solver.joint_names,joint_limits_rad=limits.tolist(),recorded_positions_modified=False,scope='Actual action-rate obs plus final next_obs; recorded finger_joint drives locked mimic joints; direct enabled cuRobo self-collision cost on unmodified recorded arm q must be zero. Joint limits separately use 1e-5 rad floating-point tolerance. Not a physics-rate self-collision proof. Separate external FCL checks are physics-rate.')
destination.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k not in ('violations','exact_bound_exceptions')},indent=2),flush=True)
# Lock remains held until this process exits and destroys the GPU context.
raise SystemExit(0 if report['passed'] else 1)
