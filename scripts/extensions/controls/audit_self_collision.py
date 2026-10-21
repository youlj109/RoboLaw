"""Separate actual-state self collision from float-precision joint-limit noise.

No recorded positions are clamped or changed. The numerical position tolerance
is 1e-5 rad (0.000573 degrees); exact-bound exceptions remain in the report.
"""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.curobo_backend import CuroboPlanner
from curobo.types.state import JointState

parser = argparse.ArgumentParser()
parser.add_argument('episodes', nargs='+')
parser.add_argument('--report-name', default='self_collision_detailed_audit.json')
args = parser.parse_args()
overall = True
POSITION_TOLERANCE_RAD = 1e-5
for directory in args.episodes:
    episode = Path(directory)
    destination = episode / args.report_name
    if destination.exists():
        raise FileExistsError('Refusing to overwrite ' + str(destination))
    result = json.loads((episode / 'result.json').read_text())
    records = json.loads((episode / 'records.json').read_text())
    pose = result['initial_robot_pose']
    kin = DroidKinematics(pose[:3], Rotation.from_quat(pose[3:7]).as_euler('xyz', degrees=True)[2])
    world = SimpleNamespace(objects=[], meshes={}, attachment=None, gripper_joint_position=0.)
    planner = CuroboPlanner(world, kin, dt=result['dt'])
    solver = planner.motion_gen.ik_solver
    rollout = solver.rollout_fn
    limits = rollout.kinematics.get_joint_limits().position.detach().cpu().numpy()
    names = result['robot_joint_names']
    arm = [names.index(name) for name in solver.joint_names]
    assert limits.shape == (2, len(arm)), limits.shape
    assert rollout.robot_self_collision_constraint.enabled
    finger = names.index('finger_joint')
    violations, exact_bound_exceptions = [], []
    max_error = 0.0
    for index, row in enumerate(records):
        joints = np.array(row['after']['q'])
        world.gripper_joint_position = float(joints[finger])
        planner._sync_gripper_and_attachment(1)
        actual = joints[arm]
        excess = np.maximum(limits[0] - actual, actual - limits[1]).clip(min=0)
        error = float(excess.max())
        max_error = max(max_error, error)
        state = rollout._get_augmented_state(JointState(position=planner.args.to_device(actual[None])))
        self_cost = float(rollout.robot_self_collision_constraint.forward(state.robot_spheres).max().item())
        entry = dict(action_index=index, phase=row['phase'], joint_limit_excess_rad=error,
                     self_collision_constraint=self_cost, q=actual.tolist())
        if error > 0:
            exact_bound_exceptions.append(entry)
        if not np.isfinite(actual).all() or self_cost != 0.0 or error > POSITION_TOLERANCE_RAD:
            violations.append(entry)
    report = dict(passed=not violations, samples=len(records), violations=violations,
        joint_names=solver.joint_names, joint_limits_rad=limits.tolist(),
        position_tolerance_rad=POSITION_TOLERANCE_RAD, maximum_joint_limit_excess_rad=max_error,
        exact_bound_exceptions=exact_bound_exceptions, recorded_positions_modified=False,
        scope='Actual action-rate states, measured gripper, direct cuRobo self-collision cost on unmodified q. Joint positions separately use 1e-5 rad numerical tolerance. External FCL checks remain physics-rate; no collision-distance tolerance is changed.')
    destination.write_text(json.dumps(report, indent=2))
    print(directory, json.dumps({k: v for k, v in report.items() if k not in ('violations', 'exact_bound_exceptions')}), flush=True)
    overall = overall and report['passed']
raise SystemExit(0 if overall else 2)
