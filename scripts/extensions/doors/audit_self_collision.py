"""Audit measured action states with cuRobo self-collision and joint-limit checks.

The physical-step FCL audit covers external geometry independently. No scene is
stepped here; an empty planning world isolates robot self-collision and limits.
"""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from kitchen_safety.curobo_backend import CuroboPlanner
from kitchen_safety.planning import DroidKinematics

parser = argparse.ArgumentParser()
parser.add_argument('episodes', nargs='+')
args = parser.parse_args()
overall = True
for directory in args.episodes:
    episode = Path(directory)
    metadata = json.loads((episode / 'metadata.json').read_text())
    records = json.loads((episode / 'trajectory.json').read_text())
    kin = DroidKinematics(metadata['robot_base'], metadata['robot_yaw_deg'])
    world = SimpleNamespace(objects=[], meshes={}, attachment=None,
                            gripper_joint_position=0.)
    planner = CuroboPlanner(world, kin, dt=metadata['dt'])
    names = metadata['robot_joint_names']
    arm = [names.index(f'panda_joint{i}') for i in range(1, 8)]
    finger = names.index('finger_joint')
    violations = []
    for index, row in enumerate(records):
        joints = np.asarray(row['next_obs']['q'])
        world.gripper_joint_position = float(joints[finger])
        planner._sync_gripper_and_attachment(1)
        feasible = planner.motion_gen.ik_solver.check_valid(
            planner.args.to_device(joints[arm][None]))
        if not bool(feasible.all()):
            violations.append({'action_index': index, 'phase': row['phase'],
                               'q': joints.tolist()})
    report = {
        'passed': not violations, 'samples': len(records), 'violations': violations,
        'scope': 'Measured action states at 15Hz: cuRobo self-collision and joint '
                 'limits, with measured finger_joint driving locked mimic joints. '
                 'Separate FCL world collision audits run at 120Hz.'}
    (episode / 'self_collision_audit.json').write_text(json.dumps(report, indent=2))
    print(directory, json.dumps(report), flush=True)
    overall = overall and report['passed']
raise SystemExit(0 if overall else 1)
