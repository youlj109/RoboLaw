"""Read-only minimum clearance of each accepted trajectory to its own hazard."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom

from kitchen_safety.extensions.doors.collision import DoorCollisionWorld
from kitchen_safety.planning import DroidKinematics

parser = argparse.ArgumentParser()
parser.add_argument('episodes', nargs='+')
args = parser.parse_args()
root = Path(__file__).resolve().parents[3]
config = root / 'configs/extensions/doors/open_fridge_variants.json'
specs = json.loads(config.read_text())
for directory in args.episodes:
    episode = Path(directory)
    metadata = json.loads((episode / 'metadata.json').read_text())
    if metadata['variant'] == 'base':
        continue
    expected = metadata['implementation_sha256'][str(config.relative_to(root))]
    if hashlib.sha256(config.read_bytes()).hexdigest() != expected:
        raise RuntimeError('Current hazard configuration differs from collection provenance')
    spec = specs[metadata['variant']]
    records = json.loads((episode / 'trajectory.json').read_text())
    stage = Usd.Stage.CreateInMemory()
    prim = UsdGeom.Xform.Define(stage, '/World/envs/env_0/' + spec['name'])
    prim.GetPrim().GetReferences().AddReference(str(root / spec['asset']))
    prim.AddTranslateOp().Set(Gf.Vec3d(*spec['position']))
    prim.AddRotateZOp().Set(spec.get('yaw_deg', 0))
    kin = DroidKinematics(metadata['robot_base'], metadata['robot_yaw_deg'])
    world = DoorCollisionWorld(stage, kin, env_root='/World/envs/env_0')
    best = None
    failures = []
    for i, row in enumerate(records):
        state = row['next_obs']
        angles = dict(zip(metadata['robot_joint_names'], state['q']))
        if state['hazard_pose'] is not None:
            world.set_obstacle_pose(spec['name'], state['hazard_pose'])
        measured = world.clearance(
            np.array([angles[f'panda_joint{j}'] for j in range(1, 8)]),
            gripper_q={key: angles[key] for key in world.GRIPPER_JOINT_NAMES},
            obstacle_filter=[spec['name']])
        if best is None or measured['slack_m'] < best['slack_m']:
            best = {'frame_index': i, 'phase': row['phase'], **measured}
        if not measured['valid']:
            failures.append(i)
    report = {'passed': not failures, 'samples': len(records), 'minimum': best,
              'failed_frames': failures, 'variant_spec': spec,
              'scope': 'Offline FCL surface separation at every measured 15Hz action '
                       'state. The collection audit separately checks robot/world and '
                       'moving-door/hazard geometry at every 120Hz physics update.'}
    (episode / 'hazard_clearance.json').write_text(json.dumps(report, indent=2))
    print(directory, json.dumps(report), flush=True)
    if failures:
        raise SystemExit(1)
