"""Read-only endpoint diagnosis of failed one-shot queries; no new planning."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom
from scipy.spatial.transform import Rotation

from kitchen_safety.planning import DroidKinematics
from kitchen_safety.curobo_backend import CuroboPlanner
from kitchen_safety.extensions.controls.collision import ControlsCollisionWorld


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        raise FileExistsError(a.output)
    root = Path(__file__).resolve().parents[3]
    report = dict(scope='Only the two saved endpoints in cuRobo proxy constraints; '
                       'no MotionGen call, new IK candidate, path audit, physics or layout edits.',
                  motiongen_calls=0, layout_modified=False, safety_validated=False, scenes=[])
    for result_file in sorted(a.run.glob('*/result.json')):
        result = json.loads(result_file.read_text())
        if result.get('status') != 'failed':
            continue
        config_file = root/result['config']
        before_hash = hashlib.sha256(config_file.read_bytes()).hexdigest()
        if before_hash != result['config_sha256']:
            raise ValueError('Config changed since query: '+str(config_file))
        config = json.loads(config_file.read_text())
        variant = 'property' if result_file.parent.name.endswith('property') else 'robot'
        spec = config['variants'][variant]
        asset = root/spec['asset']
        asset_hash = hashlib.sha256(asset.read_bytes()).hexdigest()
        pose = config['robot_pose_xyzw']
        kin = DroidKinematics(pose[:3], Rotation.from_quat(pose[3:]).as_euler('xyz', degrees=True)[2])
        stage = Usd.Stage.CreateInMemory()
        stage.GetRootLayer().subLayerPaths.append(str((root/result['stage']).resolve()))
        key = {'property': 'glass', 'robot': 'barrier'}[variant]
        prim = UsdGeom.Xform.Define(stage, '/World/envs/env_0/'+key)
        prim.GetPrim().GetReferences().AddReference(str(asset))
        prim.AddTranslateOp().Set(Gf.Vec3d(*spec['position']))
        prim.AddRotateZOp().Set(spec['yaw_deg'])
        world = ControlsCollisionWorld(stage, kin, env_root='/World/envs/env_0')
        world.gripper_joint_position = result['gripper_joint_position']
        planner = CuroboPlanner(world, kin, dt=1/15, seed=42, max_joint_speed=.4)
        planner._prepare_path_query((), 0 if result['phase']=='pregrasp' else 1, result['phase'])
        endpoints = planner.args.to_device(np.asarray([result['start_q'], result['goal_q']]))
        graph_valid = planner.motion_gen.graph_planner.mask_samples(endpoints).detach().cpu().numpy().reshape(-1).tolist()
        ik_valid = planner.motion_gen.ik_solver.check_valid(endpoints).detach().cpu().numpy().reshape(-1).tolist()
        entry = dict(scene=result_file.parent.name, endpoint_order=['saved_start', 'saved_fixed_branch_goal'],
                     graph_endpoint_valid=graph_valid, ik_endpoint_valid=ik_valid,
                     config=result['config'], config_sha256=before_hash, asset=str(asset.relative_to(root)),
                     asset_sha256=asset_hash, source_result=str(result_file))
        if before_hash != hashlib.sha256(config_file.read_bytes()).hexdigest() or asset_hash != hashlib.sha256(asset.read_bytes()).hexdigest():
            raise RuntimeError('Unexpected concurrent source edit')
        report['scenes'].append(entry)
        with a.output.open('w') as stream:
            json.dump(report, stream, indent=2)
            stream.write('\n')
        print('ENDPOINT_DIAGNOSIS', json.dumps(entry), flush=True)
        del planner


if __name__ == '__main__':
    main()
