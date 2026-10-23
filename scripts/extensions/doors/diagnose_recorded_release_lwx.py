"""Offline release geometry at recorded door poses; never a dynamics acceptance."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from kitchen_safety.planning import transform


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episode', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--retreats', default='0,.01,.02,.03,.04')
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    read = lambda path: json.loads(path.read_text())
    metadata = read(args.episode / 'metadata.json')
    config = read(Path(metadata['variant_config']))
    stage, kin, world, geometry = reconstruct(metadata, config, metadata.get('variant_spec'))
    margin = metadata['controller_parameters'].get('ik_limit_margin_rad', 0.)
    kin.limits[:, 0] += margin
    kin.limits[:, 1] -= margin
    samples = read(args.episode / 'physics.json')['samples']
    release = [s for s in samples if s['phase'] in ['release_handle', 'released_hold']]
    if not release:
        raise RuntimeError('No measured release poses')
    before = next(s for s in reversed(samples) if s['phase'] not in ['release_handle', 'released_hold'])
    qstart = np.asarray(before['state']['q'][:7])
    start = kin.fk(qstart)
    approach = Rotation.from_euler('z', metadata['robot_yaw_deg'], degrees=True).apply([1., 0., 0.])
    angle = before['state']['door_q'] - metadata.get('door_motion_reference_joint_rad', 0.)
    outward = -Rotation.from_rotvec(np.asarray(geometry['axis']) * angle).apply(approach)
    initial = transform(metadata['initial_door_pose'][:3], metadata['initial_door_pose'][3:])
    first_action = release[0]['action_index']
    world.allow_handle_contact = True
    output = dict(episode=str(args.episode), physics_sha256=hashlib.sha256((args.episode / 'physics.json').read_bytes()).hexdigest(),
                  scope='Offline FCL geometry at measured failed-door poses. Robot poses are IK predictions; changed contact, actual tracking, later settling and dynamic safety remain unverified.',
                  physics_replay=False, results=[])
    for distance in map(float, args.retreats.split(',')):
        details = []
        seed = qstart.copy()
        for sample in release:
            state = sample['state']
            fraction = min(1., (sample['action_index'] - first_action + 1) / 20.)
            target = start.copy()
            target[:3, 3] += outward * distance * fraction
            try:
                q = kin.ik(target, seed, attempts=1) if distance else qstart
                seed = q
            except RuntimeError as error:
                details.append(dict(physics_step=sample['physics_step'], fraction=fraction,
                                    ik_error=str(error), clearance=dict(valid=False, slack_m=-1.e9)))
                break
            pose = state['door_body_pose']
            delta = transform(pose[:3], pose[3:]) @ np.linalg.inv(initial)
            for body in geometry['moving_bodies']:
                world.move_body(body, delta)
            fingers = {name: state['q'][metadata['robot_joint_names'].index(name)] for name in world.GRIPPER_JOINT_NAMES}
            world.gripper_joint_position = fingers['finger_joint']
            detail = dict(world.clearance(q, mode='execution', gripper_q=fingers))
            details.append(dict(physics_step=sample['physics_step'], fraction=fraction, clearance=detail))
        worst = min(details, key=lambda row: row['clearance']['slack_m'])
        result = dict(retreat_m=distance, all_predicted_geometry_valid=all(row['clearance']['valid'] for row in details),
                      samples=len(details), worst=worst)
        output['results'].append(result)
        print(json.dumps(result), flush=True)
        args.output.write_text(json.dumps(output, indent=2) + '\n')


if __name__ == '__main__':
    main()
