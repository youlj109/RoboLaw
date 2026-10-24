"""Compare hypothetical checked release targets over actual recorded door motion."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.doors.release_pose_lwx import release_pose
from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from kitchen_safety.planning import transform


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--episode', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        p.error('Evidence exists')
    metadata = json.loads((a.episode / 'metadata.json').read_text())
    rows = json.loads((a.episode / 'trajectory.json').read_text())
    config_path = Path(metadata['variant_config'])
    config = json.loads(config_path.read_text())
    stage, kin, world, geometry = reconstruct(metadata, config, metadata['variant_spec'])
    world.allow_handle_contact = True
    cp = metadata['controller_parameters']
    margin = cp.get('ik_limit_margin_rad', 0.)
    kin.limits[:, 0] += margin
    kin.limits[:, 1] -= margin
    before = [row for row in rows if row['phase'] in ['success_hold', 'recover_open']][-1]['next_obs']
    release_rows = [row for row in rows if row['phase'] == 'release_handle']
    if not release_rows:
        raise ValueError('No actual release states')
    q0 = np.asarray(before['q'][:7])
    start = kin.fk(q0)
    approach = Rotation.from_euler('z', metadata['robot_yaw_deg'], degrees=True).apply([1., 0., 0.])
    normal = Rotation.from_rotvec(np.asarray(geometry['axis']) *
                                  (before['door_q'] - metadata['door_motion_reference_joint_rad'])).apply(approach)
    initial = transform(metadata['initial_door_pose'][:3], metadata['initial_door_pose'][3:])
    report = dict(episode=str(a.episode), trajectory_sha256=sha(a.episode / 'trajectory.json'),
                  metadata_sha256=sha(a.episode / 'metadata.json'), config=str(config_path), config_sha256=sha(config_path),
                  implementation_sha256=sha(__file__),
                  helper_sha256=sha('kitchen_safety/extensions/doors/release_pose_lwx.py'),
                  scope='Hypothetical CPU IK/world checks over observed door and finger states. Perfect arm tracking, no GPU self collision or dynamics. Not an execution or acceptance claim.',
                  physics_replay=False, unsafe_path_executed=False, modes=[])
    for follow, follow_rotation in [(False, False), (True, False), (True, True)]:
        q = q0.copy()
        mode = dict(follow_door=follow, follow_rotation=follow_rotation, samples=[])
        for index, row in enumerate(release_rows):
            obs = row['obs']
            pose = row['next_obs']['door_body_pose']
            delta = transform(pose[:3], pose[3:]) @ np.linalg.inv(initial)
            for body in geometry['moving_bodies']:
                world.move_body(body, delta)
            target = release_pose(start, before['door_q'], obs['door_q'], geometry['hinge'], geometry['axis'],
                                  normal, cp['release_retreat_m'], cp['release_tilt_deg'], (index + 1) / 20., follow, follow_rotation)
            sample = dict(index=index, measured_door_q=obs['door_q'])
            try:
                command = kin.ik(target, q, attempts=1)
                dq = command - q
                command = q + dq / max(1., float(np.max(np.abs(dq))) / .025)
                fingers = {name: row['next_obs']['q'][metadata['robot_joint_names'].index(name)]
                           for name in world.GRIPPER_JOINT_NAMES}
                sample.update(arm_q=command.tolist(), conservative_world=world.clearance(command, mode='execution', gripper_q=fingers))
                q = command
            except RuntimeError as exc:
                sample['error'] = str(exc)
            mode['samples'].append(sample)
        mode['ik_failures'] = sum('error' in sample for sample in mode['samples'])
        mode['invalid_world_samples'] = sum(not sample.get('conservative_world', {}).get('valid', False) for sample in mode['samples'])
        mode['minimum_slack_m'] = min((sample['conservative_world']['slack_m'] for sample in mode['samples'] if 'conservative_world' in sample), default=None)
        report['modes'].append(mode)
    result = json.loads((a.episode / 'result.json').read_text())
    failure = result.get('failure') or ''
    if 'EXECUTION_COLLISION ' in failure:
        reference = json.loads(failure.split('EXECUTION_COLLISION ', 1)[1])
        pose = reference['door_body_pose']
        delta = transform(pose[:3], pose[3:]) @ np.linalg.inv(initial)
        for body in geometry['moving_bodies']:
            world.move_body(body, delta)
        fingers = {name: reference['q'][metadata['robot_joint_names'].index(name)] for name in world.GRIPPER_JOINT_NAMES}
        check = world.clearance(np.asarray(reference['q'][:7]), mode='execution', gripper_q=fingers)
        report['actual_failed_transition_reconstruction'] = dict(recorded=reference['failure'], reconstructed=check,
                                                                 distance_error_m=abs(check['distance_m'] - reference['failure']['distance_m']))
    report['complete'] = True
    a.output.write_text(json.dumps(report, indent=2) + '\n')
    print('RELEASE_DIAGNOSIS', [{k: mode[k] for k in ['follow_door', 'follow_rotation', 'ik_failures', 'invalid_world_samples', 'minimum_slack_m']} for mode in report['modes']], flush=True)


if __name__ == '__main__':
    main()
