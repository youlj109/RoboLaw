"""Compare bounded release directions at recorded poses, without running physics."""
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
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    read = lambda p: json.loads(p.read_text())
    m = read(args.episode / 'metadata.json')
    stage, kin, world, geometry = reconstruct(m, read(Path(m['variant_config'])), m.get('variant_spec'))
    margin = m['controller_parameters'].get('ik_limit_margin_rad', 0.)
    kin.limits[:, 0] += margin
    kin.limits[:, 1] -= margin
    samples = read(args.episode / 'physics.json')['samples']
    release = [s for s in samples if s['phase'] in ['release_handle', 'released_hold']]
    before = next(s for s in reversed(samples) if s['phase'] not in ['release_handle', 'released_hold'])
    qstart = np.asarray(before['state']['q'][:7])
    start = kin.fk(qstart)
    angle = before['state']['door_q'] - m.get('door_motion_reference_joint_rad', 0.)
    outward = -Rotation.from_rotvec(np.asarray(geometry['axis']) * angle).apply(
        Rotation.from_euler('z', m['robot_yaw_deg'], degrees=True).apply([1., 0., 0.]))
    initial = transform(m['initial_door_pose'][:3], m['initial_door_pose'][3:])
    world.allow_handle_contact = True
    grouped = {}
    for sample in release:
        grouped.setdefault(sample['action_index'], []).append(sample)
    result = dict(episode=str(args.episode), physics_sha256=hashlib.sha256(
        (args.episode / 'physics.json').read_bytes()).hexdigest(),
        scope='Offline IK and exact source geometry at recorded failed door poses. '
        'The bounded commands are treated as immediately tracked; new dynamics and safety are unverified.',
        physics_replay=False, results=[])
    for yaw in [-60., -30., 0., 30., 60.]:
        horizontal = Rotation.from_euler('z', yaw, degrees=True).apply(outward)
        for pitch in [-45., 0., 45.]:
            direction = horizontal * np.cos(np.deg2rad(pitch)) + np.array([0., 0., np.sin(np.deg2rad(pitch))])
            for distance in [.02, .03]:
                command = qstart.copy()
                worst = None
                max_increment = 0.
                ik_error = None
                for action_number, group in enumerate(grouped.values()):
                    target = start.copy()
                    fraction = min(1., (action_number + 1) / 20.)
                    target[:3, 3] += direction * distance * fraction
                    try:
                        desired = kin.ik(target, command, attempts=1)
                    except RuntimeError as exc:
                        ik_error = str(exc)
                        break
                    delta = desired - command
                    delta /= max(1., float(np.max(np.abs(delta))) / .025)
                    max_increment = max(max_increment, float(np.max(np.abs(delta))))
                    command += delta
                    for sample in group:
                        state = sample['state']
                        pose = state['door_body_pose']
                        delta_door = transform(pose[:3], pose[3:]) @ np.linalg.inv(initial)
                        for body in geometry['moving_bodies']:
                            world.move_body(body, delta_door)
                        fingers = {n: state['q'][m['robot_joint_names'].index(n)] for n in world.GRIPPER_JOINT_NAMES}
                        world.gripper_joint_position = fingers['finger_joint']
                        clearance = dict(world.clearance(command, mode='execution', gripper_q=fingers))
                        row = dict(physics_step=sample['physics_step'], clearance=clearance)
                        if worst is None or clearance['slack_m'] < worst['clearance']['slack_m']:
                            worst = row
                actual = kin.fk(command)[:3, 3] - start[:3, 3]
                item = dict(yaw_deg=yaw, pitch_deg=pitch, retreat_m=distance,
                            direction_world=direction.tolist(), worst=worst, ik_error=ik_error,
                            max_command_increment_rad=max_increment,
                            achieved_translation_m=actual.tolist(),
                            achieved_outward_m=float(actual @ outward))
                result['results'].append(item)
                args.output.write_text(json.dumps(result, indent=2) + '\n')
                print(json.dumps(item), flush=True)


if __name__ == '__main__':
    main()
