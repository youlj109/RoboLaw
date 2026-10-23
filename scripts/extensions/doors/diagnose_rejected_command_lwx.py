"""Isolate saved rejected-command constraints without executing a robot action."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from kitchen_safety.extensions.doors.task_planner import TaskPlanner
from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from kitchen_safety.planning import transform


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('episode', type=Path)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        raise FileExistsError(a.output)
    read = lambda path: json.loads(path.read_text())
    rejection = read(a.episode / 'rejected_command.json')
    m = read(a.episode / 'metadata.json')
    stage, kin, world, geometry = reconstruct(m, read(Path(m['variant_config'])), m.get('variant_spec'))
    margin = m['controller_parameters'].get('ik_limit_margin_rad', 0.)
    kin.limits[:, 0] += margin
    kin.limits[:, 1] -= margin
    state = rejection['measured_state']
    initial = transform(m['initial_door_pose'][:3], m['initial_door_pose'][3:])
    pose = state['door_body_pose']
    delta = transform(pose[:3], pose[3:]) @ np.linalg.inv(initial)
    for body in geometry['moving_bodies']:
        world.move_body(body, delta)
    if m['variant'] == 'property':
        world.set_obstacle_pose('glass', state['hazard_pose'])
    world.gripper_joint_position = state['q'][m['robot_joint_names'].index('finger_joint')]
    planner = TaskPlanner(world, kin, dt=m['dt'], seed=m['controller_parameters']['seed'], max_joint_speed=.5)
    planner._prepare_path_query(tuple(geometry['handle_colliders']), 0., rejection['phase'])
    enabled = [n for n in planner.obstacle_names if n not in geometry['handle_colliders']]
    qt = planner.args.to_device(np.asarray(rejection['command_q'])[None])

    def valid(names):
        selected = set(names)
        for name in planner.obstacle_names:
            planner.motion_gen.world_coll_checker.enable_obstacle(name, enable=name in selected)
        return bool(planner.motion_gen.ik_solver.check_valid(qt).all())

    report = dict(episode=str(a.episode),
                  rejection_sha256=hashlib.sha256((a.episode / 'rejected_command.json').read_bytes()).hexdigest(),
                  physics_replay=False, robot_actions_executed=0,
                  scope='Offline constraint isolation at the saved measured pose. Obstacle toggles are diagnostic only; '
                        'they never authorize executing a disabled-obstacle path or relaxing production safety checks.',
                  full_world_valid=valid(enabled), world_disabled_valid=valid([]), blocking_obstacles=[])

    def isolate(names):
        if not names or valid(names):
            return
        if len(names) == 1:
            report['blocking_obstacles'].append(names[0])
            return
        middle = len(names) // 2
        isolate(names[:middle])
        isolate(names[middle:])

    try:
        if not report['full_world_valid'] and report['world_disabled_valid']:
            isolate(enabled)
    finally:
        report['restored_full_world_valid'] = valid(enabled)
    a.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
