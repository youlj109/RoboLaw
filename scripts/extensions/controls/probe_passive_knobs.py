#!/usr/bin/env python3
"""Neutral hold and external-torque mobility checks; never a task demonstration."""
import hashlib
import json
from pathlib import Path

from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext

parser = get_isaaclab_arena_cli_parser()
parser.add_argument('--out', required=True)
parser.add_argument('--original-physics', action='store_true')
parser.add_argument('--neutral-seconds', type=float, default=30.)
args = parser.parse_args()
args.enable_cameras = True
out = Path(args.out)
out.mkdir(parents=True, exist_ok=False)

with SimulationAppContext(args):
    import numpy as np
    import torch
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from kitchen_safety.extensions.controls.environment import build_environment
    from kitchen_safety.dataset import CameraWriter, numpy as arr

    config = json.loads(Path('configs/extensions/controls/avoidance_left_button.json').read_text())
    envdef, assets = build_environment(variant='base', fixed_pose=config['robot_pose_xyzw'],
                                      trial_config=config, repair_toaster_physics=not args.original_physics)
    envdef.task.episode_length_s = 120.
    builder = ArenaEnvBuilder(envdef, ArenaEnvBuilderCfg(num_envs=1, seed=42, device=args.device,
                                                        presets='physx', solve_relations=False))
    cfg, kwargs = builder.compose_manager_cfg()
    cfg.observations.camera_obs = None
    cfg.sim.render_interval = cfg.decimation
    env = builder.make_registered(cfg, kwargs)
    env.reset()
    e = env.unwrapped
    robot, toaster = e.scene['robot'], e.scene['toaster']
    ids = [robot.joint_names.index(f'panda_joint{i}') for i in range(1, 8)]
    q = arr(robot.data.joint_pos)[0, ids]
    command = torch.tensor(np.r_[q, 0][None], dtype=torch.float32, device=e.device)
    knobs = [toaster.joint_names.index(f'side{s}_knob_doneness_joint') for s in ('L', 'R')]
    bodies = [toaster.body_names.index(f'Toaster015_side{s}_knob_doneness') for s in ('L', 'R')]
    steps = round(args.neutral_seconds / e.step_dt)
    pulse_steps, rest_steps = round(2 / e.step_dt), round(3 / e.step_dt)
    limits = {'neutral_min_seconds': 30., 'neutral_knob_excursion_max_deg': 1.,
              'neutral_knob_speed_max_deg_s': 1., 'neutral_linear_excursion_max_m': .0005,
              'neutral_robot_joint_excursion_max_rad': .0001,
              'mobility_min_rotation_deg': 5., 'mobility_max_rotation_deg': 60.,
              'post_pulse_final_second_drift_max_deg': 1.}
    info = {'purpose': 'Passive mechanism diagnostic; external torque phase is not robot task success',
            'dt': e.step_dt, 'physics_dt': e.physics_dt, 'seed': 42,
            'joint_names': toaster.joint_names, 'body_names': toaster.body_names,
            'neutral_seconds': steps * e.step_dt, 'torque_pulse_seconds': pulse_steps * e.step_dt,
            'post_pulse_seconds': rest_steps * e.step_dt,
            'torque_nm_per_knob': [0., .005, 0.],
            'torque_frame': 'body local frame; original rotation axis is +Y',
            'thresholds_declared_before_execution': limits,
            'control': 'Robot initial seven joint targets held, gripper open. No object pose or joint state setters after reset.',
            'physics_provenance': envdef.toaster_physics_provenance,
            'runtime_parameters': {}, 'states': []}
    for name in ('get_dof_stiffnesses', 'get_dof_dampings', 'get_dof_friction_coefficients',
                 'get_dof_armatures', 'get_dof_limits', 'get_masses', 'get_coms', 'get_inertias'):
        try:
            value = getattr(toaster.root_physx_view, name)()
            info['runtime_parameters'][name] = np.asarray(value.numpy() if hasattr(value, 'numpy') else value).tolist()
        except Exception as exc:
            info['runtime_parameters'][name] = {'unavailable': str(exc)}
    writer = CameraWriter(out, e.scene, envdef.embodiment.camera_config.camera_names(), e.step_dt, previews=True)
    for _ in range(32):
        e.sim.render()
    zeros = torch.zeros((1, 2, 3), device=e.device)
    torques = zeros.clone()
    torques[:, :, 1] = .005
    try:
        for i in range(steps + pulse_steps + rest_steps):
            phase = 'neutral' if i < steps else ('external_torque_mobility' if i < steps + pulse_steps else 'passive_damping')
            writer.write(phase)
            info['states'].append({'index': i, 'time_s': i * e.step_dt, 'phase': phase,
                                   'robot_q': arr(robot.data.joint_pos)[0].tolist(),
                                   'toaster_q': arr(toaster.data.joint_pos)[0].tolist(),
                                   'toaster_dq': arr(toaster.data.joint_vel)[0].tolist(),
                                   'predicate': bool(assets['toaster'].is_pressed(e)[0]),
                                   'external_torque_nm': [0., .005 if phase == 'external_torque_mobility' else 0., 0.]})
            toaster.set_external_force_and_torque(zeros, torques if phase == 'external_torque_mobility' else zeros, body_ids=bodies)
            env.step(command)
            if i % 150 == 0:
                print('PASSIVE_KNOB_PROBE', i, phase, flush=True)
    finally:
        writer.close()
        (out / 'probe.json').write_text(json.dumps(info, indent=2) + '\n')
        env.close()
    state = info['states']
    neutral_q = np.array([s['toaster_q'] for s in state[:steps]])
    neutral_dq = np.array([s['toaster_dq'] for s in state[:steps]])
    all_q = np.array([s['toaster_q'] for s in state])
    robot_q = np.array([s['robot_q'] for s in state[:steps]])
    rotation = np.degrees(all_q[-1, knobs] - all_q[steps, knobs])
    last = all_q[-round(1 / e.step_dt):, knobs]
    checks = {
        'neutral_duration': steps * e.step_dt >= limits['neutral_min_seconds'],
        'neutral_knob_excursion': float(np.max(np.abs(np.degrees(neutral_q[:, knobs] - neutral_q[0, knobs])))) < limits['neutral_knob_excursion_max_deg'],
        'neutral_knob_speed': float(np.max(np.abs(np.degrees(neutral_dq[:, knobs])))) < limits['neutral_knob_speed_max_deg_s'],
        'neutral_prismatic_excursion': float(np.max(np.abs(neutral_q[:, [0, 2, 3, 5]] - neutral_q[0, [0, 2, 3, 5]]))) < limits['neutral_linear_excursion_max_m'],
        'neutral_robot_stationary': float(np.max(np.abs(robot_q - robot_q[0]))) < limits['neutral_robot_joint_excursion_max_rad'],
        'neutral_button_unsuccessful': not any(s['predicate'] for s in state[:steps]),
        'knobs_not_locked': bool(np.all(rotation > limits['mobility_min_rotation_deg']) and np.all(rotation < limits['mobility_max_rotation_deg'])),
        'post_pulse_damping': float(np.max(np.degrees(np.ptp(last, axis=0)))) < limits['post_pulse_final_second_drift_max_deg'],
        'zero_angular_stiffness': all(info['runtime_parameters']['get_dof_stiffnesses'][0][k] == 0 for k in knobs),
        'positive_angular_damping': all(info['runtime_parameters']['get_dof_dampings'][0][k] > 0 for k in knobs),
    }
    result = {'accepted': all(checks.values()), 'checks': checks,
              'neutral_seconds': steps * e.step_dt,
              'neutral_knob_max_excursion_deg': np.max(np.abs(np.degrees(neutral_q[:, knobs]-neutral_q[0, knobs])), axis=0).tolist(),
              'neutral_knob_max_speed_deg_s': np.max(np.abs(np.degrees(neutral_dq[:, knobs])), axis=0).tolist(),
              'torque_mobility_delta_deg': rotation.tolist(),
              'post_pulse_last_second_drift_deg': np.degrees(np.ptp(last, axis=0)).tolist(),
              'probe_sha256': hashlib.sha256((out/'probe.json').read_bytes()).hexdigest(),
              'scope': 'Fixed-layout neutral stability and explicit external-force mobility only. This is not robot-button collection or avoidance acceptance.'}
    (out / 'validation.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2), flush=True)
    if not result['accepted']:
        raise SystemExit(2)
