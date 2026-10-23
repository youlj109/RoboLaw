"""Known external torque demonstrates that the mounted refrigerator doors move.

This diagnostic does not claim robot operation or task success. No body/joint
state is assigned after reset; a native force is applied before each physics step.
"""
import json
from pathlib import Path
from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext

parser = get_isaaclab_arena_cli_parser()
parser.add_argument('--layout', required=True)
parser.add_argument('--output', required=True)
args = parser.parse_args()
args.enable_cameras = True
out = Path(args.output)
out.mkdir(parents=True, exist_ok=False)

with SimulationAppContext(args):
    import numpy as np
    import torch
    import warp as wp
    from scipy.spatial.transform import Rotation
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from isaaclab.sensors import ContactSensorCfg
    from kitchen_safety.extensions.replicator.environment import KitchenSafetyCfg, KitchenSafetyEnvironment
    from kitchen_safety.extensions.replicator.overview import configure_overview
    from kitchen_safety.dataset import CameraWriter, numpy as arr, decode_video

    layout = json.loads(Path(args.layout).read_text())
    envdef = KitchenSafetyEnvironment().build(KitchenSafetyCfg(
        variant='human', enable_cameras=True, collection_seed=42,
        obstacle_id='human_default_mpfb', fixed_layout=layout, episode_length_s=120))
    builder = ArenaEnvBuilder(envdef, ArenaEnvBuilderCfg(num_envs=1, seed=42,
        device=args.device, presets='physx', solve_relations=False))
    cfg, kwargs = builder.compose_manager_cfg()
    cfg.observations.camera_obs = None
    cfg.sim.render_interval = cfg.decimation
    cfg.scene.robot_contacts = ContactSensorCfg(prim_path='{ENV_REGEX_NS}/Robot/.*', update_period=0, history_length=1)
    overview = configure_overview(cfg.scene, eye=(1.0, -2.3, 1.5), target=(2.4, -1.4, .85))
    env = builder.make_registered(cfg, kwargs)
    env.reset()
    e = env.unwrapped
    robot = e.scene['robot']
    ids = [robot.joint_names.index(f'panda_joint{i}') for i in range(1, 8)]
    q = arr(robot.data.joint_pos)[0, ids]
    command = torch.tensor(np.r_[q, 0][None], dtype=torch.float32, device=e.device)
    view = e.scene['robot_contacts']._physics_sim_view.create_rigid_body_view('/World/envs/env_0/kitchen/Fridge/**')
    paths = list(view.prim_paths)
    left = next(i for i, p in enumerate(paths) if p.endswith('door_left_01_obj_00'))
    right = next(i for i, p in enumerate(paths) if p.endswith('door_right_01_obj_00'))
    housing = next(i for i, p in enumerate(paths) if p.endswith('body_01_obj_00'))
    def array(value):
        value = value.torch if hasattr(value, 'torch') else value
        return value.detach().cpu().numpy() if hasattr(value, 'detach') else np.asarray(value.numpy() if hasattr(value, 'numpy') else value)
    zero = wp.zeros((len(paths), 3), dtype=wp.float32, device=e.device)
    indices = wp.array(np.arange(len(paths)), dtype=wp.int32, device=e.device)
    torque = np.zeros((len(paths), 3), dtype=np.float32)
    torque[left, 2], torque[right, 2] = .3, -.3
    torques = wp.array(torque, dtype=wp.float32, device=e.device)
    current_torque = zero
    writes = []
    original_write = e.scene.write_data_to_sim
    def write_with_diagnostic_wrench():
        original_write()
        view.apply_forces_and_torques_at_position(force_data=zero, torque_data=current_torque,
                                                position_data=None, indices=indices, is_global=True)
        writes.append('pulse' if current_torque is torques else 'zero')
    e.scene.write_data_to_sim = write_with_diagnostic_wrench
    writer = CameraWriter(out, e.scene, envdef.embodiment.camera_config.camera_names(), e.step_dt, previews=True)
    (out / 'overview').mkdir()
    extra = CameraWriter(out / 'overview', e.scene, ['overview_camera'], e.step_dt, previews=True)
    for _ in range(32):
        e.sim.render()
    rows = []
    try:
        for i in range(90):
            phase = 'neutral' if i < 30 else 'known_external_torque' if i < 60 else 'coasting_after_torque'
            current_torque = torques if phase == 'known_external_torque' else zero
            writer.write(phase)
            extra.write(phase)
            rows.append({'i': i, 'time_s': i * e.step_dt, 'phase': phase,
                         'poses': array(view.get_transforms()).tolist(),
                         'velocities': array(view.get_velocities()).tolist(),
                         'robot_q': arr(robot.data.joint_pos)[0].tolist()})
            _, _, done, truncated, _ = env.step(command)
            if bool(arr(done)[0]) or bool(arr(truncated)[0]):
                raise RuntimeError('Unexpected task termination in diagnostic')
    finally:
        writer.close()
        extra.close()
        (out/'recorded_states.json').write_text(json.dumps({'paths': paths, 'rows': rows,
            'background_physics_provenance': envdef.background_physics_provenance}, indent=2) + '\n')
        env.close()
    poses = np.array([r['poses'] for r in rows])
    changes = {}
    for name, index in [('left', left), ('right', right), ('housing', housing)]:
        angle = np.degrees((Rotation.from_quat(poses[:, index, 3:]) * Rotation.from_quat(poses[0, index, 3:]).inv()).magnitude())
        changes[name] = {'initial_hold_max_rotation_deg': float(angle[:31].max()),
                         'rotation_at_pulse_end_deg': float(angle[60]),
                         'maximum_rotation_deg': float(angle.max()),
                         'maximum_translation_m': float(np.linalg.norm(poses[:, index, :3] - poses[0, index, :3], axis=1).max())}
    checks = {'doors_neutral_before_torque': all(changes[n]['initial_hold_max_rotation_deg'] < 1 for n in ['left', 'right']),
              'doors_both_movable': all(5 < changes[n]['rotation_at_pulse_end_deg'] < 60 for n in ['left', 'right']),
              'housing_remains_installed': changes['housing']['maximum_rotation_deg'] < 1 and changes['housing']['maximum_translation_m'] < .002,
              'force_applied_each_physics_substep': len(writes) == len(rows) * cfg.decimation and writes.count('pulse') == 30 * cfg.decimation}
    info = {'passed': all(checks.values()), 'checks': checks, 'background_physics_provenance': envdef.background_physics_provenance,
            'source_joint_limits_deg': {'left': [0., 110.], 'right': [-110., 0.]},
            'torques_nm_world': {'left': [0, 0, .3], 'right': [0, 0, -.3]},
            'pulse_seconds': 2., 'neutral_seconds': 2., 'post_pulse_seconds': 2.,
            'dt': e.step_dt, 'physics_dt': e.physics_dt, 'paths': paths, 'rows': rows,
            'changes': changes, 'force_applications': len(writes), 'overview': overview,
            'scope': 'External torque is diagnostic, not robot task completion. The original un-driven door may coast after a deliberate pulse; this is separate from the 40s no-torque neutral validation.'}
    info['decoded_frames'] = {name: decode_video(out / (name + '.mp4'), 90, expected_dt=e.step_dt)
                              for name in [*envdef.embodiment.camera_config.camera_names(), 'all_cameras']}
    info['decoded_frames']['overview_camera'] = decode_video(out/'overview/overview_camera.mp4', 90, expected_dt=e.step_dt)
    (out/'mobility_probe.json').write_text(json.dumps(info, indent=2) + '\n')
    print(json.dumps({k: v for k, v in info.items() if k != 'rows'}, indent=2), flush=True)
    if not info['passed']:
        raise SystemExit(2)
