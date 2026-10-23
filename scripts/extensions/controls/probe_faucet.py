"""Background faucet mobility diagnostic, never a robot task demonstration."""
import json
from pathlib import Path
from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext

parser = get_isaaclab_arena_cli_parser()
parser.add_argument('--out', required=True)
args = parser.parse_args()
args.enable_cameras = True
out = Path(args.out)
out.mkdir(parents=True, exist_ok=False)

with SimulationAppContext(args):
    import numpy as np
    import torch
    import warp as wp
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from isaaclab_physx.physics import PhysxManager
    from kitchen_safety.extensions.controls.environment import build_environment
    from kitchen_safety.extensions.controls.mechanism_monitor import MechanismMonitor
    from kitchen_safety.grasp_contacts import _numpy
    from kitchen_safety.dataset import CameraWriter

    config = json.loads(Path('configs/extensions/controls/repaired_left_button.json').read_text())
    definition, assets = build_environment(fixed_pose=config['robot_pose_xyzw'], trial_config=config)
    definition.task.episode_length_s = 120.
    builder = ArenaEnvBuilder(definition, ArenaEnvBuilderCfg(num_envs=1, seed=42,
        device=args.device, presets='physx', solve_relations=False))
    cfg, kwargs = builder.compose_manager_cfg()
    cfg.observations.camera_obs = None
    cfg.sim.render_interval = cfg.decimation
    env = builder.make_registered(cfg, kwargs)
    env.reset()
    e = env.unwrapped
    robot = e.scene['robot']
    arm = [robot.joint_names.index(f'panda_joint{i}') for i in range(1, 8)]
    command = torch.tensor(np.r_[_numpy(robot.data.joint_pos)[0, arm], 0][None],
                           dtype=torch.float32, device=e.device)
    simview = PhysxManager.get_physics_sim_view()
    faucet = simview.create_articulation_view('/World/envs/env_0/kitchen/sink_left_group')
    body = simview.create_rigid_body_view('/World/envs/env_0/kitchen/sink_left_group/Sink009_handle')
    assert faucet.count == body.count == 1
    names = list(faucet.shared_metatype.dof_names)
    indices = [names.index(name) for name in ('handle_temp_joint', 'handle_joint')]
    monitor = MechanismMonitor(e.sim.stage)
    torque = wp.array(np.array([[.01, .01, 0]], dtype='f4'), device=e.device)
    zero = wp.zeros((1, 3), dtype=wp.float32, device=e.device)
    selected = wp.array([0], dtype=wp.uint32, device=e.device)
    phase = 'neutral'
    rows = []
    scene_update = e.scene.update
    def checked_update(dt):
        scene_update(dt)
        rows.append({'phase': phase, 'q': _numpy(faucet.get_dof_positions())[0].tolist(),
                     'dq': _numpy(faucet.get_dof_velocities())[0].tolist(),
                     'next_substep_local_torque_nm': [.01, .01, 0] if phase == 'torque' else [0, 0, 0]})
        monitor.sample(phase, len(rows))
        body.apply_forces_and_torques_at_position(None, torque if phase == 'torque' else zero,
                                                 None, selected, False)
    e.scene.update = checked_update
    parameters = {name: _numpy(getattr(faucet, name)()).tolist() for name in
                  ('get_dof_stiffnesses', 'get_dof_dampings', 'get_masses', 'get_coms', 'get_inertias')}
    writer = CameraWriter(out, e.scene, definition.embodiment.camera_config.camera_names(), e.step_dt, previews=True)
    for _ in range(32):
        e.sim.render()
    try:
        for label, seconds in [('neutral', 3.), ('torque', 2.), ('rest', 3.)]:
            phase = label
            for _ in range(round(seconds / e.step_dt)):
                writer.write(phase)
                env.step(command)
    finally:
        writer.close()
        env.close()
    neutral = np.array([r['q'] for r in rows if r['phase'] == 'neutral'])[:, indices]
    rest = np.array([r['q'] for r in rows if r['phase'] == 'rest'])[:, indices]
    rotation = np.degrees(rest[-1] - neutral[0])
    checks = {
        'neutral_stationary': float(np.max(np.abs(np.degrees(neutral-neutral[0])))) < 1.,
        'both_dofs_movable': bool(np.all(np.abs(rotation)>2.) and np.all(np.abs(rotation)<60.)),
        'post_pulse_damped': float(np.max(np.degrees(np.ptp(rest[-120:], axis=0)))) < 1.,
        'zero_stiffness': all(parameters['get_dof_stiffnesses'][0][i] == 0 for i in indices),
        'positive_damping': all(parameters['get_dof_dampings'][0][i] > 0 for i in indices),
    }
    report = {'accepted': all(checks.values()), 'checks': checks, 'joint_names': names,
              'rotation_deg': rotation.tolist(), 'physics_dt': cfg.sim.dt, 'dt': e.step_dt,
              'runtime_parameters': parameters, 'states': rows,
              'faucet_physics_provenance': definition.faucet_physics_provenance,
              'scope': 'Explicit external body torque of 0.01 Nm per local X/Y axis; '
              'zero stiffness. Diagnostic only, not robot contact or task success. '
              'Torque is applied for the next physics substep in the scene-update callback.'}
    (out/'validation.json').write_text(json.dumps(report, indent=2)+'\n')
    (out/'mechanism_audit.json').write_text(json.dumps(monitor.report(('handle_joint',)), indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('states','runtime_parameters')}, indent=2))
    if not report['accepted']:
        raise SystemExit(2)
