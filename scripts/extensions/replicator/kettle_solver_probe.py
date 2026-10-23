"""Predeclared local solver convergence diagnostic; never changes task defaults.

Run all 16/1, 32/2 and 64/4 cases. A failed world-space stability gate is retained
and gives nonzero exit status; parent-relative measurements are diagnostic only.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

PAIRS = ((16, 1), (32, 2), (64, 4))
KETTLE = '/World/KettleA01/KettleA01_OnlineVisual/Geometry/'
BODY_NAMES = ('sm_kitchenware_kettle_a01_body_obj_01', 'sm_kitchenware_kettle_a01_button_obj_00',
              'sm_kitchenware_kettle_a01_lid_obj_01', 'sm_kitchenware_kettle_a01_switch_obj_00')
FIELDS = ('physxRigidBody:solverPositionIterationCount', 'physxRigidBody:solverVelocityIterationCount')
ROOT = Path(__file__).resolve().parents[3]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_overlay(layer, source, position, velocity):
    """Only four API-schema additions and eight solver attributes are allowed."""
    from pxr import Sdf
    if (position, velocity) not in PAIRS or list(layer.subLayerPaths) != [str(Path(source).resolve())]:
        raise ValueError('Unexpected solver pair or source sublayer')
    paths = {KETTLE + name for name in BODY_NAMES}
    found, api_paths = {}, set()
    def visit(path):
        spec = layer.GetObjectAtPath(path)
        if isinstance(spec, Sdf.AttributeSpec):
            if str(spec.owner.path) not in paths or spec.name not in FIELDS or spec.typeName != Sdf.ValueTypeNames.Int:
                raise ValueError('Disallowed authored property: ' + str(path))
            found[str(path)] = int(spec.default)
        elif isinstance(spec, Sdf.RelationshipSpec):
            raise ValueError('Diagnostic may not author joints/relationships')
        elif isinstance(spec, Sdf.PrimSpec):
            if path == Sdf.Path.absoluteRootPath:
                if set(spec.ListInfoKeys()) - {'subLayers', 'subLayerOffsets', 'defaultPrim'} or layer.defaultPrim != 'World':
                    raise ValueError('Unexpected diagnostic layer metadata')
                return
            if set(spec.ListInfoKeys()) - {'specifier', 'apiSchemas'}:
                raise ValueError('Disallowed prim metadata: ' + str(path))
            if spec.HasInfo('apiSchemas'):
                if str(path) not in paths or spec.GetInfo('apiSchemas').GetAddedOrExplicitItems() != ['PhysxRigidBodyAPI']:
                    raise ValueError('Unexpected API schema edit: ' + str(path))
                api_paths.add(str(path))
    layer.Traverse(Sdf.Path.absoluteRootPath, visit)
    expected = {path+'.'+field: value for path in paths for field, value in zip(FIELDS, (position, velocity))}
    if found != expected or api_paths != paths:
        raise ValueError('Missing or extra solver authoring')
    return dict(passed=True, authored_solver_attributes=found, api_schema_added_paths=sorted(api_paths),
                allowed_changes='Only PhysxRigidBodyAPI and two solver iteration attributes on exactly four Kettle rigid bodies')


def create_overlay(source, output, position, velocity):
    from pxr import Sdf, Usd, UsdPhysics
    source, output = Path(source).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError(output)
    if (position, velocity) not in PAIRS:
        raise ValueError('Only predeclared solver pairs 16/1, 32/2, 64/4 are permitted')
    original = Usd.Stage.Open(str(source))
    actual = {str(p.GetPath()) for p in original.Traverse()
              if str(p.GetPath()).startswith('/World/KettleA01/') and p.HasAPI(UsdPhysics.RigidBodyAPI)}
    expected = {KETTLE + name for name in BODY_NAMES}
    if actual != expected:
        raise ValueError('Source Kettle rigid-body inventory differs')
    stage = Usd.Stage.CreateNew(str(output))
    stage.GetRootLayer().subLayerPaths = [str(source)]
    stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
    before = {}
    for path in sorted(expected):
        prim = stage.GetPrimAtPath(path)
        before[path] = {field: dict(authored=original.GetPrimAtPath(path).GetAttribute(field).HasAuthoredValueOpinion(),
                                   value=original.GetPrimAtPath(path).GetAttribute(field).Get()) for field in FIELDS}
        prim.AddAppliedSchema('PhysxRigidBodyAPI')
        for field, value in zip(FIELDS, (position, velocity)):
            prim.CreateAttribute(field, Sdf.ValueTypeNames.Int, custom=False).Set(value)
    layer = stage.GetRootLayer()
    validation = validate_overlay(layer, source, position, velocity)
    layer.Save()
    return dict(source=str(source), source_sha256=sha(source), overlay=str(output), overlay_sha256=sha(output),
                source_solver_attributes=before, authoring_validation=validation, authored_layer_text=layer.ExportToString())


def motion(poses):
    poses = np.asarray(poses)
    rotation = Rotation.from_quat(poses[:, 3:7])
    translations = np.linalg.norm(poses[:, :3]-poses[0, :3], axis=1)
    angles = np.rad2deg((rotation * rotation[0].inv()).magnitude())
    return dict(max_translation_m=float(translations.max()), max_rotation_deg=float(angles.max()),
                translation_peak_sample=int(translations.argmax()), rotation_peak_sample=int(angles.argmax()))


def analyze(data, metadata):
    steps, paths, dt = data['physics_step'], metadata['background_paths'], metadata['physics_dt_s']
    n = len(steps)
    if n < 2 or dt <= 0 or not np.isfinite(dt):
        raise ValueError('Missing actual post-reset physics records')
    for key, shape in [('q', (n, len(metadata['joint_names']))), ('dq', (n, len(metadata['joint_names']))),
                       ('background', (n, len(paths), 7)), ('background_velocity', (n, len(paths), 6)),
                       ('mustard', (n, 13)), ('bowl', (n, 13))]:
        if data[key].shape != shape or not np.isfinite(data[key]).all():
            raise ValueError('Missing/nonfinite native evidence: ' + key)
    elapsed = (steps-steps[0])*dt
    after_five = np.flatnonzero(elapsed >= 5.-1e-8)
    after_two = np.flatnonzero(elapsed >= 2.-1e-8)
    if not len(after_five) or not len(after_two):
        raise ValueError('Declared settling intervals were not observed')
    linked = set(metadata['background_topology']['joint_connected_native_paths'])
    world = []
    for i, path in enumerate(paths):
        start = 0 if path in linked else int(after_five[0])
        row = motion(data['background'][start:, i])
        world.append(dict(path=path, reference_seconds=float(elapsed[start]), **row,
            passed=row['max_translation_m'] <= .002 and row['max_rotation_deg'] <= 1.))
    kettle_paths = ['/World/envs/env_0/kitchen'+(KETTLE+name)[len('/World'):] for name in BODY_NAMES]
    if not set(kettle_paths) <= set(paths):
        raise ValueError('Kettle four native rigid bodies are not all observed')
    parent = data['background'][:, paths.index(kettle_paths[0])]
    parent_rotation = Rotation.from_quat(parent[:, 3:7])
    relative = []
    for path in kettle_paths[1:]:
        child = data['background'][:, paths.index(path)]
        xyz = parent_rotation.inv().apply(child[:, :3]-parent[:, :3])
        quat = (parent_rotation.inv()*Rotation.from_quat(child[:, 3:7])).as_quat()
        relative.append(dict(path=path, parent=kettle_paths[0], **motion(np.column_stack([xyz, quat]))))
    task = {name: motion(data[name][after_two[0]:, :7]) for name in ['mustard', 'bowl']}
    checks = dict(full_post_reset_physics_coverage=int(steps[0]) == metadata['reset_physics_step'] and
        np.all(np.diff(steps) == 1) and n == metadata['actions']*metadata['decimation']+1 and
        int(steps[-1]) == metadata['final_physics_step'],
        complete_predeclared_duration=elapsed[-1] >= metadata['requested_duration_seconds']-1e-8,
        native_background_joint_coverage=bool(linked) and linked <= set(paths) and
            not metadata['background_topology']['unobserved_joint_connected_bodies'],
        original_world_background_stability=all(row['passed'] for row in world),
        robot_neutral_hold=float(np.abs(data['q']-data['q'][0]).max()) <= .001,
        task_objects_settled=all(row['max_translation_m'] <= .001 and row['max_rotation_deg'] <= 1. for row in task.values()),
        no_reset_or_callback_failure=not metadata['callback_errors'] and not metadata.get('runtime_error'))
    checks = {key: bool(value) for key, value in checks.items()}
    return dict(passed=all(checks.values()), checks=checks, samples=n, observed_seconds=float(elapsed[-1]),
        world_background_results=world, kettle_parent_relative_diagnostic=relative,
        task_objects=task, robot_max_joint_change_rad=float(np.abs(data['q']-data['q'][0]).max()),
        thresholds=dict(background_translation_m=.002, background_rotation_deg=1.,
                        free_background_settling_seconds=5., robot_joint_rad=.001, task_settling_seconds=2.,
                        task_translation_m=.001, task_rotation_deg=1.),
        parent_relative_scope='Diagnostic only; never substitutes for the predeclared world-space pass/fail gate')


def run(args):
    import torch
    from pxr import PhysxSchema, Usd
    from isaaclab.sensors import ContactSensorCfg
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from kitchen_safety.extensions.replicator.environment import KitchenSafetyCfg, KitchenSafetyEnvironment
    from kitchen_safety.extensions.replicator.startup_recorder import array, background_topology
    code_paths = [Path(__file__), ROOT/'kitchen_safety/extensions/replicator/environment.py',
                  ROOT/'kitchen_safety/extensions/replicator/background_physics.py',
                  ROOT/'kitchen_safety/extensions/replicator/startup_recorder.py']
    snapshots = {str(path.resolve()): path.read_text() for path in code_paths}
    hashes = {key: hashlib.sha256(value.encode()).hexdigest() for key, value in snapshots.items()}
    layout = json.loads(args.layout.read_text())
    if layout != json.loads((ROOT/'configs/extensions/replicator/g_shape_mustard.json').read_text()) or layout.get('obstacle'):
        raise ValueError('Require the unchanged G-mustard base source layout without an obstacle')
    layout_sha = sha(args.layout)
    envdef = KitchenSafetyEnvironment().build(KitchenSafetyCfg(variant='base', enable_cameras=True,
        collection_seed=42, fixed_layout=layout, episode_length_s=max(120., args.duration_seconds+30.)))
    kitchen = envdef.scene.assets['kitchen']
    overlay = create_overlay(kitchen.usd_path, args.output/'kettle_solver.usda', args.position_iterations, args.velocity_iterations)
    kitchen.usd_path = overlay['overlay']
    kitchen.object_cfg.spawn.usd_path = overlay['overlay']
    builder = ArenaEnvBuilder(envdef, ArenaEnvBuilderCfg(num_envs=1, seed=42, device=args.device, presets='physx', solve_relations=False))
    cfg, kwargs = builder.compose_manager_cfg()
    cfg.observations.camera_obs = None
    cfg.sim.render_interval = cfg.decimation
    cfg.scene.robot_contacts = ContactSensorCfg(prim_path='{ENV_REGEX_NS}/Robot/.*', update_period=0, history_length=1)
    env = builder.make_registered(cfg, kwargs)
    info = dict(schema_version=1, passed=False, task_id=layout['task_id'], seed=42,
        requested_duration_seconds=args.duration_seconds, requested_solver_pair=[args.position_iterations, args.velocity_iterations],
        predeclared_comparison_pairs=[list(pair) for pair in PAIRS], background_physics=envdef.background_physics_provenance,
        diagnostic_overlay=overlay, layout=str(args.layout.resolve()), layout_sha256=layout_sha,
        code_sha256=hashes, code_snapshots=snapshots, callback_errors=[],
        scope='Solver convergence diagnostic only, not a task collection or new accepted physical profile. '
              'All three predeclared pairs must be retained, including failures. No object/joint state setters after reset.')
    rows = {key: [] for key in ['physics_step', 'q', 'dq', 'background', 'background_velocity', 'mustard', 'bowl']}
    original_update = None
    try:
        env.reset()
        e, robot = env.unwrapped, env.unwrapped.scene['robot']
        reset_physics_step = int(e.sim.get_physics_step_count())
        view = e.scene['robot_contacts']._physics_sim_view.create_rigid_body_view('/World/envs/env_0/kitchen/**')
        def live_solver():
            values = {}
            for name in BODY_NAMES:
                path = '/World/envs/env_0/kitchen'+(KETTLE+name)[len('/World'):]
                api = PhysxSchema.PhysxRigidBodyAPI(e.sim.stage.GetPrimAtPath(path))
                values[path] = dict(position=int(api.GetSolverPositionIterationCountAttr().Get()),
                                    velocity=int(api.GetSolverVelocityIterationCountAttr().Get()))
            return values
        info['live_composed_solver_parameters_initial'] = live_solver()
        info['live_solver_parameter_scope'] = ('Typed PhysxSchema reads from the actual composed runtime stage after reset. '
            'The available native tensor RigidBodyView exposes no solver iteration getter; internal solver readback is unverified.')
        info['native_solver_readback_methods'] = [name for name in dir(view) if 'solver' in name.lower() and name.startswith('get_')]
        temporary = Usd.Stage.CreateInMemory()
        default_api = PhysxSchema.PhysxRigidBodyAPI.Apply(temporary.DefinePrim('/Defaults'))
        info['registered_schema_defaults'] = dict(position=default_api.GetSolverPositionIterationCountAttr().Get(),
            velocity=default_api.GetSolverVelocityIterationCountAttr().Get())
        dt, action_dt = float(e.physics_dt), float(e.step_dt)
        info.update(physics_dt_s=dt, action_dt_s=action_dt, decimation=round(action_dt/dt),
            actions=int(np.ceil(args.duration_seconds/action_dt)), reset_physics_step=reset_physics_step,
            background_paths=list(view.prim_paths), joint_names=list(robot.joint_names),
            background_topology=background_topology(e.sim.stage, view))
        if abs(info['decimation']*dt-action_dt) > 1e-9:
            raise ValueError('Nonintegral physics/action sampling ratio')
        ids = [robot.joint_names.index(f'panda_joint{i}') for i in range(1, 8)]
        command = torch.tensor(np.r_[array(robot.data.joint_pos)[0, ids], 0.][None], dtype=torch.float32, device=e.device)
        def capture():
            rows['physics_step'].append(int(e.sim.get_physics_step_count()))
            rows['q'].append(array(robot.data.joint_pos)[0]); rows['dq'].append(array(robot.data.joint_vel)[0])
            rows['background'].append(array(view.get_transforms()))
            rows['background_velocity'].append(array(view.get_velocities()))
            rows['mustard'].append(array(e.scene['mustard_bottle'].data.root_state_w)[0])
            rows['bowl'].append(array(e.scene['bowl'].data.root_state_w)[0])
        capture()
        original_update = e.scene.update
        def update(dt, *positional, **keywords):
            value = original_update(dt, *positional, **keywords)
            if int(e.sim.get_physics_step_count()) != rows['physics_step'][-1]:
                try:
                    capture()
                except Exception as error:
                    info['callback_errors'].append(repr(error)); raise
            return value
        e.scene.update = update
        for _ in range(info['actions']):
            _, _, done, truncated, _ = env.step(command)
            if bool(array(done)[0]) or bool(array(truncated)[0]):
                raise RuntimeError('Neutral diagnostic terminated/reset unexpectedly')
        info['live_composed_solver_parameters_final'] = live_solver()
        info['final_physics_step'] = int(e.sim.get_physics_step_count())
    except Exception as error:
        info['runtime_error'] = repr(error)
    finally:
        if original_update is not None:
            env.unwrapped.scene.update = original_update
        env.close()
    data = {key: np.asarray(value) for key, value in rows.items()}
    np.savez_compressed(args.output/'physics_states.npz', **data,
        joint_names=np.asarray(info.get('joint_names', [])), background_paths=np.asarray(info.get('background_paths', [])))
    try:
        info.update(analyze(data, info))
        requested = dict(position=args.position_iterations, velocity=args.velocity_iterations)
        info['checks']['runtime_composed_solver_matches_requested'] = all(row == requested for
            key in ['live_composed_solver_parameters_initial', 'live_composed_solver_parameters_final'] for row in info[key].values())
        info['checks']['all_inputs_unchanged_during_probe'] = (hashes == {key: sha(key) for key in hashes} and
            sha(args.layout) == layout_sha and sha(overlay['source']) == overlay['source_sha256'] and
            sha(overlay['overlay']) == overlay['overlay_sha256'] and
            sha(envdef.background_physics_provenance['source']) == envdef.background_physics_provenance['source_sha256'])
        info['passed'] = all(info['checks'].values())
    except Exception as error:
        info.update(passed=False, audit_error=repr(error))
    info['input_sha256'] = {'physics_states.npz': sha(args.output/'physics_states.npz')}
    (args.output/'solver_probe.json').write_text(json.dumps(info, indent=2, allow_nan=False)+'\n')
    print(json.dumps({key: info.get(key) for key in ['passed', 'checks', 'requested_solver_pair', 'samples', 'observed_seconds', 'runtime_error', 'audit_error']}, indent=2))
    if not info['passed']:
        raise RuntimeError('Solver probe failed original gates; retained full report and samples')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--layout', type=Path, default=ROOT/'configs/extensions/replicator/g_shape_mustard.json')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--position-iterations', type=int, choices=[16, 32, 64], required=True)
    parser.add_argument('--velocity-iterations', type=int, choices=[1, 2, 4], required=True)
    parser.add_argument('--duration-seconds', type=float, default=10.)
    parser.add_argument('--device', default='cpu')
    args = parser.parse_args()
    if (args.position_iterations, args.velocity_iterations) not in PAIRS or not np.isfinite(args.duration_seconds) or args.duration_seconds < 10.:
        parser.error('Require predeclared pair 16/1, 32/2, or 64/4 and duration at least 10 seconds')
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'predeclared_plan.json').write_text(json.dumps(dict(pairs=PAIRS, duration_seconds=args.duration_seconds,
        seed=42, task_id='kitchen_bench_replicator_g_shape_mustard_bowl', no_automatic_parameter_selection=True,
        preserve_all_results=True, original_world_thresholds=dict(translation_m=.002, rotation_deg=1.)), indent=2)+'\n')
    # Use the original Arena defaults while keeping --help CPU-only.
    from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
    from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
    simulator_args = get_isaaclab_arena_cli_parser().parse_args([])
    simulator_args.device, simulator_args.enable_cameras = args.device, True
    try:
        with SimulationAppContext(simulator_args):
            run(args)
    except Exception as error:
        path = args.output/'solver_probe.json'
        if not path.exists():
            path.write_text(json.dumps(dict(passed=False, error=repr(error)), indent=2)+'\n')
        raise


if __name__ == '__main__':
    main()
