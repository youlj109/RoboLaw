"""Neutral hold observing original task objects, dynamic joints and static anchors."""
import json
from pathlib import Path
from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext

p = get_isaaclab_arena_cli_parser()
p.add_argument('--layout', required=True)
p.add_argument('--output', type=Path, required=True)
p.add_argument('--duration-seconds', type=float, default=40.)
p.add_argument('--background-settling-seconds', type=float, default=5.)
a = p.parse_args()
if not 2. <= a.background_settling_seconds < a.duration_seconds:
    p.error('Require 2 <= background settling seconds < duration seconds')
a.enable_cameras = True
a.output.mkdir(parents=True, exist_ok=False)
with SimulationAppContext(a):
    import numpy as np
    import torch
    from scipy.spatial.transform import Rotation
    from pxr import Usd, UsdGeom, UsdPhysics
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from isaaclab.sensors import ContactSensorCfg
    from kitchen_safety.extensions.replicator.environment import KitchenSafetyCfg, KitchenSafetyEnvironment
    from kitchen_safety.extensions.replicator.overview import configure_overview
    from kitchen_safety.dataset import CameraWriter, numpy as arr

    layout = json.loads(Path(a.layout).read_text())
    envdef = KitchenSafetyEnvironment().build(KitchenSafetyCfg(
        variant='human', enable_cameras=True, collection_seed=42,
        obstacle_id='human_default_mpfb', fixed_layout=layout, episode_length_s=120))
    builder = ArenaEnvBuilder(envdef, ArenaEnvBuilderCfg(
        num_envs=1, seed=42, device=a.device, presets='physx', solve_relations=False))
    cfg, kwargs = builder.compose_manager_cfg()
    cfg.observations.camera_obs = None
    cfg.sim.render_interval = cfg.decimation
    cfg.scene.robot_contacts = ContactSensorCfg(
        prim_path='{ENV_REGEX_NS}/Robot/.*', update_period=0, history_length=1)
    overview_spec = configure_overview(cfg.scene, eye=(1., -2.3, 1.5), target=(2.4, -1.4, .85))
    env = builder.make_registered(cfg, kwargs)
    env.reset()
    e = env.unwrapped
    robot = e.scene['robot']
    ids = [robot.joint_names.index(f'panda_joint{i}') for i in range(1, 8)]
    q = arr(robot.data.joint_pos)[0, ids]
    command = torch.tensor(np.r_[q, 0][None], dtype=torch.float32, device=e.device)
    source = Usd.Stage.Open(str(Path(layout['kitchen_usd']).resolve()))
    source_root = source.GetDefaultPrim()
    if not source_root:
        raise RuntimeError('Kitchen source has no default prim for reference mapping')
    live_root = '/World/envs/env_0/kitchen'
    joint_connected, joint_targets, static_anchors = set(), [], {}

    def transform(prim):
        return np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(prim), dtype=float)

    def animated_ancestors(prim):
        paths = []
        while prim and not prim.IsPseudoRoot():
            xform = UsdGeom.Xformable(prim)
            if xform and xform.TransformMightBeTimeVarying():
                paths.append(str(prim.GetPath()))
            prim = prim.GetParent()
        return paths

    for prim in e.sim.stage.Traverse():
        if not prim.IsA(UsdPhysics.Joint):
            continue
        joint = UsdPhysics.Joint(prim)
        for role, relation in [('body0', joint.GetBody0Rel()), ('body1', joint.GetBody1Rel())]:
            for target in relation.GetTargets():
                target_path = str(target)
                if not target_path.startswith(live_root + '/'):
                    continue
                endpoint = e.sim.stage.GetPrimAtPath(target)
                if not endpoint:
                    raise RuntimeError(f'Missing joint target {target}')
                body = endpoint
                while body and not body.IsPseudoRoot() and not body.HasAPI(UsdPhysics.RigidBodyAPI):
                    body = body.GetParent()
                record = dict(joint=str(prim.GetPath()), role=role, authored_target=target_path)
                if body and not body.IsPseudoRoot():
                    resolved = str(body.GetPath())
                    joint_connected.add(resolved)
                    record.update(kind='dynamic_rigid_body', rigid_body=resolved)
                else:
                    # USD joints may legally attach to a static cabinet frame (world anchor).
                    source_path = str(source_root.GetPath()) + target_path[len(live_root):]
                    source_prim = source.GetPrimAtPath(source_path)
                    if not source_prim:
                        raise RuntimeError(f'Static anchor has no source counterpart: {source_path}')
                    source_matrix, live_matrix = transform(source_prim), transform(endpoint)
                    static_anchors[target_path] = dict(
                        source_path=source_path, source_world_transform=source_matrix.tolist(),
                        initial_world_transform=live_matrix.tolist(),
                        source_initial_max_matrix_difference=float(np.abs(source_matrix-live_matrix).max()),
                        animated_source_ancestors=animated_ancestors(source_prim),
                        animated_runtime_ancestors=animated_ancestors(endpoint))
                    record.update(kind='static_anchor', source_path=source_path)
                joint_targets.append(record)
    background = e.scene['robot_contacts']._physics_sim_view.create_rigid_body_view(live_root + '/**')

    def native_array(value):
        value = value.torch if hasattr(value, 'torch') else value
        return (value.detach().cpu().numpy() if hasattr(value, 'detach') else
                np.asarray(value.numpy() if hasattr(value, 'numpy') else value)).copy()

    # PhysX may expose an articulation's root rigid link under its articulation
    # root path. Prove the unique directed-joint root, mass and initial pose;
    # never silently classify an enabled missing rigid body as a static anchor.
    observed_paths = list(background.prim_paths)
    aliases = {}
    initial_poses = native_array(background.get_transforms())
    native_masses = native_array(background.get_masses()).reshape(-1)
    for missing in sorted(joint_connected-set(observed_paths)):
        body = e.sim.stage.GetPrimAtPath(missing)
        parent = body.GetParent()
        while parent and not parent.IsPseudoRoot() and not parent.HasAPI(UsdPhysics.ArticulationRootAPI):
            parent = parent.GetParent()
        if not parent or parent.IsPseudoRoot() or str(parent.GetPath()) not in observed_paths:
            continue
        root_path = str(parent.GetPath())
        edges = [item for item in joint_targets if item.get('rigid_body') and item['joint'].startswith(root_path+'/')]
        roots = {x['rigid_body'] for x in edges if x['role']=='body0'} - {x['rigid_body'] for x in edges if x['role']=='body1'}
        if roots != {missing}:
            continue
        native_index = observed_paths.index(root_path)
        authored_mass = UsdPhysics.MassAPI(body).GetMassAttr().Get()
        native_mass = float(native_masses[native_index])
        frame = transform(body).T
        rotation = frame[:3, :3]/np.linalg.norm(frame[:3, :3], axis=0)
        measured = initial_poses[native_index]
        translation_error = float(np.linalg.norm(frame[:3, 3]-measured[:3]))
        rotation_error = float(np.rad2deg((Rotation.from_matrix(rotation)*Rotation.from_quat(measured[3:7]).inv()).magnitude()))
        if authored_mass is None or abs(native_mass-authored_mass)>1e-4*max(1., authored_mass) or translation_error>.002 or rotation_error>1.:
            raise RuntimeError(f'Unverified articulation root alias for {missing}')
        aliases[missing] = dict(native_path=root_path, unique_joint_graph_root=sorted(roots),
            source_mass_kg=authored_mass, native_mass_kg=native_mass,
            initial_translation_error_m=translation_error, initial_rotation_error_deg=rotation_error)
    observed_joint_connected = {aliases.get(path, {}).get('native_path', path) for path in joint_connected}
    for record in joint_targets:
        if record.get('rigid_body') in aliases:
            record['native_rigid_body'] = aliases[record['rigid_body']]['native_path']
    info = dict(
        layout=a.layout, background_physics=envdef.background_physics_provenance,
        control='Initial robot arm command held, gripper open; no object/joint-state setters after reset',
        frames=int(np.ceil(a.duration_seconds/e.step_dt)), dt=e.step_dt, settling_frames=30, background_settling_seconds=a.background_settling_seconds,
        background_settling_frames=int(round(a.background_settling_seconds/e.step_dt)), articulation_root_aliases=aliases,
        predeclared_tolerances=dict(task_translation_m=.001, task_rotation_deg=1., robot_joint_rad=.001),
        background_paths=list(background.prim_paths), joint_connected_bodies=sorted(joint_connected),
        joint_targets=joint_targets, static_anchor_paths=list(static_anchors), static_anchors=static_anchors,
        static_anchor_matrix_tolerance=1e-6, overview=overview_spec,
        background_tolerances=dict(translation_m=.002, rotation_deg=1.,
            applies='Free background rigid bodies after predeclared 5s settling; joint-connected bodies from reset'),
        scope='Neutral observation over the requested duration. Native poses/velocities cover dynamic '
              'bodies; composed USD world transforms cover legal static anchors with no rigid body '
              'ancestor and no transform animation. Dynamic joint endpoints cannot be omitted.')
    writer = CameraWriter(a.output, e.scene, envdef.embodiment.camera_config.camera_names(), e.step_dt, previews=True)
    (a.output/'overview').mkdir()
    overview_writer = CameraWriter(a.output/'overview', e.scene, ['overview_camera'], e.step_dt, previews=True)
    for _ in range(32):
        e.sim.render()
    rows = {key: [] for key in ['q', 'dq', 'mustard', 'bowl', 'background', 'background_velocity',
                               'robot_contact_force', 'static_anchor_transforms', 'capture_physics_steps']}
    try:
        for i in range(info['frames']):
            before = int(e.sim.get_physics_step_count())
            writer.write('neutral_hold')
            overview_writer.write('neutral_hold')
            after = int(e.sim.get_physics_step_count())
            if before != after:
                raise RuntimeError('Camera capture advanced physics')
            rows['capture_physics_steps'].append([before, after])
            rows['q'].append(arr(robot.data.joint_pos)[0])
            rows['dq'].append(arr(robot.data.joint_vel)[0])
            rows['mustard'].append(arr(e.scene['mustard_bottle'].data.root_state_w)[0])
            rows['bowl'].append(arr(e.scene['bowl'].data.root_state_w)[0])
            rows['background'].append(native_array(background.get_transforms()))
            rows['background_velocity'].append(native_array(background.get_velocities()))
            rows['robot_contact_force'].append(arr(e.scene['robot_contacts'].data.net_forces_w)[0])
            rows['static_anchor_transforms'].append([
                transform(e.sim.stage.GetPrimAtPath(path)) for path in static_anchors])
            _, _, done, truncated, _ = env.step(command)
            if bool(arr(done)[0]) or bool(arr(truncated)[0]):
                raise RuntimeError('Neutral hold terminated/reset unexpectedly; reject this diagnostic')
    finally:
        writer.close()
        overview_writer.close()
        env.close()
    rows = {key: np.asarray(value) for key, value in rows.items()}
    np.savez_compressed(a.output/'neutral_states.npz', **rows)

    def difference(values, start_index=30):
        start = values[start_index, :7]
        x = values[start_index:, :7]
        angles = np.rad2deg((Rotation.from_quat(x[:, 3:7])*Rotation.from_quat(start[3:7]).inv()).magnitude())
        return dict(max_translation_m=float(np.linalg.norm(x[:, :3]-start[:3], axis=1).max()),
                    max_rotation_deg=float(angles.max()))

    info['task_objects'] = {name: difference(rows[name]) for name in ['mustard', 'bowl']}
    info['robot_max_joint_change_rad'] = float(np.abs(rows['q']-rows['q'][0]).max())
    info['background_after_settling'] = [dict(path=path, **difference(rows['background'][:, j], info['background_settling_frames']))
                                      for j, path in enumerate(info['background_paths'])]
    info['background_from_reset'] = [dict(path=path, **difference(rows['background'][:, j], 0))
                                    for j, path in enumerate(info['background_paths'])]
    info['articulated_bodies_from_reset'] = [x for x in info['background_from_reset'] if x['path'] in observed_joint_connected]
    info['background_settled_passed'] = all(x['max_translation_m'] <= .002 and x['max_rotation_deg'] <= 1
                                          for x in info['background_after_settling'])
    info['unobserved_joint_connected_bodies'] = sorted(observed_joint_connected-set(info['background_paths']))
    info['unoperated_articulated_bodies_passed'] = (
        not info['unobserved_joint_connected_bodies'] and bool(info['articulated_bodies_from_reset']) and
        all(x['max_translation_m'] <= .002 and x['max_rotation_deg'] <= 1
            for x in info['articulated_bodies_from_reset']))
    for j, path in enumerate(static_anchors):
        record = static_anchors[path]
        record['runtime_max_matrix_difference'] = float(np.abs(
            rows['static_anchor_transforms'][:, j]-np.asarray(record['initial_world_transform'])).max())
        record['passed'] = (record['source_initial_max_matrix_difference'] <= 1e-6 and
            record['runtime_max_matrix_difference'] <= 1e-6 and
            not record['animated_source_ancestors'] and not record['animated_runtime_ancestors'])
    info['static_anchors_passed'] = all(x['passed'] for x in static_anchors.values())
    info['task_objects_passed'] = all(x['max_translation_m'] <= .001 and x['max_rotation_deg'] <= 1
                                      for x in info['task_objects'].values())
    info['robot_hold_passed'] = info['robot_max_joint_change_rad'] <= .001
    info['overview_capture_alignment'] = bool(np.all(rows['capture_physics_steps'][:, 0] == rows['capture_physics_steps'][:, 1]))
    info['passed'] = bool(info['task_objects_passed'] and info['robot_hold_passed'] and
        info['background_settled_passed'] and info['unoperated_articulated_bodies_passed'] and
        info['static_anchors_passed'] and info['overview_capture_alignment'])
    (a.output/'neutral_probe.json').write_text(json.dumps(info, indent=2)+'\n')
    print(json.dumps({k: v for k, v in info.items() if not k.startswith('background') and k not in
        ['articulated_bodies_from_reset', 'joint_connected_bodies', 'joint_targets', 'static_anchors']}, indent=2))

    if not info['passed']:
        raise SystemExit(2)
