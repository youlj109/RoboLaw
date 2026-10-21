"""Replay recorded actions for contact diagnosis, without planning or success scoring.

Example (run only when the simulation GPU is free)::

    bash scripts/arena_python.sh scripts/diagnose_contact_replay.py

The default source has 411 rows; its last action was interrupted by an FCL
exception. Only the first 410 complete actions are replayed by default.
"""

import json
from pathlib import Path
import traceback

from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = (
    "datasets/kitchen_auto/human_lower_first_validation/"
    "human_broad_workwear_raised_wrist/lower_first_v4/video"
)


def main():
    parser = get_isaaclab_arena_cli_parser()
    parser.add_argument('--source', default=DEFAULT_SOURCE,
                        help='Directory containing original result.json and trajectory.h5')
    parser.add_argument('--output', default=(
        'reports/human_lower_first_validation/broad_v4_contact_replay'))
    parser.add_argument('--num-actions', type=int, default=410,
                        help='Replay this many saved actions, excluding the interrupted last row')
    args = parser.parse_args()
    if args.num_actions <= 0:
        parser.error('--num-actions must be positive')
    source = ROOT / args.source
    output = ROOT / args.output
    source_result = json.loads((source / 'result.json').read_text())
    # Source metadata, not CLI defaults, determines the original environment.
    args.seed = int(source_result['seed'])
    args.enable_cameras = False
    output.mkdir(parents=True, exist_ok=True)

    with SimulationAppContext(args):
        import h5py
        import numpy as np
        import torch
        import omni.usd
        from pxr import Usd, UsdPhysics

        from isaaclab.sensors import ContactSensorCfg
        from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
        from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
        from kitchen_safety.environment import KitchenSafetyCfg, KitchenSafetyEnvironment

        with h5py.File(source / 'trajectory.h5') as saved:
            actions = saved['action'][:args.num_actions].astype(np.float32)
            phases = saved['phase'].asstr()[:len(actions)]
            reference_q = saved['next_obs/q'][:len(actions)]
            reference_initial_q = saved['obs/q'][0]
            reference_joint_names = json.loads(saved.attrs['joint_names'])
            reference_dt = float(saved.attrs['dt'])
            source_action_count = len(saved['action'])
        if actions.ndim != 2 or actions.shape[1] != 8 or not np.isfinite(actions).all():
            raise ValueError('Source actions must be finite absolute arm7 + binary gripper commands')

        environment = KitchenSafetyEnvironment().build(KitchenSafetyCfg(
            variant=source_result['variant'], enable_cameras=False,
            object_jitter_m=float(source_result['jitter_m']),
            collection_seed=args.seed, obstacle_id=source_result['obstacle_id']))
        builder = ArenaEnvBuilder(environment, ArenaEnvBuilderCfg(
            device=args.device, seed=args.seed, presets='physx', num_envs=1,
            solve_relations=False, disable_fabric=False))
        cfg, kwargs = builder.compose_manager_cfg()
        cfg.sim.render_interval = cfg.decimation
        cfg.scene.robot_contacts = ContactSensorCfg(
            prim_path='{ENV_REGEX_NS}/Robot/.*', update_period=0, history_length=1)
        env = builder.make_registered(cfg, kwargs)
        env.reset()
        e = env.unwrapped
        scene = e.scene
        robot = scene['robot']
        e.render_enabled = False
        ids, _ = robot.find_joints(['panda_joint[1-7]'], preserve_order=True)

        def arr(value):
            value = value.torch if hasattr(value, 'torch') else value
            if hasattr(value, 'detach'):
                return value.detach().cpu().numpy().copy()
            # Raw PhysX tensor views use Warp arrays in this Arena version.
            return np.asarray(value.numpy() if hasattr(value, 'numpy') else value).copy()

        # Each PhysX filter pattern must resolve to exactly one actor. Group
        # wildcards such as kitchen/* resolve to many and are not accepted.
        stage = omni.usd.get_context().get_stage()
        actor_colliders = {}
        for prim in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            if not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():
                continue
            actor = prim
            while actor and not actor.IsPseudoRoot():
                if actor.HasAPI(UsdPhysics.RigidBodyAPI):
                    break
                actor = actor.GetParent()
            # Static geometry without a rigid ancestor is its own actor.
            if not actor or actor.IsPseudoRoot():
                actor = prim
            actor_colliders.setdefault(str(actor.GetPath()), []).append(str(prim.GetPath()))
        contact_filters = sorted(actor_colliders)
        wrist_path = '/World/envs/env_0/Robot/panda_link7'
        # Persist paths before stepping so a failed actor mapping is inspectable.
        (output / 'contact_actor_paths.json').write_text(json.dumps(dict(
            sensor_actor=wrist_path, filter_actor_paths=contact_filters,
            actor_colliders=actor_colliders), indent=2))
        contact_view = scene['robot_contacts']._physics_sim_view.create_rigid_contact_view(
            [wrist_path], filter_patterns=[contact_filters], max_contact_data_count=64)

        # Exactly the collector's unrecorded initialization: capture q once,
        # then send that same absolute target and open gripper for eight steps.
        neutral_q = arr(robot.data.joint_pos)[0, ids]
        neutral_action = np.r_[neutral_q, 0.].astype(np.float32)
        for _ in range(8):
            env.step(torch.tensor(neutral_action[None], device=e.device))

        joint_names = list(robot.joint_names)
        body_names = list(robot.body_names)
        contact_names = list(scene['robot_contacts'].body_names)
        source_indices = [reference_joint_names.index(name) for name in joint_names]
        reference_q = reference_q[:, source_indices]
        reference_initial_q = reference_initial_q[source_indices]
        wrist_index = body_names.index('panda_link7')
        aggregate_wrist_index = contact_names.index('panda_link7')
        physical_parameters = {
            'joint_stiffness': dict(zip(joint_names, arr(robot.data.joint_stiffness)[0].tolist())),
            'joint_damping': dict(zip(joint_names, arr(robot.data.joint_damping)[0].tolist())),
            # Passive USD mimic joints can have an infinite authored limit.
            # Preserve that meaning while keeping the report valid JSON.
            'joint_effort_limits': dict(zip(joint_names, [
                float(value) if np.isfinite(value) else str(float(value))
                for value in arr(robot.data.joint_effort_limits)[0]])),
            'mustard_body_mass_kg': arr(scene['mustard_bottle'].data.body_mass)[0].tolist(),
            'mustard_body_com_position_local_m': arr(scene['mustard_bottle'].data.body_com_pos_b)[0].tolist(),
            'mustard_body_com_quaternion_xyzw': arr(scene['mustard_bottle'].data.body_com_quat_b)[0].tolist(),
            'source': 'Live initialized articulation/rigid-object data, read without changing parameters',
        }
        (output / 'physical_parameters.json').write_text(json.dumps(physical_parameters, indent=2))

        def snapshot():
            return dict(
                q=arr(robot.data.joint_pos)[0],
                dq=arr(robot.data.joint_vel)[0],
                robot_body_pose_w=np.concatenate((arr(robot.data.body_pos_w)[0],
                                                  arr(robot.data.body_quat_w)[0]), axis=-1),
                mustard=arr(scene['mustard_bottle'].data.root_state_w)[0],
                bowl=arr(scene['bowl'].data.root_state_w)[0],
                robot_contact_forces=arr(scene['robot_contacts'].data.net_forces_w)[0])

        initial = snapshot()
        contacts = []
        physics_contacts = []
        records = []
        terminal = {}
        action_index = -1
        physics_updates = 0
        physics_within_action = 0
        error = None

        def contact_record():
            force = arr(contact_view.get_net_contact_forces(dt=cfg.sim.dt))[0]
            aggregate_force = arr(scene['robot_contacts'].data.net_forces_w)[0, aggregate_wrist_index]
            if max(float(np.linalg.norm(force)), float(np.linalg.norm(aggregate_force))) <= 1.:
                return None
            matrix = arr(contact_view.get_contact_force_matrix(dt=cfg.sim.dt))[0]
            raw_forces, raw_points, raw_normals, raw_separations, counts, starts = (
                contact_view.get_contact_data(dt=cfg.sim.dt))
            raw_forces, raw_points, raw_normals, raw_separations = map(
                arr, (raw_forces, raw_points, raw_normals, raw_separations))
            counts, starts = arr(counts).reshape(-1), arr(starts).reshape(-1)
            partner_contacts = []
            for partner_index, (count, start) in enumerate(zip(counts, starts)):
                count, start = int(count), int(start)
                if not count and np.linalg.norm(matrix[partner_index]) <= 1.e-8:
                    continue
                sl = slice(start, start + count)
                partner_contacts.append(dict(
                    actor_path=contact_filters[partner_index], filter_index=partner_index,
                    normal_force_w_n=matrix[partner_index].tolist(), contact_count=count,
                    points_world_m=raw_points[sl].tolist(), normals_world=raw_normals[sl].tolist(),
                    normal_force_magnitudes_n=raw_forces[sl].tolist(),
                    separations_m=raw_separations[sl].tolist()))
            body_pose = np.r_[arr(robot.data.body_pos_w)[0, wrist_index],
                              arr(robot.data.body_quat_w)[0, wrist_index]]
            return dict(
                action_index=action_index, phase=str(phases[action_index]),
                physics_update=physics_updates, physics_within_action=physics_within_action,
                net_force_w_n=force.tolist(),
                original_sensor_link7_force_w_n=aggregate_force.tolist(),
                normal_force_matrix_w_n=matrix.tolist(),
                partner_contacts=partner_contacts,
                articulation_body_pose_xyz_xyzw=body_pose.tolist(),
                arm_q_rad=arr(robot.data.joint_pos)[0, ids].tolist())

        scene_update_original = scene.update
        reset_original = e._reset_idx

        def record_scene_update(dt):
            nonlocal physics_updates, physics_within_action
            scene_update_original(dt)
            physics_updates += 1
            physics_within_action += 1
            record = contact_record()
            if record is not None:
                physics_contacts.append(record)

        def capture_reset(env_ids):
            # Mirror the collector's success latch and preserve a terminal
            # state before any non-success automatic reset.
            if len(env_ids):
                terminal['state'] = snapshot()
                terminal['terms'] = {
                    name: bool(arr(e.termination_manager.get_term(name))[0])
                    for name in e.termination_manager.active_terms}
            if not terminal.get('terms', {}).get('success', False):
                return reset_original(env_ids)

        scene.update = record_scene_update
        e._reset_idx = capture_reset
        try:
            for action_index, action in enumerate(actions):
                physics_within_action = 0
                _, _, done, truncated, _ = env.step(torch.tensor(action[None], device=e.device))
                terminated = bool(arr(truncated)[0] or (
                    arr(done)[0] and not terminal.get('terms', {}).get('success', False)))
                state = terminal['state'] if terminated else snapshot()
                records.append(state)
                record = contact_record()
                if record is not None:
                    contacts.append(record)
                if len(records) % 30 == 0 or len(records) == len(actions):
                    delta = np.abs(state['q'] - reference_q[action_index])
                    print('CONTACT_REPLAY', len(records), '/', len(actions),
                          'phase', phases[action_index], 'max_q_error_rad', float(delta.max()),
                          'link7_force_n', float(np.linalg.norm(
                              state['robot_contact_forces'][aggregate_wrist_index])), flush=True)
                if terminated:
                    error = f'Environment terminated at action {action_index}: {terminal.get("terms")}'
                    break
        except Exception:
            error = traceback.format_exc()
            print('CONTACT_REPLAY_ERROR', error, flush=True)
        finally:
            scene.update = scene_update_original
            e._reset_idx = reset_original
            if records:
                replay_q = np.asarray([state['q'] for state in records])
                difference = np.abs(replay_q - reference_q[:len(records)])
                worst_frame, worst_joint = np.unravel_index(difference.argmax(), difference.shape)
                comparison = dict(
                    compared_actions=len(records),
                    max_abs_joint_error_rad=float(difference.max()),
                    max_abs_arm_error_rad=float(difference[:, ids].max()),
                    max_abs_error_by_joint_rad=dict(zip(joint_names, difference.max(axis=0).tolist())),
                    rms_error_by_joint_rad=dict(zip(joint_names, np.sqrt((difference**2).mean(axis=0)).tolist())),
                    worst_action_index=int(worst_frame), worst_joint=joint_names[worst_joint])
            else:
                comparison = {'compared_actions': 0}
            report = dict(
                diagnostic_only=True, collection_success_evaluated=False,
                source_directory=str(source), source_action_count=source_action_count,
                requested_actions=args.num_actions, replayed_actions=len(records),
                omitted_source_rows=source_action_count-len(actions),
                completion=(error is None and len(records) == len(actions)), error=error,
                seed=args.seed, variant=source_result['variant'],
                obstacle_id=source_result['obstacle_id'], jitter_m=source_result['jitter_m'],
                initialization={'env_reset_calls': 1, 'neutral_steps': 8,
                                'neutral_action': neutral_action.tolist(),
                                'initial_q_max_error_rad': float(np.abs(initial['q']-reference_initial_q).max())},
                simulation={'presets': 'physx', 'num_envs': 1, 'device': str(e.device),
                            'solve_relations': False, 'disable_fabric': False,
                            'physics_dt_s': float(cfg.sim.dt), 'decimation': int(cfg.decimation),
                            'render_interval': int(cfg.sim.render_interval),
                            'step_dt_s': float(e.step_dt), 'source_step_dt_s': reference_dt,
                            'robot_spawn_usd': str(cfg.scene.robot.spawn.usd_path),
                            'cameras_enabled': False, 'render_enabled': False},
                deliberate_differences=['Cameras and rendering disabled',
                                        'No planning or FCL safety checks',
                                        'Added raw link7 contact view and read-only physics-step logging'],
                original_partial_row_note='Default410 excludes source action410, interrupted by original FCL check.',
                joint_names=joint_names, robot_body_names=body_names,
                contact_body_names=contact_names, comparison=comparison,
                physical_parameters=physical_parameters,
                filter_actor_paths=contact_filters, sensor_body_names=['panda_link7'],
                filter_count=int(contact_view.filter_count),
                force_matrix_shape=list(arr(contact_view.get_contact_force_matrix(dt=cfg.sim.dt)).shape),
                physics_updates=physics_updates,
                action_contacts_over_1n=len(contacts), physics_contacts_over_1n=len(physics_contacts),
                terminations=terminal.get('terms', {}))
            with h5py.File(output / 'replay.h5', 'w') as saved:
                saved.attrs['diagnostic_only'] = True
                saved.attrs['dt'] = e.step_dt
                saved.attrs['joint_names'] = json.dumps(joint_names)
                saved.attrs['robot_body_names'] = json.dumps(body_names)
                saved.attrs['contact_body_names'] = json.dumps(contact_names)
                saved.attrs['quaternion_order'] = 'xyzw'
                saved.create_dataset('action', data=actions[:len(records)])
                saved.create_dataset('reference_next_q', data=reference_q[:len(records)])
                saved.create_dataset('phase', data=np.asarray(phases[:len(records)], dtype=h5py.string_dtype()))
                for key in initial:
                    saved.create_dataset('initial/'+key, data=initial[key])
                    if records:
                        saved.create_dataset('next_obs/'+key, data=np.asarray([r[key] for r in records]))
            (output / 'contact_diagnostics.json').write_text(json.dumps(dict(
                filter_actor_paths=contact_filters, filter_count=report['filter_count'],
                sensor_body_names=['panda_link7'],
                action_frames=contacts, physics_frames=physics_contacts), indent=2, allow_nan=False))
            (output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False))
            print('CONTACT_REPLAY_RESULT', json.dumps(report), flush=True)
            env.close()
        if error is not None:
            raise SystemExit(1)


if __name__ == '__main__':
    main()
