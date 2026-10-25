"""Task-scale neutral hold for the lime placement scene.

Holds the DROID arm at the authored reset command (gripper open). Records lime,
bowl, distractors, robot q/dq and cameras over a duration comparable to a
successful physical episode. Predeclared tolerances accept/reject independently
of task predicates.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext

ROOT = Path(__file__).resolve().parents[3]

p = get_isaaclab_arena_cli_parser()
p.add_argument('--solver-velocity-iterations',type=int,default=None,help='Diagnostic PhysX scene minimum velocity solver iterations; no joint locks or parameter changes')
p.add_argument('--joint-inventory-only',action='store_true',help='Read actual PhysX joint parameters without a hold trial')
p.add_argument('--repair-fixture-schemas',action='store_true',help='Diagnostic: map authored drive channels to canonical USD names and apply the authored PhysxJointAPI friction schema; preserve all numeric values')
p.add_argument('--native-background-reset',action='store_true',help='Diagnostic: use the upstream Background default nested physics reset, without locking joints or changing assets')
p.add_argument('--task', default='kitchen_bench_lightwheel_place_lime_in_wooden_bowl_l_shaped_scandinavian')
p.add_argument('--fixed-layout', type=Path, default=None,
               help='Defaults to task-specific fixed_layout path; omit for graph-solved reset')
p.add_argument('--output', type=Path, required=True)
p.add_argument('--duration-seconds', type=float, default=35.0,
               help='Observation window; default covers ~one successful B01 episode (~31.5s) with margin')
p.add_argument('--background-settling-seconds', type=float, default=5.0)
p.add_argument('--reference-episode', type=Path, default=None,
               help='Optional successful episode whose duration is used as a lower bound')
a = p.parse_args()
from kitchen_safety.extensions.lightwheel_pick_place.task_config import resolve_task
TASK_SPEC = resolve_task(a.task)
TASK_OBJECTS = TASK_SPEC.rigid_objects
if a.fixed_layout is None:
    a.fixed_layout = TASK_SPEC.default_fixed_layout if TASK_SPEC.default_fixed_layout.exists() else None
if not 2.0 <= a.background_settling_seconds < a.duration_seconds:
    p.error('Require 2 <= background settling seconds < duration seconds')
a.enable_cameras = True
a.output = Path(a.output)
a.output.mkdir(parents=True, exist_ok=False)

duration = float(a.duration_seconds)
if a.reference_episode is not None:
    ref = json.loads((Path(a.reference_episode) / 'result.json').read_text())
    frames = int(ref.get('frames') or 0)
    dt = float(ref.get('dt') or (1.0 / 15.0))
    duration = max(duration, frames * dt)

with SimulationAppContext(a):
    import numpy as np
    import torch
    from scipy.spatial.transform import Rotation
    from pxr import UsdGeom, UsdPhysics
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from isaaclab.sensors import ContactSensorCfg
    from kitchen_safety.extensions.lightwheel_pick_place.environment import build_environment
    from kitchen_safety.dataset import CameraWriter, numpy as arr

    envdef, assets = build_environment(
        enable_cameras=True,
        variant='base',
        native_background_reset=a.native_background_reset,
        task_id=TASK_SPEC.task_id,
        fixed_layout_path=str(a.fixed_layout) if a.fixed_layout is not None else None,
    )
    builder = ArenaEnvBuilder(
        envdef,
        ArenaEnvBuilderCfg(
            num_envs=1,
            seed=42,
            device=a.device,
            presets='physx',
            solve_relations=True,
            resolve_on_reset=False,
            placement_seed=42,
        ),
    )
    cfg, kwargs = builder.compose_manager_cfg()
    if a.solver_velocity_iterations is not None:
        cfg.sim.physics.min_velocity_iteration_count=int(a.solver_velocity_iterations)
    cfg.observations.camera_obs = None
    cfg.sim.render_interval = cfg.decimation
    cfg.scene.robot_contacts = ContactSensorCfg(
        prim_path='{ENV_REGEX_NS}/Robot/.*', update_period=0, history_length=1
    )
    fixture_schema_records=[]
    if a.repair_fixture_schemas:
        from kitchen_safety.extensions.lightwheel_pick_place.fixture_schema import install_fixture_schema_repair
        fixture_schema_records=install_fixture_schema_repair(cfg.scene)
    env = builder.make_registered(cfg, kwargs)
    (a.output/'fixture_schema_repair.json').write_text(json.dumps(fixture_schema_records,indent=2))
    env.reset()
    e = env.unwrapped
    robot = e.scene['robot']
    ids = [robot.joint_names.index(f'panda_joint{i}') for i in range(1, 8)]
    q0 = arr(robot.data.joint_pos)[0, ids]
    command = torch.tensor(np.r_[q0, 0.0][None], dtype=torch.float32, device=e.device)

    # Track kitchen joint-connected rigid bodies via USD (world transforms).
    live_root = '/World/envs/env_0/kitchen'
    joint_connected = []
    for prim in e.sim.stage.Traverse():
        if not prim.IsA(UsdPhysics.Joint):
            continue
        joint = UsdPhysics.Joint(prim)
        for relation in (joint.GetBody0Rel(), joint.GetBody1Rel()):
            for target in relation.GetTargets():
                target_path = str(target)
                if not target_path.startswith(live_root + '/'):
                    continue
                endpoint = e.sim.stage.GetPrimAtPath(target)
                body = endpoint
                while body and not body.IsPseudoRoot() and not body.HasAPI(UsdPhysics.RigidBodyAPI):
                    body = body.GetParent()
                if body and not body.IsPseudoRoot():
                    path = str(body.GetPath())
                    if path not in joint_connected:
                        joint_connected.append(path)

    cache = UsdGeom.XformCache()

    def world_tf(path: str):
        prim = e.sim.stage.GetPrimAtPath(path)
        return np.asarray(cache.GetLocalToWorldTransform(prim), dtype=float)

    from kitchen_safety.extensions.lightwheel_pick_place.joint_state import KitchenJointRecorder
    joint_recorder=KitchenJointRecorder(e.sim.stage,e.sim)
    (a.output/'source_joint_inventory.json').write_text(json.dumps(joint_recorder.inventory,indent=2))

    if a.joint_inventory_only:
        print('JOINT_INVENTORY_ONLY_COMPLETE',flush=True)
        env.close()
        raise SystemExit(0)

    info = dict(
        scope=(
            'Neutral hold: same task-specific fixed_layout reset and PhysX backend as collection; '
            'arm command held at reset q with open gripper; no object/joint setters after reset. '
            'Task objects + distractors use RigidObject root states; kitchen joint-connected '
            'bodies retain USD transforms for comparison; actual PhysX body/joint state is recorded independently.'
        ),
        task=TASK_SPEC.task_id,
        native_background_reset=bool(a.native_background_reset),
        repair_fixture_schemas=bool(a.repair_fixture_schemas),
        solver_velocity_iterations=a.solver_velocity_iterations,
        fixture_schema_repair=fixture_schema_records,
        fixed_layout=str(a.fixed_layout),
        fixed_layout_sha256=hashlib.sha256(Path(a.fixed_layout).read_bytes()).hexdigest(),
        reference_episode=str(a.reference_episode) if a.reference_episode else None,
        duration_seconds=duration,
        background_settling_seconds=a.background_settling_seconds,
        control='Initial robot arm command held, gripper open; no object/joint-state setters after reset',
        predeclared_tolerances=dict(
            task_translation_m=0.001,
            task_rotation_deg=1.0,
            distractor_translation_m=0.002,
            distractor_rotation_deg=1.0,
            robot_joint_rad=0.001,
            articulated_translation_m=0.002,
            articulated_rotation_deg=1.0,
            actual_joint_angular_delta_rad=float(np.deg2rad(1.)),
            actual_joint_linear_delta_m=.001,
            actual_joint_angular_speed_rad_s_after_settling=float(np.deg2rad(1.)),
            actual_joint_linear_speed_m_s_after_settling=.001,
            actual_body_translation_m=.002,
            actual_body_rotation_deg=1.,
        ),
        joint_connected_bodies=joint_connected,
    )

    writer = CameraWriter(
        a.output,
        e.scene,
        envdef.embodiment.camera_config.camera_names(),
        e.step_dt,
        previews=True,
    )
    for _ in range(32):
        e.sim.render()

    frames = int(np.ceil(duration / e.step_dt))
    settle = int(round(a.background_settling_seconds / e.step_dt))
    info.update(frames=frames, dt=float(e.step_dt), background_settling_frames=settle)

    rows = {
        'q': [],
        'dq': [],
        'robot_contact_force': [],
        'capture_physics_steps': [],
        **{name: [] for name in TASK_OBJECTS},
        'articulated_transforms': [],
        **{name:[] for name in ('kitchen_body_pose','kitchen_body_velocity','kitchen_joint_q','kitchen_joint_dq')},
    }
    completed=False
    try:
        for i in range(frames):
            before = int(e.sim.get_physics_step_count())
            writer.write('neutral_hold')
            after = int(e.sim.get_physics_step_count())
            if before != after:
                raise RuntimeError('Camera capture advanced physics')
            for name,value in joint_recorder.read().items():rows[name].append(value)
            if i%150==0:print('NEUTRAL_STEP',i,'/',frames,flush=True)
            rows['capture_physics_steps'].append([before, after])
            rows['q'].append(arr(robot.data.joint_pos)[0])
            rows['dq'].append(arr(robot.data.joint_vel)[0])
            rows['robot_contact_force'].append(arr(e.scene['robot_contacts'].data.net_forces_w)[0])
            for name in TASK_OBJECTS:
                rows[name].append(arr(e.scene[name].data.root_state_w)[0])
            cache.Clear()
            rows['articulated_transforms'].append(
                [world_tf(path) for path in joint_connected] if joint_connected else []
            )
            _, _, done, truncated, _ = env.step(command)
            if bool(arr(done)[0]) or bool(arr(truncated)[0]):
                raise RuntimeError('Neutral hold terminated/reset unexpectedly; reject this diagnostic')
        completed=True
    finally:
        if not completed:
            np.savez_compressed(a.output/'neutral_states.partial.npz',**{k:np.asarray(v) for k,v in rows.items()})
        writer.close()
        env.close()

    rows = {k: np.asarray(v) for k, v in rows.items()}
    np.savez_compressed(a.output / 'neutral_states.npz', **rows)
    joint_state_report=joint_recorder.summarize(rows,settle)
    info['physx_joint_state']=joint_state_report
    info['unoperated_joint_state_passed']=joint_state_report['passed']

    def pose_delta(values, start_index=0):
        start = values[start_index, :7]
        x = values[start_index:, :7]
        angles = np.rad2deg((Rotation.from_quat(x[:, 3:7]) * Rotation.from_quat(start[3:7]).inv()).magnitude())
        return dict(
            max_translation_m=float(np.linalg.norm(x[:, :3] - start[:3], axis=1).max()),
            max_rotation_deg=float(angles.max()),
        )

    tol = info['predeclared_tolerances']
    pick_name = TASK_SPEC.pick_object
    dest_name = TASK_SPEC.destination_object
    distractors = tuple(n for n in TASK_OBJECTS if n not in (pick_name, dest_name))
    info['task_objects'] = {
        pick_name: pose_delta(rows[pick_name], settle),
        dest_name: pose_delta(rows[dest_name], settle),
    }
    info['distractors_after_settling'] = {
        name: pose_delta(rows[name], settle) for name in distractors
    }
    info['robot_max_joint_change_rad'] = float(np.abs(rows['q'] - rows['q'][0]).max())
    info['robot_hold_passed'] = bool(info['robot_max_joint_change_rad'] <= tol['robot_joint_rad'])
    info['task_objects_passed'] = bool(
        all(
            x['max_translation_m'] <= tol['task_translation_m'] and x['max_rotation_deg'] <= tol['task_rotation_deg']
            for x in info['task_objects'].values()
        )
    )
    info['distractors_passed'] = bool(
        all(
            x['max_translation_m'] <= tol['distractor_translation_m']
            and x['max_rotation_deg'] <= tol['distractor_rotation_deg']
            for x in info['distractors_after_settling'].values()
        )
    )

    articulated = []
    if joint_connected and rows['articulated_transforms'].ndim == 4:
        mats = rows['articulated_transforms']
        for j, path in enumerate(joint_connected):
            start = mats[0, j]
            delta = float(np.abs(mats[:, j] - start).max())
            # also translation of origin
            origins = mats[:, j, :3, 3]
            rot = []
            for i in range(len(mats)):
                r0 = np.asarray(start[:3, :3], dtype=float)
                r1 = np.asarray(mats[i, j, :3, :3], dtype=float)
                rot.append(np.rad2deg(Rotation.from_matrix(r1 @ r0.T).magnitude()))
            record = dict(
                path=path,
                max_matrix_difference=delta,
                max_translation_m=float(np.linalg.norm(origins - origins[0], axis=1).max()),
                max_rotation_deg=float(np.max(rot)),
            )
            record['passed'] = bool(
                record['max_translation_m'] <= tol['articulated_translation_m']
                and record['max_rotation_deg'] <= tol['articulated_rotation_deg']
            )
            articulated.append(record)
    info['articulated_bodies_from_reset'] = articulated
    info['unoperated_articulated_bodies_passed'] = bool(
        (not joint_connected) or (bool(articulated) and all(x['passed'] for x in articulated))
    )
    info['camera_capture_alignment'] = bool(
        np.all(rows['capture_physics_steps'][:, 0] == rows['capture_physics_steps'][:, 1])
    )
    info['passed'] = bool(
        info['task_objects_passed']
        and info['distractors_passed']
        and info['robot_hold_passed']
        and info['unoperated_articulated_bodies_passed']
        and info['camera_capture_alignment']
        and info['unoperated_joint_state_passed']
    )
    (a.output / 'neutral_hold.json').write_text(json.dumps(info, indent=2) + '\n')
    print(
        json.dumps(
            {
                k: info[k]
                for k in (
                    'passed',
                    'duration_seconds',
                    'frames',
                    'robot_hold_passed',
                    'task_objects_passed',
                    'distractors_passed',
                    'unoperated_articulated_bodies_passed',
                    'camera_capture_alignment',
                    'robot_max_joint_change_rad',
                    'task_objects',
                    'distractors_after_settling',
                )
            },
            indent=2,
        ),
        flush=True,
    )
    if not info['passed']:
        raise SystemExit(2)
