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
TASK_OBJECTS = ('lime', 'wooden_bowl', 'broccoli', 'sweet_potato', 'red_bell_pepper')

p = get_isaaclab_arena_cli_parser()
p.add_argument('--fixed-layout', type=Path, default=ROOT / 'configs/extensions/lightwheel_pick_place/fixed_layout.json')
p.add_argument('--output', type=Path, required=True)
p.add_argument('--duration-seconds', type=float, default=35.0,
               help='Observation window; default covers ~one successful B01 episode (~31.5s) with margin')
p.add_argument('--background-settling-seconds', type=float, default=5.0)
p.add_argument('--reference-episode', type=Path, default=None,
               help='Optional successful episode whose duration is used as a lower bound')
a = p.parse_args()
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
        fixed_layout_path=str(a.fixed_layout),
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
    cfg.observations.camera_obs = None
    cfg.sim.render_interval = cfg.decimation
    cfg.scene.robot_contacts = ContactSensorCfg(
        prim_path='{ENV_REGEX_NS}/Robot/.*', update_period=0, history_length=1
    )
    env = builder.make_registered(cfg, kwargs)
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

    info = dict(
        scope=(
            'Neutral hold: same fixed_layout reset and PhysX backend as B01 collects; '
            'arm command held at reset q with open gripper; no object/joint setters after reset. '
            'Task objects + distractors use RigidObject root states; kitchen joint-connected '
            'bodies use composed USD world transforms.'
        ),
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
    }
    try:
        for i in range(frames):
            before = int(e.sim.get_physics_step_count())
            writer.write('neutral_hold')
            after = int(e.sim.get_physics_step_count())
            if before != after:
                raise RuntimeError('Camera capture advanced physics')
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
    finally:
        writer.close()
        env.close()

    rows = {k: np.asarray(v) for k, v in rows.items()}
    np.savez_compressed(a.output / 'neutral_states.npz', **rows)

    def pose_delta(values, start_index=0):
        start = values[start_index, :7]
        x = values[start_index:, :7]
        angles = np.rad2deg((Rotation.from_quat(x[:, 3:7]) * Rotation.from_quat(start[3:7]).inv()).magnitude())
        return dict(
            max_translation_m=float(np.linalg.norm(x[:, :3] - start[:3], axis=1).max()),
            max_rotation_deg=float(angles.max()),
        )

    tol = info['predeclared_tolerances']
    info['task_objects'] = {
        'lime': pose_delta(rows['lime'], settle),
        'wooden_bowl': pose_delta(rows['wooden_bowl'], settle),
    }
    info['distractors_after_settling'] = {
        name: pose_delta(rows[name], settle) for name in ('broccoli', 'sweet_potato', 'red_bell_pepper')
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
