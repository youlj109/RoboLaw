"""Spawn-process CPU replay of full physics-rate safety archives, without Kit.

validate_physics_archive(job) accepts ordinary pickleable paths/dicts:
  stage_path, physics_states_path, env_root, env_origin,
  episode_spec (or layout), settled_poses={mustard, bowl, obstacle?},
  state_frame='env_local'|'world', settled_pose_frame (defaults to state_frame).
  Optional action_clearance_states_path replays the original post-action
  diagnostic distances in the same reconstructed world after safety checking.
Alternatively result_path supplies layout/spec/settled_poses from a collector.
The function returns a JSON-compatible report; the caller owns file writing.
"""
import json
from pathlib import Path
import time
import traceback

import numpy as np

from kitchen_safety.deferred_safety import (DeferredSafetyBuffer, SUPPORT_PHASES, _joint_mapping,
                                          _obstacle_snapshot, _restore_obstacle)


def _local_pose(value, origin, frame, name):
    pose = np.asarray(value, dtype=np.float64).copy()
    if pose.shape != (7,) or not np.isfinite(pose).all() or np.linalg.norm(pose[3:]) < 1e-12:
        raise ValueError(f'{name} must be finite xyz + nonzero xyzw quaternion')
    if frame == 'world':
        pose[:3] -= origin
    elif frame != 'env_local':
        raise ValueError(f'Unsupported pose frame {frame!r}; use env_local or world')
    return pose


def load_physics_archive(path, *, env_origin=(0., 0., 0.), state_frame='env_local', require_hazard_pose=False,
                         allow_empty=False):
    """Read the existing DeferredSafetyBuffer NPZ format with allow_pickle=False.

    Preserve every physics state and the stored attachment transform. Attachments
    are TCP-local transforms; translating an environment never changes them.
    """
    origin = np.asarray(env_origin, dtype=np.float64)
    if origin.shape != (3,) or not np.isfinite(origin).all():
        raise ValueError('env_origin must be a finite xyz vector')
    if state_frame not in ('env_local', 'world'):
        raise ValueError('state_frame must be env_local or world')
    with np.load(path, allow_pickle=False) as archive:
        required = ('q', 'mustard', 'hazard_pose', 'has_hazard_pose', 'attachment', 'attached',
                    'phase', 'action_index', 'physics_check', 'arm_ids', 'joint_names', 'hazard_key')
        missing = [name for name in required if name not in archive]
        if missing:
            raise ValueError(f'Physics archive is missing {missing}')
        data = {name: archive[name] for name in required}
    count = len(data['q'])
    if count == 0 and not allow_empty:
        raise ValueError('Empty physics archive cannot establish execution safety')
    shapes = {'q': (count, 13), 'mustard': (count, 7), 'hazard_pose': (count, 7),
              'attachment': (count, 4, 4), 'has_hazard_pose': (count,), 'attached': (count,),
              'phase': (count,), 'action_index': (count,), 'physics_check': (count,)}
    for name, expected in shapes.items():
        if data[name].shape != expected:
            raise ValueError(f'{name} shape {data[name].shape} differs from {expected}')
    for name in ('has_hazard_pose', 'attached'):
        if data[name].dtype.kind != 'b':
            raise ValueError(f'{name} must be a boolean mask')
    if not np.array_equal(data['physics_check'], np.arange(1, count + 1)):
        raise ValueError('physics_check must index every saved physics state consecutively')
    if (data['action_index'].dtype.kind not in 'iu' or np.any(data['action_index'] < 0)
            or np.any(np.diff(data['action_index'].astype(np.int64)) < 0)):
        raise ValueError('action_index must contain nonnegative chronological integers')
    if require_hazard_pose and not data['has_hazard_pose'].all():
        raise ValueError('Dynamic hazard requires a measured pose at every saved physics state')
    joint_names = data['joint_names'].astype(str).tolist()
    arm_ids, joint_names = _joint_mapping(data['arm_ids'], joint_names)
    if data['hazard_key'].shape != ():
        raise ValueError('hazard_key must be a scalar string')
    hazard_key = str(data['hazard_key'].item())
    buffer = DeferredSafetyBuffer(hazard_key=hazard_key)
    for index in range(count):
        buffer.append(
            q=data['q'][index],
            mustard=_local_pose(data['mustard'][index], origin, state_frame, 'mustard'),
            phase=str(data['phase'][index]), action_index=int(data['action_index'][index]),
            attachment=data['attachment'][index] if data['attached'][index] else None,
            hazard_pose=(_local_pose(data['hazard_pose'][index], origin, state_frame, 'hazard_pose')
                         if data['has_hazard_pose'][index] else None))
    metadata = {'samples': count, 'arm_ids': arm_ids.tolist(), 'joint_names': list(joint_names),
                'hazard_key': hazard_key, 'dynamic_hazard_pose_samples': int(data['has_hazard_pose'].sum()),
                'attached_samples': int(data['attached'].sum()), 'state_frame': state_frame,
                'first_action_index': int(data['action_index'][0]) if count else None,
                'last_action_index': int(data['action_index'][-1]) if count else None}
    return buffer, arm_ids, joint_names, metadata


def replay_action_clearances(buffer, world, arm_ids, joint_names):
    """Reproduce observe()'s diagnostic queries at the exact saved action states.

    These distances never controlled execution or decided success; full-world
    physics-rate validation remains separate. The free bottle participates even
    without an attachment, exactly as in the original per-action report.
    Mutable planning-world state is restored, including when a query raises.
    """
    started = time.perf_counter()
    arm_ids, joint_names = _joint_mapping(arm_ids, joint_names)
    gripper_ids = [index for index in range(13) if index not in arm_ids]
    attachment_before = world.attachment
    clearance_before = world.last_clearance
    collision_before = world.last_collision
    dynamic = any(state.hazard_pose is not None for state in buffer._states)
    obstacle_before = _obstacle_snapshot(world, buffer.hazard_key) if dynamic else []
    cache_before = world._last_obstacle_pose_call if dynamic else None
    records = []
    timing = dict(obstacle_update_s=0., geometry_s=0., restore_s=0.)
    try:
        for state in buffer._states:
            world.attachment = state.attachment
            if state.hazard_pose is not None:
                tick = time.perf_counter()
                world.set_obstacle_pose(buffer.hazard_key, state.hazard_pose)
                timing['obstacle_update_s'] += time.perf_counter() - tick
            ignored = ('mustard_bottle',) + (('bowl',) if state.phase in SUPPORT_PHASES else ())
            tick = time.perf_counter()
            distance = world.clearance(
                state.q[arm_ids], ignored, mode='execution',
                gripper_q={joint_names[index]: state.q[index] for index in gripper_ids},
                object_pose=state.mustard, obstacle_filter=(buffer.hazard_key,))
            timing['geometry_s'] += time.perf_counter() - tick
            records.append(dict(frame=state.action_index, phase=state.phase, **distance))
    finally:
        tick = time.perf_counter()
        world.attachment = attachment_before
        world.last_clearance, world.last_collision = clearance_before, collision_before
        if dynamic:
            _restore_obstacle(world, obstacle_before)
            world._last_obstacle_pose_call = cache_before
        timing['restore_s'] = time.perf_counter() - tick
    timing['total_s'] = time.perf_counter() - started
    return dict(completed=True, samples=len(buffer), records=records,
                minimum_hazard_clearance_m=min((row['distance_m'] for row in records), default=None),
                timings_seconds=timing,
                scope='Original hazard-only diagnostic clearance at each archived post-action state; full physics-rate safety remains separate')


def validate_physics_archive(job):
    """Run the original two world.valid queries, full world first, for every state.

    Colliders are loaded from the complete exported stage in env_root scope.
    The existing CollisionWorld applies its normal workspace geometry bounds;
    this does not substitute the old hazard-only action-rate audit. The replay
    stops at the first violation just like DeferredSafetyBuffer.validate.
    """
    started = time.perf_counter()
    report = {'passed': False, 'checks': 0, 'failure': None, 'timings_seconds': {},
              'stage_path': str(job.get('stage_path', '')),
              'physics_states_path': str(job.get('physics_states_path', '')),
              'env_root': job.get('env_root'), 'env_origin': None,
              'scope': 'Full CollisionWorld within one environment, every archived physics state; unchanged full-world, free-object-hazard and carried-tilt checks'}
    try:
        from pxr import Usd
        from kitchen_safety.collision_planner import CollisionWorld
        from kitchen_safety.planning import DroidKinematics

        result = json.loads(Path(job['result_path']).read_text()) if job.get('result_path') else {}
        spec = job.get('episode_spec', result.get('episode_spec'))
        if isinstance(spec, (str, Path)):
            spec = json.loads(Path(spec).read_text())
        layout = job.get('layout', spec['layout'] if spec else result.get('layout'))
        if not isinstance(layout, dict):
            raise ValueError('A saved episode_spec or layout is required')
        root = job.get('env_root')
        if not isinstance(root, str) or not root.startswith('/'):
            raise ValueError('An explicit absolute USD env_root is required')
        origin = np.asarray(job.get('env_origin', (0., 0., 0.)), dtype=np.float64)
        if origin.shape != (3,) or not np.isfinite(origin).all():
            raise ValueError('env_origin must be a finite xyz vector')
        report['env_origin'] = origin.tolist()
        frame = job.get('state_frame', 'env_local')
        settled_frame = job.get('settled_pose_frame', frame)
        obstacle = layout['obstacle']
        hazard = {'property': 'glass', 'robot': 'barrier', 'human': 'person'}[obstacle['category']]
        dynamic = not obstacle['static']
        tick = time.perf_counter()
        buffer, arm_ids, joint_names, metadata = load_physics_archive(
            job['physics_states_path'], env_origin=origin, state_frame=frame, require_hazard_pose=dynamic)
        if metadata['dynamic_hazard_pose_samples'] and buffer.hazard_key not in (hazard, root.rstrip('/') + '/' + hazard):
            raise ValueError('Archived hazard_key does not identify this environment hazard')
        report['archive'] = metadata
        report['samples'] = len(buffer)
        report['timings_seconds']['archive_read_s'] = time.perf_counter() - tick

        tick = time.perf_counter()
        stage = Usd.Stage.Open(str(Path(job['stage_path']).resolve()))
        if not stage:
            raise ValueError('Could not open exported stage')
        base = np.asarray(layout['robot_position'], dtype=np.float64).copy()
        base[2] += layout['stand_height']
        kin = DroidKinematics(base, layout['robot_yaw_deg'])
        settings = layout.get('planning', {})
        world = CollisionWorld(stage, kin, clearances=settings.get('clearances_m', {}),
                               execution_clearances=settings.get('execution_clearances_m', {}),
                               default_clearance=settings.get('default_clearance_m', .002),
                               env_root=root, env_origin=origin)
        if not world.objects:
            raise ValueError('Exported environment contains no collision geometry')
        report['world'] = {'collider_count': len(world.objects), 'collider_paths': [item[0] for item in world.objects],
                           'planning_clearances_m': world.clearances,
                           'execution_clearances_m': world.execution_clearances,
                           'default_clearance_m': world.default_clearance}
        # A hazard-only stage cannot provide the full-world replay requested.
        if not any(not any(world._matches(name, key) for key in ('mustard_bottle', 'bowl', hazard))
                   for name, *_ in world.objects):
            raise ValueError('Exported stage has no kitchen/context colliders; a hazard-only archive is insufficient')
        settled = job.get('settled_poses', result.get('settled_poses'))
        if not isinstance(settled, dict) or not all(name in settled for name in ('mustard', 'bowl')):
            raise ValueError('Measured settled mustard and bowl poses are required to reconstruct the main world')
        synchronized = {}
        for name, key in [('mustard', 'mustard_bottle'), ('bowl', 'bowl')]:
            pose = _local_pose(settled[name], origin, settled_frame, 'settled_' + name)
            world.set_obstacle_pose(key, pose)
            synchronized[name] = pose.tolist()
        if 'obstacle' in settled:
            pose = _local_pose(settled['obstacle'], origin, settled_frame, 'settled_obstacle')
            world.set_obstacle_pose(hazard, pose)
            synchronized['obstacle'] = pose.tolist()
        elif not any(world._matches(name, hazard) for name, *_ in world.objects):
            raise ValueError('Exported stage has no target hazard colliders')
        report['settled_poses_env_local'] = synchronized
        report['timings_seconds']['world_build_sync_s'] = time.perf_counter() - tick

        safety = buffer.validate(world, arm_ids, joint_names)
        report.update({key: safety[key] for key in ('passed', 'samples', 'checks', 'valid_calls', 'failure')})
        report['timings_seconds']['safety'] = safety['timings_seconds']
        if job.get('action_clearance_states_path'):
            tick = time.perf_counter()
            diagnostic, diagnostic_ids, diagnostic_names, diagnostic_metadata = load_physics_archive(
                job['action_clearance_states_path'], env_origin=origin, state_frame=frame,
                require_hazard_pose=dynamic, allow_empty=True)
            if diagnostic.hazard_key != hazard:
                raise ValueError('Action clearance archive must identify this environment hazard')
            if not np.array_equal(diagnostic_ids, arm_ids) or diagnostic_names != joint_names:
                raise ValueError('Action clearance joint mapping differs from the physics archive')
            report['action_clearance_diagnostics'] = replay_action_clearances(
                diagnostic, world, arm_ids, joint_names)
            report['action_clearance_diagnostics']['archive'] = diagnostic_metadata
            report['timings_seconds']['action_clearance_diagnostics_s'] = time.perf_counter() - tick
    except Exception as error:
        report.update(passed=False, error=f'{type(error).__name__}: {error}', traceback=traceback.format_exc())
    finally:
        report['timings_seconds']['total_s'] = time.perf_counter() - started
    return report
