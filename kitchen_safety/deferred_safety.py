"""Record physics-rate states and replay the collector's existing safety queries.

This moves checking out of the execution loop; it does not make unchecked
execution safe or reduce the amount of FCL work. A failed replay must discard
the episode. No simulator or GPU imports are needed to read the NPZ records.
"""
from dataclasses import dataclass
from pathlib import Path
import time

import numpy as np


HAZARDS = ('person', 'glass', 'plate', 'phone', 'barrier')
SUPPORT_PHASES = ('lower', 'release', 'retreat', 'settle')


@dataclass
class _PhysicsState:
    q: np.ndarray
    mustard: np.ndarray
    phase: str
    action_index: int
    attachment: object
    hazard_pose: object


def _copy_array(value, shape, name):
    result = np.asarray(value, dtype=np.float64)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f'{name} must be a finite array with shape {shape}')
    return result.copy()


def _joint_mapping(arm_ids, joint_names):
    names = tuple(joint_names)
    indices = np.asarray(arm_ids)
    if len(names) != 13 or len(set(names)) != 13:
        raise ValueError('joint_names must contain all 13 distinct articulation joint names')
    if (indices.shape != (7,) or not np.issubdtype(indices.dtype, np.integer)
            or len(set(indices.tolist())) != 7 or np.any(indices < 0) or np.any(indices >= 13)):
        raise ValueError('arm_ids must contain seven distinct integer indices into the full joint state')
    return indices, names


def _obstacle_snapshot(world, key):
    """Retain actual geometry, including a pose not represented by the pose cache.

CollisionWorld does not expose a root-pose getter. Its most recent pose call
may refer to a different root or alias, so that cache cannot restore geometry.
Only the colliders changed by set_obstacle_pose(key, ...) are copied here.
"""
    saved = []
    for index, item in enumerate(world.objects):
        name, obj, _, _ = item
        if world._matches(name, key):
            saved.append((index, item, np.array(obj.getRotation()).copy(),
                          np.array(obj.getTranslation()).copy(), world.low[index].copy(),
                          world.high[index].copy(), name in world.meshes, world.meshes.get(name)))
    return saved


def _restore_obstacle(world, saved):
    for index, item, rotation, translation, low, high, had_mesh, mesh in saved:
        name, obj, _, _ = item
        obj.setTransform(world.fcl.Transform(rotation, translation))
        world.objects[index] = item
        world.low[index] = low
        world.high[index] = high
        if had_mesh:
            world.meshes[name] = mesh
        else:
            world.meshes.pop(name, None)


class DeferredSafetyBuffer:
    """Append copied post-physics states; validate sequentially after execution.

``action_index`` is zero-based and may repeat for all eight substeps of an
action. ``mustard`` and optional ``hazard_pose`` are world xyz + xyzw poses.
``attachment`` is the same bottle-to-TCP 4x4 transform used by CollisionWorld,
or None for a free bottle. The world must not be shared by concurrent queries
while validate() is running. Validation stops at the first violation, matching
the online checker; the NPZ still contains every recorded physics state.
    """

    def __init__(self, *, hazard_key='glass'):
        self.hazard_key = str(hazard_key)
        self._states = []

    def __len__(self):
        return len(self._states)

    def append(self, *, q, mustard, phase, action_index, attachment=None, hazard_pose=None):
        """Copy the measured state before the simulator reuses its buffers."""
        if not isinstance(phase, str) or not phase:
            raise ValueError('phase must be a nonempty string')
        if not isinstance(action_index, (int, np.integer)) or action_index < 0:
            raise ValueError('action_index must be a nonnegative integer')
        q = _copy_array(q, (13,), 'q')
        mustard = _copy_array(mustard, (7,), 'mustard')
        if np.linalg.norm(mustard[3:]) < 1e-12:
            raise ValueError('mustard must have a nonzero xyzw quaternion')
        if attachment is not None:
            attachment = _copy_array(attachment, (4, 4), 'attachment')
        if hazard_pose is not None:
            hazard_pose = _copy_array(hazard_pose, (7,), 'hazard_pose')
            if np.linalg.norm(hazard_pose[3:]) < 1e-12:
                raise ValueError('hazard_pose must have a nonzero xyzw quaternion')
        self._states.append(_PhysicsState(q, mustard, phase, int(action_index), attachment, hazard_pose))

    def validate(self, world, arm_ids, joint_names):
        """Run unchanged world.valid execution queries and restore world state.

Returns passed, samples, checks, valid_calls, failure and timings_seconds.
``checks`` counts physics states visited, not individual world.valid calls.
Unexpected query errors propagate after restoration, rather than passing a
trajectory whose safety could not be evaluated. No clearances or tilt limits
are changed; the carried-bottle limit remains inside CollisionWorld.valid.
        """
        started = time.perf_counter()
        arm_ids, joint_names = _joint_mapping(arm_ids, joint_names)
        gripper_ids = [i for i in range(13) if i not in arm_ids]
        attachment_before = world.attachment
        collision_before = world.last_collision
        clearance_before = world.last_clearance
        has_dynamic_hazard = any(state.hazard_pose is not None for state in self._states)
        obstacle_before = _obstacle_snapshot(world, self.hazard_key) if has_dynamic_hazard else []
        cache_before = world._last_obstacle_pose_call if has_dynamic_hazard else None
        report = {'passed': True, 'samples': len(self), 'checks': 0, 'valid_calls': 0,
                  'failure': None, 'timings_seconds': {'obstacle_update_s': 0., 'geometry_s': 0.,
                                                     'restore_s': 0., 'validate_s': 0.}}
        timing = report['timings_seconds']
        try:
            for index, state in enumerate(self._states):
                world.attachment = state.attachment
                if state.hazard_pose is not None:
                    tick = time.perf_counter()
                    world.set_obstacle_pose(self.hazard_key, state.hazard_pose)
                    timing['obstacle_update_s'] += time.perf_counter() - tick
                ignored = ('mustard_bottle',) + (('bowl',) if state.phase in SUPPORT_PHASES else ())
                kwargs = dict(mode='execution',
                              gripper_q={joint_names[i]: state.q[i] for i in gripper_ids},
                              object_pose=state.mustard if state.attachment is not None else None)
                q_arm = state.q[arm_ids]
                report['checks'] += 1
                tick = time.perf_counter()
                try:
                    report['valid_calls'] += 1
                    if not world.valid(q_arm, ignored, **kwargs):
                        reason = world.last_collision
                        failure = dict(world.clearance(q_arm, ignored, **kwargs))
                        failure.update(geometry_valid=failure['valid'], valid=False, reason=reason,
                                       check='executed_constraint')
                    elif state.attachment is None:
                        # Intentional counter/bowl support remains excluded. The
                        # released bottle still participates in hazard queries.
                        kwargs['object_pose'] = state.mustard
                        report['valid_calls'] += 1
                        if not world.valid(q_arm, ignored, obstacle_filter=HAZARDS, **kwargs):
                            reason = world.last_collision
                            failure = dict(world.clearance(q_arm, ignored, obstacle_filter=HAZARDS, **kwargs))
                            failure.update(valid=False, reason=reason, check='free_object_hazard')
                        else:
                            failure = None
                    else:
                        failure = None
                finally:
                    timing['geometry_s'] += time.perf_counter() - tick
                if failure is not None:
                    failure.update(phase=state.phase, action_index=state.action_index,
                                   physics_check=index + 1, state_index=index)
                    report.update(passed=False, failure=failure)
                    break
        finally:
            tick = time.perf_counter()
            world.attachment = attachment_before
            if has_dynamic_hazard:
                _restore_obstacle(world, obstacle_before)
                world._last_obstacle_pose_call = cache_before
            world.last_collision = collision_before
            world.last_clearance = clearance_before
            timing['restore_s'] = time.perf_counter() - tick
            timing['validate_s'] = time.perf_counter() - started
        return report

    def write_npz(self, path, *, arm_ids, joint_names):
        """Save all physics states in uncompressed arrays readable without pickle.

Absent optional poses/transforms use zero-filled slots plus explicit boolean
masks. ``physics_check`` is one-based, matching online failure reports.
        """
        started = time.perf_counter()
        arm_ids, joint_names = _joint_mapping(arm_ids, joint_names)
        count = len(self)
        q = np.empty((count, 13), dtype=np.float64)
        mustard = np.empty((count, 7), dtype=np.float64)
        hazard = np.zeros((count, 7), dtype=np.float64)
        attachment = np.zeros((count, 4, 4), dtype=np.float64)
        has_hazard = np.zeros(count, dtype=bool)
        attached = np.zeros(count, dtype=bool)
        for index, state in enumerate(self._states):
            q[index], mustard[index] = state.q, state.mustard
            if state.hazard_pose is not None:
                hazard[index], has_hazard[index] = state.hazard_pose, True
            if state.attachment is not None:
                attachment[index], attached[index] = state.attachment, True
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('wb') as output:
            np.savez(output, q=q, mustard=mustard, hazard_pose=hazard, has_hazard_pose=has_hazard,
                     attachment=attachment, attached=attached,
                     phase=np.asarray([state.phase for state in self._states], dtype=str),
                     action_index=np.asarray([state.action_index for state in self._states], dtype=np.int64),
                     physics_check=np.arange(1, count + 1, dtype=np.int64),
                     arm_ids=arm_ids, joint_names=np.asarray(joint_names, dtype=str),
                     hazard_key=np.asarray(self.hazard_key), pose_convention=np.asarray('world xyz + xyzw'),
                     sampling=np.asarray('after each physics update, before the next physics step'))
        return {'path': str(path), 'samples': count, 'bytes': path.stat().st_size,
                'elapsed_seconds': time.perf_counter() - started}
