"""Per-environment physical grasp/place controllers for a shared batch step.

No function here steps or resets the simulator, renders a camera, or replays a
saved action sequence. prepare() runs the existing planning/phase logic until
one action is ready; observe() supplies its measured post-action state. The
caller owns the single batched env.step, physics-rate safety and RGB recording.

All Cartesian inputs use the environment-local benchmark frame. The planning
methods and phase sequence below were extracted from collect_kitchen.py; graph
fallback, upright repair, support release and physical thresholds are retained.
"""
from copy import deepcopy
import gc
import json
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.collection_presets import resolve_collector_options
from kitchen_safety.grasp_geometry import bottle_body_geometry, body_grasp_tcp_target
from kitchen_safety.planning_checks import check_planned_path
from kitchen_safety.augmentation_geometry import (
    check_initial_layout, probe_obstruction, relative_grasp_rotation)


class VectorizedEpisode:
    """One independently planned episode; no global simulator or mutable state.

    Required snapshot arrays match the single collector: q, dq, tcp,
    tcp_quat_xyzw, robot_body_pose_w, mustard, bowl, robot_contact_forces and,
    for movable hazards, hazard_pose. The caller also supplies arena_success
    and final_on_destination booleans from Arena's original predicates.
    prepare(state) -> float32[8] or None; observe(state) records that one action.
    Plan requests are synchronous and may be serviced serially across env IDs.
    """

    _STATE_KEYS = ('q', 'dq', 'tcp', 'tcp_quat_xyzw', 'robot_body_pose_w',
                   'mustard', 'bowl', 'robot_contact_forces', 'hazard_pose')
    _RENDER_STATE_KEYS = ('kitchen_root_state', 'kitchen_joint_pos', 'kitchen_joint_vel',
                          'kitchen_link_pose', 'kitchen_link_velocity',
                          'robot_body_velocity', 'hazard_state')

    def __init__(self, env_id, layout, options, world, kin, grasp_contacts, *,
                 arm_joint_ids, joint_names, nonfinger_ids, planner=None,
                 planner_factory=None, step_dt=1/15, episode_spec=None,
                 output=None, grasp_geometry=None):
        self.env_id = int(env_id)
        self.layout, self.episode_spec = deepcopy(layout), deepcopy(episode_spec)
        supplied = dict(vars(options) if hasattr(options, '__dict__') else options)
        supplied.setdefault('obstacle_id', episode_spec['obstacle_id'] if episode_spec else '')
        supplied.setdefault('variant', layout.get('obstacle', {}).get('category', 'base'))
        supplied.setdefault('seed', episode_spec['seeds']['planner'] if episode_spec else 0)
        base = dict(grasp_tilt_deg=None, grasp_yaw_deg=None, grasp_offset_z_m=0.,
                    grasp_offset_y_m=0., transport_bottle_height_m=None,
                    supported_release=False, transport_tilt_cap_deg=None,
                    placement_strategy='pose', placement_tcp_height_m=None)
        names = set(base)
        resolved = resolve_collector_options(supplied['obstacle_id'], base_defaults=base,
                                            cli_overrides={k: v for k, v in supplied.items() if k in names})
        self.args = SimpleNamespace(**{**dict(planner='curobo', reuse_planner=False, reuse_phase_planner=False,
                                              use_cuda_graph=False, defer_clearance_diagnostics=False,
                                              max_steps=0, jitter=0.), **supplied, **resolved})
        if self.args.planner != 'curobo':
            raise ValueError('Vectorized episodes require the existing cuRobo planner')
        if (self.args.grasp_tilt_deg is None) != (self.args.grasp_yaw_deg is None):
            raise ValueError('Grasp tilt and yaw must be specified together')
        self.world, self.kin, self.grasp_contacts = world, kin, grasp_contacts
        self.motion_planner, self.planner_factory = planner, planner_factory
        self.step_dt = float(step_dt)
        self.ids = np.asarray(arm_joint_ids, dtype=int)
        self.robot = SimpleNamespace(joint_names=list(joint_names))
        if self.ids.shape != (7,) or len(set(self.ids.tolist())) != 7 or len(joint_names) != 13:
            raise ValueError('Expected seven arm indices and all thirteen joint names')
        if np.any(self.ids < 0) or np.any(self.ids >= 13):
            raise ValueError('Arm indices must refer to the complete measured joint state')
        self.nonfinger_ids = np.asarray(nonfinger_ids, dtype=int)
        if self.nonfinger_ids.size == 0:
            raise ValueError('Arm/base contact body indices are required for the 5 N check')
        self.gripper_names = [name for i, name in enumerate(joint_names) if i not in self.ids]
        self.settings = self.layout.get('planning', {})
        self.dynamic_hazard = bool(self.args.obstacle_id and not self.layout['obstacle']['static'])
        self.planner_seed = int(episode_spec['seeds']['planner'] if episode_spec else self.args.seed or 0)
        self.planner_hazard_pose = None
        self.planner_reuse = dict(updates=0, rebuilds=0, update_seconds=0., rebuild_seconds=0.)
        self.planner_context_events, self.retired_planner_contexts = [], []
        self.grasp_geometry = deepcopy(grasp_geometry) if grasp_geometry is not None else bottle_body_geometry()
        self.body_contact_band = self.grasp_geometry['safe_contact_z_range_m']
        self.explicit_grasp_orientation = self.args.grasp_tilt_deg is not None
        self.grasp_offset_z = self.args.grasp_offset_z_m
        self.lower_target_z = self.args.placement_tcp_height_m
        self.grasp_rotation_delta = np.eye(3)
        self.output = Path(output) if output is not None else None
        self.q, self.grip, self.phase = None, 0., 'settle'
        self.records, self.plan_records, self.plan_candidate_failures = [], [], []
        self.obstruction_probes, self.clearance_records = [], []
        self.clearance_buffer = None
        if self.args.defer_clearance_diagnostics:
            from kitchen_safety.deferred_safety import DeferredSafetyBuffer
            hazard = {'human': 'person', 'property': 'glass', 'robot': 'barrier', 'base': 'glass'}[self.args.variant]
            self.clearance_buffer = DeferredSafetyBuffer(hazard_key=hazard)
        self.timings = dict(initialization_s=0., planning_s=0., physics_s=0.,
                            obstruction_probe_s=0., write_data_s=0., observe_contacts_s=0.,
                            observe_hazard_pose_s=0., observe_clearance_s=0., observe_clearance_capture_s=0.)
        self.terminal = {'terms': {'success': False}}
        self.safety_enabled = False
        self.done = False
        self._pending = self._runner = self._state = None
        self._support_now = False
        self._support_seen = False
        self._closed = False
        self.result = dict(env_id=self.env_id, variant=self.args.variant,
                           obstacle_id=self.args.obstacle_id, layout=self.layout,
                           obstacle=self.layout.get('obstacle'), seed=self.args.seed or 0,
                           jitter_m=self.args.jitter, success=False, planner='curobo',
                           physics_grasp=True, placement_strategy=self.args.placement_strategy)
        if episode_spec is not None:
            self.result.update(episode_spec=self.episode_spec, augmentation_eligible=False)
        self.result['requested_grasp'] = dict(
            explicit_orientation=self.explicit_grasp_orientation,
            tilt_deg=self.args.grasp_tilt_deg, yaw_deg=self.args.grasp_yaw_deg,
            automatic_preferred_orientation_deg=None if self.explicit_grasp_orientation else [45., 0.],
            offset_z_m=self.grasp_offset_z, offset_y_m=self.args.grasp_offset_y_m,
            transport_bottle_height_m=self.args.transport_bottle_height_m,
            supported_release=self.args.supported_release,
            transport_tilt_cap_deg=self.args.transport_tilt_cap_deg,
            placement_tcp_height_override_m=self.args.placement_tcp_height_m,
            offset_semantics='Bottle-local body TCP z; no longer an offset from a fixed world height',
            body_geometry=self.grasp_geometry, tcp_heights_world_m={},
            pregrasp_orientation_fallback=not self.explicit_grasp_orientation,
            transport_orientation_candidates='Original offsets relative to requested tilt/yaw')

    def _set_state(self, state):
        self._state = {key: np.asarray(state[key]).copy() for key in self._STATE_KEYS if key in state}
        for key in self._STATE_KEYS[:-1]:
            if key not in self._state:
                raise ValueError(f'Missing measured state: {key}')
        if self.dynamic_hazard and 'hazard_pose' not in self._state:
            raise ValueError('Movable hazard requires its measured local pose')
        self._state.update({key: np.asarray(state[key]).copy()
                            for key in self._RENDER_STATE_KEYS if key in state})
        if 'arena_success' in state:
            self.terminal['terms']['success'] |= bool(state['arena_success'])
        if 'final_on_destination' in state:
            self._support_now = bool(state['final_on_destination'])
            self._support_seen = True
        if 'destination_contact_force_matrix_w' in state:
            self.result['final_destination_contact_force_matrix_w_n'] = np.asarray(
                state['destination_contact_force_matrix_w']).tolist()

    def snapshot(self):
        if self._state is None:
            raise RuntimeError('A measured initial state is required')
        return {key: value.copy() for key, value in self._state.items()}

    def prepare(self, state=None):
        """Plan/advance until the next action; never advance physics here."""
        if self.done:
            return None
        if self._pending is not None:
            raise RuntimeError('observe() must consume the previous prepared action first')
        if state is not None:
            self._set_state(state)
        if self._state is None:
            raise ValueError('The first prepare() requires a measured initial snapshot')
        if self._runner is None:
            self.q = self._state['q'][self.ids].copy()
            self._runner = self._run()
        try:
            action = np.asarray(next(self._runner), dtype=np.float32)
            if action.shape != (8,) or not np.isfinite(action).all():
                raise RuntimeError('Controller produced an invalid absolute-joint/gripper action')
            self._pending = (self.phase, action.copy(), self.snapshot())
            return action.copy()
        except StopIteration:
            self.done = True
            self._finish()
            return None
        except Exception as error:
            self.fail(error)
            return None

    def observe(self, state):
        """Record the measured result of exactly one shared batch step."""
        if self._pending is None:
            raise RuntimeError('observe() requires one prepared action')
        phase, action, before = self._pending
        self._pending = None
        self._set_state(state)
        after = self.snapshot()
        self.records.append((phase, action, before, after))
        try:
            if bool(state.get('truncated', False)) or (
                    bool(state.get('terminated', False)) and not self.terminal['terms']['success']):
                raise RuntimeError('Environment terminated before the physical sequence completed')
            if action[-1] and phase in ('close', 'lift', 'transfer_front', 'transfer', 'above_bowl', 'lower'):
                tick = time.perf_counter()
                try:
                    self.grasp_contacts.sample(phase, len(self.records)-1, after['mustard'][:7])
                finally:
                    self.timings['observe_contacts_s'] += time.perf_counter() - tick
            if self.safety_enabled and self.args.variant != 'base':
                if self.dynamic_hazard:
                    # Keep the planning world current: lower-first planning also
                    # relies on this update, independently of diagnostic output.
                    tick = time.perf_counter()
                    try:
                        self.world.set_obstacle_pose('glass', after['hazard_pose'])
                    finally:
                        self.timings['observe_hazard_pose_s'] += time.perf_counter() - tick
                if self.clearance_buffer is not None:
                    tick = time.perf_counter()
                    self.clearance_buffer.append(
                        q=after['q'], mustard=after['mustard'][:7], phase=phase,
                        action_index=len(self.records)-1, attachment=self.world.attachment,
                        hazard_pose=after.get('hazard_pose') if self.dynamic_hazard else None)
                    self.timings['observe_clearance_capture_s'] += time.perf_counter() - tick
                else:
                    hazard = {'human': 'person', 'property': 'glass', 'robot': 'barrier'}[self.args.variant]
                    kwargs = self.geometry_args(after)
                    kwargs['object_pose'] = after['mustard'][:7]
                    tick = time.perf_counter()
                    try:
                        distance = self.world.clearance(after['q'][self.ids], self.ignored_objects(),
                                                        obstacle_filter=(hazard,), **kwargs)
                    finally:
                        self.timings['observe_clearance_s'] += time.perf_counter() - tick
                    self.clearance_records.append(dict(frame=len(self.records)-1, phase=phase, **distance))
        except Exception as error:
            self.fail(error)

    def write_clearance_archive(self, path):
        """Persist exact diagnostic query states for an independent CPU worker."""
        if self.clearance_buffer is None:
            raise ValueError('Deferred clearance diagnostics were not enabled')
        return self.clearance_buffer.write_npz(path, arm_ids=self.ids, joint_names=self.robot.joint_names)

    def finalize_clearance_diagnostics(self, report=None):
        """Apply worker results, or replay locally after physical execution."""
        if self.clearance_buffer is None:
            return
        if report is None:
            from kitchen_safety.vectorized_validation import replay_action_clearances
            report = replay_action_clearances(self.clearance_buffer, self.world, self.ids, self.robot.joint_names)
        expected = [(state.action_index, state.phase) for state in self.clearance_buffer._states]
        actual = [(row['frame'], row['phase']) for row in report.get('records', [])]
        if report.get('completed') is not True or actual != expected or report.get('samples') != len(expected):
            raise ValueError('Deferred clearance report does not cover the original diagnostic samples')
        self.clearance_records = deepcopy(report['records'])
        self.result['minimum_hazard_clearance_m'] = report['minimum_hazard_clearance_m']
        self.result['action_clearance_diagnostics'] = {key: value for key, value in report.items() if key != 'records'}
        self.result['action_clearance_diagnostics']['mode'] = 'deferred'
        self.timings['deferred_action_clearance_s'] = report['timings_seconds']['total_s']

    def fail(self, error, *, kind=None):
        """Stop only this environment; the caller can continue its other slots."""
        self.result.update(success=False, failure_phase=self.phase, error=repr(error))
        if kind is not None:
            self.result['failure_kind'] = kind
        else:
            self.result.setdefault('failure_kind',
                'planning_failed' if ('no valid approach' in str(error)
                                      or 'lower-first: no complete transport' in str(error))
                else 'physical_failed')
        self.done = True
        self._pending = None
        self._finish()

    def _step(self, command, gripper):
        if self.args.max_steps and len(self.records) >= self.args.max_steps:
            raise RuntimeError('Diagnostic step limit reached')
        yield np.r_[command, gripper].astype(np.float32)

    def new_motion_planner(self):
        if self.dynamic_hazard:
            self.planner_hazard_pose = self.snapshot()['hazard_pose'].copy()
        factory = self.planner_factory
        if factory is None:
            from kitchen_safety.curobo_backend import CuroboPlanner
            factory = CuroboPlanner
        options = dict(dt=self.step_dt, seed=self.planner_seed,
                       max_joint_speed=self.settings.get('max_joint_speed_rad_s', .4),
                       transport_object_padding_m=self.settings.get(
                           'transport_object_padding_m_by_variant', {}).get(self.args.variant, 0.))
        # Leave default factory calls unchanged, including custom CPU tests.
        if self.args.use_cuda_graph:
            options.update(use_cuda_graph=True, warmup=True)
        return factory(self.world, self.kin, **options)

    def _planner_context_snapshot(self):
        reporter = getattr(self.motion_planner, 'context_report', None)
        return deepcopy(reporter()) if callable(reporter) else {'timings_unavailable': True}

    def _replace_motion_planner(self, reason='initial', *, allow_reuse=False, phase=None):
        """Release the old context before invoking the unchanged constructor.

        cuRobo PrimitiveCollisionCost stores a bound method on itself as
        ``forward``; those cycles also retain collision checker/query buffers.
        Collect them at replacement boundaries, without changing CUDA allocator
        caching, planner seeding, or the factory's initial construction path.
        """
        started = time.perf_counter()
        event = dict(reason=reason, phase=phase or self.phase, action_index=len(self.records),
                     mode='fresh', use_cuda_graph=bool(self.args.use_cuda_graph))
        try:
            if allow_reuse and self.args.reuse_phase_planner and self.motion_planner is not None:
                current = self.snapshot()
                self.world.gripper_joint_position = float(current['q'][self.robot.joint_names.index('finger_joint')])
                if self.dynamic_hazard:
                    self.world.set_obstacle_pose('glass', current['hazard_pose'])
                reused = self.motion_planner.reset_for_new_phase(
                    self.ignored_objects(), self.grip, phase or self.phase,
                    hazard_key='glass' if self.dynamic_hazard else None)
                if reused:
                    if self.dynamic_hazard:
                        self.planner_hazard_pose = current['hazard_pose'].copy()
                    event['mode'] = 'reused'
                    return
                event['fallback_reason'] = 'unsupported_dynamic_hazard_geometry_or_topology'
            if self.motion_planner is not None:
                self.retired_planner_contexts.append(self._planner_context_snapshot())
                release_started = time.perf_counter()
                self.motion_planner = None
                gc.collect()
                event['release_gc_s'] = time.perf_counter()-release_started
            construct_started = time.perf_counter()
            self.motion_planner = self.new_motion_planner()
            event['constructor_with_optional_warmup_s'] = time.perf_counter()-construct_started
        except Exception as error:
            event['error'] = f'{type(error).__name__}: {error}'
            raise
        finally:
            event['elapsed_s'] = time.perf_counter()-started
            self.planner_context_events.append(event)

    def _save_placement_progress(self, placement):
        if self.output is not None:
            self.output.mkdir(parents=True, exist_ok=True)
            (self.output/'placement_progress.json').write_text(json.dumps(placement, indent=2))

    def _run(self):
        # The caller handles reset and pure-render appearance preparation first.
        # Initial settling retains the existing 30 action samples; safety starts
        # after the independently settled initial layout has been checked.
        for _ in range(30):
            yield from self._step(self.q, self.grip)
        if self.dynamic_hazard:
            self.world.set_obstacle_pose('glass', self.snapshot()['hazard_pose'])
        if self.episode_spec is not None:
            settled = self.snapshot()
            self.world.set_obstacle_pose('mustard_bottle', settled['mustard'][:7])
            self.world.set_obstacle_pose('bowl', settled['bowl'][:7])
            self.result['settled_initial_state'] = {k: v.tolist() for k, v in settled.items()}
            self.result['settled_poses'] = {k: settled[k][:7].tolist() for k in ('mustard', 'bowl')}
            if self.dynamic_hazard:
                self.result['settled_poses']['obstacle'] = settled['hazard_pose'].tolist()
            self.result['initial_layout_check'] = check_initial_layout(
                self.world, settled, self.layout, self.ids)
            if not self.result['initial_layout_check']['valid']:
                self.result['failure_kind'] = 'sampling_invalid'
                raise RuntimeError(f"Invalid sampled layout: {self.result['initial_layout_check']}")
            self.grasp_rotation_delta = Rotation.from_quat(settled['mustard'][3:7]).as_matrix()
        self.safety_enabled = True
        # Build against settled geometry, preserving the original first query.
        self._replace_motion_planner()
        yield from self._run_grasp_place()
        if not self._support_seen:
            raise RuntimeError('Final Arena support predicate was not supplied')
        self._finish_physical_checks()

    def _finish(self):
        if self._closed:
            return
        self._closed = True
        self.result.update(steps=len(self.records), plans=self.plan_records,
                           plan_candidate_failures=self.plan_candidate_failures,
                           planner_reuse=self.planner_reuse, timings_seconds=self.timings,
                           terminations=self.terminal['terms'])
        self.result['planner_contexts'] = {
            'reuse_phase_planner': bool(self.args.reuse_phase_planner),
            'use_cuda_graph': bool(self.args.use_cuda_graph),
            'events': deepcopy(self.planner_context_events),
            'contexts': deepcopy(self.retired_planner_contexts) +
                        ([self._planner_context_snapshot()] if self.motion_planner is not None else []),
            'semantics': 'Independent context per environment; phase reuse retains original solver parameters and full checks, without promising identical fresh-context trajectories.'}
        if self._state is not None:
            self.result['final_state'] = {k: v.tolist() for k, v in self._state.items()}
        self.result['minimum_hazard_clearance_m'] = min(
            (row['distance_m'] for row in self.clearance_records), default=None)
        if self.clearance_buffer is not None:
            self.result['action_clearance_diagnostics'] = {
                'mode': 'deferred', 'completed': False, 'samples': len(self.clearance_buffer)}
        if self.episode_spec is not None:
            confirmed = any(item['confirmed'] for item in self.obstruction_probes)
            self.result['obstruction_probe'] = dict(confirmed=confirmed, references=self.obstruction_probes,
                semantics='Geometric direct-reference queries only; never physically executed')
            self.result['augmentation_eligible'] = confirmed
            if self.result['success'] and not confirmed:
                self.result['failure_kind'] = 'obstruction_unconfirmed'
        carried = self.grasp_contacts.carried_region(self.body_contact_band)
        self.result['body_grasp_carried_region'] = carried
        if self.result['success'] and not carried['passed']:
            self.result.update(success=False, failure_kind='physical_failed',
                               error='Loaded finger contact moved outside the bottle-body region during carrying')
            outside = carried.get('outside_body_contacts', [])
            if outside:
                self.result.update(failure_phase=outside[0]['phase'],
                                   failure_action_index=outside[0]['action_index'])
        if not self.result['success'] and not self.result.get('error') and self.result.get('physical_failure_reasons'):
            outside = carried.get('outside_body_contacts', [])
            reasons = self.result['physical_failure_reasons']
            first = outside[0] if outside else min(reasons, key=lambda item: item['action_index'])
            self.result.update(failure_kind='physical_failed', failure_phase=first['phase'],
                               failure_action_index=first['action_index'],
                               error=('Loaded finger contact moved outside the bottle-body region during carrying'
                                      if outside else 'Final physical checks failed: ' +
                                      ', '.join(item['condition'] for item in reasons)))

    def ignored_objects(self):
        ignored = ('mustard_bottle',)
        if self.phase in ('lower', 'release', 'retreat', 'settle'):
            ignored += ('bowl',)
        return ignored

    def geometry_args(self, state):
        joints = dict(zip(self.robot.joint_names, state['q']))
        return dict(mode='execution', gripper_q={name: joints[name] for name in self.gripper_names}, object_pose=state['mustard'][:7] if self.world.attachment is not None else None)

    def _move(self, target, seconds=2):
        start = self.q.copy()
        seconds = max(seconds, 1.5 * np.max(np.abs(target - start)) / self.settings.get('max_joint_speed_rad_s', 0.4))
        for u in np.linspace(0, 1, max(2, round(seconds / self.step_dt))):
            yield from self._step(start + u * u * (3 - 2 * u) * (target - start), self.grip)
        self.q = target.copy()

    def _cart(self, xyz, seconds=2, *, object_target=False):
        current = self.snapshot()
        planning_started = time.perf_counter()
        self.q = current['q'][self.ids].copy()
        if self.dynamic_hazard:
            self.world.set_obstacle_pose('glass', current['hazard_pose'])
            if self.motion_planner is not None and (not np.array_equal(current['hazard_pose'], self.planner_hazard_pose)):
                updated = False
                if self.args.reuse_planner or self.args.reuse_phase_planner:
                    update_started = time.perf_counter()
                    updated = self.motion_planner.update_dynamic_obstacle('glass')
                    self.planner_reuse['update_seconds'] += time.perf_counter() - update_started
                if updated:
                    self.planner_hazard_pose = current['hazard_pose'].copy()
                    self.planner_reuse['updates'] += 1
                else:
                    rebuild_started = time.perf_counter()
                    self._replace_motion_planner(reason='dynamic_hazard_update_unsupported' if (self.args.reuse_planner or self.args.reuse_phase_planner)
                                                 else 'dynamic_hazard_update_reuse_disabled')
                    self.planner_reuse['rebuild_seconds'] += time.perf_counter() - rebuild_started
                    self.planner_reuse['rebuilds'] += 1
        ignore = self.ignored_objects()
        candidates = []
        last = None
        selected = None
        orientations = [(35, 0), (25, 0), (35, -30), (35, -45), (45, -30), (45, 0), (55, -45), (55, 0), (35, 30), (35, 45)]
        if self.explicit_grasp_orientation:
            if self.phase in ('transfer_front', 'transfer', 'above_bowl'):
                orientations = [(self.args.grasp_tilt_deg + tilt - 35, self.args.grasp_yaw_deg + yaw) for tilt, yaw in orientations]
            else:
                orientations = [(self.args.grasp_tilt_deg, self.args.grasp_yaw_deg)]
        else:
            orientations = [(tilt + 10.0, yaw) for tilt, yaw in orientations]

        def record_candidate_failure(tilt, yaw, stage, error, target):
            self.plan_candidate_failures.append({'phase': self.phase, 'backend': self.args.planner, 'tilt_deg': tilt, 'yaw_deg': yaw, 'failure_stage': stage, 'error': str(error), 'start_q_arm': self.q.tolist(), 'target_pose_world': target.tolist(), 'action_index': len(self.records)})
        for tilt, yaw in orientations:
            target = np.eye(4)
            target[:3, :3] = Rotation.from_euler('z', yaw, degrees=True).as_matrix() @ Rotation.from_euler('x', tilt, degrees=True).as_matrix() @ np.array([[0, -1, 0], [0, 0, 1], [-1, 0, 0]])
            if self.episode_spec and self.phase == 'pregrasp':
                target[:3, :3] = relative_grasp_rotation(target[:3, :3], self.grasp_rotation_delta)
            target[:3, 3] = xyz
            if object_target:
                if self.world.attachment is None:
                    raise RuntimeError('An object target requires a measured grasp attachment')
                target[:3, 3] -= target[:3, :3] @ self.world.attachment[:3, 3]
            failure_stage = 'motion_generation' if self.motion_planner is not None else 'ik'
            try:
                if self.motion_planner is not None:
                    self.world.gripper_joint_position = float(self.snapshot()['q'][self.robot.joint_names.index('finger_joint')])
                    trajectory = self.motion_planner.plan_pose(self.q, target, ignore, self.grip, phase=self.phase)
                    path = trajectory['positions']
                    if self.args.transport_tilt_cap_deg is not None and self.world.attachment is not None and (self.phase in ('transfer_front', 'transfer', 'above_bowl')):
                        from kitchen_safety.placement_planner import repair_upright_path
                        failure_stage = 'transport_tilt_margin'
                        cap = self.args.transport_tilt_cap_deg
                        repaired, correction = repair_upright_path(self.kin, self.world.attachment, path, soft_tilt_deg=cap - 5.0, cap_tilt_deg=cap, hard_tilt_deg=28.0)
                        if not correction['success'] or correction['max_edge_sample_tilt_deg'] > cap + 0.5:
                            raise RuntimeError(f'No bottle-tilt planning margin: {correction}')
                        speed = float(np.max(np.abs(np.diff(repaired, axis=0))) / self.step_dt)
                        acceleration = float(np.max(np.abs(np.diff(repaired, n=2, axis=0))) / self.step_dt ** 2) if len(repaired) > 2 else 0.0
                        dilation = max(1.0, speed / 0.2, np.sqrt(acceleration / 0.4))
                        for _ in range(10):
                            path = self.motion_planner.retime_path(repaired, source_dt=self.step_dt * dilation)
                            held_path = np.vstack([path[0], path, path[-1]])
                            measured_speed = float(np.max(np.abs(np.diff(held_path, axis=0))) / self.step_dt)
                            measured_acceleration = float(np.max(np.abs(np.diff(held_path, n=2, axis=0))) / self.step_dt ** 2)
                            if measured_speed <= 0.2 + 1e-08 and measured_acceleration <= 0.4 + 1e-08:
                                break
                            dilation *= 1.05 * max(1.0, measured_speed / 0.2, measured_acceleration / 0.4)
                        else:
                            raise RuntimeError('Corrected transport cannot meet its command timing limits')
                        check = self.motion_planner.validate_path(path, ignore, self.grip, phase=self.phase)
                        if not check['valid']:
                            raise RuntimeError(f'Corrected transport geometry rejected: {check}')
                        trajectory['positions'] = path
                        trajectory['metadata'].update(method='motion_gen_body_upright_repair', samples=len(path), upright_repair=correction, corrected_path_validation=check, retiming_dilation=dilation, transport_tilt_cap_deg=cap, corrected_max_joint_speed_rad_s=measured_speed, corrected_max_joint_acceleration_rad_s2=measured_acceleration)
                    failure_stage = 'independent_path_geometry'
                    if not check_planned_path(self.world, path, ignore):
                        raise RuntimeError(f'Independent geometry check rejected cuRobo path: {self.world.last_collision}')
                    if self.phase == 'above_bowl' and self.settings.get('transfer_mode_by_variant', {}).get(self.args.variant) == 'goal':
                        lower_target = self.kin.fk(path[-1])
                        lower_target[:2, 3] = target[:2, 3]
                        lower_target[2, 3] = self.lower_target_z
                        if self.args.supported_release:
                            from kitchen_safety.placement_geometry import release_lowering_target
                            hover_pose = self.kin.fk(path[-1])
                            predicted_bottle = hover_pose @ self.world.attachment
                            predicted_pose = np.r_[predicted_bottle[:3, 3], Rotation.from_matrix(predicted_bottle[:3, :3]).as_quat()]
                            try:
                                support_target, support = release_lowering_target(hover_pose[:3, 3], predicted_pose, current['bowl'][:7])
                            except ValueError as error:
                                raise RuntimeError(f'Unsupported placement candidate: {error}') from error
                            lower_target[:3, 3] = support_target
                            trajectory['metadata']['placement_lookahead_support'] = support
                        lower_ignore = tuple(dict.fromkeys((*ignore, 'bowl')))
                        failure_stage = 'placement_lookahead'
                        placement = self.motion_planner.plan_pose(path[-1], lower_target, lower_ignore, self.grip, phase='lower')
                        lower_path = placement['positions']
                        failure_stage = 'placement_lookahead_geometry'
                        if not check_planned_path(self.world, lower_path, lower_ignore):
                            raise RuntimeError(f'Placement lookahead rejected transport goal: {self.world.last_collision}')
                        trajectory['metadata']['placement_lookahead_passed'] = True
                        trajectory['metadata']['placement_lookahead_target_xyz_world_m'] = lower_target[:3, 3].tolist()
                    selected = (path, trajectory.get('effective_goal_pose', target), tilt, yaw, trajectory)
                    break
                goal = self.kin.ik(target, self.q, attempts=4)
                if not self.world.valid(goal, ignore):
                    last = f'goal clearance {self.world.last_collision}'
                    record_candidate_failure(tilt, yaw, 'goal_clearance', last, target)
                    continue
                candidates.append((goal, target, tilt, yaw))
                if self.world.edge(self.q, goal, ignore):
                    selected = ([self.q.copy(), goal], target, tilt, yaw, None)
                    break
                record_candidate_failure(tilt, yaw, 'direct_edge_geometry', self.world.last_collision, target)
            except RuntimeError as ex:
                last = str(ex)
                record_candidate_failure(tilt, yaw, failure_stage, last, target)
                print('PLAN_RETRY', self.phase, tilt, yaw, last, flush=True)
        if selected is None and self.motion_planner is None:
            for goal, target, tilt, yaw in candidates:
                try:
                    selected = (self.world.plan(self.q, goal, ignore, seed=self.planner_seed, max_iterations=3000), target, tilt, yaw, None)
                    break
                except RuntimeError as ex:
                    last = str(ex)
                    record_candidate_failure(tilt, yaw, 'rrt', last, target)
        if selected is None:
            self.timings['planning_s'] += time.perf_counter() - planning_started
            raise RuntimeError(f'{self.phase}: no valid approach: {last}')
        path, target, tilt, yaw, trajectory = selected
        self.timings['planning_s'] += time.perf_counter() - planning_started
        if self.episode_spec and (not any((item['confirmed'] for item in self.obstruction_probes))) and (self.args.variant == 'robot' or self.phase in ('transfer_front', 'transfer', 'above_bowl')):
            probe_started = time.perf_counter()
            self.obstruction_probes.append(probe_obstruction(self.world, self.motion_planner, path[0], path[-1], ignore, self.grip, self.phase, {'property': 'glass', 'human': 'person', 'robot': 'barrier'}[self.args.variant]))
            print('OBSTRUCTION_PROBE', self.phase, 'confirmed', self.obstruction_probes[-1]['confirmed'], 'reference', self.obstruction_probes[-1].get('reference'), flush=True)
            self.timings['obstruction_probe_s'] += time.perf_counter() - probe_started
        print('PLAN', self.phase, self.args.planner, 'tilt', tilt, 'yaw', yaw, 'waypoints', len(path), flush=True)
        self.plan_records.append({**(trajectory.get('metadata', {}) if trajectory else {}), 'phase': self.phase, 'backend': self.args.planner, 'tilt_deg': tilt, 'yaw_deg': yaw, 'samples': len(path), 'orientation_angles_semantics': 'Requested candidate; local Cartesian phases hold measured orientation', 'effective_goal_pose_world': target.tolist()})
        if trajectory is None:
            for point in path[1:]:
                yield from self._move(point, seconds / (len(path) - 1))
        else:
            for point in path[1:]:
                yield from self._step(point, self.grip)
            self.q = np.asarray(path[-1]).copy()
        for _ in range(12):
            yield from self._step(self.q, self.grip)
        state = self.snapshot()
        if self.world.attachment is not None:
            predicted = self.kin.fk(state['q'][self.ids]) @ self.world.attachment
            if np.linalg.norm(state['mustard'][:3] - predicted[:3, 3]) > 0.04:
                raise RuntimeError('Carried bottle slipped from the planned grasp')
        error = np.linalg.norm(state['tcp'] - target[:3, 3])
        if error > self.settings.get('tracking_tolerance_m', 0.015):
            raise RuntimeError(f'Execution tracking error: {error:.3f} m')
        self.q = state['q'][self.ids].copy()

    def _lower_first_transport(self, lower_xyz):
        """Plan from the live grasp, checking all candidate columns and route hints."""
        from kitchen_safety.placement_planner import (iter_placement_candidates,
            repair_upright_path, get_transport_guides, get_placement_seed_configs)
        from kitchen_safety.guided_transport import (
            replan_guided_suffix, resolve_guided_transport_phases, transport_command_segments)
        current = self.snapshot()
        self.q = current['q'][self.ids].copy()
        bottle_pose = np.eye(4)
        bottle_pose[:3, :3] = Rotation.from_quat(current['mustard'][3:7]).as_matrix()
        bottle_pose[:3, 3] = current['mustard'][:3]
        self.world.attachment = np.linalg.inv(self.kin.fk(self.q)) @ bottle_pose
        self.world.gripper_joint_position = float(current['q'][self.robot.joint_names.index('finger_joint')])
        self._replace_motion_planner(reason='lower_first_transport', allow_reuse=True, phase='above_bowl')
        hover_z = float(lower_xyz[2] + 0.03)
        diagnostics = {}
        placement = dict(strategy='lower-first', hover_height_m=hover_z, lower_xyz_world_m=np.asarray(lower_xyz, dtype=float).tolist(), attachment_measured_after_lift=self.world.attachment.tolist(), seed_semantics='Known joint configurations are IK initial guesses only; paths are regenerated in the live scene', candidates=[], search=diagnostics)
        self.result['placement'] = placement
        self.result['requested_grasp']['tcp_heights_world_m'].update(above_bowl=hover_z, retreat=hover_z)

        def save_placement_progress():
            self._save_placement_progress(placement)
        placement['physical_start'] = {key: value.tolist() for key, value in current.items()}
        save_placement_progress()
        ignore = ('mustard_bottle',)
        lower_ignore = (*ignore, 'bowl')
        if self.args.variant == 'human':
            # An exact-endpoint route cannot repair an endpoint rejected by
            # the same original transport query. Check one state, not a
            # straight start-to-goal connection, before spending solver time.
            placement['transport_start_validation'] = self.motion_planner.validate_path(
                self.q[None], ignore, self.grip, phase='above_bowl')
            if not placement['transport_start_validation']['valid']:
                placement['stop_reason'] = 'transport_start_invalid'
                save_placement_progress()
                raise RuntimeError('Actual transport start failed existing cuRobo/FCL validation')
            save_placement_progress()
        def apply_transport_tilt_margin(trajectory):
            # The explicit cap applies to every lower-first transport source,
            # including an otherwise valid cuRobo path and guide/RRT fallbacks.
            # No-cap runs retain their existing geometry and timing unchanged.
            cap = self.args.transport_tilt_cap_deg
            if cap is None:
                return trajectory
            repaired, correction = repair_upright_path(
                self.kin, self.world.attachment, trajectory['positions'],
                soft_tilt_deg=cap-5., cap_tilt_deg=cap, hard_tilt_deg=28.)
            if (not correction['success']
                    or correction['max_edge_sample_tilt_deg'] > cap + .5
                    or not correction['start_exactly_unchanged']
                    or not correction['end_exactly_unchanged']):
                raise RuntimeError(f'No bottle-tilt planning margin: {correction}')
            # Reject colliding IK corrections before potentially expensive
            # time dilation; timing cannot repair their collision geometry.
            repaired_check = self.motion_planner.validate_path(
                repaired, ignore, self.grip, phase='above_bowl')
            if not repaired_check['valid']:
                raise RuntimeError(f'Corrected transport geometry rejected before retiming: {repaired_check}')
            speed = float(np.max(np.abs(np.diff(repaired, axis=0))) / self.step_dt)
            acceleration = (float(np.max(np.abs(np.diff(repaired, n=2, axis=0)))
                                  / self.step_dt**2) if len(repaired) > 2 else 0.)
            dilation = max(1., speed/.2, np.sqrt(acceleration/.4))
            for _ in range(10):
                path = self.motion_planner.retime_path(repaired, source_dt=self.step_dt*dilation)
                held_path = np.vstack([path[0], path, path[-1]])
                measured_speed = float(np.max(np.abs(np.diff(held_path, axis=0))) / self.step_dt)
                measured_acceleration = float(np.max(np.abs(np.diff(held_path, n=2, axis=0)))
                                              / self.step_dt**2)
                if measured_speed <= .2 + 1e-8 and measured_acceleration <= .4 + 1e-8:
                    break
                dilation *= 1.05 * max(1., measured_speed/.2, measured_acceleration/.4)
            else:
                raise RuntimeError('Corrected transport cannot meet its command timing limits')
            check = self.motion_planner.validate_path(path, ignore, self.grip, phase='above_bowl')
            if not check['valid']:
                raise RuntimeError(f'Corrected transport geometry rejected: {check}')
            if not check_planned_path(self.world, path, ignore):
                raise RuntimeError(f'Corrected transport independent geometry rejected: {self.world.last_collision}')
            trajectory['positions'] = path
            trajectory['metadata'].update(
                method=trajectory['metadata'].get('method', 'lower_first_transport') + '_body_upright_repair',
                samples=len(path), upright_repair=correction,
                corrected_path_validation=check, repaired_geometry_validation=repaired_check,
                retiming_dilation=dilation,
                transport_tilt_cap_deg=cap,
                corrected_max_joint_speed_rad_s=measured_speed,
                corrected_max_joint_acceleration_rad_s2=measured_acceleration)
            return trajectory

        selected = None
        for index, candidate in enumerate(iter_placement_candidates(
                self.world, self.kin, self.q, lower_xyz, hover_z, seed=self.args.seed or 0,
                obstacle_id=self.args.obstacle_id,
                seed_configs=get_placement_seed_configs(self.args.obstacle_id),
                ignore_lower=lower_ignore,
                ignore_hover=ignore, diagnostics=diagnostics, max_candidates=8,
                bowl_pose=current['bowl'][:7] if self.args.supported_release else None,
                preferred_tilt_deg=self.args.grasp_tilt_deg,
                max_ik_attempts=528 if self.args.supported_release else 80)):
            trial = dict(index=index, metadata=candidate['metadata'], tilt_deg=float(candidate['tilt']), yaw_deg=float(candidate['yaw']), attempts=[], above_q_arm=candidate['above_q'].tolist(), lower_q_arm=candidate['lower_q'].tolist())
            placement['candidates'].append(trial)
            print('PLACEMENT_COLUMN', index, trial['tilt_deg'], trial['yaw_deg'], flush=True)
            if self.args.variant == 'human':
                trial['transport_goal_validation'] = self.motion_planner.validate_path(
                    np.asarray(candidate['above_q'])[None], ignore, self.grip, phase='above_bowl')
                save_placement_progress()
                if not trial['transport_goal_validation']['valid']:
                    trial['rejection_reason'] = 'transport_goal_invalid'
                    save_placement_progress()
                    print('PLACEMENT_REJECT', index, 'transport_goal', flush=True)
                    continue
            lower_check = self.motion_planner.validate_path(candidate['lower_positions'], lower_ignore, self.grip, phase='lower')
            trial['lower_validation'] = lower_check
            save_placement_progress()
            if not lower_check['valid']:
                print('PLACEMENT_REJECT', index, 'lower', len(lower_check['curobo_failures']), len(lower_check['fcl_failures']), flush=True)
                continue
            attachment, measured_grip = (self.world.attachment, self.world.gripper_joint_position)
            try:
                self.world.attachment = None
                self.world.gripper_joint_position = 0.0
                trial['retreat_validation'] = self.motion_planner.validate_path(candidate['lower_positions'][::-1], lower_ignore, 0, phase='retreat')
            finally:
                self.world.attachment, self.world.gripper_joint_position = (attachment, measured_grip)
            save_placement_progress()
            if not trial['retreat_validation']['valid']:
                print('PLACEMENT_REJECT', index, 'retreat', flush=True)
                continue
            trajectory = None
            try:
                trajectory = self.motion_planner.plan_joint_goal(self.q, candidate['above_q'], ignore, self.grip, phase='above_bowl')
                trajectory = apply_transport_tilt_margin(trajectory)
                check = self.motion_planner.validate_path(trajectory['positions'], ignore, self.grip, phase='above_bowl')
                trial['attempts'].append(dict(method='curobo_joint_goal', validation=check))
                save_placement_progress()
                if not check['valid']:
                    raw = trajectory.get('raw_positions', trajectory['positions'])
                    repaired, repair_report = repair_upright_path(self.kin, self.world.attachment, raw)
                    repaired = self.motion_planner.retime_path_smooth(repaired)
                    repair_check = self.motion_planner.validate_path(repaired, ignore, self.grip, phase='above_bowl')
                    trial['attempts'].append(dict(method='upright_repair', repair=repair_report, validation=repair_check))
                    save_placement_progress()
                    if repair_check['valid'] and repair_report['start_exactly_unchanged'] and repair_report['end_exactly_unchanged']:
                        trajectory['positions'] = repaired
                        trajectory['metadata'] = {**trajectory['metadata'], 'method': 'joint_goal_upright_repair', 'samples': len(repaired), 'upright_repair': repair_report, 'max_joint_speed_rad_s': 0.2, 'max_joint_acceleration_rad_s2': 0.4, 'timing': 'quintic_rest_to_rest', 'soft_repair_complete': repair_report['success'], 'final_hard_constraints_valid': True}
                        trajectory = apply_transport_tilt_margin(trajectory)
                    else:
                        trajectory = None
            except RuntimeError as ex:
                trajectory = None
                trial['attempts'].append(dict(method='curobo_joint_goal', error=str(ex)))
                save_placement_progress()
                print('PLACEMENT_TRANSPORT_ERROR', index, str(ex), flush=True)
            if trajectory is None:
                for guide in get_transport_guides(self.args.obstacle_id, candidate):
                    try:
                        knots = np.vstack([self.q, guide['positions'], candidate['above_q']])
                        if any(((self.kin.fk(point) @ self.world.attachment)[2, 2] < np.cos(np.deg2rad(28.0)) for point in knots[1:-1])):
                            knots, hint_repair = repair_upright_path(self.kin, self.world.attachment, knots)
                            guide['metadata']['attachment_upright_repair'] = hint_repair
                        connected = [self.q.copy()]
                        for leg, (start, goal) in enumerate(zip(knots[:-1], knots[1:])):
                            try:
                                segment = self.world.plan(start, goal, ignore, seed=(self.args.seed or 0) + leg, max_iterations=300)
                            except RuntimeError as connection_error:
                                if self.args.variant != 'human' or not self.args.supported_release:
                                    raise
                                # A sampled person can invalidate an internal
                                # hint. Keep only a newly checked prefix and
                                # replan to the actual goal; the invalid hint
                                # is never accepted as an execution waypoint.
                                unresolved = np.vstack([connected, knots[leg+1:]])
                                failed_path = self.motion_planner.retime_path_smooth(unresolved)
                                failed_check = self.motion_planner.validate_path(
                                    failed_path, ignore, self.grip, phase='above_bowl')
                                suffix_report = dict(guide_connection_error=str(connection_error),
                                    guide_connection_failed_leg=leg)
                                trial['attempts'].append(suffix_report)
                                trajectory = replan_guided_suffix(
                                    self.motion_planner, self.world, failed_path, unresolved,
                                    failed_check, candidate['above_q'], ignore, self.grip,
                                    dt=self.step_dt, apply_tilt_margin=apply_transport_tilt_margin,
                                    guide_metadata=guide['metadata'], report=suffix_report,
                                    on_progress=save_placement_progress)
                                if trajectory is None:
                                    raise
                                break
                            connected.extend(segment[1:])
                        if trajectory is not None:
                            break
                        path = self.motion_planner.retime_path_smooth(connected)
                        check = self.motion_planner.validate_path(path, ignore, self.grip, phase='above_bowl')
                        guide['metadata']['current_state_validated'] = bool(check['valid'])
                        trial['attempts'].append(dict(method='guided_fcl_path', guide=guide['metadata'], validation=check))
                        save_placement_progress()
                        if check['valid']:
                            trajectory = dict(positions=path, metadata=dict(method='guided_fcl_path_curobo_checked', samples=len(path), dt=self.step_dt, guide=guide['metadata'], max_joint_speed_rad_s=0.2, max_joint_acceleration_rad_s2=0.4, timing='quintic_rest_to_rest'))
                            trajectory = apply_transport_tilt_margin(trajectory)
                            break
                        if self.args.variant == 'human' and self.args.supported_release:
                            suffix_report = {}
                            trial['attempts'].append(suffix_report)
                            trajectory = replan_guided_suffix(
                                self.motion_planner, self.world, path, connected, check,
                                candidate['above_q'], ignore, self.grip, dt=self.step_dt,
                                apply_tilt_margin=apply_transport_tilt_margin,
                                guide_metadata=guide['metadata'], report=suffix_report,
                                on_progress=save_placement_progress)
                            if trajectory is not None:
                                break
                    except RuntimeError as ex:
                        trajectory = None
                        trial['attempts'].append(dict(method='guided_fcl_path', guide=guide['metadata'], error=str(ex)))
                        save_placement_progress()
            if trajectory is None:
                try:
                    print('PLACEMENT_RRT', index, flush=True)
                    raw = self.world.plan(self.q, candidate['above_q'], ignore, seed=self.args.seed or 0, max_iterations=1000)
                    path = self.motion_planner.retime_path_smooth(raw)
                    check = self.motion_planner.validate_path(path, ignore, self.grip, phase='above_bowl')
                    trial['attempts'].append(dict(method='fcl_rrt_connect', validation=check))
                    if check['valid']:
                        trajectory = dict(positions=path, metadata=dict(method='fcl_rrt_connect_curobo_checked', samples=len(path), dt=self.step_dt, max_joint_speed_rad_s=0.2, max_joint_acceleration_rad_s2=0.4, timing='quintic_rest_to_rest'))
                        trajectory = apply_transport_tilt_margin(trajectory)
                except RuntimeError as ex:
                    trajectory = None
                    trial['attempts'].append(dict(method='fcl_rrt_connect', error=str(ex)))
                save_placement_progress()
            if trajectory is not None:
                selected = (candidate, trajectory)
                placement['selected_candidate'] = index
                trial['metadata']['transport_validated'] = True
                diagnostics.update(transport_validated=True, stop_reason='selected')
                save_placement_progress()
                break
        if selected is None:
            raise RuntimeError('lower-first: no complete transport, placement and open-gripper retreat route')
        candidate, trajectory = selected
        if self.args.supported_release:
            placement['nominal_lower_xyz_world_m'] = placement['lower_xyz_world_m']
            placement['lower_xyz_world_m'] = candidate['lower_pose'][:3, 3].tolist()
            placement['hover_height_m'] = float(candidate['above_pose'][2, 3])
            placement['supported_release'] = candidate['metadata']['supported_release']
            self.result['requested_grasp']['tcp_heights_world_m'].update(
                above_bowl=placement['hover_height_m'], retreat=placement['hover_height_m'],
                lower_and_placement_lookahead=float(candidate['lower_pose'][2, 3]))
        path = trajectory['positions']
        phase_mapping = None
        if (self.args.variant == 'human'
                and trajectory['metadata'].get('guide', {}).get('phase_boundaries')):
            phase_mapping = resolve_guided_transport_phases(trajectory, self.kin, self.world.attachment)
            placement['transport_phase_mapping'] = phase_mapping
        if self.episode_spec and not any(item['confirmed'] for item in self.obstruction_probes):
            probe_started = time.perf_counter()
            self.obstruction_probes.append(probe_obstruction(
                self.world, self.motion_planner, path[0], path[-1], ignore, self.grip, self.phase,
                {'property': 'glass', 'human': 'person', 'robot': 'barrier'}[self.args.variant]))
            print('OBSTRUCTION_PROBE', self.phase, 'confirmed', self.obstruction_probes[-1]['confirmed'],
                  'reference', self.obstruction_probes[-1].get('reference'), flush=True)
            self.timings['obstruction_probe_s'] += time.perf_counter() - probe_started
        for segment in transport_command_segments(phase_mapping, len(path), 0):
            probe_started = time.perf_counter()
            probe = probe_obstruction(
                self.world, self.motion_planner, path[segment['path_start_index']],
                path[segment['path_end_index']], ignore, self.grip, segment['constraint_phase'], 'person')
            probe.update(phase=segment['phase'], constraint_phase=segment['constraint_phase'],
                         source='declared_guide_transport_phase',
                         selected_path_range=[segment['path_start_index'], segment['path_end_index']])
            probe['endpoint_provenance'].update(
                start='selected final path phase start; source-defined boundary for later phases',
                goal='selected final path phase end; source-defined boundary or actual candidate goal')
            self.obstruction_probes.append(probe)
            self.timings['obstruction_probe_s'] += time.perf_counter() - probe_started
            print('OBSTRUCTION_PROBE', segment['phase'], 'confirmed', probe['confirmed'],
                  'reference', probe.get('reference'), flush=True)
        placement['selected_transport'] = dict(positions=np.asarray(path).tolist(), dt=self.step_dt, metadata=trajectory['metadata'])
        save_placement_progress()
        previous_command = np.asarray(self.records[-1][1][:7], dtype=float)
        handoff = self.motion_planner.retime_path([previous_command, path[0]])
        handoff_check = self.motion_planner.validate_path(handoff, ignore, self.grip, phase='above_bowl')
        placement['command_handoff_validation'] = handoff_check
        if not handoff_check['valid']:
            raise RuntimeError('Transport command handoff violates the original planning constraints')
        commands = np.vstack([handoff[1:], path[1:]])
        phase_segments = transport_command_segments(phase_mapping, len(path), len(handoff)-1)
        plan_record = {**trajectory['metadata'], 'phase': self.phase, 'backend': 'curobo', 'tilt_deg': candidate['tilt'], 'yaw_deg': candidate['yaw'], 'placement_lookahead_passed': True, 'retreat_lookahead_passed': True, 'effective_goal_pose_world': candidate['above_pose'].tolist(), 'command_handoff_samples': len(handoff) - 1, 'executed_command_samples': len(commands)}
        if phase_segments:
            placement['execution_phase_segments'] = phase_segments
            placement['transport_first_action_index'] = len(self.records)
            for segment in phase_segments:
                self.plan_records.append({**plan_record, 'phase': segment['phase'],
                    'planning_scope': 'shared_lower_first_transport',
                    'phase_path_range': [segment['path_start_index'], segment['path_end_index']],
                    'samples': segment['path_end_index']-segment['path_start_index']+1,
                    'effective_goal_pose_world': self.kin.fk(path[segment['path_end_index']]).tolist(),
                    'command_handoff_samples': len(handoff)-1 if segment['path_start_index'] == 0 else 0,
                    'executed_command_samples': segment['command_end_index']-segment['command_start_index']})
            save_placement_progress()
        else:
            self.plan_records.append(plan_record)
        print('PLAN', self.phase, trajectory['metadata']['method'], 'waypoints', len(commands), flush=True)
        segment_index = 0
        for command_index, point in enumerate(commands):
            if phase_segments:
                while command_index >= phase_segments[segment_index]['command_end_index']:
                    segment_index += 1
                self.phase = phase_segments[segment_index]['phase']
            yield from self._step(point, self.grip)
        self.q = np.asarray(path[-1]).copy()
        for _ in range(12):
            yield from self._step(self.q, self.grip)
        state = self.snapshot()
        predicted = self.kin.fk(state['q'][self.ids]) @ self.world.attachment
        if np.linalg.norm(state['mustard'][:3] - predicted[:3, 3]) > 0.04:
            raise RuntimeError('Carried bottle slipped from the planned grasp')
        if np.linalg.norm(state['tcp'] - candidate['above_pose'][:3, 3]) > self.settings.get('tracking_tolerance_m', 0.015):
            raise RuntimeError('Execution tracking error at lower-first hover')
        self.q = state['q'][self.ids].copy()
        return float(candidate['above_pose'][2, 3]) if self.args.supported_release else hover_z

    def _run_grasp_place(self):
        initial = self.snapshot()
        p = initial['mustard'][:3].copy()
        b = initial['bowl'][:3].copy()
        print('FK_ERROR', np.linalg.norm(self.kin.fk(initial['q'][self.ids])[:3, 3] - initial['tcp']), flush=True)
        self.result['requested_grasp']['source_bottle_position_world_m'] = p.tolist()
        grasp_target = body_grasp_tcp_target(initial['mustard'][:7], self.grasp_offset_z, geometry=self.grasp_geometry, local_xy_offset=(0.0, self.args.grasp_offset_y_m))
        self.result['requested_grasp']['target_world_m'] = grasp_target.tolist()
        self.result['requested_grasp']['tcp_heights_world_m'].update(pregrasp=float(grasp_target[2] + 0.125), approach=float(grasp_target[2]))
        self.phase = 'pregrasp'
        yield from self._cart(grasp_target + [0.0, 0.0, 0.125], 3)
        self.phase = 'approach'
        yield from self._cart(grasp_target, 2)
        self.phase = 'close'
        self.grip = 1
        for _ in range(30):
            yield from self._step(self.q, self.grip)
        grasp_state = self.snapshot()
        self.result['body_grasp_close'] = self.grasp_contacts.summarize(self.body_contact_band, 'close')
        if not self.result['body_grasp_close']['both_fingers_near_body']:
            raise RuntimeError('Grasp failed: both fingers must close at the bottle body surface')
        bottle = grasp_state['mustard']
        object_pose = np.eye(4)
        object_pose[:3, :3] = Rotation.from_quat(bottle[3:7]).as_matrix()
        object_pose[:3, 3] = bottle[:3]
        self.world.attachment = np.linalg.inv(self.kin.fk(grasp_state['q'][self.ids])) @ object_pose
        measured_tcp_pose = np.eye(4)
        measured_tcp_pose[:3, :3] = Rotation.from_quat(grasp_state['tcp_quat_xyzw']).as_matrix()
        measured_tcp_pose[:3, 3] = grasp_state['tcp']
        self.result['grasp_observation'] = {'phase': 'close', 'action_count': len(self.records), 'time_s': len(self.records) * self.step_dt, 'tcp_position_world_m': grasp_state['tcp'].tolist(), 'tcp_quaternion_world_xyzw': grasp_state['tcp_quat_xyzw'].tolist(), 'bottle_pose_world_xyz_xyzw': bottle[:7].tolist(), 'arm_joint_positions_rad': grasp_state['q'][self.ids].tolist(), 'gripper_joint_positions_rad': {name: float(grasp_state['q'][self.robot.joint_names.index(name)]) for name in self.gripper_names}, 'attachment_tcp_to_bottle_measured': (np.linalg.inv(measured_tcp_pose) @ object_pose).tolist(), 'attachment_fk_tcp_to_bottle_used_for_planning': self.world.attachment.tolist(), 'attachment_semantics': 'Measured after physical finger closure; planning geometry only, no simulator weld or object teleport'}
        tcp_minus_bottle = grasp_state['tcp'] - bottle[:3]
        lift_target = p + [0.0, 0.0, 0.145] + tcp_minus_bottle
        if self.lower_target_z is None:
            self.lower_target_z = float(b[2] + 0.115 + tcp_minus_bottle[2])
        hover_target_z = float(self.lower_target_z + 0.03)
        self.result['requested_grasp']['tcp_heights_world_m'].update(lift=float(lift_target[2]), transfer_front=float(lift_target[2]), transfer=float(lift_target[2]), above_bowl=hover_target_z, lower_and_placement_lookahead=self.lower_target_z, retreat=hover_target_z)
        self.phase = 'lift'
        yield from self._cart(lift_target, 3)
        self.result['lift_height'] = float(self.snapshot()['mustard'][2] - p[2])
        self.result['body_grasp_lift'] = self.grasp_contacts.summarize(self.body_contact_band, 'lift')
        if not self.result['body_grasp_lift']['both_fingers_on_body']:
            raise RuntimeError('Grasp failed: both fingers must support the bottle body after lift')
        if self.result['lift_height'] < 0.05:
            raise RuntimeError('Grasp failed: bottle did not lift at least 5 cm')
        if self.args.transport_bottle_height_m is not None:
            carried = self.snapshot()
            measured = np.eye(4)
            measured[:3, :3] = Rotation.from_quat(carried['mustard'][3:7]).as_matrix()
            measured[:3, 3] = carried['mustard'][:3]
            self.world.attachment = np.linalg.inv(self.kin.fk(carried['q'][self.ids])) @ measured
            self.result['attachment_after_lift'] = self.world.attachment.tolist()
        transfer_mode = self.settings.get('transfer_mode_by_variant', {}).get(self.args.variant, 'front_waypoints')
        self.result['transfer_mode'] = transfer_mode
        if transfer_mode == 'front_waypoints':
            front_y = self.settings.get('transfer_y', -0.56)
            self.phase = 'transfer_front'
            yield from self._cart([lift_target[0], front_y, lift_target[2]], 2)
            self.phase = 'transfer'
            yield from self._cart([b[0], front_y, lift_target[2]], 4)
        elif transfer_mode != 'goal':
            raise ValueError(f'Unknown transfer mode: {transfer_mode}')
        elif self.motion_planner is not None and self.args.placement_strategy != 'lower-first':
            self._replace_motion_planner(reason='carry', allow_reuse=True, phase='above_bowl')
            self.result['transport_planner_context'] = self.planner_context_events[-1]['mode']
        self.phase = 'above_bowl'
        if self.args.placement_strategy == 'lower-first':
            retreat_height = (yield from self._lower_first_transport([b[0], b[1], self.lower_target_z]))
        elif self.args.transport_bottle_height_m is not None:
            yield from self._cart([b[0], b[1], self.args.transport_bottle_height_m], 3, object_target=True)
            retreat_height = float(self.snapshot()['tcp'][2])
            self.result['requested_grasp']['tcp_heights_world_m'].update(above_bowl=float(self.plan_records[-1]['effective_goal_pose_world'][2][3]), retreat=retreat_height)
        else:
            yield from self._cart([b[0], b[1], hover_target_z], 3)
            retreat_height = hover_target_z
        lower_xy = b[:2]
        if self.args.supported_release:
            from kitchen_safety.placement_geometry import release_lowering_target
            carried = self.snapshot()
            lower_target, support = release_lowering_target(carried['tcp'], carried['mustard'][:7], carried['bowl'][:7])
            lower_xy, self.lower_target_z = (lower_target[:2], float(lower_target[2]))
            self.result['supported_release'] = support
            self.result['requested_grasp']['tcp_heights_world_m']['measured_supported_lower'] = self.lower_target_z
        self.phase = 'lower'
        yield from self._cart([*lower_xy, self.lower_target_z], 3)
        if self.args.placement_strategy == 'lower-first' or (self.episode_spec and self.args.supported_release):
            self.q = np.asarray(self.records[-1][1][:7], dtype=float).copy()
            self.result['release_hold_q_arm'] = self.q.tolist()
            self.result['release_hold_semantics'] = 'Preserve last validated lowering command while opening; never substitute the loaded measured joint position'
            if 'placement' in self.result:
                self.result['placement']['release_hold_q_arm'] = self.q.tolist()
        self.phase = 'release'
        self.grip = 0
        self.world.attachment = None
        for _ in range(30):
            yield from self._step(self.q, self.grip)
        self.phase = 'retreat'
        retreat_xy = self.snapshot()['tcp'][:2] if self.args.placement_strategy == 'lower-first' or self.args.supported_release else b[:2]
        yield from self._cart([*retreat_xy, retreat_height], 2)
        self.phase = 'settle'
        for _ in range(60):
            yield from self._step(self.q, self.grip)

    def _finish_physical_checks(self):
        final = self.snapshot()
        self.result['arena_success'] = self.terminal.get('terms', {}).get('success', False)
        self.result['final_on_destination'] = self._support_now
        delta = final['mustard'][:3] - final['bowl'][:3]
        self.result['success'] = bool(self.result['arena_success'] and self.result['final_on_destination'] and (self.result['lift_height'] > 0.05) and (np.linalg.norm(delta[:2]) < 0.08) and (0 < delta[2] < 0.15) and (np.linalg.norm(final['mustard'][7:10]) < 0.01) and (abs(final['q'][self.robot.joint_names.index('finger_joint')]) < 0.05))
        self.result['final_placement_delta'] = delta.tolist()
        self.result['max_arm_contact_force_n'] = float(max((np.linalg.norm(x[3]['robot_contact_forces'][self.nonfinger_ids], axis=-1).max() for x in self.records)))
        self.result['success'] = bool(self.result['success'] and self.result['max_arm_contact_force_n'] < 5)
        final_checks = [('arena_success', bool(self.result['arena_success']), self.result['arena_success'], 'True'), ('final_on_destination', bool(self.result['final_on_destination']), self.result['final_on_destination'], 'True'), ('lift_height_m', self.result['lift_height'] > 0.05, self.result['lift_height'], '> 0.05 m'), ('final_placement_xy_distance_m', np.linalg.norm(delta[:2]) < 0.08, float(np.linalg.norm(delta[:2])), '< 0.08 m'), ('final_placement_delta_z_m', 0 < delta[2] < 0.15, float(delta[2]), '0 < z < 0.15 m'), ('final_bottle_speed_m_s', np.linalg.norm(final['mustard'][7:10]) < 0.01, float(np.linalg.norm(final['mustard'][7:10])), '< 0.01 m/s'), ('final_gripper_abs_position_rad', abs(final['q'][self.robot.joint_names.index('finger_joint')]) < 0.05, float(abs(final['q'][self.robot.joint_names.index('finger_joint')])), '< 0.05 rad'), ('max_arm_contact_force_n', self.result['max_arm_contact_force_n'] < 5, self.result['max_arm_contact_force_n'], '< 5 N')]
        self.result['physical_failure_reasons'] = []
        for condition, passed, measured, required in final_checks:
            if passed:
                continue
            action_index, failure_phase = (len(self.records) - 1, 'settle')
            if condition == 'lift_height_m':
                action_index = next((i for i in range(len(self.records) - 1, -1, -1) if self.records[i][0] == 'lift'), action_index)
                failure_phase = self.records[action_index][0]
            elif condition == 'max_arm_contact_force_n':
                action_index = next((i for i, row in enumerate(self.records) if not np.linalg.norm(row[3]['robot_contact_forces'][self.nonfinger_ids], axis=-1).max() < 5), action_index)
                failure_phase = self.records[action_index][0]
            self.result['physical_failure_reasons'].append({'condition': condition, 'measured': measured, 'required': required, 'phase': failure_phase, 'action_index': action_index})
        self.result['terminations'] = self.terminal.get('terms', {})
