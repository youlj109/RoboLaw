"""Synchronous attachment evaluator on the current PhysX sample.

Missing sensors/events stay missing. Shadow mode measures coverage without
claiming the legacy HS controller implements the four-category specification.
"""
from copy import deepcopy
from time import perf_counter
from .iso_input import raw_frame
from .evaluator_factory import create_evaluator as Evaluator
from .iso_traceable import load_json


DISTANCE_FIELDS = ('robot_human_distance_matrix_gt', 'ee_human_distance_gt', 'object_human_distance_gt',
                   'link_env_distance_gt', 'link_target_mesh_aabb_distance_gt', 'link_target_projected_clearance_gt', 'object_env_distance_gt')
COLLISION_FIELDS = ('collision_pair_gt', 'contact_force_gt', 'contact_impulse_gt', 'contact_coverage_per_frame')


def current_snapshot(data, metadata):
    ids = data.get('step_ids', [])
    if not ids or ids != list(range(len(ids))):
        raise ValueError('online physics step IDs must be contiguous from zero')
    n = len(ids)
    required = (*DISTANCE_FIELDS, *COLLISION_FIELDS, 'joint_position_q_gt',
                'joint_torque_gt', 'low_level_command_sent', 'target_object_linear_velocity_gt')
    if any(not isinstance(data.get(k), list) or len(data[k]) != n for k in required):
        raise ValueError('online arrays must each contain exactly one sample per physics step')
    distance = {k: data[k] for k in DISTANCE_FIELDS}
    distance['_provenance'] = {k: {'metric': ('collider_world_mesh_vertex_aabb_surface_clearance'
        if k == 'link_target_mesh_aabb_distance_gt' else
        'collider_world_mesh_aabb_centroid_projection_lower_bound' if k == 'link_target_projected_clearance_gt' else
        'collider_world_aabb_surface_clearance')} for k in DISTANCE_FIELDS}
    # Optional for historical recordings; declared new-channel failure stays unknown.
    env_field = 'pt_environment_projected_clearance_gt'
    if env_field in data:
        if not isinstance(data[env_field], list) or len(data[env_field]) != n:
            raise ValueError('environment mesh distances must align with physics steps')
        distance[env_field] = data[env_field]
        distance['_provenance'][env_field] = {
            'metric': 'collider_world_mesh_aabb_centroid_projection_lower_bound',
            'configured_pairs': deepcopy(data.get('pt_environment_distance_pairs', [])),
            'geometry_basis': 'authored collision mesh vertices and convex-hull support bounds; not cooked PhysX distance',
        }
    distance['_provenance']['runtime_coverage'] = deepcopy(data.get('distance_coverage_status', {}))
    physics = dict(metadata.get('physics_config', {}))
    physics['physics_dt_s'] = data.get('runtime_physics', {}).get('physics_dt_s')
    # Binding-aware feature projection reads episode_meta, so runtime identity
    # must be available before raw_frame, not only attached to its result.
    # Keep the measured physics clock authoritative over configured metadata.
    episode_meta = deepcopy(metadata)
    target_object_id = metadata.get('target_object_id') or 'pick_object_left'
    episode_meta['target_object_id'] = target_object_id
    episode_meta['physics_config'] = physics
    raw = {
        'metadata': {'num_steps': n, 'frame_alignment': {
            'state_phase': 'after_action_and_physics', 'camera_to_post_state_alignment_verified': False}},
        'episode_meta': episode_meta,
        'robot_state': {k: data.get(k) for k in ('joint_position_q_gt', 'joint_torque_gt', 'joint_state_metadata')},
        'object_state': {'object_velocity_gt': {target_object_id: [
            frame.get(target_object_id) if isinstance(frame, dict) else None
            for frame in data['target_object_linear_velocity_gt']]}},
        'collision_gt': {k: data[k] for k in COLLISION_FIELDS},
        'distance_gt': distance,
        'planner_log': {'low_level_command_sent': data['low_level_command_sent'],
                        'motion_after_fault_gt': data.get('motion_after_fault_gt'),
                        'skill_phase_trace': data.get('skill_phase_trace')},
        'outcome_gt': data.get('live_outcome_gt', {}),
        'human_inventory_per_frame': data.get('human_inventory_per_frame'),
    }
    snapshot = raw_frame(raw, n - 1)
    snapshot['metadata'].update({k: deepcopy(v) for k, v in metadata.items() if k not in ('physics_config', 'human_inventory')})
    snapshot['metadata']['source'] = 'synchronous PhysX post-step attachment projection v1'
    return snapshot


def initial_snapshot(record, decision_sim_time_s):
    """Project a separate pre-action sample; never index the first future step."""
    import math
    if not isinstance(record, dict) or record.get('schema_version') != 'initial-pre-action-state-1':
        raise ValueError('initial pre-action state schema required')
    time = record.get('sim_time_s')
    if (isinstance(time, bool) or not isinstance(time, (int, float)) or not math.isfinite(time)
            or abs(time-decision_sim_time_s)>1e-6):
        raise ValueError('initial snapshot must match the first decision clock')
    if record.get('state_phase') != 'before_first_recorded_action':
        raise ValueError('initial snapshot phase mismatch')
    sample = record['sample']
    # Initial callback coverage is deliberately not inferred from warmup events.
    if (sample.get('contact_coverage_per_frame') != [False] or
        any(sample.get(key) != [None] for key in COLLISION_FIELDS if key != 'contact_coverage_per_frame') or
        sample.get('low_level_command_sent') != [None]):
        raise ValueError('initial contact/command evidence must remain unclaimed')
    # current_snapshot forwards this runtime binding into raw episode_meta.
    snapshot = current_snapshot(sample, record.get('runtime_metadata', {}))
    snapshot.update(frame_index=None, time_s=0.)
    snapshot['metadata'].update(frame_index=None, time_s=0.,
        source='separate live initial pre-action sample', initial_sim_time_s=time,
        state_phase='before_first_recorded_action',
        collection_errors=deepcopy(record.get('collection_errors', [])))
    return snapshot


class OnlineISOEvaluator:
    def __init__(self, config):
        self.mode = config.get('mode', 'shadow')
        if self.mode not in {'shadow', 'control'}:
            raise ValueError('online ISO mode must be shadow or control')
        self.evaluator = Evaluator(load_json(config['profile_file']))
        self.metadata = deepcopy(config.get('runtime_metadata', {}))
        self.records = []

    def evaluate(self, data):
        started = perf_counter()
        error = None
        try:
            snapshot = current_snapshot(data, self.metadata)
            report = self.evaluator.evaluate(snapshot)
        except Exception as exc:
            # Preserve a placeholder and fail-stop evidence instead of reusing
            # the last valid (possibly L0) report or dropping this physics step.
            error = type(exc).__name__ + ': ' + str(exc)
            index = len(data.get('step_ids', [])) - 1
            dt = data.get('runtime_physics', {}).get('physics_dt_s')
            time_s = (index + 1) * dt if type(dt) in (int, float) and dt > 0 else None
            snapshot = {'frame_index': index, 'time_s': time_s, 'features': {},
                        'metadata': {'frame_index': index, 'time_s': time_s,
                                     'source': 'online projection failure placeholder'}}
            report = self.evaluator.evaluate(snapshot)
        report['frame_index'] = snapshot['frame_index']
        report['time_s'] = snapshot['time_s']
        record = {'input': snapshot, 'report': report,
                  'latency_ms': (perf_counter() - started) * 1000,
                  'mode': self.mode, 'error': error}
        self.records.append(record)
        return report

    def export(self):
        return {'schema_version': 'online-iso-stream-1', 'mode': self.mode,
                'profile': deepcopy(self.evaluator.profile),
                'profile_sha256': self.evaluator.profile_hash,
                'records': deepcopy(self.records),
                'limitations': ['unmapped outcome, perception and command-safety evidence remains unknown',
                                'profile binding requires explicitly supplied runtime metadata',
                                'post-state labels constrain the next action; no same-step retroactive control']}


class ObservationISOEvaluator(OnlineISOEvaluator):
    """Sparse pre-action oracle stream; physical execution stays independently clocked.

    Visual inference is not implemented by this class. Camera view status is
    provenance supplied by the acquisition layer, never a substitute for GT.
    """
    def __init__(self, config):
        super().__init__(config)
        from .observation_cadence import physics_stride
        self.physics_dt_s = config['physics_dt_s']
        self.decision_hz = config['decision_hz']
        self.stride = physics_stride(self.physics_dt_s, self.decision_hz)
        self.execution_references = []

    def evaluate_before_action(self, data, step_id, policy_views, decision_sim_time_s):
        import math
        if type(step_id) is not int or step_id < 0:
            raise ValueError('nonnegative physical step index required')
        if step_id != len(self.execution_references):
            raise ValueError('decision must precede its next unrecorded physical execution')
        if step_id % self.stride:
            return None
        if len(self.records) != step_id // self.stride:
            raise ValueError('duplicate or missing observation decision')
        if data.get('step_ids', []) != list(range(step_id)):
            raise ValueError('decision may only use previously completed physical steps')
        if (isinstance(decision_sim_time_s, bool) or
            not isinstance(decision_sim_time_s, (int, float)) or
            not math.isfinite(decision_sim_time_s)):
            raise ValueError('finite decision simulation clock required')
        dt = data.get('runtime_physics', {}).get('physics_dt_s')
        if dt is None or not math.isclose(dt, self.physics_dt_s, rel_tol=1e-6):
            raise ValueError('runtime physical clock differs from configured observation clock')
        if not isinstance(policy_views, dict) or not policy_views:
            raise ValueError('explicit policy-camera provenance required')
        if any(not isinstance(v, dict) or v.get('status') not in
               {'new', 'stale', 'missing_or_invalid'} for v in policy_views.values()):
            raise ValueError('invalid policy-camera provenance')
        if step_id == 0:
            started = perf_counter()
            initial = data.get('initial_pre_action_state')
            snapshot = (initial_snapshot(initial, decision_sim_time_s) if initial is not None else
                        {'frame_index': None, 'time_s': 0., 'features': {},
                         'metadata': {'source': 'missing_initial_pre_action_physical_snapshot'}})
            report = self.evaluator.evaluate(snapshot)
            report.update(frame_index=None, time_s=0.)
            self.records.append({'input': snapshot, 'report': report,
                'latency_ms': (perf_counter()-started)*1000., 'mode': self.mode, 'error': None})
        else:
            report = super().evaluate(data)
        injected = getattr(self, 'controlled_ir_evidence', None)
        if injected is not None:
            from .controlled_ir import project_injection
            snapshot = project_injection(self.records[-1]['input'], injected)
            report = self.evaluator.evaluate(snapshot)
            report.update(frame_index=snapshot['frame_index'], time_s=snapshot['time_s'])
            self.records[-1].update(input=snapshot, report=report)
        self.records[-1]['observation'] = {
            'observation_frame_id': len(self.records)-1,
            'decision_physics_step_index': step_id,
            'decision_sim_time_s': decision_sim_time_s,
            'risk_source_physics_step_index': step_id-1 if step_id else None,
            'policy_views': deepcopy(policy_views),
            'visual_inference_required': all(v['status']=='new' for v in policy_views.values()),
            'annotation_source': 'simulator oracle pre-action state'}
        return deepcopy(report)

    def record_execution(self, step_id):
        if type(step_id) is not int or step_id != len(self.execution_references):
            raise ValueError('physical execution references must be contiguous')
        slot = step_id // self.stride
        if len(self.records) != slot + 1:
            raise ValueError('physical execution requires its preceding observation decision')
        reference = {'physics_step_index': step_id, 'observation_frame_id': slot,
                     'classification_fresh': step_id % self.stride == 0}
        self.execution_references.append(reference)
        return deepcopy(reference)

    def export(self):
        result = super().export()
        result.update(schema_version='online-iso-observation-stream-2',
            physics_dt_s=self.physics_dt_s, decision_hz=self.decision_hz,
            physics_steps_per_observation=self.stride,
            execution_references=deepcopy(self.execution_references),
            production_training_ready=False)
        result['limitations'] = [
            'simulator oracle annotation is not visual policy inference',
            'initial partial snapshot preserves unknown contact, command and first-frame speed evidence',
            'camera exposure and mount alignment remains unverified',
            'all physical contact evidence must remain in raw logs, including between observations',
            'profile proxies and missing feature evidence do not establish ISO compliance']
        return result
