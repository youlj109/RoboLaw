"""One deterministic timing-only candidate per failed native physical episode.

This module is CPU-only. It retains every original joint-space edge and vertex,
the nominal gripper configuration, and all geometric planning inputs. It does
not claim that retiming establishes collision clearance or physical success.
"""
from __future__ import annotations

from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import numpy as np

PHASES = ('pregrasp', 'approach', 'lift', 'transfer', 'lower', 'retreat')
DT = 1 / 30
VELOCITY_LIMIT = .4
ACCELERATION_LIMIT = .8
# Leave numerical room for the executor's float32 actuator commands.
NUMERICAL_MARGIN = .99
ENDPOINT_HOLDS = {'approach': .5, 'lower': .5}
EXECUTION_SETTINGS = dict(device='cpu', seed=42, physics_threads=8,
    initial_settle_seconds=5., close_seconds=2., release_seconds=2.,
    final_settle_seconds=3., episode_seconds=200., max_actions=6000)


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def validate_source(data):
    q = np.asarray(data['q'], dtype=float)
    n = len(q)
    if q.shape != (n, 7) or n < 6 or not np.isfinite(q).all():
        raise ValueError('Expected finite (N,7) source joint targets')
    if (np.asarray(data['phase']).shape != (n,) or np.asarray(data['gripper']).shape != (n,)
            or np.asarray(data['tcp_world']).shape != (n, 4, 4)
            or np.asarray(data['dt']).shape != () or not np.isclose(float(data['dt']), DT, atol=1e-12, rtol=0)):
        raise ValueError('Source arrays must be aligned at 30Hz')
    if tuple(dict.fromkeys(data['phase'].tolist())) != PHASES:
        raise ValueError('Source must contain exactly the ordered six phases')
    if not np.isfinite(data['gripper']).all() or not np.isfinite(data['tcp_world']).all():
        raise ValueError('Nonfinite source gripper or TCP')
    for phase in PHASES:
        grips = np.unique(data['gripper'][data['phase'] == phase])
        if len(grips) != 1:
            raise ValueError('Timing-only policy requires constant nominal grip within each phase')
        if phase in ('pregrasp', 'approach', 'retreat') and grips[0] != 0:
            raise ValueError('Original open-gripper phases changed')
    carry = np.unique(data['gripper'][np.isin(data['phase'], ['lift','transfer','lower'])])
    if len(carry) != 1 or carry[0] <= 0:
        raise ValueError('Original carry phases must share the nominal closed angle')
    return q


def command_bounds(q, dt=DT):
    """Discrete command bounds include starts/stops, also after float32 casting."""
    q = np.asarray(q, dtype=float)
    extended = np.vstack([q[0], q, q[-1]])
    velocity = np.diff(extended, axis=0) / dt
    acceleration = np.diff(velocity, axis=0) / dt
    return dict(peak_joint_speed_rad_s=float(np.max(np.abs(velocity))),
        peak_joint_acceleration_rad_s2=float(np.max(np.abs(acceleration))))


def retime_candidate(data, *, endpoint_holds=None):
    """Quintic rest-to-rest traversal of every unchanged original q edge.

    The blend is 10u^3-15u^4+6u^5, whose exact derivative maxima are 1.875
    and 10/sqrt(3). Integer 30Hz edge durations are rounded upward. Hence the
    continuous reference and sampled command differences obey both limits.
    This stops at all original vertices instead of cutting across corners.
    """
    q = validate_source(data)
    holds = ENDPOINT_HOLDS if endpoint_holds is None else endpoint_holds
    if set(holds) - set(PHASES) or any(not np.isfinite(v) or v < 0 for v in holds.values()):
        raise ValueError('Invalid endpoint hold policy')
    points, phases, grips, starts, ends, fractions = [], [], [], [], [], []
    vertex_indices, edge_samples = [], []
    for i, target in enumerate(q):
        phase = str(data['phase'][i])
        grip = float(data['gripper'][i])
        if i == 0:
            chunk = target[None].copy(); blend = np.array([0.]); start = 0
        else:
            start = i - 1
            delta = target - q[start]
            distance = float(np.max(np.abs(delta)))
            duration = max(DT, 1.875 * distance / (VELOCITY_LIMIT * NUMERICAL_MARGIN),
                np.sqrt((10 / np.sqrt(3)) * distance / (ACCELERATION_LIMIT * NUMERICAL_MARGIN)))
            count = max(1, int(np.ceil(duration / DT)))
            u = np.arange(1, count + 1, dtype=float) / count
            blend = 10*u**3 - 15*u**4 + 6*u**5
            chunk = q[start] + blend[:, None] * delta
            chunk[-1] = target
            edge_samples.append(count)
        points.extend(chunk)
        phases.extend([phase] * len(chunk)); grips.extend([grip] * len(chunk))
        starts.extend([start] * len(chunk)); ends.extend([i] * len(chunk)); fractions.extend(blend)
        vertex_indices.append(len(points)-1)
        if i == len(q)-1 or data['phase'][i+1] != phase:
            count = int(np.ceil(holds.get(phase, 0.) / DT))
            points.extend([target.copy() for _ in range(count)])
            phases.extend([phase] * count); grips.extend([grip] * count)
            starts.extend([i] * count); ends.extend([i] * count); fractions.extend([0.] * count)
    candidate = dict(q=np.asarray(points, dtype=np.float64), phase=np.asarray(phases),
        gripper=np.asarray(grips), dt=np.asarray(DT),
        source_edge_start_index=np.asarray(starts, dtype=np.int64),
        source_edge_end_index=np.asarray(ends, dtype=np.int64),
        source_edge_fraction=np.asarray(fractions),
        source_vertex_output_index=np.asarray(vertex_indices, dtype=np.int64))
    validation = validate_candidate(data, candidate)
    validation.update(edge_sample_counts=edge_samples, source_samples=len(q), samples=len(points),
        nominal_trajectory_seconds=len(points)*DT,
        added_endpoint_holds_seconds=dict(holds),
        continuous_reference_velocity_bound_rad_s=VELOCITY_LIMIT*NUMERICAL_MARGIN,
        continuous_reference_acceleration_bound_rad_s2=ACCELERATION_LIMIT*NUMERICAL_MARGIN)
    return candidate, validation


def validate_candidate(source, candidate):
    q = validate_source(source)
    output = np.asarray(candidate['q'])
    a, b, f, vertices = (np.asarray(candidate[k]) for k in (
        'source_edge_start_index','source_edge_end_index','source_edge_fraction','source_vertex_output_index'))
    n = len(output)
    if (output.shape != (n,7) or not np.isfinite(output).all() or a.shape != (n,)
            or b.shape != (n,) or f.shape != (n,) or vertices.shape != (len(q),)
            or np.asarray(candidate['phase']).shape != (n,) or np.asarray(candidate['gripper']).shape != (n,)):
        raise ValueError('Misaligned retimed edge provenance')
    if any(v.dtype.kind not in 'iu' for v in (a,b,vertices)):
        raise ValueError('Source indices must be integers')
    if (np.any(a < 0) or np.any(b >= len(q)) or np.any((b-a < 0)|(b-a > 1))
            or np.any((f < 0)|(f > 1)) or not np.isfinite(f).all()
            or np.any(vertices < 0) or np.any(vertices >= n) or np.any(np.diff(vertices) <= 0)):
        raise ValueError('Invalid or reversed source-edge traversal')
    location = a + f*(b-a)
    if np.any(np.diff(location) < -1e-12) or location[0] != 0 or location[-1] != len(q)-1:
        raise ValueError('Retimed path skips or reverses the source polyline')
    expected = q[a] + f[:,None]*(q[b]-q[a])
    deviation = float(np.max(np.abs(output-expected)))
    if deviation > 1e-12 or not np.array_equal(output[vertices],q):
        raise ValueError('Geometric edge/vertex changed')
    if (not np.array_equal(candidate['phase'],source['phase'][b])
            or not np.array_equal(candidate['gripper'],source['gripper'][b])):
        raise ValueError('Phase or nominal gripper changed')
    if not np.isclose(float(candidate['dt']),DT,atol=1e-12,rtol=0):
        raise ValueError('Retimed command interval changed')
    bounds = command_bounds(output)
    cast_bounds = command_bounds(output.astype(np.float32))
    for values in (bounds,cast_bounds):
        if values['peak_joint_speed_rad_s'] > VELOCITY_LIMIT+1e-9 or values['peak_joint_acceleration_rad_s2'] > ACCELERATION_LIMIT+1e-9:
            raise ValueError('Retimed actuator command bound exceeded')
    return dict(passed=True, geometric_polyline_unchanged=True, all_source_vertices_exact=True,
        source_edge_max_error_rad=deviation, phase_sequence_unchanged=True,
        nominal_gripper_unchanged=True, **bounds, float32_command_bounds=cast_bounds)


def fk_batch(kin, q):
    """CPU FK through the unchanged source-USD chain; no interpolated TCP poses."""
    from scipy.spatial.transform import Rotation
    q = np.asarray(q, dtype=float)
    transforms = np.broadcast_to(kin.base, (len(q),4,4)).copy()
    for index,(a,b,_) in enumerate(kin.chain):
        rotations = np.broadcast_to(np.eye(4),(len(q),4,4)).copy()
        if index < 7:
            rotations[:,:3,:3] = Rotation.from_euler('x',q[:,index,None]).as_matrix()
        transforms = transforms @ a @ rotations @ b
    transforms[:,:3,3] += transforms[:,:3,:3] @ np.array([.131,0.,0.])
    return transforms


def prepare_plan(root, first_manifest, camera_manifest, observations, output, plan_path):
    """Freeze exactly one timing candidate for all 84 first-pass failures."""
    from kitchen_safety.planning import DroidKinematics, ROBOT_USD
    root, output, plan_path = Path(root).resolve(), Path(output).resolve(), Path(plan_path).resolve()
    frozen = {}
    def local(p):
        value=(root/str(p).removeprefix('/workspace/')).resolve()
        if not value.is_relative_to(root): raise ValueError('Input outside repository')
        return value
    def bind(p, expected=None):
        p=local(p); digest=sha(p)
        if expected is not None and digest != expected: raise ValueError('Source SHA changed: '+str(p))
        key=str(p.relative_to(root))
        if key in frozen and frozen[key] != digest: raise ValueError('Conflicting source SHA')
        frozen[key]=digest
        return dict(path=key,sha256=digest)
    def load(p): return json.loads(local(p).read_text())
    first_ref, camera_ref, observation_ref = bind(first_manifest),bind(camera_manifest),bind(observations)
    first,camera,observed = load(first_manifest),load(camera_manifest),load(observations)
    if (first['stage'] != 'complete' or len(first['entries']) != 154
            or Counter(r['execution_status'] for r in first['entries']) != {'success':70,'failed':84}):
        raise ValueError('Expected the fixed first-pass 70 successes / 84 failures')
    if camera['stage'] != 'complete' or len(camera['entries']) != 11:
        raise ValueError('Completed independent camera history is required')
    if observed['source_first_pass_manifest'] != first_ref or observed['source_camera_retake_manifest'] != camera_ref:
        raise ValueError('Observation snapshot belongs to different physical evidence')
    camera_rows={r['scene_id']:r for r in camera['entries']}
    observations_by_id={r['scene_id']:r for r in observed['entries']}
    if len(camera_rows)!=11 or len(observations_by_id)!=154: raise ValueError('Duplicate source scene IDs')
    for r in first['entries']+camera['entries']:
        bind(r['execution_result_path'],r['execution_result_sha256'])
    failed=[r for r in first['entries'] if r['execution_status']=='failed']
    sources=[]
    for old in failed:
        sid=old['scene_id']; original=load(old['execution_result_path']); observation=observations_by_id[sid]['failure_observation']
        if original['executed'] is not True or original['task_success'] is not False or original.get('physical_attempt_number',1)!=1:
            raise ValueError('Retry cohort must contain actual first-pass failures')
        for path_key,hash_key in [('result_path','result_sha256'),('config_path','config_sha256'),('trajectory_path','trajectory_sha256'),('source_scene_path','source_scene_sha256')]:
            bind(old[path_key],old[hash_key])
        plan=load(old['result_path']); layout=load(old['config_path'])
        if plan['scene_id']!=sid or plan['status']!='generated' or not plan['trajectory_complete']:
            raise ValueError('Incomplete or mismatched original planning result')
        for p,digest in plan['composition']['source_paths_sha256'].items(): bind(p,digest)
        for item in observation['evidence'].values(): bind(item['path'],item['sha256'])
        if observation['root_cause_confirmed'] is not False: raise ValueError('Expected observational, not causal, labels')
        with np.load(local(old['trajectory_path']),allow_pickle=False) as data: source={k:data[k].copy() for k in data.files}
        validate_source(source)
        if np.max(np.abs(source['q'][0]-plan['start_q']))>1e-5: raise ValueError('Source initial q differs')
        sources.append((old,plan,layout,source,observation))
    bind(ROBOT_USD)
    if output.exists() or plan_path.exists(): raise FileExistsError('Existing candidate files are frozen; choose a new authorized round')
    if not output.is_relative_to(root) or not plan_path.is_relative_to(root): raise ValueError('Outputs must remain in repository')
    output.mkdir(parents=True,exist_ok=False)
    plan_path.parent.mkdir(parents=True,exist_ok=True)
    entries=[]; kin_cache={}
    for old,plan,layout,source,observation in sources:
        sid=old['scene_id']; candidate,validation=retime_candidate(source)
        base=tuple(np.asarray(layout['robot_position'])+[0,0,layout['stand_height']]); key=(base,float(layout['robot_yaw_deg']))
        if key not in kin_cache: kin_cache[key]=DroidKinematics(base,key[1])
        kin=kin_cache[key]
        fk_source=fk_batch(kin,source['q'])
        if not np.allclose(fk_source,source['tcp_world'],atol=2e-6,rtol=0): raise ValueError('Source TCP/FK mismatch: '+sid)
        candidate['tcp_world']=fk_batch(kin,candidate['q'])
        if not np.allclose(candidate['tcp_world'][candidate['source_vertex_output_index']],source['tcp_world'],atol=2e-6,rtol=0):
            raise ValueError('Geometric TCP endpoints changed')
        validation['tcp_recomputed_from_source_kinematics']=True
        validation['source_command_bounds']=command_bounds(source['q'])
        total_actions=len(candidate['q'])+sum(int(np.ceil(EXECUTION_SETTINGS[k]/DT)) for k in
            ('initial_settle_seconds','close_seconds','release_seconds','final_settle_seconds'))
        if total_actions>=EXECUTION_SETTINGS['max_actions'] or total_actions*DT+10>=EXECUTION_SETTINGS['episode_seconds']:
            raise ValueError('Retimed candidate exceeds the frozen execution budget')
        validation.update(expected_total_actions=total_actions,expected_total_seconds=total_actions*DT,
            runtime_budget_margin_seconds=EXECUTION_SETTINGS['episode_seconds']-total_actions*DT)
        npz_path=output/(sid+'.npz')
        with npz_path.open('xb') as stream: np.savez_compressed(stream,**candidate)
        with np.load(npz_path,allow_pickle=False) as saved: validate_candidate(source,saved)
        description={'F1':'原记录未达到代理抬升阈值；降低接近/抬升命令速度与加速度，并在抓取前停止到位。',
            'F2':'原记录抬升且物体-夹爪距离增大；保持抓姿/闭合角，仅降低携物段速度与加速度。',
            'F3':'原记录最终放置判据失败；保持最终目标，仅降低放置/撤离速度与加速度并增加释放前到位停留。'}[observation['label']]
        entry={k:old[k] for k in ('scene_id','task_id','variant_id','result_path','result_sha256','config_path','config_sha256','source_scene_path','source_scene_sha256')}
        entry.update(source_trajectory_path=old['trajectory_path'],source_trajectory_sha256=old['trajectory_sha256'],
            trajectory_path=str(npz_path.relative_to(root)),trajectory_sha256=sha(npz_path),
            physical_attempt_number=3 if sid in camera_rows else 2,
            first_pass_result_path=old['execution_result_path'],first_pass_result_sha256=old['execution_result_sha256'],
            adjustment=dict(kind='timing_only_quintic_original_polyline',description=description,
                evidence_path=observation_ref['path'],evidence_sha256=observation_ref['sha256'],
                failure_observation_label=observation['label'],failure_observation=observation['observed_symptom_zh'],
                root_cause_confirmed=False,gripper_policy='original nominal angles and original mimic unchanged',
                velocity_limit_rad_s=VELOCITY_LIMIT,acceleration_limit_rad_s2=ACCELERATION_LIMIT,
                endpoint_holds_seconds=dict(ENDPOINT_HOLDS),candidate_count=1,planning_calls=0),
            timing_validation=validation)
        entries.append(entry)
    for p,digest in frozen.items():
        if sha(root/p)!=digest: raise ValueError('Frozen source changed during preparation: '+p)
    result=dict(schema_version=1,kind='physical_failure_retry_plan',created_at=datetime.now(timezone.utc).isoformat(),
        source_first_pass_manifest=first_ref,source_camera_retake_manifest=camera_ref,
        source_observation_snapshot=observation_ref,scope='One deterministic timing-only candidate and one authorized physical retry for each of the 84 first-pass failures; original results remain unchanged.',
        expected_scene_count=84,candidate_count_per_scene=1,physical_retries_per_scene=1,
        geometric_polyline_unchanged=True,layout_changed=False,nominal_gripper_changed=False,planning_calls=0,
        execution_settings=EXECUTION_SETTINGS.copy(),retiming_policy=dict(method='quintic_rest_to_rest_per_original_edge',
            velocity_limit_rad_s=VELOCITY_LIMIT,acceleration_limit_rad_s2=ACCELERATION_LIMIT,
            numerical_margin=NUMERICAL_MARGIN,endpoint_holds_seconds=dict(ENDPOINT_HOLDS)),
        summary=dict(first_pass_successes_excluded=70,scenes=84,
            expected_physical_attempt_counts=dict(Counter(e['physical_attempt_number'] for e in entries)),
            maximum_expected_total_seconds=max(e['timing_validation']['expected_total_seconds'] for e in entries),
            total_expected_simulation_seconds=sum(e['timing_validation']['expected_total_seconds'] for e in entries)),
        checks_not_performed=['No new collision/planning solve','No physical retry executed during CPU preparation','No assertion of grasp repair, success, or safety'],
        frozen_input_sha256=frozen,entries=entries)
    with plan_path.open('x') as stream: json.dump(result,stream,ensure_ascii=False,indent=2,allow_nan=False); stream.write('\n')
    return result
