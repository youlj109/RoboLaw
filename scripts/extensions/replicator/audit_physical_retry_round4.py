#!/usr/bin/env python3
"""Independent read-only CPU audit of the 6-scene fourth physical retry.

Only the requested audit JSON is written. A planning failure must have zero
physical attempts. Completed native episodes are checked against their new
candidate, exact actuator commands, original predicates, and all prior trials.
No Kit, physics, planning, rendering or GPU is initialized here. Source USD
joint frames and bowl/floor geometry are read on CPU for independent checks.
"""
import argparse
import math
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from scripts.extensions.replicator import audit_physical_retry_round3 as numeric
DEFAULT_DEMOS = 'exports/replicator_physical_retry_round4_20261029'
DEFAULT_OUTPUT = 'reports/replicator_physical_retry_round4_20261029/audit_execution.json'
TERMINAL = {'success', 'failed', 'error', 'planning_failed'}
PHASES = ('pregrasp', 'approach', 'lift', 'transfer', 'lower', 'retreat')
SOURCE_KEYS = ('scene_id', 'task_id', 'variant_id', 'result_path', 'result_sha256',
               'config_path', 'config_sha256', 'source_scene_path', 'source_scene_sha256')
HASHES = {}


def require(value, message):
    if not value:
        raise ValueError(message)


def local(value):
    path = (ROOT / str(value).removeprefix('/workspace/')).resolve()
    require(path.is_relative_to(ROOT), 'Path outside checkout: ' + str(path))
    return path


def digest(path):
    path = Path(path)
    stat = path.stat()
    key = (str(path), stat.st_size, stat.st_mtime_ns)
    if key not in HASHES:
        with path.open('rb') as stream:
            HASHES[key] = hashlib.file_digest(stream, 'sha256').hexdigest()
    return HASHES[key]


def bound(path, expected):
    path = local(path)
    require(path.is_file() and path.stat().st_size > 0, 'Missing/empty file: ' + str(path))
    require(digest(path) == expected, 'SHA mismatch: ' + str(path))
    return path


def read(path):
    return json.loads(Path(path).read_text())


def ref(path):
    path = local(path)
    return dict(path=str(path.relative_to(ROOT)), sha256=digest(path), bytes=path.stat().st_size)


def file_binding(path):
    """Use the exact two-field binding expected by downstream publishers."""
    value = ref(path)
    return {key: value[key] for key in ('path', 'sha256')}


def bound_json(value):
    return read(bound(value['path'], value['sha256']))


def rows_by_id(document):
    rows = document['entries']
    if isinstance(rows, dict):
        rows = list(rows.values())
    require(len({row['scene_id'] for row in rows}) == len(rows), 'Duplicate scene IDs')
    return {row['scene_id']: row for row in rows}


def merged_settings(settings, overrides):
    require(isinstance(overrides, dict) and not set(overrides) - {'close_seconds', 'release_seconds'},
            'Only close/release duration overrides are authorized')
    values = dict(settings)
    for key, value in overrides.items():
        require(not isinstance(value, bool) and isinstance(value, (int, float))
                and np.isfinite(value) and .1 <= value <= 10., 'Invalid hold duration override')
        values[key] = value
    return values


def prior_binding(row):
    return dict(result_path=row['execution_result_path'], result_sha256=row['execution_result_sha256'],
        execution_status=row['execution_status'], task_success=row['task_success'], executed=row['executed'],
        physical_attempt_number=row.get('physical_attempt_number', 1), video_sha256=row.get('video_sha256'))


def audit_history(row, recipe, context):
    """Reconstruct only actual episodes through all three bound prior retries."""
    sid = row['scene_id']
    original = context['original'][sid]
    previous = context['previous'][sid]
    prior_rows = [(original, None)]
    if sid in context['retakes']:
        prior_rows.append((context['retakes'][sid], None))
    history = [prior_binding(prior) for prior, _ in prior_rows]
    for manifest, rows in context['prior_rounds']:
        prior = rows[sid]
        require(prior['prior_physical_attempts'] == history, 'Previous round omitted earlier actual history')
        for key in SOURCE_KEYS:
            require(prior[key] == original[key], 'Historical original source changed: '+key)
        if prior['execution_status'] == 'failed':
            prior_rows.append((prior,manifest));history.append(prior_binding(prior))
        else:
            require(prior['execution_status'] == 'planning_failed' and prior['planning_status'] == 'failed'
                    and type(prior['attempts']) is int and prior['attempts'] == 0 and prior['executed'] is False
                    and prior['task_success'] is False and not prior.get('execution_result_path')
                    and not prior.get('execution_result_sha256'), 'Previous failed plan invented a physical episode')
            candidate = bound_json(prior['candidate_planning_result'])
            require(candidate['status'] == 'failed' and candidate['trajectory_complete'] is False
                    and candidate['executed'] is False and candidate['attempts'] == 1,
                    'Previous failed planning provenance differs')
        require(prior['physical_attempt_number'] == len(prior['prior_physical_attempts'])+1,
                'Prior reserved/actual attempt number omits history')
    require(row['prior_physical_attempts'] == history, 'Prior physical history differs from bound manifests')
    expected_previous = {'path':history[-1]['result_path'],'sha256':history[-1]['result_sha256']}
    require(row['previous_retry_result'] == recipe['previous_retry_result'] == expected_previous,
            'Wrong latest actual failure binding')
    require(row['physical_attempt_number'] == recipe['physical_attempt_number'] == len(history)+1,
            'Next physical attempt number counts planning failure or omits actual history')
    for key in SOURCE_KEYS:
        require(row[key] == recipe[key] == original[key] == previous[key], 'Historical scene source differs: '+key)
    require(row['source_trajectory_path'] == original['trajectory_path']
            and row['source_trajectory_sha256'] == original['trajectory_sha256'], 'Original trajectory identity differs')
    prior_results=[]
    for number,((prior,manifest),saved) in enumerate(zip(prior_rows,history),1):
        result=read(bound(saved['result_path'],saved['result_sha256']))
        require(result.get('physical_attempt_number',1) == saved['physical_attempt_number'] == number,
                'Physical lineage is not consecutive')
        require(result['scene_id'] == sid and result['status'] == 'complete' and result['executed'] is True
                and result['execution_status'] == 'failed' and result['task_success'] is False,
                'History contains an unexecuted, incomplete, or successful trial')
        for key in ('task_id','variant_id','config_path','config_sha256','trajectory_path','trajectory_sha256'):
            require(result[key] == prior[key], 'Prior result differs from its own manifest: '+key)
        for key in SOURCE_KEYS:
            require(prior[key] == original[key], 'Historical original source changed: '+key)
        expected_plan=(prior['candidate_planning_result'] if prior.get('retry_schema_version') == 2
                       else {'path':original['result_path'],'sha256':original['result_sha256']})
        require(result['planning_result_path'] == expected_plan['path']
                and result['planning_result_sha256'] == expected_plan['sha256'], 'Prior actual planning source differs')
        if manifest is not None:
            require(result['kind'] == 'physical_failure_retry_execution' and result['attempts'] == prior['attempts'] == 1,
                    'Previous retry is not one native trial')
            require(result['prior_physical_attempts'] == prior['prior_physical_attempts'] == history[:number-1],
                    'Previous result history differs')
            require(result['source_trajectory_path'] == prior['source_trajectory_path'] == row['source_trajectory_path']
                    and result['source_trajectory_sha256'] == prior['source_trajectory_sha256'] == row['source_trajectory_sha256'],
                    'Previous retry original trajectory binding differs')
            for key in ('retry_plan_path','retry_plan_sha256'):
                require(result[key] == prior[key] == manifest[key], 'Previous retry plan differs')
            for flag in ('runtime_state_setters_used','physical_weld_used','payload_attachment_used','planning_rerun','rendering_enabled'):
                require(result[flag] is False, 'Previous retry substituted execution or rendered: '+flag)
            require(result['render_generation_delta'] == 0 and result['source_inputs_unchanged'] is True
                    and result['execution_sources_unchanged'] is True, 'Previous retry provenance differs')
            if prior.get('retry_schema_version') == 2:
                require(result['candidate_planning_result'] == expected_plan
                        and result['original_planning_result'] == {'path':original['result_path'],'sha256':original['result_sha256']},
                        'Previous candidate/original binding differs')
                bound_json(expected_plan)
        else:
            require(result['kind'] == 'physical_execution', 'Unexpected original physical result kind')
            require(result['trajectory_sha256'] == row['source_trajectory_sha256'], 'Original/camera trajectory differs')
            if number == 2:
                require(prior['first_pass']['result_sha256'] == history[0]['result_sha256'], 'Camera history omits first trial')
        bound(result['trajectory_path'],result['trajectory_sha256']);prior_results.append(result)
    return prior_results


def audit_candidate(row, recipe):
    """The new planner output is distinct from the retained original result."""
    for key, value in recipe.items():
        require(row.get(key) == value, 'Manifest differs from frozen retry recipe: ' + key)
    require(row.get('retry_schema_version') == 2, 'Wrong retry entry schema')
    for path_key, hash_key in (('result_path', 'result_sha256'), ('config_path', 'config_sha256'),
            ('source_scene_path', 'source_scene_sha256'), ('source_trajectory_path', 'source_trajectory_sha256')):
        bound(row[path_key], row[hash_key])
    adjustment = recipe['adjustment']
    require(adjustment['candidate_count'] == 1 and adjustment['description'], 'Missing single-candidate adjustment')
    bound(adjustment['evidence_path'], adjustment['evidence_sha256'])
    recipe_document = bound_json(adjustment['recipe_source'])
    declared = recipe_document['recipes'][row['scene_id']]
    candidate = bound_json(row['candidate_planning_result'])
    old = read(local(row['result_path']))
    require(local(row['candidate_planning_result']['path']) != local(row['result_path']), 'New candidate overwrites historical planning result')
    for key in ('scene_id', 'task_id', 'variant_id', 'config_path', 'config_sha256'):
        require(candidate[key] == row[key], 'Candidate scene identity differs: ' + key)
    require(candidate['recipe'] == declared, 'Candidate recipe differs from its frozen source')
    audit_candidate_targets(candidate, declared, old)
    require(candidate['prior_result_path'] == row['result_path']
            and candidate['prior_result_sha256'] == row['result_sha256'], 'Candidate omits old planning source')
    for key, value in dict(planning_only=True, executed=False, attempts=1, per_stage_max_attempts=1,
                           candidate_count=1, layout_fixed=True, task_success_verified=False).items():
        require(candidate.get(key) == value, 'Candidate scope differs: ' + key)
    require(candidate.get('scene_settings_changed_once', False) is False, 'Candidate changed scene settings')
    require(np.asarray(candidate['start_q']).shape == (7,)
            and np.allclose(candidate['start_q'], old['start_q'], atol=1e-5, rtol=0), 'Candidate changed frozen start q')
    require(candidate['composition']['source_paths_sha256'] == old['composition']['source_paths_sha256'],
            'Candidate source geometry differs')
    for path, sha in candidate['composition']['source_paths_sha256'].items():
        bound(path, sha)
    require(candidate.get('collision_geometry_unchanged') is True and candidate.get('added_hazard_masks') is False,
            'Candidate changed collision geometry or added hazard masks')
    phases = candidate['phases']
    require(tuple(phase['phase'] for phase in phases) == PHASES[:len(phases)] and len(phases) <= len(PHASES),
            'Planner repeated or reordered phases')
    old_masks = {phase['phase']: phase['contact_masks'] for phase in old['phases']}
    for phase in phases:
        require(phase['contact_masks'] == old_masks[phase['phase']], 'Candidate changed phase contact masks')
        require(phase.get('vertical_motion_cost_requested') is (phase['phase'] in declared.get('linear_motion_phases', [])),
                'Candidate vertical-motion request differs from frozen recipe')
    require(row['execution_overrides'] == declared.get('execution_overrides', {}), 'Execution override differs from declared recipe')
    merged_settings({}, row['execution_overrides'])
    require(candidate['status'] == row['planning_status'], 'Planning state mismatch')
    audit_declared_segments(candidate,declared,row)
    grip = candidate['nominal_gripper_joint_rad']
    require(0 < grip <= .785 and grip == declared['nominal_gripper_joint_rad'], 'Invalid nominal execution gripper')
    planning_grip = candidate['planning_gripper_joint_rad']
    require(0 < planning_grip <= .785
            and planning_grip == declared.get('planning_gripper_joint_rad', grip), 'Invalid estimated planning gripper')
    if row['planning_status'] == 'failed':
        require(candidate['trajectory_complete'] is False and row.get('failure_reason')
                and row['trajectory_path'] is None and row['trajectory_sha256'] is None,
                'Failed candidate has an executable trajectory or no reason')
        return candidate
    require(row['planning_status'] == 'generated' and candidate['trajectory_complete'] is True, 'Incomplete generated candidate')
    require(tuple(phase['phase'] for phase in phases) == PHASES
            and all(phase['solver_success'] is True and phase['status'] == 'generated' for phase in phases),
            'Candidate lacks six successful phases')
    raw_ref = candidate['artifacts']['trajectory']
    raw_path = bound(raw_ref['path'], raw_ref['sha256'])
    retimed_path = bound(row['trajectory_path'], row['trajectory_sha256'])
    require(len({raw_path, retimed_path, local(row['source_trajectory_path'])}) == 3, 'Raw/new/original trajectory paths overlap')
    with np.load(raw_path, allow_pickle=False) as raw, np.load(retimed_path, allow_pickle=False) as retimed:
        audit_retiming(raw, retimed, grip, candidate['start_q'])
        ordinary=dict(candidate,phases=[p for p in candidate['phases'] if 'segments' not in p])
        audit_vertical_cost_record(ordinary, raw)
    return candidate


def state_pose(state):
    """Independent xyzw pose conversion for recorded states and USD joints."""
    value=np.asarray(state,dtype=float);x,y,z,w=value[3:7]
    require(np.isfinite(value).all() and abs(x*x+y*y+z*z+w*w-1)<1e-3,'Invalid pose quaternion')
    q=np.array([x,y,z,w]);q=q/np.linalg.norm(q);x,y,z,w=q
    t=np.eye(4);t[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]];t[:3,3]=value[:3];return t


class SourceFK:
    """CPU-only FK reconstructed from pinned source USD joint frames."""
    def __init__(self,layout):
        from pxr import Usd,UsdPhysics
        from kitchen_safety.planning import ROBOT_USD
        self.source=file_binding(ROBOT_USD)
        stage=Usd.Stage.Open(str(local(self.source['path'])))
        joints={str(p.GetName()):p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)}
        angle=np.deg2rad(float(layout['robot_yaw_deg']));self.base=np.eye(4)
        self.base[:3,:3]=[[math.cos(angle),-math.sin(angle),0],[math.sin(angle),math.cos(angle),0],[0,0,1]]
        self.base[:3,3]=np.asarray(layout['robot_position'])+[0,0,layout['stand_height']]
        self.chain=[]
        def joint_pose(joint,side):
            p=getattr(joint,'GetLocalPos'+side+'Attr')().Get();q=getattr(joint,'GetLocalRot'+side+'Attr')().Get()
            return state_pose([*p,*q.GetImaginary(),q.GetReal()])
        for name in [f'panda_joint{i}' for i in range(1,9)]+['panda_hand_joint']:
            prim=joints[name];joint=UsdPhysics.Joint(prim)
            axis=None
            if prim.IsA(UsdPhysics.RevoluteJoint):
                axis=np.eye(3)['XYZ'.index(str(UsdPhysics.RevoluteJoint(prim).GetAxisAttr().Get()))]
            self.chain.append((joint_pose(joint,'0'),np.linalg.inv(joint_pose(joint,'1')),axis))
    def fk(self,q):
        result=self.base.copy();cursor=0
        for left,right,axis in self.chain:
            rotation=np.eye(4)
            if axis is not None:
                theta=float(q[cursor]);cursor+=1;x,y,z=axis
                k=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
                rotation[:3,:3]=np.eye(3)+math.sin(theta)*k+(1-math.cos(theta))*(k@k)
            result=result@left@rotation@right
        require(cursor==7,'Unexpected source arm DOF')
        result[:3,3]+=result[:3,:3]@np.array([.131,0,0]);return result


def segment_deviation(positions,start,end):
    xyz=np.asarray(positions,dtype=float);a=np.asarray(start,dtype=float);b=np.asarray(end,dtype=float)
    require(xyz.ndim==2 and xyz.shape[1]==3 and len(xyz)>0 and np.isfinite(xyz).all(),'Invalid segment positions')
    d=b-a;length_squared=float(np.dot(d,d))
    closest=np.broadcast_to(a,xyz.shape) if length_squared<=1e-20 else a+np.clip(np.dot(xyz-a,d)/length_squared,0,1)[:,None]*d
    return float(np.linalg.norm(xyz-closest,axis=1).max())


def audit_destination_compensation(candidate,declared,nominal_goal):
    limit=float(declared.get('destination_nominal_z_limit_m',.06))
    if limit==.06:return limit
    allowed={'l_shape_bowl_sink__human_default_mpfb','u_shape_bowl_sink__human_default_mpfb'}
    require(limit==.09 and declared['scene_id'] in allowed and candidate['payload_name']=='bowl',
            '90mm nominal compensation is restricted to the two authorized human bowl scenes')
    region=declared['destination_region_check'];require(region.get('passed') is True,'Missing compensation evidence')
    evidence=read(bound(region['evidence_path'],region['sha256']))
    require(evidence.get('passed') is True and np.allclose(evidence['nominal_goal_world'],nominal_goal,atol=1e-7,rtol=0),
            'Nominal compensation target differs from bound evidence')
    config=evidence['original_config'];require(config=={'path':candidate['config_path'],'sha256':candidate['config_sha256']},'Compensation config differs')
    layout=bound_json(config);source_scene=bound(evidence['original_source_scene']['path'],evidence['original_source_scene']['sha256'])
    reference=evidence['observed_reference'];actual=bound_json(reference['execution_result'])
    require(actual['scene_id']==candidate['scene_id'] and actual['config_sha256']==candidate['config_sha256'] and actual['executed'] is True,
            'Compensation reference is not actual execution of this scene')
    for name,key_path,key_sha in (('physics_states','q_path','q_sha256'),('physics_metadata','metadata_path','metadata_sha256')):
        require(reference[name]=={'path':actual['physics_evidence'][key_path],'sha256':actual['physics_evidence'][key_sha]},'Compensation native reference differs')
    metadata=bound_json(reference['physics_metadata']);index=reference['state_index']
    with np.load(bound(reference['physics_states']['path'],reference['physics_states']['sha256']),allow_pickle=False) as states:
        require(type(index) is int and index==int(np.flatnonzero(states['phase']=='transfer')[-1]),'Compensation does not use the bound final transfer state')
        for key in ('phase','physics_step','action_index','source_trajectory_index'):
            value=str(states[key][index]) if key=='phase' else int(states[key][index])
            require(reference[key]==value,'Compensation observation label differs: '+key)
        palm=state_pose(states['robot_body_pose_w'][index,metadata['robot_body_names'].index('base_link')])
        tcp=palm.copy();tcp[:3,3]+=tcp[:3,:3]@np.array([.131,0,0])
        bowl=state_pose(states['task_object_states'][index,metadata['object_names'].index('bowl'),:7])
    attachment=np.linalg.inv(tcp)@bowl
    for field,value in (('measured_tcp_world',tcp),('measured_bowl_world',bowl),('measured_attachment',attachment)):
        require(np.allclose(reference[field],value,atol=1e-7,rtol=0),'Compensation reference differs from actual native state: '+field)
    predicted=np.asarray(candidate['targets']['lower'])@attachment
    require(np.allclose(evidence['measured_reference_predicted_bowl_world'],predicted,atol=1e-7,rtol=0),'Predicted actual-offset bowl pose differs')
    from pxr import Usd,UsdGeom
    from kitchen_safety.extensions.replicator.task_collision import PayloadGeometry
    stage=Usd.Stage.Open(str(source_scene));prim=stage.GetPrimAtPath(evidence['sink_floor_source_path'])
    require(prim.IsValid() and str(prim.GetPath()).endswith('/Sink/BasinBottom'),'Unexpected source sink floor')
    transform=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(prim)).T
    if prim.IsA(UsdGeom.Mesh):
        points=np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(),dtype=float)@transform[:3,:3].T+transform[:3,3]
        floor=np.array([points.min(0),points.max(0)])
    else:
        box=UsdGeom.BBoxCache(Usd.TimeCode(0),[UsdGeom.Tokens.default_,UsdGeom.Tokens.render,UsdGeom.Tokens.proxy,UsdGeom.Tokens.guide],False,True).ComputeWorldBound(prim).ComputeAlignedRange()
        floor=np.array([box.GetMin(),box.GetMax()])
    asset=local(layout['objects']['bowl']['usd_path']);geometry=PayloadGeometry(asset)
    points=geometry.vertices@predicted[:3,:3].T+predicted[:3,3];bounds=np.array([points.min(0),points.max(0)])
    require(np.allclose(evidence['sink_floor_world_bounds_m'],floor,atol=1e-7,rtol=0)
            and np.allclose(evidence['predicted_source_bowl_world_bounds_m'],bounds,atol=1e-7,rtol=0), 'Compensation geometry differs from original source vertices')
    require(np.all(bounds[0,:2]>=floor[0,:2]) and np.all(bounds[1,:2]<=floor[1,:2])
            and abs(bounds[0,2]-floor[1,2]-.003)<=1e-6,'Measured-reference complete bowl is not within original floor XY at floor+3mm')
    return limit


def audit_declared_segments(candidate,declared,row,kin=None):
    """Audit every declared solver segment and rejected path using independent FK."""
    waypoints=declared.get('phase_waypoints_world',{});limits=declared.get('segment_deviation_limits_m',{})
    require(isinstance(waypoints,dict) and isinstance(limits,dict) and set(waypoints)<=set(PHASES) and set(limits)<=set(PHASES),'Unknown segment phase')
    require(all(isinstance(points,list) and len(points)<=8 for points in waypoints.values()) and sum(map(len,waypoints.values()))<=16,'Too many declared waypoints')
    require(all(not isinstance(v,bool) and isinstance(v,(int,float)) and np.isfinite(v) and 0<v<=.02 for v in limits.values()),'Invalid segment hard gate')
    for points in waypoints.values():
        for value in points:
            t=np.asarray(value);require(t.shape==(4,4) and np.isfinite(t).all() and np.allclose(t[3],[0,0,0,1],atol=1e-8,rtol=0)
                and np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-7,rtol=0) and np.linalg.det(t[:3,:3])>0,'Invalid waypoint pose')
    has_segments=any(bool(waypoints.get(p)) or p in limits for p in PHASES)
    if not has_segments:
        require(all('segments' not in p for p in candidate['phases']),'Undeclared segments')
        return dict(segments_audited=0,independent_fk_samples=0,segments=[])
    if kin is None:kin=SourceFK(read(local(row['config_path'])))
    key='trajectory' if candidate['status']=='generated' else 'partial_trajectory'
    artifact=candidate.get('artifacts',{}).get(key);raw=None
    if artifact:
        with np.load(bound(artifact['path'],artifact['sha256']),allow_pickle=False) as data:raw=dict(data)
    q=np.asarray(candidate['start_q']);checked=[];used_paths=set();total_fk=0
    for phase in candidate['phases']:
        name=phase['phase'];expected=bool(waypoints.get(name)) or name in limits
        require(('segments' in phase)==expected,'Segment structure differs from declaration: '+name)
        phase_q=np.empty((0,7)) if raw is None else raw['q'][raw['phase']==name]
        if not expected:
            if phase['status']=='generated':
                require(len(phase_q)==phase['samples'] and np.allclose(phase_q[0],q,atol=1e-4,rtol=0),'Unsegmented phase source differs')
                q=phase_q[-1].copy()
            else:require(len(phase_q)==0,'Failed phase entered executable partial path')
            continue
        targets=[np.asarray(t) for t in waypoints.get(name,[])]+[np.asarray(candidate['targets'][name])]
        segments=phase['segments'];require(0<len(segments)<=len(targets) and phase['segment_count_declared']==len(targets)
            and phase['segment_count_attempted']==len(segments) and phase['per_segment_max_attempts']==1,'Declared/attempted segment budget differs')
        accepted=[];cursor=0;failed=False
        for index,segment in enumerate(segments):
            target=targets[index];start=kin.fk(q)
            require(segment['segment_index']==index and type(segment['attempts']) is int and segment['attempts']==segment['max_attempts']==1,'Segment repeated or reordered')
            require(np.allclose(segment['target_world'],target,atol=1e-10,rtol=0) and np.allclose(segment['start_world'],start,atol=2e-6,rtol=0)
                and np.allclose(segment['start_q'],q,atol=1e-8,rtol=0),'Segment target/start differs from frozen path')
            require(segment['contact_masks']==phase['contact_masks'] and segment['nominal_payload_attached'] is (name in ('lift','transfer','lower')),'Segment collision/mode changed')
            require(segment['vertical_motion_cost_requested'] is (name in declared.get('linear_motion_phases',[])),'Segment vertical request differs')
            n=segment['samples'];require(type(n) is int and n>=0 and segment['phase_sample_start']==cursor and segment['phase_sample_stop']==cursor+n,'Segment path range differs')
            item=dict(phase=name,segment_index=index,solver_success=segment['solver_success'],solver_status=segment['solver_status'],samples=n)
            if segment['solver_success'] is True:
                reference=segment['trajectory'];path=bound(reference['path'],reference['sha256'])
                require(path.parent==local(row['candidate_planning_result']['path']).parent and path not in used_paths,'Segment artifact reused or outside candidate')
                used_paths.add(path)
                with np.load(path,allow_pickle=False) as data:joints=data['q'].copy();saved=data['tcp_world'].copy()
                require(joints.shape==(n,7) and n>0 and np.isfinite(joints).all() and np.allclose(joints[0],q,atol=1e-4,rtol=0),'Segment joint path differs')
                computed=np.asarray([kin.fk(v) for v in joints]);total_fk+=n
                require(saved.shape==computed.shape and np.isfinite(saved).all() and np.allclose(saved,computed,atol=2e-6,rtol=0),'Saved segment TCP differs from independent source FK')
                deviation=segment_deviation(computed[:,:3,3],start[:3,3],target[:3,3]);limit=limits.get(name);passed=limit is None or deviation<=limit
                require(segment['deviation_limit_m']==limit and np.isclose(segment['max_tcp_segment_deviation_m'],deviation,atol=1e-9,rtol=0)
                    and segment['deviation_gate_passed'] is passed,'Hard segment gate or measured deviation differs')
                if segment['vertical_motion_cost_requested']:
                    temporary=dict(targets={name:target},phases=[segment])
                    numeric.audit_vertical_cost_record(temporary,dict(phase=np.array([name]*n),tcp_world=computed))
                item.update(trajectory=reference,max_tcp_segment_deviation_m=deviation,deviation_limit_m=limit,deviation_gate_passed=passed,
                    start_q=q.tolist(),start_world=start.tolist(),target_world=target.tolist(),independent_fk_verified=True)
                if passed:
                    require(segment['status']=='generated','Accepted segment marked failed');accepted.append(joints);cursor+=n;q=joints[-1].copy()
                else:
                    require(segment['status']=='failed' and segment['solver_status']=='TCP_SEGMENT_DEVIATION_LIMIT'
                        and segment.get('solver_status_before_gate') is not None,'Rejected gate did not retain real solver outcome');failed=True
            else:
                require(segment['solver_success'] is False and segment['status']=='failed' and n==0 and not segment.get('trajectory'),'Failed solver has a fabricated path')
                failed=True
            checked.append(item)
            if failed:require(index==len(segments)-1,'Solver continued after failed segment')
        if failed:
            require(phase['status']=='failed' and phase['failed_segment_index']==len(segments)-1 and phase['samples']==cursor
                and candidate['status']=='failed' and candidate.get('failed_phase')==name and phase is candidate['phases'][-1]
                and len(phase_q)==0,'Rejected segment phase was executed, omitted, or retried')
        else:
            require(len(segments)==len(targets) and phase['status']=='generated' and phase['samples']==cursor
                and np.array_equal(phase_q,np.concatenate(accepted)),'Successful phase does not contain every declared segment vertex')
    return dict(segments_audited=len(checked),independent_fk_samples=total_fk,source_robot_model=getattr(kin,'source',None),segments=checked)


def audit_candidate_targets(candidate, declared, old):
    """Check target/attachment arithmetic without importing the executing planner."""
    def pose(value):
        value = np.asarray(value, dtype=float)
        require(value.shape == (4,4) and np.isfinite(value).all()
                and np.allclose(value[3], [0,0,0,1], atol=1e-8, rtol=0)
                and np.allclose(value[:3,:3].T@value[:3,:3], np.eye(3), atol=1e-7, rtol=0)
                and np.isclose(np.linalg.det(value[:3,:3]), 1., atol=1e-7, rtol=0), 'Invalid frozen target pose')
        return value
    require(set(declared['target_poses_world']) == set(candidate['targets']) == set(PHASES), 'Incomplete target phases')
    targets = {phase: pose(declared['target_poses_world'][phase]) for phase in PHASES}
    for phase, target in targets.items():
        require(np.allclose(pose(candidate['targets'][phase]), target, atol=1e-10, rtol=0), 'Planner changed frozen target: '+phase)
    attachment = pose(declared['nominal_attachment'])
    require(np.allclose(pose(candidate['nominal_attachment']), attachment, atol=1e-10, rtol=0), 'Planner changed attachment transform')
    initial = pose(old['targets']['approach'])@pose(old['nominal_attachment'])
    require(np.allclose(targets['approach']@attachment, initial, atol=1e-7, rtol=0), 'Attachment does not preserve original payload pose')
    original_goal = pose(old['targets']['lower'])@pose(old['nominal_attachment'])
    adjusted_goal = targets['lower']@attachment
    z_limit = audit_destination_compensation(candidate, declared, adjusted_goal)
    require(np.linalg.norm(adjusted_goal[:2,3]-original_goal[:2,3]) <= .06
            and abs(adjusted_goal[2,3]-original_goal[2,3]) <= z_limit, 'New release goal exceeds original 60mm neighborhood')
    require(candidate['payload_name'] == old['payload_name'] == declared['payload_name'], 'Candidate changes payload')
    linear = declared.get('linear_motion_phases', [])
    require(isinstance(linear, list) and len(linear) == len(set(linear)) and set(linear) <= set(PHASES),
            'Invalid vertical-motion phase declaration')



audit_vertical_cost_record = numeric.audit_vertical_cost_record


audit_retiming = numeric.audit_retiming


expected_command_stream = numeric.expected_command_stream


predicate_components = numeric.predicate_components


def audit_entry(row, recipe, revisions, demos, retry_plan, context):
    # The manifest, not a guessed scene directory, selects recovery child results.
    prior_results = audit_history(row, recipe, context)
    plan = audit_candidate(row, recipe)
    rp = bound(row['execution_result_path'], row['execution_result_sha256'])
    require(rp.is_relative_to(demos / 'executions'), 'Physical result outside execution directory')
    result = read(rp)
    for key in ('scene_id', 'task_id', 'variant_id', 'config_path', 'config_sha256',
                'trajectory_path', 'trajectory_sha256', 'execution_status', 'task_success',
                'attempts', 'executed', 'runtime_revision_id', 'physical_attempt_number',
                'source_trajectory_path', 'source_trajectory_sha256', 'prior_physical_attempts',
                'retry_plan_path', 'retry_plan_sha256', 'retry_schema_version', 'candidate_planning_result',
                'previous_retry_result', 'execution_overrides', 'adjustment', 'planning_status'):
        require(row.get(key) == result.get(key), 'Manifest/result mismatch: ' + key)
    require(result['kind'] == 'physical_failure_retry_execution' and result['status'] == 'complete', 'Not a terminal physical result')
    require(result['executed'] is True and type(result['attempts']) is int and result['attempts'] == 1,
            'Expected one actual physical episode')
    require(result['physical_execution'] is True, 'Missing native execution flag')
    for key in ('runtime_state_setters_used', 'physical_weld_used', 'payload_attachment_used', 'planning_rerun'):
        require(result.get(key) is False, 'Forbidden physical substitution: ' + key)
    require(result.get('source_inputs_unchanged') is True and result.get('execution_sources_unchanged') is True,
            'Inputs or code changed during execution')
    revision = revisions[result['runtime_revision_id']]
    require(result['execution_source_sha256_start'] == result['execution_source_sha256_end'] == revision['source_sha256'],
            'Execution code differs from its recorded runtime revision')
    require(result['runtime_settings'] == merged_settings(revision['settings'], recipe['execution_overrides']),
            'Runtime settings do not match revision plus declared close/release overrides')
    # Revision 0 predates the gate summary fields. Audit actual states below in
    # every revision; never manufacture a missing legacy runtime gate result.
    if 'reset_state_validation' in result:
        require(result['reset_state_validation']['passed'] is True, 'Frozen initial state gate failed')
    for path_key, hash_key in (('result_path', 'result_sha256'), ('config_path', 'config_sha256'),
                               ('trajectory_path', 'trajectory_sha256'), ('source_scene_path', 'source_scene_sha256')):
        bound(row[path_key], row[hash_key])
    require(result['original_planning_result'] == {'path': row['result_path'], 'sha256': row['result_sha256']},
            'Result omits original historical planning source')
    require(local(result['planning_result_path']) == local(row['candidate_planning_result']['path'])
            and result['planning_result_sha256'] == row['candidate_planning_result']['sha256'],
            'Actual execution does not bind its new candidate planning result')
    for path, sha in plan['composition']['source_paths_sha256'].items():
        bound(path, sha)
    overlay = result['physical_overlay_binding']
    require(overlay['unchanged'] is True and overlay['sha256_start'] == overlay['sha256_end'], 'Overlay changed')
    bound(overlay['path'], overlay['sha256_start'])
    physics = result['physics_evidence']
    require(physics['continuous'] is True and physics['complete'] is True, 'Incomplete native physics evidence')
    if 'physics_evidence_gate' in result:
        require(result['physics_evidence_gate'] == {'required': True, 'passed': True}, 'Native physics gate failed')
    qp = bound(physics['q_path'], physics['q_sha256'])
    mp = bound(physics['metadata_path'], physics['metadata_sha256'])
    require(qp.parent == mp.parent == rp.parent, 'Physics files do not belong to selected episode')
    metadata = read(mp)
    for key in ('runtime_state_setters_used', 'physical_weld_used', 'payload_attachment_used'):
        require(metadata.get(key) is False, 'Native recorder reports forbidden substitution: ' + key)
    actions_path = rp.parent / 'actions.json'
    actions = read(actions_path)
    require(len(actions) == result['action_count'] > 0, 'Missing or mismatched actuator commands')
    before = np.array([a['before_physics_step'] for a in actions], dtype=np.int64)
    after = np.array([a['after_physics_step'] for a in actions], dtype=np.int64)
    require([a['action_index'] for a in actions] == list(range(len(actions))), 'Non-contiguous action indices')
    require(np.all(after - before == 4) and np.array_equal(before[1:], after[:-1]), 'Missing/duplicated physics substeps between actions')
    require(before[0] == metadata['reset_physics_step'], 'Action stream does not start at recorded reset')
    expected = 1 + int(np.sum(after - before))
    with np.load(qp, allow_pickle=False) as data:
        steps = data['physics_step']
        n = len(steps)
        require(n == expected == physics['samples'] == physics['expected_post_reset_samples'] == metadata['samples'], 'Physics sample count mismatch')
        require(steps[0] == before[0] and steps[-1] == after[-1] and np.all(np.diff(steps) == 1), 'Native physics stream is discontinuous')
        require(data['q'].shape == data['dq'].shape == (n, len(metadata['joint_names'])), 'Joint record shape mismatch')
        for key in data.files:
            values = data[key]
            require(values.shape[0] == n, 'Physics field sample count mismatch: ' + key)
            if values.dtype.kind in 'fiu':
                require(np.isfinite(values).all(), 'Nonfinite physics values: ' + key)
        physics_action_index = data['action_index'].copy()
        physics_phase = data['phase'].copy()
        physics_source_index = data['source_trajectory_index'].copy()
        require(physics_action_index[0] == physics_source_index[0] == -1 and physics_phase[0] == 'reset',
                'Native stream does not begin with the sole reset observation')
        native_object_states = data['task_object_states'].copy()
        native_destination_force = data['destination_contact_force'][:, 0, 0].copy()
        arm_ids = [metadata['joint_names'].index('panda_joint' + str(i)) for i in range(1, 8)]
        require(np.allclose(data['q'][0, arm_ids], result['reset_observed_arm_q'], atol=1e-7, rtol=0), 'Reset q does not match physical record')
        initial_q_error = float(np.max(np.abs(data['q'][0, arm_ids] - np.asarray(plan['start_q']))))
        require(initial_q_error <= .01, 'Recorded initial q differs from frozen planning q')
        layout = read(local(row['config_path']))
        for name, state in zip(metadata['object_names'], data['task_object_states'][0]):
            spec = layout['objects'][name]
            half = np.deg2rad(spec.get('yaw_deg', 0.)) / 2
            expected_quat = np.array([0., 0., np.sin(half), np.cos(half)])
            quat = state[3:7]
            require(.99 < np.linalg.norm(quat) < 1.01, 'Invalid initial object quaternion')
            angle = float(np.rad2deg(2*np.arccos(np.clip(abs(np.dot(quat/np.linalg.norm(quat), expected_quat)), 0., 1.))))
            require(np.linalg.norm(state[:3]-spec['position']) <= .002 and angle <= 1.,
                    'Recorded initial object pose differs from frozen scene: ' + name)
    require(metadata['continuous_substeps'] is True and metadata['callback_errors'] == [], 'Native recorder errors')
    require(abs(metadata['physics_dt_s'] - 1/120) < 1e-12 and abs(metadata['control_dt_s'] - 1/30) < 1e-12, 'Unexpected execution timing')
    with np.load(local(row['trajectory_path']), allow_pickle=False) as planned:
        expected_actions = expected_command_stream(plan, planned, result['runtime_settings'])
        require(len(actions) <= len(expected_actions), 'Commands exceed the single planned attempt')
        for action, expected_action in zip(actions, expected_actions):
            require(action['phase'] == expected_action['phase'] and action['source_trajectory_index'] == expected_action['source_trajectory_index']
                    and np.array_equal(np.asarray(action['target'],dtype=np.float32),expected_action['target']),
                    'Actual command/hold differs from adjusted NPZ and original holds')
        if result.get('sequence_complete'):
            require(len(actions) == len(expected_actions), 'Complete episode omitted commands/holds')
        source_actions = [a for a in actions if a['source_trajectory_index'] >= 0]
        require([a['source_trajectory_index'] for a in source_actions] == list(range(len(source_actions))), 'Planned action sequence skipped/repeated')
        for action in source_actions:
            index = action['source_trajectory_index']
            target = np.r_[planned['q'][index], planned['gripper'][index]].astype(np.float32)
            require(action['phase'] == str(planned['phase'][index])
                    and np.array_equal(np.asarray(action['target'], dtype=np.float32), target), 'Executed target differs from frozen trajectory')
        if result.get('sequence_complete'):
            require(len(source_actions) == len(planned['q']), 'Complete sequence omitted planning samples')
    predicate_metadata_path = rp.parent / 'task_predicate_metadata.json'
    predicate_metadata = read(predicate_metadata_path)
    require(predicate_metadata['task_id'] == row['task_id'], 'Wrong task predicate metadata')
    for name, sha in predicate_metadata['input_sha256'].items():
        bound(rp.parent / name, sha)
    with np.load(rp.parent / 'task_predicate_actions.npz', allow_pickle=False) as predicates:
        require(np.array_equal(predicates['physics_step'], after), 'Action predicates not synchronized to native steps')
        require(np.array_equal(predicates['action_index'], np.arange(len(actions))), 'Predicate action indices differ')
        observed = bool(np.any(predicates['terminal_success']))
        final = bool(predicates['original_object_on_destination'][-1])
        if 'original_task_success' in result:
            require(result['original_task_success'] is observed, 'Original task success differs from recorded predicate')
        if 'final_original_destination_predicate' in result:
            require(result['final_original_destination_predicate'] is final, 'Final destination predicate mismatch')
        require(result['task_success'] is (result['execution_status'] == 'success'), 'Success/status contradiction')
        if result['task_success']:
            require(observed and final and result.get('sequence_complete') is True, 'Success lacks original terminal/final predicate')
    with np.load(rp.parent / 'task_predicate_physics.npz', allow_pickle=False) as predicates:
        require(np.array_equal(predicates['physics_step'], steps), 'Physics predicate timeline differs')
    require(np.array_equal(physics_action_index[1:], np.repeat(np.arange(len(actions)),4)), 'Physics/action indices differ')
    require(np.array_equal(physics_phase[1:], np.repeat([x['phase'] for x in actions],4)), 'Physics phase differs from command')
    require(np.array_equal(physics_source_index[1:], np.repeat([x['source_trajectory_index'] for x in actions],4)), 'Physics source index differs')
    for key in ('source_first_pass_manifest','source_camera_retake_manifest','source_previous_retry_manifest'):
        require(result[key] == retry_plan[key], 'Original manifest binding differs')
        bound(result[key]['path'],result[key]['sha256'])
    for key,value in recipe.items():
        require(row.get(key) == value, 'Manifest differs from frozen retry recipe: '+key)
    bound(row['source_trajectory_path'],row['source_trajectory_sha256'])
    evidence = result['evidence']
    for name,binding in evidence.items():
        require(local(binding['path']).parent == rp.parent and local(binding['path']).name == name, 'Evidence path outside selected episode')
        bound(binding['path'],binding['sha256'])
    required_evidence={'actions.json','physics_states.npz','physics_metadata.json','grasp_contacts.json',
        'task_predicate_actions.npz','task_predicate_physics.npz','task_predicate_metadata.json'}
    require(required_evidence <= set(evidence), 'Incomplete evidence file manifest')
    first = prior_results[0]
    require(result['physical_attempt_number']==len(result['prior_physical_attempts'])+1,'New attempt number omits history')
    require(result['controller']==first['controller'],'Controller/physics settings changed')
    require(result['initial_object_pose_reference']==first['initial_object_pose_reference'],'Object initial poses changed')
    require(result['physical_overlay_binding']['sha256_start']==first['physical_overlay_binding']['sha256_start'],'Physical kitchen overlay changed')
    first_metadata_path = local(result['prior_physical_attempts'][0]['result_path']).with_name('task_predicate_metadata.json')
    first_metadata=read(first_metadata_path)
    previous_metadata_binding = prior_results[-1]['evidence']['task_predicate_metadata.json']
    previous_metadata = bound_json(previous_metadata_binding)
    require(predicate_metadata['thresholds']==first_metadata['thresholds'],'Native predicate thresholds changed')
    require(predicate_metadata['actual_spawned_local_bounds']==first_metadata['actual_spawned_local_bounds'],'Native predicate geometry changed')
    require(predicate_metadata['sensor_identity']==predicate_metadata['sensor_identity_at_end']==first_metadata['sensor_identity'],'Original contact target/filter identity changed')
    for key in ('thresholds', 'actual_spawned_local_bounds', 'sensor_identity', 'pick_up_object', 'destination_location'):
        require(predicate_metadata[key] == previous_metadata[key], 'Predicate differs from latest actual source-bound evidence: ' + key)
    for source, expected_sha in first['execution_source_sha256_start'].items():
        if '/tasks/' in source or source.endswith('task_collection_evidence.py'):
            require(result['execution_source_sha256_start'].get(source) == expected_sha,
                    'Original task/predicate source changed: ' + source)
    for source, expected_sha in prior_results[-1]['execution_source_sha256_start'].items():
        if source.endswith(('planned_execution.py', 'task_collection_evidence.py')):
            require(result['execution_source_sha256_start'].get(source) == expected_sha,
                    'Latest actual native recorder/controller source changed: ' + source)
    recomputed_samples=0
    for name in ('actions','physics'):
        with np.load(rp.parent/('task_predicate_'+name+'.npz'),allow_pickle=False) as predicate_data:
            indexes = np.arange(n) if name == 'physics' else after-steps[0]
            for key in predicate_data.files:
                values = predicate_data[key]
                require(values.shape[0] == len(indexes), 'Predicate sample count differs: ' + key)
                if values.dtype.kind in 'fiub':
                    require(np.isfinite(values).all(), 'Nonfinite predicate samples: ' + key)
            require(np.array_equal(predicate_data['phase'], physics_phase[indexes])
                    and np.array_equal(predicate_data['action_index'], physics_action_index[indexes]),
                    'Predicate phase/action differs from native physics')
            payload_index = metadata['object_names'].index(predicate_metadata['pick_up_object'])
            measured = native_object_states[indexes, payload_index]
            for key, native in (('object_pose', measured[:, :7]), ('object_linear_velocity', measured[:, 7:10]),
                                ('destination_contact_force', native_destination_force[indexes])):
                require(predicate_data[key].shape == native.shape
                        and np.allclose(predicate_data[key], native, atol=1e-6, rtol=0),
                        'Predicate field differs from native physics: ' + name + '/' + key)
            if predicate_metadata['destination_location'] in metadata['object_names']:
                destination_index = metadata['object_names'].index(predicate_metadata['destination_location'])
                require(np.allclose(predicate_data['destination_pose'], native_object_states[indexes, destination_index, :7],
                                    atol=1e-6, rtol=0), 'Destination pose differs from native physics')
            box = predicate_metadata['actual_spawned_local_bounds'][predicate_metadata['pick_up_object']]
            center = (np.asarray(box['min']) + np.asarray(box['max'])) / 2
            quaternion = measured[:, 3:7]
            t = 2*np.cross(quaternion[:, :3], center)
            centroid = measured[:, :3] + center + quaternion[:, 3:4]*t + np.cross(quaternion[:, :3], t)
            require(np.allclose(centroid, predicate_data['object_centroid_world'], atol=1e-6, rtol=0),
                    'Predicate centroid differs from native pose and spawned bounds')
            components=predicate_components(predicate_data,predicate_metadata)
            on=components['within_destination'] & components['upward_support'] & components['slow']
            require(np.array_equal(on,predicate_data['original_object_on_destination']),'Recomputed native destination predicate differs: '+name)
            recomputed_samples+=len(on)
            above = predicate_data['has_original_rest_pose'] & (predicate_data['object_pose'][:, 2]
                    > predicate_data['original_rest_position'][:, 2]+.01)
            require(np.array_equal(above, predicate_data['original_above_rest']), 'Original strict lift predicate differs')
            if name=='actions':
                final_components={k:bool(v[-1]) for k,v in components.items()}
                audit_terminal_order(predicate_data, predicate_metadata)
    if result.get('sequence_complete'):
        require(result['task_success'] is (observed and final),'Complete task outcome does not equal original ever-success AND final placement')
    for key in ('rendering_enabled','camera_entities_enabled','video_expected'):
        require(result[key] is False,'Rendering/video was enabled: '+key)
    require(row.get('video_path') is None and result.get('video_path') is None and result.get('video_sha256') is None,'Unexpected video')
    for name in ('render_state_start','render_state_end'):
        state=result[name]
        require(state['passed'] is True and not any(state[k] for k in ('is_rendering','render_enabled','has_gui','rtx_sensors','offscreen',
            'visualizer_types','visualizer_count','camera_sensors','render_generation')),'Nonzero rendering/camera state: '+name)
    require(result['render_generation_delta']==0,'Render generation advanced')
    require(not any(p.suffix.lower() in ('.mp4','.png','.jpg','.jpeg') for p in rp.parent.rglob('*')),'Unexpected rendered frame/video')
    for source,expected_sha in result['execution_source_sha256_end'].items():bound(source,expected_sha)
    bound(row['execution_result_path'],row['execution_result_sha256'])
    return dict(scene_id=row['scene_id'],task_id=row['task_id'],status='passed',execution_status=result['execution_status'],
        executed=True,attempts=1,physical_attempt_number=result['physical_attempt_number'],task_success=result['task_success'],
        runtime_revision_id=result['runtime_revision_id'],physics_samples=n,actions=len(actions),
        recorded_initial_state_independently_checked=True,initial_q_error_rad=initial_q_error,
        adjusted_commands=len(source_actions),full_expected_actions=len(expected_actions),
        original_task_terminal_observed=observed,final_original_destination_predicate=final,
        final_native_predicate_components=final_components,native_predicate_samples_recomputed=recomputed_samples,
        rendered_frames=0,no_render_gate_passed=True,source_geometry_unchanged=True,controller_unchanged=True,
        result=file_binding(rp),planning_status=row['planning_status'],candidate_planning_result=row['candidate_planning_result'],
        previous_retry_result=row['previous_retry_result'],execution_overrides=row['execution_overrides'],
        nominal_gripper_joint_rad=plan['nominal_gripper_joint_rad'],
        planning_gripper_joint_rad=plan['planning_gripper_joint_rad'],
        runtime_substitution_flags_verified=True,
        physics=ref(qp),actions_evidence=ref(actions_path),predicates_evidence=ref(predicate_metadata_path))


audit_terminal_order = numeric.audit_terminal_order


def audit_planning_failure(row, recipe, demos, context):
    require(row['execution_status'] == 'planning_failed' and row['planning_status'] == 'failed',
            'Planning-failure status mismatch')
    require(type(row['attempts']) is int and row['attempts'] == 0 and row['executed'] is False
            and row['task_success'] is False, 'Failed planning consumed a physical attempt')
    require(not row.get('execution_result_path') and not row.get('execution_result_sha256')
            and row.get('video_path') is None and row.get('video_sha256') is None,
            'Planning failure has unexpected physical result/video')
    require(not (demos/'executions'/row['scene_id']).exists(), 'Planning failure has an unexpected physical episode directory')
    audit_history(row, recipe, context)
    candidate = audit_candidate(row, recipe)
    return dict(scene_id=row['scene_id'], task_id=row['task_id'], status='passed',
        planning_status='failed', execution_status='planning_failed', executed=False, attempts=0,
        task_success=False, physical_attempt_number=None,
        reserved_next_physical_attempt_number=row['physical_attempt_number'],
        candidate_planning_result=row['candidate_planning_result'], previous_retry_result=row['previous_retry_result'],
        failure_reason=candidate['failure_reason'], physics_samples=0, native_predicate_samples_recomputed=0,
        no_render_gate_passed=True, no_physical_episode_created=True,
        scope='A single failed planning candidate; no physical runtime, action, or render was created.')


def audit_manifest(manifest, plan):
    require(manifest['kind'] == 'physical_failure_retry' and plan['kind'] == 'physical_failure_retry_plan', 'Wrong retry kind')
    require(manifest['schema_version'] == plan['schema_version'] == 2
            and manifest['retry_round'] == plan['retry_round'] == 4, 'Wrong physical retry round/schema')
    for key,value in dict(expected_scene_count=6,candidate_count_per_scene=1,physical_retries_per_scene=1,
                          layout_changed=False,success_predicate_changed=False).items():
        require(plan.get(key) == value, 'Frozen authorization differs: '+key)
    original_document=bound_json(plan['source_first_pass_manifest'])
    retake_document=bound_json(plan['source_camera_retake_manifest'])
    round3_document=bound_json(plan['source_previous_retry_manifest'])
    round2_document=bound_json(round3_document['source_previous_retry_manifest'])
    round1_document=bound_json(round2_document['source_previous_retry_manifest'])
    original,retakes=rows_by_id(original_document),rows_by_id(retake_document)
    original_failed={sid for sid,row in original.items() if row['execution_status']=='failed'}
    require(len(original)==154 and len(original_failed)==84, 'Unexpected original 154-scene history')
    unresolved=original_failed;prior_rounds=[]
    expected_counts=({'success':36,'failed':48},{'success':19,'failed':10,'planning_failed':19},
                     {'success':23,'failed':4,'planning_failed':2})
    for number,(document,expected) in enumerate(zip((round1_document,round2_document,round3_document),expected_counts),1):
        historical=rows_by_id(document)
        require(document['kind']=='physical_failure_retry' and document['stage']=='complete'
                and document['schema_version']==(1 if number==1 else 2)
                and (number==1 or document['retry_round']==number)
                and set(historical)==unresolved
                and Counter(row['execution_status'] for row in historical.values())==expected,
                'Historical completed retry cohort/counts differ: round'+str(number))
        frozen=read(bound(document['retry_plan_path'],document['retry_plan_sha256']));declared=rows_by_id(frozen)
        require(set(declared)==set(historical),'Historical frozen plan cohort changed')
        for sid,row in declared.items():
            for key,value in row.items():
                require(historical[sid].get(key)==value,'Historical manifest differs from frozen plan: '+sid+' '+key)
        for key in ('source_first_pass_manifest','source_camera_retake_manifest'):
            require(document[key]==frozen[key]==plan[key],'Previous round historical source differs')
        if number>1:
            require(frozen['source_previous_retry_manifest']==document['source_previous_retry_manifest'],
                    'Historical retry plan does not bind previous manifest')
        unresolved={sid for sid,row in historical.items() if row['execution_status'] in ('failed','planning_failed')}
        prior_rounds.append((document,historical))
    require(retake_document['stage']=='complete','Camera retake history is not complete')
    rows,recipes=rows_by_id(manifest),rows_by_id(plan)
    require(len(rows)==len(recipes)==manifest['total']==6 and set(rows)==set(recipes)==unresolved,
            'Round4 whitelist differs from exactly the 6 round3 unresolved scenes')
    for key in ('source_first_pass_manifest','source_camera_retake_manifest','source_previous_retry_manifest'):
        require(manifest[key]==plan[key],'Manifest historical source binding differs: '+key)
    require(manifest['rendering_enabled'] is False and manifest['video_expected'] is False
            and manifest['one_additional_attempt_per_scene'] is True,'Manifest runtime scope differs')
    counts=Counter(row['execution_status'] for row in rows.values())
    require(set(counts)<=TERMINAL|{'pending','running'},'Unknown execution state')
    require(all(manifest['counts'].get(key)==counts[key] for key in ('pending','running','success','failed','error','planning_failed')),
            'Manifest count mismatch')
    progress=bound_json(plan['planning_progress']);progress_rows=rows_by_id(progress)
    require(progress['stage']=='complete' and progress['retry_round']==4 and progress['total']==6
            and set(progress_rows)==set(recipes) and progress['source_previous_retry_manifest']==plan['source_previous_retry_manifest'],
            'Frozen planning progress cohort or provenance differs')
    for sid,row in rows.items():
        require(row['retry_plan_path']==manifest['retry_plan_path'] and row['retry_plan_sha256']==manifest['retry_plan_sha256'],
                'Row retry-plan binding differs')
        if row['execution_status'] in ('pending','planning_failed'):
            require(type(row['attempts']) is int and row['attempts']==0 and row['executed'] is False,'Unexecuted candidate consumed a trial')
        else:require(type(row['attempts']) is int and row['attempts']==1,'Attempted candidate must consume exactly one trial')
        for key,value in recipes[sid].items():
            require(row.get(key)==value,'Manifest differs from frozen plan: '+key)
        planned=progress_rows[sid]
        require(planned['attempts']==1 and planned['status']==row['planning_status']
                and planned['candidate_planning_result']==row['candidate_planning_result']
                and planned['recipe_source']==row['adjustment']['recipe_source'],'Candidate differs from frozen planning progress')
        for startup in planned.get('initialization_errors',[]):
            error=bound_json(startup)
            require(error.get('phases')==[] and error.get('artifacts')=={} and error.get('stage')=='initializing'
                    and error.get('failure_reason')=="RuntimeError('No CUDA GPUs are available')"
                    and error['recipe']==planned['recipe'],'Startup recovery is not an identical zero-solver recipe')
    revisions=manifest['runtime_revisions']
    require(len(revisions)==len({revision['id'] for revision in revisions})>0,'Missing/duplicate runtime revision')
    for revision in revisions:
        settings=revision['settings']
        for key,value in dict(device='cpu',headless=True,enable_cameras=False,visualizers=[],rendering_enabled=False,
                              physics_dt=1/120,control_dt=1/30).items():
            require(settings.get(key)==value,'Revision runtime scope differs: '+key)
        require(settings['retry_plan_path']==manifest['retry_plan_path'] and settings['retry_plan_sha256']==manifest['retry_plan_sha256'],
                'Revision retry-plan binding differs')
        for key,value in plan['execution_settings'].items():
            require(settings[key]==value,'Revision changes frozen base execution settings: '+key)
    return dict(original=original,retakes=retakes,previous=prior_rounds[-1][1],prior_rounds=prior_rounds,
                previous_manifest=round3_document,recipes=recipes,counts=counts)


def audit_episode_inventory(manifest, demos):
    """Reject extra or orphaned native episodes outside the selected result paths."""
    selected = {local(row['execution_result_path']) for row in manifest['entries'] if row.get('execution_result_path')}
    recorded = {path.resolve() for path in (demos/'executions').rglob('result.json')}
    require(recorded == selected, 'Execution directory contains missing, extra, or unbound physical result files')
    expected_directories = {path.parent for path in selected}
    for filename in ('physics_states.npz', 'physics_metadata.json', 'actions.json'):
        observed = {path.parent.resolve() for path in (demos/'executions').rglob(filename)}
        require(observed == expected_directories, 'Native evidence contains an extra or orphaned episode: '+filename)


def audit_startup_failure_reviews(plan):
    """Retain startup observations without inventing a saved manifest snapshot."""
    documents = []
    recipes = rows_by_id(plan)
    folder = ROOT/'reports/replicator_physical_retry_round4_20261029'
    for path in sorted(folder.glob('pre_episode_startup_failure*_review.json')):
        review = read(path)
        require(review['kind'] == 'pre_episode_process_startup_failure_review'
                and review['exit_code'] == 139 and review['physical_attempts_consumed'] == 0
                and review['runtime_modified'] is False, 'Unexpected startup failure review scope')
        log_path = bound(review['source_log']['path'],review['source_log']['sha256'])
        bound(review['driver_source']['path'],review['driver_source']['sha256'])
        log = log_path.read_text()
        require("appState = 'startup'" in log and 'exit code 139' in log
                and 'PHYSICAL_RETRY_START' not in log and 'PHYSICAL_RETRY_DONE' not in log,
                'Crash log does not support a failure before any scene execution marker')
        selected = review['selected']
        require(selected and len({row['scene_id'] for row in selected}) == len(selected), 'Invalid startup selection')
        for row in selected:
            require(row['scene_id'] in recipes and recipes[row['scene_id']]['planning_status'] == 'generated'
                    and row['execution_status'] == 'pending' and row['attempts'] == 0
                    and row['executed'] is False and row['episode_paths'] == [],
                    'Startup observation contains an attempted or unselected scene')
        # This was a digest observed before resume, not an archived copy. The
        # live manifest legitimately advances and must not be SHA-checked here.
        documents.append(dict(review=file_binding(path),source_log=review['source_log'],driver_source=review['driver_source'],
            exit_code=review['exit_code'],physical_attempts_consumed=0,
            selected_scene_ids=[row['scene_id'] for row in selected],
            pre_resume_manifest_observation=review['manifest_before_resume'],
            pre_resume_manifest_bytes_preserved=False,pre_resume_manifest_digest_revalidated=False,
            log_independently_checked=True,
            limitation='The pre-resume full manifest bytes were not saved. Its historical digest and pending-row observations are retained as observations, not claimed as a revalidated file. Final actual lineage and all episode directories are independently audited.'))
    return documents


def main(argv=None):
    started = time.monotonic()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demos', default=DEFAULT_DEMOS)
    parser.add_argument('--output', default=DEFAULT_OUTPUT)
    parser.add_argument('--require-complete', action='store_true')
    parser.add_argument('--scene', default='')
    args = parser.parse_args(argv)
    demos = local(args.demos)
    manifest_path = demos/'manifest.json'
    raw = manifest_path.read_bytes()
    manifest_sha = hashlib.sha256(raw).hexdigest()
    manifest = json.loads(raw)
    errors, checked = [], []
    plan = None
    context = None
    startup_reviews = []
    counts = Counter(row.get('execution_status') for row in manifest.get('entries', []))
    try:
        plan = read(bound(manifest['retry_plan_path'], manifest['retry_plan_sha256']))
        context = audit_manifest(manifest, plan)
        startup_reviews = audit_startup_failure_reviews(plan)
        if args.require_complete:
            require(manifest['stage'] == 'complete' and sum(counts[key] for key in TERMINAL) == 6,
                    'All 6 candidates are not terminal')
            audit_episode_inventory(manifest, demos)
        if args.scene:
            require(args.scene in context['recipes'], 'Unknown audit scene')
    except Exception as error:
        errors.append(dict(scope='manifest', error=f'{type(error).__name__}: {error}'))
    revisions = {revision['id']: revision for revision in manifest.get('runtime_revisions', [])}
    if context is not None:
        for row in manifest['entries']:
            if row['execution_status'] not in TERMINAL or (args.scene and row['scene_id'] != args.scene):
                continue
            try:
                recipe = context['recipes'][row['scene_id']]
                if row['execution_status'] == 'planning_failed':
                    checked.append(audit_planning_failure(row, recipe, demos, context))
                else:
                    checked.append(audit_entry(row, recipe, revisions, demos, plan, context))
            except Exception as error:
                errors.append(dict(scene_id=row['scene_id'], error=f'{type(error).__name__}: {error}'))
    if args.scene and not checked:
        errors.append(dict(scope='selection', error='Requested scene is not a fully audited terminal outcome'))
    changed = digest(manifest_path) != manifest_sha
    if changed:
        errors.append(dict(scope='manifest', error='Manifest changed during audit; this snapshot cannot certify the new manifest'))
    require_complete_coverage = args.require_complete and not args.scene
    if require_complete_coverage and len(checked) != 6:
        errors.append(dict(scope='coverage', error='Complete audit must cover all 6 candidates, including planning failures'))
    report = dict(schema_version=2, kind='physical_retry_round4_independent_audit',
        created_at=datetime.now(timezone.utc).isoformat(), elapsed_seconds=time.monotonic()-started, passed=not errors,
        scope=('Independent CPU read-only audit: exact 6 unresolved-scene cohort; full physical history; one new candidate and at most one native trial; '
               'actual new-grip/hold commands; continuous 120Hz state and original predicates; frozen geometry/controller; zero render/camera runtime; source SHA. '
               'No new physics/GPU, collision validation, or safety acceptance.'),
        manifest_snapshot={'path': str(manifest_path.relative_to(ROOT)), 'sha256': manifest_sha},
        manifest_snapshot_sha256=manifest_sha, manifest_changed_during_audit=changed,
        require_complete=args.require_complete, selected_scene=args.scene or None, manifest_snapshot_counts=dict(counts),
        audited_scenes=len(checked), physical_executions=sum(entry['executed'] for entry in checked),
        planning_failures=sum(entry['execution_status'] == 'planning_failed' for entry in checked),
        physical_successes=sum(entry['task_success'] for entry in checked),
        native_physics_samples=sum(entry['physics_samples'] for entry in checked),
        native_predicate_samples_recomputed=sum(entry['native_predicate_samples_recomputed'] for entry in checked),
        all6_audited=len(checked) == 6 and not errors, errors=errors, entries=checked,
        startup_failure_reviews=startup_reviews,
        auditor_source=file_binding(Path(__file__)),
        numerical_auditor_source=file_binding(Path(numeric.__file__)))
    if plan is not None:
        report.update(original_first_pass_manifest=plan['source_first_pass_manifest'],
            original_camera_retake_manifest=plan['source_camera_retake_manifest'],
            source_previous_retry_manifest=plan['source_previous_retry_manifest'])
    output = local(args.output)
    require(not output.is_relative_to(demos), 'Audit output must not modify physical execution evidence')
    require(output.suffix == '.json' and output not in {Path(key[0]) for key in HASHES},
            'Audit output must be a separate JSON, never a bound input or source file')
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix+'.tmp')
    temporary.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    temporary.replace(output)
    print(json.dumps({key: value for key, value in report.items() if key != 'entries'}, ensure_ascii=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
