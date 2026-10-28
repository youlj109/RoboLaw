"""Read-only CPU facts review of existing native water execution records.

No runtime modules, simulator, CUDA, FCL, or scene state are loaded. Captured
source text is audited rather than whichever controller happens to exist now.
This report never grants physical acceptance or turns a partial trial into success.
"""
import argparse
import ast
import hashlib
import json
import math
from pathlib import Path

import numpy as np

TASK = 'kitchen_bench_replicator_g_shape_bowl_sink'
FINGERS = ['finger_joint', 'right_outer_knuckle_joint', 'left_inner_finger_joint',
           'right_inner_finger_joint', 'left_inner_finger_knuckle_joint',
           'right_inner_finger_knuckle_joint']
BOWL_PHASES = {'pregrasp', 'approach', 'close', 'lift', 'transport', 'lower',
               'release', 'retreat', 'final_settle'}
REVIEWED_SOURCES = [
    'scripts/extensions/replicator/hard_execution_v1/water_collect.py',
    'kitchen_safety/extensions/replicator/hard_execution_v1/water_runtime.py',
    'kitchen_safety/extensions/replicator/hard_execution_v1/water_control.py',
    'kitchen_safety/extensions/replicator/hard_execution_v1/water_actions.py',
]
STATE_WRITERS = {'write_joint_state_to_sim', 'write_joint_position_to_sim',
    'write_joint_velocity_to_sim', 'write_root_state_to_sim',
    'write_root_pose_to_sim', 'write_root_velocity_to_sim',
    'write_root_com_pose_to_sim', 'write_root_link_pose_to_sim',
    'set_joint_positions', 'set_joint_velocities', 'set_world_poses',
    'set_local_poses', 'set_joint_efforts', 'apply_forces_and_torques_at_pos'}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def review(episode):
    episode = Path(episode)
    files = ['water_physics.json', 'physics_states.npz', 'physics_metadata.json',
        'result.json', 'task_predicate_metadata.json', 'source_relation_audit.json',
        'task_collection_provenance.json', 'valve_plans.json', 'actions.json']
    hashes = {name: digest(episode/name) for name in files}
    read = lambda name: json.loads((episode/name).read_text())
    water, meta, result = (read(n) for n in files[:1]+files[2:4])
    predicate, relation, provenance = (read(n) for n in files[4:7])
    plans, actions = read('valve_plans.json'), read('actions.json')
    with np.load(episode/'physics_states.npz', allow_pickle=False) as archive:
        arrays = {name: archive[name] for name in archive.files}
    rows, contacts = water['rows'], water['raw_contacts']
    steps = arrays['physics_step'].astype(np.int64)
    water_steps = np.array([r['physics_step'] for r in rows], dtype=np.int64)
    contact_steps = np.array([r['physics_step'] for r in contacts], dtype=np.int64)
    angles = np.array([r['angle_deg'] for r in rows], dtype=float)
    speeds = np.array([r['speed_deg_s'] for r in rows], dtype=float)
    lever = np.array([r['lever_pose'] for r in rows], dtype=float)
    dt = float(meta['physics_dt_s'])
    config = result['water_config']
    initial_angle = float(config['initial_angle_deg'])
    off_angle = float(config['flow_off_angle_deg'])
    low, high = np.array(config['contact_region_lever_local_m'], dtype=float)
    force_threshold = float(config['close_success']['minimum_loaded_contact_n'])
    near_threshold = .00025
    step_index = {int(s): i for i,s in enumerate(steps)}
    loaded = []
    all_contact_arrays_valid = True
    for row in contacts:
        intentional = []
        for name, finger in row['fingers'].items():
            points = np.asarray(finger['points_object_local_m'], dtype=float).reshape(-1,3)
            force = np.asarray(finger['normal_forces_n'], dtype=float)
            gaps = np.asarray(finger['separations_m'], dtype=float)
            valid = (len(points) == len(force) == len(gaps) == finger['count'] and
                all(np.isfinite(v).all() for v in (points, force, gaps)))
            all_contact_arrays_valid &= valid
            if not valid:
                continue
            keep = (np.abs(force)>force_threshold) & (gaps<=near_threshold) & np.all((points>=low)&(points<=high), axis=1)
            intentional.extend(dict(finger=name, signed_force_n=float(force[i]),
                separation_m=float(gaps[i]), point_lever_local_m=points[i].tolist())
                for i in np.flatnonzero(keep))
        if intentional:
            loaded.append(dict(physics_step=int(row['physics_step']), contacts=intentional))
    first_contact = loaded[0] if loaded else None
    moved = np.flatnonzero(initial_angle-angles>.5)
    first_movement = int(water_steps[moved[0]]) if len(moved) else None
    first_contact_step = first_contact['physics_step'] if first_contact else None
    causal = (first_contact_step is not None and first_movement is not None and first_contact_step<first_movement)
    before = angles[water_steps<first_contact_step] if first_contact_step is not None else angles
    x,y,z,w = lever[:,3:].T
    body_yaw = np.degrees(np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z)))
    body_angle = (body_yaw-float(config['closed_world_yaw_deg'])+180)%360-180
    angular_error = np.abs((body_angle-angles+180)%360-180)
    flowing = np.array([bool(r['flowing']) for r in rows])
    fractions = np.array([r['open_fraction'] for r in rows], dtype=float)
    expected_fraction = np.clip((angles-off_angle)/(initial_angle-off_angle),0.,1.)
    finger_ids = [meta['joint_names'].index(name) for name in FINGERS]
    observed_fingers = np.array([r['gripper_observed_joint_positions_rad'] for r in rows], dtype=float)
    aligned = np.array_equal(steps,water_steps)
    finger_error = float(np.max(np.abs(observed_fingers-arrays['q'][:,finger_ids]))) if aligned and observed_fingers.shape==(len(steps),6) else None
    shape_checks = [r.get('flow_geometry') for r in rows if r['flowing']]
    complete_flow = all(v is not None for v in shape_checks)
    flow_values = [dict(physics_step=r['physics_step'],phase=r['phase'],**r['flow_geometry']) for r in rows if r.get('flow_geometry',{}).get('active')]
    min_flow = min(flow_values,key=lambda r:r['distance_m']) if flow_values else None
    manifest = provenance['entries']
    source_review = []
    reviewed_sources = list(REVIEWED_SOURCES)
    lower_adapter = 'kitchen_safety/extensions/replicator/hard_execution_v1/water_bowl_lower.py'
    if lower_adapter in manifest:
        reviewed_sources.append(lower_adapter)
    for path in reviewed_sources:
        entry = manifest.get(path,{})
        source = entry.get('source_text')
        source_hash = hashlib.sha256(source.encode()).hexdigest() if source is not None else None
        calls=[]
        if source is not None:
            for node in ast.walk(ast.parse(source)):
                if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute) and node.func.attr in STATE_WRITERS:
                    calls.append(dict(line=node.lineno,call=ast.unparse(node.func)))
        source_review.append(dict(path=path,captured_source_present=source is not None,
            captured_sha256=source_hash,declared_sha256=entry.get('initial_sha256'),
            recorded_unchanged=entry.get('unchanged'),state_setter_or_direct_force_calls=calls,
            hash_matches=source_hash is not None and source_hash==entry.get('initial_sha256')==entry.get('checked_sha256')))
    runtime_source=manifest[REVIEWED_SOURCES[1]]['source_text']
    controller_source=manifest[REVIEWED_SOURCES[2]]['source_text']
    source_methods = {node.name:ast.get_source_segment(controller_source,node) for node in ast.walk(ast.parse(controller_source)) if isinstance(node,(ast.FunctionDef,ast.AsyncFunctionDef))}
    single_contact_controller = 'each_step_any_loaded_near_in_region' in source_methods.get('close_valve','')
    old_bilateral_reject = any('both_fingers' in ast.unparse(node.test) and any(isinstance(x,ast.Raise) for x in ast.walk(node))
        for node in ast.walk(ast.parse(source_methods.get('close_valve','def missing(): pass'))) if isinstance(node,ast.If))
    source_task = relation['layout']
    relation_on_counter = any(r.get('kind')=='on' and r.get('subject')=='bowl' and r.get('reference')=='sink_counter' and r.get('passed') for r in relation['checks'])
    identity=predicate['sensor_identity']
    bowl_steps = [r['physics_step'] for r in rows if r['phase'] in BOWL_PHASES]
    bowl_flow_steps = [r['physics_step'] for r in rows if r['phase'] in BOWL_PHASES and r['flowing']]
    stable_n = int(math.ceil(float(config['close_success']['stable_seconds'])/dt))
    tail=slice(max(0,len(rows)-stable_n),len(rows))
    # User-defined completion is measured closure/flow-off, maintained during
    # the bowl task. Residual speed remains diagnostic, never a success gate.
    stable_off=bool(len(rows)>=stable_n and np.all(angles[tail]>=-.5) and np.all(angles[tail]<=off_angle) and np.all(~flowing[tail]))
    off_indices = np.flatnonzero(~flowing)
    first_off_index = int(off_indices[0]) if len(off_indices) else None
    continuous_off = bool(first_off_index is not None and not flowing[first_off_index:].any())
    speed_diagnostic=dict(used_as_success_gate=False,
        final_speed_deg_s=float(speeds[-1]),
        final_window_maximum_absolute_speed_deg_s=float(np.max(np.abs(speeds[tail]))),
        legacy_config_max_speed_deg_s=config['close_success'].get('max_speed_deg_s'),
        statement='Residual speed is diagnostic only under the user-defined closure criterion; actual renewed flow remains a failure.')
    rejected=[]
    for plan in plans:
        if plan.get('status')=='failed':
            start=int(plan['start_physics_step'])
            after=[a for a in actions if a.get('after_physics_step') is not None and int(a['after_physics_step'])>start]
            rejected.append(dict(phase=plan['phase'],start_physics_step=start,error=plan.get('error'),
                rejection=plan.get('validation'),observed_actions_after_rejected_request=len(after)))
    checks=dict(
        continuous_native_steps=bool(len(steps)>0 and np.all(np.diff(steps)==1)),
        reset_return_included=bool(len(steps)>0 and steps[0]==meta['reset_physics_step']),
        aligned_water_robot_raw_contacts=bool(aligned and np.array_equal(steps,contact_steps)),
        finite_robot_q_dq=bool(np.isfinite(arrays['q']).all() and np.isfinite(arrays['dq']).all()),
        finite_valve_q_dq_pose=bool(np.isfinite(angles).all() and np.isfinite(speeds).all() and np.isfinite(lever).all()),
        six_native_finger_channels_match=bool(finger_error is not None and finger_error<=1e-12),
        raw_contact_arrays_valid=bool(all_contact_arrays_valid),
        valve_joint_readback_matches_body_pose=bool(np.max(angular_error)<.001),
        flow_boolean_coupled_to_actual_q=bool(np.array_equal(flowing,angles>off_angle)),
        flow_fraction_coupled_to_actual_q=bool(np.max(np.abs(fractions-expected_fraction))<1e-12),
        all_active_flow_samples_have_17_part_records=bool(complete_flow and all(v.get('parts_checked')==17 for v in shape_checks)),
        recorded_actual_flow_checks_safe=bool(complete_flow and all(v.get('valid') and not v.get('intersecting') and v.get('distance_m',-1)>=.002 for v in shape_checks)),
        captured_water_sources_hash_bound=bool(all(r['hash_matches'] and r['recorded_unchanged'] for r in source_review)),
        no_state_setter_or_direct_force_calls_in_captured_water_adapters=bool(all(not r['state_setter_or_direct_force_calls'] for r in source_review)),
        passive_valve_no_actuators=water['actuators']==[],
        initial_reset_only_declared=bool(water['reset_event']=='reset_articulation_pose_and_joints' and not water['state_setters_used_after_reset'] and not result['physics_state_setters_used'] and not result['physical_weld_used']),
        unilateral_contact_controller_supported=bool(single_contact_controller and not old_bilateral_reject),
        original_counter_bowl_to_sink_goal=bool(result['task_id']==predicate['task_id']==relation['task_id']==TASK and predicate['pick_up_object']=='bowl' and predicate['destination_location']=='sink' and relation_on_counter and identity['actual_pick_up_path']=='/World/envs/env_0/bowl' and identity['actual_destination_path']=='/World/envs/env_0/kitchen/Sink/BasinBottom'),
        no_recorded_bowl_actions_while_flowing=not bowl_flow_steps,
    )
    return dict(schema_version=2,episode=str(episode),accepted=False,physics_launched=False,
        review_implementation_sha256=digest(__file__),
        scope='Independent CPU facts review of recorded native data and hash-bound captured water adapter text. Not runtime call tracing, neutral-duration matching, geometry re-evaluation, video review, full dynamics acceptance, or proof of original bowl-task completion.',
        fact_integrity_checks_passed=all(checks.values()),checks=checks,input_sha256=hashes,
        recording=dict(samples=len(steps),first_step=int(steps[0]),last_step=int(steps[-1]),duration_s=float((steps[-1]-steps[0])*dt),physics_dt_s=dt,reset_internal_substeps_not_observed=True),
        valve_motion=dict(initial_angle_deg=float(angles[0]),final_angle_deg=float(angles[-1]),range_deg=[float(angles.min()),float(angles.max())],net_closing_deg=float(angles[0]-angles[-1]),maximum_valve_body_angle_error_deg=float(angular_error.max()),maximum_pre_contact_drift_deg=float(np.max(np.abs(before-initial_angle))) if len(before) else None,
            first_allowed_loaded_near_contact=first_contact,first_closing_movement_over_half_degree_step=first_movement,strict_allowed_contact_precedes_movement=causal,contact_to_movement_seconds=(first_movement-first_contact_step)*dt if causal else None,first_any_contact_as_reported=water['closure'].get('first_contact_step')),
        fingers=dict(joint_names=FINGERS,maximum_native_channel_difference_rad=finger_error,final_observed_q_rad=observed_fingers[-1].tolist()),
        flow=dict(off_threshold_deg=off_angle,minimum_recorded_clearance=min_flow,ever_off=bool((~flowing).any()),final_flowing=bool(flowing[-1]),stable_final_off_recomputed=stable_off,stable_required_samples=stable_n,
            first_off_step=None if first_off_index is None else int(water_steps[first_off_index]),
            continuous_off_since_first_shutoff=continuous_off,
            continuous_off_observed_seconds=float((water_steps[-1]-water_steps[first_off_index])*dt) if continuous_off else None,
            completion_definition='Measured valve q in the closed range and actual flow off throughout the final window; no dq threshold. Recorded bowl actions must remain flow off.',
            speed_diagnostic=speed_diagnostic),
        task=dict(instruction=config['instruction'],pick_up_object=predicate['pick_up_object'],destination_location=predicate['destination_location'],initial_bowl_on_counter=relation_on_counter,native_destination=identity['actual_destination_path'],bowl_sequence_physics_samples=len(bowl_steps),bowl_steps_while_flowing=bowl_flow_steps,original_task_success=result.get('original_task_success'),final_original_destination_predicate=result.get('final_original_destination_predicate')),
        source_review=dict(files=source_review,reset_configuration_present='reset_articulation_pose_and_joints' in runtime_source,limitation='Static inspection of captured water adapters and source-hash manifest; not an instrumented trace of every engine/API call.'),
        outcome=dict(status=result['status'],error=result.get('error'),actual_safety_failure=result['safety'].get('failure'),rejected_candidates=rejected,original_accepted=result['accepted']))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episode',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():
        raise FileExistsError('Preserve historical review; choose a new output: '+str(args.output))
    report=review(args.episode)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(args.output),fact_integrity_checks_passed=report['fact_integrity_checks_passed'],failed_checks=[k for k,v in report['checks'].items() if not v],motion=report['valve_motion'],flow=report['flow'],task=report['task']),ensure_ascii=False))


if __name__=='__main__':
    main()
