#!/usr/bin/env python3
"""CPU-only measured grasp/contact diagnosis; writes only its new report.

No Kit, rendering, physics, candidate search, or input modification. Frame
indices address the real physics_states.npz for subsequent diagnostic replay.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import os
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[3]
DEFAULT_SCENES=['g_shape_bowl_sink__human_default_mpfb','g_shape_bowl_sink__property_tall_glass','g_shape_bowl_sink__property_glass_bottle']
FINGERS=['left_inner_finger','right_inner_finger']
THRESHOLD=.01  # Descriptive contact threshold, not the original task predicate.
NO_CONTACT_STEPS=12  # 0.1 s at native 120 Hz.
DISTANCE_INCREASE_M=.02  # Diagnostic event marker, never a success criterion.


def local(value):
    p=(ROOT/str(value).removeprefix('/workspace/')).resolve();p.relative_to(ROOT);return p


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def ref(p):
    p=local(p);return dict(path=str(p.relative_to(ROOT)),sha256=sha(p))


def read(p):return json.loads(Path(p).read_text())


def check(binding):
    p=local(binding['path'])
    if sha(p)!=binding['sha256']:raise ValueError('Evidence hash changed: '+str(p))
    return p


def atomic_json(path, value):
    path=local(path);path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_name(path.name+'.tmp.'+str(os.getpid()))
    try:
        temp.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
        os.replace(temp,path)
    finally:
        temp.unlink(missing_ok=True)


def canonical_sha(value):
    return hashlib.sha256(json.dumps(value,ensure_ascii=False,sort_keys=True,
        separators=(',',':'),allow_nan=False).encode()).hexdigest()


def cache_inputs(row):
    """Validate current small result plus streamed hashes, never parse large JSON."""
    rp=check(dict(path=row['execution_result_path'],sha256=row['execution_result_sha256']))
    result=read(rp)
    if (result['execution_status']!='failed' or result['executed'] is not True
            or result['status']!='complete' or result['scene_id']!=row['scene_id']):
        raise ValueError('Cache accepts only the selected completed actual failed execution')
    evidence={name:dict(binding) for name,binding in result['evidence'].items()}
    for binding in evidence.values():check(binding)
    for required in ('physics_states.npz','grasp_contacts.json','physics_metadata.json',
                     'actions.json','task_predicate_physics.npz','task_predicate_metadata.json'):
        if required not in evidence:raise ValueError('Missing analysis evidence: '+required)
    planning=dict(path=result['planning_result_path'],sha256=result['planning_result_sha256'])
    check(planning)
    return dict(scene_id=row['scene_id'],execution_result=ref(rp),
        analysis_script=ref(__file__),evidence=evidence,planning_result=planning)


def cached_analyze(row, cache_dir, manifest_ref, compute=None):
    """Cache is independent of a changing run manifest, bound to all scene inputs."""
    inputs=cache_inputs(row)
    path=local(cache_dir)/(row['scene_id']+'.json')
    cached=None;invalid_reason=None
    if path.exists():
        try:
            cached=read(path)
            if cached.get('kind')!='measured_grasp_analysis_cache':
                raise ValueError('Different cache kind')
            if cached.get('inputs')!=inputs:
                raise ValueError('Scene evidence or analysis script SHA changed')
            if canonical_sha(cached['entry'])!=cached.get('entry_sha256'):
                raise ValueError('Cached analysis entry digest mismatch')
            if cached['entry']['scene_id']!=row['scene_id']:
                raise ValueError('Cached scene identity mismatch')
        except (ValueError,KeyError,TypeError) as exc:
            invalid_reason=str(exc);cached=None
    if cached is None:
        entry=(compute or analyze)(row)
        # Do not persist an analysis if any of its source bytes changed meanwhile.
        if cache_inputs(row)!=inputs:
            raise ValueError('Analysis inputs changed during computation')
        cached=dict(schema_version=1,kind='measured_grasp_analysis_cache',
            created_at=datetime.now(timezone.utc).isoformat(),
            origin_manifest_snapshot=manifest_ref,inputs=inputs,
            entry_sha256=canonical_sha(entry),entry=entry)
        atomic_json(path,cached)
        status='computed'
    else:status='reused'
    provenance=dict(scene_id=row['scene_id'],status=status,cache=ref(path),
        origin_manifest_snapshot=cached['origin_manifest_snapshot'],
        analysis_script=cached['inputs']['analysis_script'])
    if invalid_reason:provenance['previous_cache_rejected_reason']=invalid_reason
    return cached['entry'],provenance


def first_sustained(mask, count=NO_CONTACT_STEPS):
    """Return the first run's start index; missing evidence remains None."""
    streak=0
    for i,value in enumerate(mask):
        streak=streak+1 if value else 0
        if streak>=count:return i-count+1
    return None


def default_frame_indices(phases, maximum=6):
    """Mirror the renderer's existing six-frame selection, without rendering."""
    selected=[]
    for phase in ('close','lift','transfer','lower','release'):
        ids=np.flatnonzero(phases==phase)
        if len(ids):selected.append(int(ids[-1]))
    selected=sorted(set([*selected,len(phases)-1]))
    for i in np.linspace(0,len(phases)-1,min(maximum,len(phases))).astype(int):
        if len(selected)>=maximum:break
        selected=sorted(set([*selected,int(i)]))
    return selected[:maximum]


def analyze(row):
    rp=check(dict(path=row['execution_result_path'],sha256=row['execution_result_sha256']));result=read(rp)
    if result['execution_status']!='failed' or result['executed'] is not True or result['status']!='complete':
        raise ValueError('Only completed actual failed executions may be diagnosed')
    for binding in result['evidence'].values():check(binding)
    p=rp.parent;m=read(p/'physics_metadata.json');pm=read(p/'task_predicate_metadata.json')
    contacts=read(p/'grasp_contacts.json');actions=read(p/'actions.json')
    with np.load(p/'physics_states.npz',allow_pickle=False) as ar:
        d={k:ar[k] for k in ('physics_step','phase','action_index','source_trajectory_index',
                            'q','robot_body_pose_w','task_object_states')}
    with np.load(p/'task_predicate_physics.npz',allow_pickle=False) as ar:pred={k:ar[k] for k in ar.files}
    planning_path=check(dict(path=result['planning_result_path'],sha256=result['planning_result_sha256']));plan=read(planning_path)
    n=len(d['physics_step'])
    if len(contacts)!=n or not np.array_equal([x['physics_step'] for x in contacts],d['physics_step']) or not np.array_equal(pred['physics_step'],d['physics_step']):
        raise ValueError('Contact/predicate/native physics streams are misaligned')
    palm=d['robot_body_pose_w'][:,m['robot_body_names'].index('base_link')]
    rotation=Rotation.from_quat(palm[:,3:7])
    tcp=palm[:,:3]+rotation.apply(np.array([.131,0.,0.]))
    payload_index=m['object_names'].index(m['payload_name']);obj=d['task_object_states'][:,payload_index]
    centroid=pred['object_centroid_world'];phase=d['phase'].astype(str);baseline_ids=np.flatnonzero(phase=='initial_settle')[-120:]
    if not len(baseline_ids):raise ValueError('No measured initial-settle baseline; cannot invent a lift reference')
    baseline_z=float(np.median(centroid[baseline_ids,2]));baseline_xyz=np.median(obj[baseline_ids,:3],axis=0)
    force=np.array([[sum(c['fingers'][f]['normal_forces_n']) for f in FINGERS] for c in contacts])
    counts=np.array([[c['fingers'][f]['count'] for f in FINGERS] for c in contacts])
    loaded=force>THRESHOLD;bilateral=np.all(loaded,axis=1);any_loaded=np.any(loaded,axis=1)
    finger_id=m['joint_names'].index('finger_joint');arm_ids=[m['joint_names'].index('panda_joint'+str(i)) for i in range(1,8)]
    obj_tcp_local=rotation.inv().apply(obj[:,:3]-tcp)
    palm_distance=np.linalg.norm(obj[:,:3]-palm[:,:3],axis=1)
    close_ids=np.flatnonzero(phase=='close')
    distance_baseline=float(np.median(palm_distance[close_ids[-60:]])) if len(close_ids) else None
    distance_delta=palm_distance-distance_baseline if distance_baseline is not None else None
    up=Rotation.from_quat(obj[:,3:7]).apply(np.array([0.,0.,1.]));tilt=np.rad2deg(np.arccos(np.clip(up[:,2],-1.,1.)))
    def frame(index,label):
        i=int(index);ai=int(d['action_index'][i]);target=np.asarray(actions[ai]['target']) if ai>=0 else np.full(8,np.nan)
        details={}
        for f in FINGERS:
            c=contacts[i]['fingers'][f];points=np.asarray(c['points_object_local_m']).reshape(-1,3)
            details[f]=dict(count=c['count'],summed_normal_force_n=float(sum(c['normal_forces_n'])),
                points_object_local_m=c['points_object_local_m'],normals_object_local=c['normals_object_local'],
                normal_forces_n=c['normal_forces_n'],separations_m=c['separations_m'])
        return dict(label=label,state_index=i,physics_step=int(d['physics_step'][i]),phase=phase[i],action_index=ai,
            source_trajectory_index=int(d['source_trajectory_index'][i]),time_since_reset_s=float((d['physics_step'][i]-d['physics_step'][0])/120),
            object_pose_xyz_xyzw=obj[i,:7].tolist(),object_centroid_world_m=centroid[i].tolist(),
            centroid_rise_m=float(centroid[i,2]-baseline_z),object_horizontal_displacement_m=float(np.linalg.norm(obj[i,:2]-baseline_xyz[:2])),
            object_tilt_from_world_up_deg=float(tilt[i]),measured_tcp_xyz_m=tcp[i].tolist(),
            object_origin_in_tcp_frame_m=obj_tcp_local[i].tolist(),object_origin_to_tcp_distance_m=float(np.linalg.norm(obj[i,:3]-tcp[i])),
            object_origin_to_gripper_base_distance_m=float(palm_distance[i]),
            distance_increase_from_close_baseline_m=float(distance_delta[i]) if distance_delta is not None else None,
            finger_joint_command_rad=float(target[7]) if ai>=0 else None,finger_joint_actual_rad=float(d['q'][i,finger_id]),
            arm_target_max_error_rad=float(np.max(np.abs(d['q'][i,arm_ids]-target[:7]))) if ai>=0 else None,
            loaded_contact_threshold_n=THRESHOLD,bilateral_loaded=bool(bilateral[i]),finger_contacts=details)
    phases={};frames=[]
    for name in dict.fromkeys(phase):
        ids=np.flatnonzero(phase==name)
        if name in ('reset','initial_settle'):continue
        last=int(ids[-1]); ai=int(d['action_index'][last]); target=np.asarray(actions[ai]['target'])
        target_phase='approach' if name=='close' else name
        endpoint=plan.get('targets',{}).get(target_phase)
        phases[name]=dict(first_state_index=int(ids[0]),last_state_index=last,samples=len(ids),seconds=len(ids)/120,
            contact_samples_by_finger={f:int(np.sum(counts[ids,k]>0)) for k,f in enumerate(FINGERS)},
            loaded_samples_by_finger={f:int(np.sum(loaded[ids,k])) for k,f in enumerate(FINGERS)},
            bilateral_loaded_samples=int(np.sum(bilateral[ids])),bilateral_loaded_seconds=float(np.sum(bilateral[ids])/120),
            normal_force_max_n={f:float(np.max(force[ids,k])) for k,f in enumerate(FINGERS)},
            normal_force_p95_n={f:float(np.percentile(force[ids,k],95)) for k,f in enumerate(FINGERS)},
            finger_joint_end_actual_rad=float(d['q'][last,finger_id]),finger_joint_end_command_rad=float(target[7]),
            arm_end_target_max_error_rad=float(np.max(np.abs(d['q'][last,arm_ids]-target[:7]))),
            endpoint_tcp_position_error_m=float(np.linalg.norm(tcp[last]-np.asarray(endpoint)[:3,3])) if endpoint is not None else None,
            max_centroid_rise_m=float(np.max(centroid[ids,2]-baseline_z)),end_centroid_rise_m=float(centroid[last,2]-baseline_z),
            end_object_horizontal_displacement_m=float(np.linalg.norm(obj[last,:2]-baseline_xyz[:2])),
            end_object_origin_to_tcp_distance_m=float(np.linalg.norm(obj[last,:3]-tcp[last])),
            original_destination_true_samples=int(np.sum(pred['original_object_on_destination'][ids])))
        if name in ('approach','close','lift','transfer','lower'):frames.append(frame(last,name+'_end'))
    carry=np.flatnonzero(np.isin(phase,['lift','transfer','lower']))
    peak=int(carry[np.argmax(centroid[carry,2])]) if len(carry) else None
    if peak is not None:frames.append(frame(peak,'maximum_carry_centroid_height'))
    carry_bilateral=carry[bilateral[carry]];carry_loaded=carry[any_loaded[carry]]
    if len(carry_bilateral):frames.append(frame(carry_bilateral[-1],'last_carry_bilateral_loaded_contact'))
    if len(carry_loaded):
        frames.append(frame(carry_loaded[-1],'last_carry_any_loaded_contact'))
        if carry_loaded[-1]+1<n:frames.append(frame(carry_loaded[-1]+1,'first_state_after_last_carry_loaded_contact'))
    final=frame(n-1,'final_state');frames.append(final)
    focused_labels=['approach_end','close_end','maximum_carry_centroid_height','last_carry_bilateral_loaded_contact',
                    'first_state_after_last_carry_loaded_contact','lift_end']
    preferred=[next(f for f in frames if f['label']==label) for label in focused_labels if any(f['label']==label for f in frames)]
    minimal=[{k:f[k] for k in ('label','state_index','physics_step','phase','action_index','source_trajectory_index')} for f in preferred]
    close=phases.get('close',{});lift_summary=phases.get('lift',{});transfer=phases.get('transfer',{});facts=[]
    if phases.get('approach',{}).get('endpoint_tcp_position_error_m') is not None:
        facts.append(f"approach末实际TCP位置误差{phases['approach']['endpoint_tcp_position_error_m']*1000:.3f}mm。")
    if close:
        facts.append(f"{close['seconds']:.3f}s闭合阶段{close['samples']}个物理样本中，双侧指爪负载接触{close['bilateral_loaded_samples']}个；实际finger_joint末值{close['finger_joint_end_actual_rad']:.6f}rad，目标{close['finger_joint_end_command_rad']:.2f}rad。")
    if peak is not None:
        facts.append(f"携物阶段物体中心最大抬升{(centroid[peak,2]-baseline_z)*1000:.3f}mm，峰值姿态倾角{tilt[peak]:.2f}度；高度和倾角均为记录值，不表示已稳定抓持。")
    if lift_summary:
        facts.append(f"lift结束物体水平移动{lift_summary['end_object_horizontal_displacement_m']*1000:.3f}mm，中心高度增量{lift_summary['end_centroid_rise_m']*1000:.3f}mm；夹爪实际角{lift_summary['finger_joint_end_actual_rad']:.6f}rad。")
    if transfer:
        facts.append(f"transfer阶段左右指对该物体的接触样本数分别为{list(transfer['contact_samples_by_finger'].values())}。")
    # Event frames are extra to the renderer's existing six. At most four new
    # actual indices; an absent event is explicitly recorded, never fabricated.
    events=[];missing={};base_frames=default_frame_indices(phase)
    def event(index,name,reason):
        if index is None:return
        i=int(index)
        events.append(dict(state_index=i,physics_step=int(d['physics_step'][i]),phase=phase[i],
            action_index=int(d['action_index'][i]),source_trajectory_index=int(d['source_trajectory_index'][i]),
            event=name,reason=reason))
    if peak is not None and centroid[peak,2]-baseline_z>.001:
        event(peak,'maximum_carry_centroid_height','Measured maximum object-centroid rise in lift/transfer/lower; >1mm noise floor, not a successful-lift declaration.')
    else:missing['maximum_carry_centroid_height']='No carry samples or centroid rise does not exceed 1mm.'
    prior_contact=np.maximum.accumulate(np.any(counts>0,axis=1))
    no_contact=np.isin(phase,['lift','transfer','lower']) & np.all(counts==0,axis=1) & prior_contact
    loss=first_sustained(no_contact)
    if loss is not None:
        event(loss,'first_sustained_no_finger_contact','First carry-phase run of at least12 consecutive120Hz samples with both raw finger/object contact counts zero, after earlier measured contact.')
    else:missing['first_sustained_no_finger_contact']='No qualifying0.1s zero-contact run after measured contact.'
    if distance_delta is not None and len(carry):
        onset=first_sustained(np.isin(phase,['lift','transfer','lower']) & (distance_delta>DISTANCE_INCREASE_M))
        if onset is not None:
            event(onset,'relative_distance_increase_onset','First0.1s carry run with object-origin to measured gripper-base distance >close-last0.5s median+2cm. Distance increase alone does not prove slip/collision.')
        else:missing['relative_distance_increase_onset']='No sustained2cm distance increase from available close baseline.'
        maximum=int(carry[np.argmax(distance_delta[carry])])
        if distance_delta[maximum]>DISTANCE_INCREASE_M:
            event(maximum,'maximum_carry_relative_distance_increase','Maximum measured carry-phase distance increase from close baseline, exceeding2cm; not a causal classification.')
        else:missing['maximum_carry_relative_distance_increase']='Maximum distance increase does not exceed2cm.'
    else:
        missing['relative_distance_increase_onset']='No measured close reference or carry samples.'
        missing['maximum_carry_relative_distance_increase']='No measured close reference or carry samples.'
    hints=[];used=set(base_frames)
    for item in events:
        if item['state_index'] in used:continue
        hints.append(item);used.add(item['state_index'])
    if len(hints)>4 or len(used)>10:raise ValueError('Event selection exceeds four additions/ten total frames')
    retention_loss=bool(close.get('bilateral_loaded_samples',0)>0 and transfer and
        not any(transfer['contact_samples_by_finger'].values()))
    inference=('已建立双侧接触，但转运阶段已无所记录的指-物体接触；优先看图核验抓取包覆、接触位置和抬升/转运期间分离。'
        if retention_loss else '根据所记录的接触、位姿和关键事件图核验抓取、保持及放置过程；当前数据不支持预先指定唯一原因。')
    result_ref=ref(rp)
    return dict(scene_id=row['scene_id'],task_id=row['task_id'],physical_attempt_number=result['physical_attempt_number'],
        result=result_ref,physics_states=ref(p/'physics_states.npz'),physics_metadata=ref(p/'physics_metadata.json'),
        grasp_contacts=ref(p/'grasp_contacts.json'),actions=ref(p/'actions.json'),predicate_physics=ref(p/'task_predicate_physics.npz'),
        predicate_metadata=ref(p/'task_predicate_metadata.json'),source_planning_result=ref(planning_path),
        payload_name=m['payload_name'],grasp_info=plan.get('grasp_info'),nominal_gripper_joint_rad=plan.get('nominal_gripper_joint_rad'),
        baseline_centroid_z_m=baseline_z,baseline_object_origin_world_m=baseline_xyz.tolist(),contact_load_threshold_n=THRESHOLD,
        facts_zh=facts,phase_summaries=phases,measured_keyframes=frames,suggested_six_diagnostic_frames=minimal,
        event_frames=events,additional_frame_hints=hints,missing_events=missing,
        default_six_state_indices=base_frames,combined_render_frame_count=len(used),
        distance_reference_close_m=distance_baseline,
        inference_zh=inference+'接触数据不能单独确定摩擦不足、穿透、外物阻挡或场景不可行。',
        root_cause_confirmed=False,scene_infeasibility_established=False,visual_verification_pending=True,
        original_task_success=bool(result.get('original_task_success')),final_original_destination_predicate=bool(result.get('final_original_destination_predicate')))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',default='exports/replicator_physical_retry_round2_20261028/manifest.json')
    parser.add_argument('--scene',action='append')
    parser.add_argument('--all-failed',action='store_true',help='Analyze all final failed rows, only after the entire selected run is complete')
    parser.add_argument('--completed-failed',action='store_true',help='Analyze only terminal failed rows in this manifest snapshot; does not claim the run is complete')
    parser.add_argument('--cache-dir',help='Reuse per-scene analyses only when every evidence hash and this script SHA still match')
    parser.add_argument('--output',default='reports/replicator_physical_retry_round2_20261028/g_bowl_grasp_analysis.json')
    parser.add_argument('--frame-hints-output',help='Write source-bound extra frame hints for the diagnostic renderer')
    args=parser.parse_args();manifest=local(args.manifest);raw=manifest.read_bytes();snapshot=json.loads(raw);rows={x['scene_id']:x for x in snapshot['entries']}
    if sum(bool(x) for x in (args.all_failed,args.completed_failed,args.scene))>1:
        parser.error('--all-failed, --completed-failed and --scene are mutually exclusive')
    if args.all_failed and (snapshot['stage']!='complete' or snapshot['counts']['pending'] or snapshot['counts']['running']):
        parser.error('Wait for all selected terminal rows before --all-failed; no partial bulk analysis was started')
    manifest_ref=dict(path=str(manifest.relative_to(ROOT)),sha256=hashlib.sha256(raw).hexdigest())
    scenes=([sid for sid,row in rows.items() if row['execution_status']=='failed']
        if args.all_failed or args.completed_failed else (args.scene or DEFAULT_SCENES))
    entries=[];cache_provenance=[]
    for index,sid in enumerate(scenes):
        if args.cache_dir:
            entry,provenance=cached_analyze(rows[sid],args.cache_dir,manifest_ref)
            cache_provenance.append(provenance)
        else:
            entry=analyze(rows[sid]);provenance=dict(status='computed_without_cache')
        entries.append(entry)
        print(json.dumps(dict(completed=index+1,total=len(scenes),scene_id=sid,
            cache_status=provenance['status']),ensure_ascii=False),flush=True)
    report=dict(schema_version=1,created_at=datetime.now(timezone.utc).isoformat(),scope='Read-only actual120Hz state/contact analysis for selected completed failures; no GPU/Kit/rendering/new trial/input modification.',
        manifest_snapshot=manifest_ref,manifest_changed_during_analysis=sha(manifest)!=manifest_ref['sha256'],scenes=len(entries),entries=entries,
        selection_mode='all_failed_final' if args.all_failed else ('completed_failed_snapshot' if args.completed_failed else 'explicit_scenes'),
        manifest_snapshot_stage=snapshot['stage'],manifest_snapshot_counts=snapshot['counts'],
        entire_run_complete_at_snapshot=bool(snapshot['stage']=='complete' and not snapshot['counts']['pending'] and not snapshot['counts']['running']),
        analysis_script=ref(__file__),cache_provenance=cache_provenance,
        tcp_definition='Measured USD Robotiq base_link pose plus local +X0.131m, matching source Droid TCP; no nominal FK substituted.',
        tcp_definition_sources=[ref('third_party/IsaacLab-Arena/isaaclab_arena/embodiments/droid/droid.py'),ref('kitchen_safety/extensions/replicator/task_recorder.py')],
        contact_scope='Filtered normal finger/object contacts only. Loaded means summed positive normal force for each finger >0.01N. Not force closure, friction proof, or global collision acceptance.',
        force_spike_caveat='Large single-step normal-force peaks are raw solver observations and may reflect transient numerical contact; causal interpretation requires visual/contact geometry review.')
    output=local(args.output);atomic_json(output,report)
    if args.frame_hints_output:
        hint_entries=[]
        for entry in entries:
            item=dict(scene_id=entry['scene_id'],frames=entry['additional_frame_hints'],
                missing_events=entry['missing_events'],default_frame_state_indices=entry['default_six_state_indices'],
                combined_render_frame_count=entry['combined_render_frame_count'])
            for name,key in [('execution_result','result'),('physics_states','physics_states'),('physics_metadata','physics_metadata')]:
                item[name+'_path']=entry[key]['path'];item[name+'_sha256']=entry[key]['sha256']
            hint_entries.append(item)
        hints=dict(schema_version=1,kind='physical_failure_frame_hints',source_manifest=manifest_ref,
            selection_mode=report['selection_mode'],entire_run_complete_at_snapshot=report['entire_run_complete_at_snapshot'],
            analysis=ref(output),event_rules=dict(native_hz=120,sustained_no_contact_steps=NO_CONTACT_STEPS,
                distance_increase_m=DISTANCE_INCREASE_M,distance_baseline='Median last60 available close physics samples, measured object-origin to Robotiq base_link origin.'),
            scope='At most4 extra actual state frames per failure. Preserve renderer default6, deduplicate by actual index, maximum10 total. Missing events are not invented.',entries=hint_entries)
        target=local(args.frame_hints_output);atomic_json(target,hints)
    print(json.dumps(dict(output=str(output.relative_to(ROOT)),sha256=sha(output),scenes=len(entries),facts={e['scene_id']:e['facts_zh'] for e in entries}),ensure_ascii=False,indent=2))

if __name__=='__main__':main()
