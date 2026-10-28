#!/usr/bin/env python3
"""Independent CPU review of an explicitly bounded banana/bowl diagnostic.

Reads completed output only; never steps physics, starts CUDA, edits raw data,
or promotes the diagnostic. Native observations and original success are separate.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scripts.extensions.replicator.task_audit_sidecars import ROOT,sha,fixed_audit,source_audit
from scripts.extensions.replicator.search_banana_grasp_stability_cpu import write
from scripts.extensions.replicator.review_task_episode import (
    prefix_audit,video_audit,neutral_audit,contact_window,sequence_audit,raw_destination,
)
from scripts.extensions.replicator.task_camera_audit import authored_camera_audit
from kitchen_safety.extensions.replicator.task_motion import pose_matrix
from kitchen_safety.extensions.replicator.task_recorder import audit_physics
from kitchen_safety.extensions.replicator.task_collection_evidence import audit_provenance,audit_predicate_evidence
from kitchen_safety.extensions.replicator.banana_destination_probe import (
    audit_declaration_archive,DECLARATION,LEGACY_DECLARATION,VECTOR_DECLARATION,BODY,DESTINATION,DESTINATION_MESH,SOURCE_ASSETS,
)
from scripts.extensions.replicator.analyze_banana_grasp_contact_lines import triangle_intersection


def read(path):return json.loads(Path(path).read_text())
def require(condition,message):
    if not condition:raise ValueError(message)


def raw_predicates(p,m,d,t,r):
    info=read(p/'task_predicate_metadata.json');ph=dict(np.load(p/'task_predicate_physics.npz',allow_pickle=False))
    ac=dict(np.load(p/'task_predicate_actions.npz',allow_pickle=False));dt=round(m['action_dt_s']/m['physics_dt_s'])
    completed=np.flatnonzero(t['after_physics_step']-t['before_physics_step']==dt)
    checks=dict(physics_steps_match=np.array_equal(ph['physics_step'],d['physics_step']),
        completed_actions_match=np.array_equal(ac['physics_step'],t['after_physics_step'][completed]),
        unchanged_sensor_identity=info['sensor_identity']==info['sensor_identity_at_end'],
        original_thresholds=info['thresholds']==dict(force_threshold=.1,velocity_threshold=.003,support_cone_half_angle_rad=float(np.pi/4),placement_consecutive_steps=1),
        physics_hash=sha(p/'task_predicate_physics.npz')==info['input_sha256']['task_predicate_physics.npz'],
        action_hash=sha(p/'task_predicate_actions.npz')==info['input_sha256']['task_predicate_actions.npz'])
    bounds=info['actual_spawned_local_bounds'];center=np.mean([bounds['banana']['min'],bounds['banana']['max']],axis=0)
    destination=np.array([bounds['bowl']['min'],bounds['bowl']['max']]);bi=m['task_object_names'].index('banana');di=m['task_object_names'].index('bowl')
    states=d['task_object_states'][:,bi];poses=d['task_object_states'][:,di,:7]
    raw=[raw_destination(states[i],d['destination_contact_force'][i,0,0],center,poses[i],destination) for i in range(len(states))]
    summaries={}
    for name,rows,steps in [('physics',ph,d['physics_step']),('complete_actions',ac,t['after_physics_step'][completed])]:
        ids=np.searchsorted(d['physics_step'],steps)
        checks[name+'_finite']=all(len(v)==len(ids) and (v.dtype.kind in 'US' or np.isfinite(v).all()) for v in rows.values())
        for field,actual in [('object_pose',states[ids,:7]),('destination_pose',poses[ids]),('object_linear_velocity',states[ids,7:10]),('destination_contact_force',d['destination_contact_force'][ids,0,0])]:
            checks[name+'_'+field]=rows[field].shape==actual.shape and np.allclose(rows[field],actual,atol=1e-6,rtol=0)
        checks[name+'_phase']=np.array_equal(rows['phase'],d['phase'][ids]);checks[name+'_action_index']=np.array_equal(rows['action_index'],d['action_index'][ids])
        checks[name+'_on']=np.array_equal(rows['original_object_on_destination'],[raw[i]['passed'] for i in ids])
        summaries[name]=dict(samples=len(ids),on_success_samples=int(rows['original_object_on_destination'].sum()),
            terminal_success_samples=int(rows['terminal_success'].sum()),final=raw[ids[-1]])
    return dict(passed=all(checks.values()),checks=checks,summaries=summaries,all_physics_values=raw,
        scope='Original raw predicate values and recording identity; successful predicate samples do not promote this diagnostic.')


def generic(p,neutral,m,d,t,r):
    details={}
    def check(name,fn):
        try:details[name]=fn()
        except Exception as exc:details[name]=dict(passed=False,error=f'{type(exc).__name__}: {exc}')
        return details[name]
    actions=read(p/'actions.json');contacts=read(p/'grasp_contacts.json')
    check('source_relations_fixed_pose',lambda:source_audit(m,d))
    check('startup_exact_prefix',lambda:prefix_audit(p,m,d))
    check('full_physics_coverage_background_contacts',lambda:audit_physics(d,m,actions,contacts))
    if np.any(t['after_physics_step']-t['before_physics_step']!=8):
        from scripts.extensions.replicator.audit_partial_task_episode import validate_partial
        check('actual_partial_extent',lambda:validate_partial(m,d,t))
    check('source_camera_alignment',lambda:authored_camera_audit(p,m,d,t))
    check('all_raw_images_and_six_video_files',lambda:video_audit(p,m,t))
    check('matching_neutral',lambda:neutral_audit(neutral,m,r,d))
    check('execution_provenance',lambda:audit_provenance(p/'task_collection_provenance.json',m,int(d['physics_step'][-1])))
    check('diagnostic_declaration_archive',lambda:audit_declaration_archive(read(p/'task_collection_provenance.json'),m['diagnostic_declaration_binding']))
    check('fixed_geometry_all_actual_states',lambda:fixed_audit(m,d))
    check('original_predicate_recording',lambda:raw_predicates(p,m,d,t,r))
    close=check('close_bilateral_10',lambda:contact_window(d,m,contacts,'banana',r['grasp_target']['contact_region_local_m'],'close'))
    lift=check('lift_bilateral_10',lambda:contact_window(d,m,contacts,'banana',r['grasp_target']['contact_region_local_m'],'lift'))
    check('full_task_sequence',lambda:sequence_audit(d,m,t,r,close,lift))
    check('full_original_success_evidence',lambda:audit_predicate_evidence(p,m,d,t,r))
    check('diagnostic_not_acceptance',lambda:dict(passed=r['accepted'] is False and r['physical_trial_passed'] is False and r['task_success'] is False and r['diagnostic_only'] is True))
    return dict(schema_version=1,accepted=False,task_success=False,physical_trial_passed=False,diagnostic_only=True,
        episode=str(p),details=details,recording_checks_passed=all(v['passed'] for k,v in details.items() if k not in ('full_task_sequence','full_original_success_evidence')),
        input_sha256={n:sha(p/n) for n in ['result.json','physics_metadata.json','physics_states.npz','trajectory.npz','actions.json','grasp_contacts.json','task_collection_provenance.json','task_predicate_metadata.json','task_predicate_physics.npz','task_predicate_actions.npz']},
        review_source_sha256=sha(__file__),scope='CPU evidence checks only; no visual or CUDA self-collision claim. Full-task failure remains independent from successful diagnostic recording.')


def original_world(m):
    from pxr import Gf,Usd,UsdGeom
    from kitchen_safety.planning import DroidKinematics
    from kitchen_safety.extensions.replicator.task_collision import PayloadCollisionWorld
    layout=m['source_layout'];stage=Usd.Stage.CreateInMemory();root='/World/envs/env_0';UsdGeom.Xform.Define(stage,root)
    UsdGeom.Xform.Define(stage,root+'/kitchen').GetPrim().GetReferences().AddReference(str(ROOT/layout['kitchen_usd']))
    for name,item in layout['objects'].items():
        xf=UsdGeom.Xform.Define(stage,root+'/'+name);xf.GetPrim().GetReferences().AddReference(str(ROOT/item['usd_path']))
        xf.ClearXformOpOrder();xf.AddTranslateOp().Set(Gf.Vec3d(*item['position']));xf.AddRotateZOp().Set(item['yaw_deg'])
    kin=DroidKinematics(np.asarray(layout['robot_position'])+[0.,0.,layout['stand_height']],layout['robot_yaw_deg'])
    return PayloadCollisionWorld(stage,kin,payload_name='banana',payload_usd=ROOT/layout['objects']['banana']['usd_path'],env_root=root,clearances={},execution_clearances={},default_clearance=.002)


def source_and_native(p,m,d,t,r):
    native=read(p/'destination_native_contacts.json');saved=read(p/'destination_source_observations.json')
    ledger=read(p/'destination_strict_failure_ledger.json')['failures'];predicate=read(p/'task_predicate_metadata.json')
    require(r['declaration'] in (LEGACY_DECLARATION,VECTOR_DECLARATION,DECLARATION),'Wrong diagnostic declaration')
    signed_vector_rule=r['declaration']['schema_version']==2
    bounded_observation_rule=r['declaration']['schema_version']==3
    require(native['identity']==predicate['sensor_identity']==predicate['sensor_identity_at_end'],'Native original identity changed')
    require([x['physics_step'] for x in native['rows']]==d['physics_step'].tolist(),'Native contacts omit actual states')
    source_indices=np.flatnonzero(np.isin(d['phase'],['lower','release']))
    require([x['physics_step'] for x in saved['rows']]==d['physics_step'][source_indices].tolist(),'Source sidecar omits lower/release substeps')
    w=original_world(m);arms=[m['joint_names'].index(f'panda_joint{i}') for i in range(1,8)]
    fingers=[m['joint_names'].index(x) for x in w.GRIPPER_JOINT_NAMES];bi=m['task_object_names'].index('banana');di=m['task_object_names'].index('bowl')
    bounds=predicate['actual_spawned_local_bounds'];low,high=np.asarray(bounds['bowl']['min']),np.asarray(bounds['bowl']['max'])
    center=np.mean([bounds['banana']['min'],bounds['banana']['max']],axis=0);cosine=np.cos(np.pi/4)
    savedby={x['physics_step']:x for x in saved['rows']};allrows=[];source_rows=[];other_failures=[];pair_count=0
    max_native_delta=max_saved_arithmetic_error=0.;negative=[];loaded_gaps=[];gaps=[];source_mismatches=[]
    for i,step in enumerate(d['physics_step']):
        step=int(step);phase=str(d['phase'][i]);row=native['rows'][i]
        points=np.asarray(row['points_world_m']).reshape(-1,3);normals=np.asarray(row['normals_world']).reshape(-1,3)
        loads=np.asarray(row['normal_forces_n']);separation=np.asarray(row['separations_m']);n=row['count']
        count,start=np.asarray(row['buffer_count']),np.asarray(row['buffer_start'])
        require(count.shape==start.shape==(1,1) and int(count[0,0])==n and 0<=int(start[0,0]) and int(start[0,0])+n<4096,'Invalid native contact inventory')
        require(len(points)==len(normals)==len(loads)==len(separation)==n and all(np.isfinite(v).all() for v in [points,normals,loads,separation]),'Native arrays missing/nonfinite')
        require(row['exact_body']==BODY and row['exact_filter']==DESTINATION and row['phase']==phase and row['action_index']==int(d['action_index'][i]),'Native identity/phase/action mismatch')
        summed=(loads[:,None]*normals).sum(0);filtered=d['destination_contact_force'][i,0,0].astype(float)
        difference=float(np.linalg.norm(summed-filtered));max_native_delta=max(max_native_delta,difference)
        max_saved_arithmetic_error=max(max_saved_arithmetic_error,float(np.max(np.abs(summed-row['native_normal_force_vector_sum_n']))),abs(difference-row['native_original_force_difference_norm_n']))
        bp=pose_matrix(d['task_object_states'][i,bi,:7]);dp=pose_matrix(d['task_object_states'][i,di,:7]);local=(points-dp[:3,3])@dp[:3,:3]
        loaded=loads>0;normalnorm=np.linalg.norm(normals,axis=1)
        individual_cone=bool(loaded.any() and np.all(normalnorm[loaded]>0) and np.all(normals[loaded,2]/normalnorm[loaded]>=cosine))
        support=lambda force:bool(np.linalg.norm(force)>=.1 and force[2]>=np.linalg.norm(force)*cosine)
        inregion=bool(n and np.all(local[:,:2]>=low[:2]) and np.all(local[:,:2]<=high[:2]) and np.all(local[:,2]>=low[2]))
        for j in np.flatnonzero(separation<0):negative.append(dict(step=step,index=int(j),separation_m=float(separation[j]),normal_force_n=float(loads[j])))
        gaps.extend(separation.tolist());loaded_gaps.extend(separation[loaded].tolist())
        w.set_phase(phase);w.payload_pose=bp;w.attachment=None;w.gripper_joint_positions=d['q'][i,fingers].copy();w.gripper_joint_position=float(w.gripper_joint_positions[0])
        w.sync_background(m['background_paths'],d['background_rigid_poses'][i])
        for j,name in enumerate(m['task_object_names']):w.set_obstacle_pose(name,d['task_object_states'][i,j,:7])
        pairs=w._pairs(d['q'][i,arms],(),'execution',w.gripper_joint_positions,bp,None,max_slack=0.)
        checked=0
        for pair in pairs:
            if pair[0]>0:break
            if phase in ('lower','release') and pair[1]=='payload/banana' and pair[2]==DESTINATION_MESH:continue
            value=w._measurement(pair);checked+=1
            if not value['valid']:other_failures.append(dict(step=step,phase=phase,measurement=value))
        pair_count+=checked
        current=dict(step=step,phase=phase,native_count=n,native_loaded_count=int(loaded.sum()),native_original_force_difference_n=difference,
            native_force_n=summed.tolist(),original_force_n=filtered.tolist(),individual_loaded_native_normals_in_cone=individual_cone,
            native_sum_original_support_cone=support(summed),original_filtered_support_cone=support(filtered),
            native_points_original_bowl_region=inregion,negative_normal_loads=int((loads<0).sum()),
            native_separation_range_m=[float(separation.min()),float(separation.max())] if n else None,
            native_loaded_separation_range_m=[float(separation[loaded].min()),float(separation[loaded].max())] if loaded.any() else None,
            banana_speed_m_s=float(np.linalg.norm(d['task_object_states'][i,bi,7:10])),
            banana_angular_speed_rad_s=float(np.linalg.norm(d['task_object_states'][i,bi,10:13])),banana_pose=d['task_object_states'][i,bi,:7].tolist(),
            actual_original_predicate=raw_destination(d['task_object_states'][i,bi],filtered,center,d['task_object_states'][i,di,:7],[low,high]),
            checked_other_potential_pairs=checked)
        contributions=loads[:,None]*normals
        current.update(raw_positive_scalar_count=int((loads>0).sum()),raw_negative_scalar_count=int((loads<0).sum()),
            raw_zero_scalar_count=int((loads==0).sum()),nonzero_contact_force_vector_count=int((np.linalg.norm(contributions,axis=1)>0).sum()),
            native_signed_contact_force_vectors_n=contributions.tolist())
        if phase in ('lower','release'):
            obstacle=next(x for x in w.objects if x[0]==DESTINATION_MESH);dv,df=w.meshes[DESTINATION_MESH]
            shape=w.fcl.CollisionObject(w.payload_triangle_model,w.fcl.Transform(bp[:3,:3],bp[:3,3]))
            strict=w._exact_measurement((0.,'payload/banana',DESTINATION_MESH,shape,obstacle[1],.002 if phase=='lower' else 0.))
            cr=w.fcl.CollisionResult();w.fcl.collide(shape,obstacle[1],w.fcl.CollisionRequest(num_max_contacts=4096,enable_contact=True),cr)
            require(len(cr.contacts)<4096,'Source pair inventory saturated')
            bv=w.payload_geometry.vertices@bp[:3,:3].T+bp[:3,3];pieces=[];contacts=[]
            for c in cr.contacts:
                bt=bv[w.payload_geometry.triangles[c.b1]];dt=dv[df[c.b2]];piece=triangle_intersection(bt,dt)
                require(len(piece)>0,'Unreconstructed source intersection');pieces.append(piece)
                contacts.append(dict(banana_triangle=int(c.b1),bowl_triangle=int(c.b2),intersection_points_world_m=piece.tolist(),
                    source_pair_depth_m=float(c.penetration_depth)))
            sourcelocal=(np.concatenate(pieces)-dp[:3,3])@dp[:3,:3] if pieces else np.empty((0,3))
            sourceinside=bool(len(sourcelocal) and np.all(sourcelocal[:,:2]>=low[:2]) and np.all(sourcelocal[:,:2]<=high[:2]) and np.all(sourcelocal[:,2]>=low[2]))
            previous=savedby[step];old=previous['source']
            equal=bool(old['intersecting']==strict['intersecting'] and abs(old['distance_m']-strict['distance_m'])<1e-12 and
                previous['original_strict_measurement']['valid']==strict['valid'] and len(old['contacts'])==len(contacts))
            mapping={(x['banana_triangle_index'],x['bowl_triangle_index']):x for x in old['contacts']}
            for c in contacts:
                key=(c['banana_triangle'],c['bowl_triangle']);equal=equal and key in mapping and np.allclose(mapping[key]['intersection_points_world_m'],c['intersection_points_world_m'],atol=1e-10,rtol=0)
            if not equal:source_mismatches.append(step)
            native_support=(support(summed) and np.any(np.linalg.norm(contributions,axis=1)>0)) if signed_vector_rule else (
                loaded.any() and not (loads<0).any() and individual_cone)
            diagnostic_allowed=bool(phase=='release' and strict['intersecting'] and native_support and
                support(filtered) and sourceinside and inregion and current['actual_original_predicate']['source_footprint'])
            if bounded_observation_rule and phase=='release':
                observation_ok=bool((n==0 or inregion) and (not strict['intersecting'] or sourceinside) and
                    bool(strict['intersecting'])==bool(contacts) and current['actual_original_predicate']['source_footprint'])
                require(previous['bounded_transition']['passed']==observation_ok,'Recorded bounded observation guard differs')
                current['bounded_observation_guard_passed']=observation_ok
                diagnostic_allowed=bool(strict['intersecting'] and observation_ok)
            require(previous['diagnostic_continuation']==diagnostic_allowed,'Recorded diagnostic continuation differs from independent native/source checks')
            current.update(source_measurement=strict,source_triangle_pairs=len(contacts),source_intersections_original_bowl_region=sourceinside,
                source_pair_depth_range_m=[min(c['source_pair_depth_m'] for c in contacts),max(c['source_pair_depth_m'] for c in contacts)] if contacts else None,
                original_strict_passed=strict['valid'],diagnostic_continuation=diagnostic_allowed,
                source_contacts=contacts)
            source_rows.append(current)
        allrows.append(current)
        if i%500==0:print('CPU_SOURCE_NATIVE',i,flush=True)
    release=[x for x in source_rows if x['phase']=='release'];lower=[x for x in source_rows if x['phase']=='lower']
    failures=[x['step'] for x in source_rows if not x['original_strict_passed']]
    checks=dict(every_actual_state_checked=len(allrows)==len(d['physics_step']),every_native_buffer_complete=True,
        actual_original_identity_preserved=True,every_native_force_sum_reproduced=max_saved_arithmetic_error<1e-12,
        all_other_source_pairs_passed=not other_failures,all_lower_source_2mm_passed=all(x['original_strict_passed'] for x in lower),
        every_recorded_source_clipping_and_measurement_reproduced=not source_mismatches,
        every_original_failure_retained=[x['physics_step'] for x in ledger]==failures,
        release_at_most_56=len(release)<=56,never_accepted=r['accepted'] is False and r['task_success'] is False and r['physical_trial_passed'] is False)
    return dict(schema_version=1,accepted=False,task_success=False,physical_trial_passed=False,diagnostic_only=True,
        independent_record_checks_passed=all(checks.values()),checks=checks,physics_samples=len(allrows),
        checked_other_potential_pairs=pair_count,other_source_failures=other_failures,source_mismatch_steps=source_mismatches,
        strict_destination_failure_steps=failures,release_steps=len(release),release_timeline=release,
        minimum_lower_source_distance_m=min(x['source_measurement']['distance_m'] for x in lower),
        native_source_rows=source_rows,native_contact_stats=[x for x in allrows if x['native_count']],
        native_original_force_difference_max_n=max_native_delta,saved_arithmetic_error_max_n=max_saved_arithmetic_error,
        separation=dict(range_m=[min(gaps),max(gaps)] if gaps else None,loaded_range_m=[min(loaded_gaps),max(loaded_gaps)] if loaded_gaps else None,negative_points=negative),
        offsets=native['offsets'],offset_capacity_not_actual_shapes=True,
        original_predicate=dict(successful_release_samples=sum(x['actual_original_predicate']['passed'] for x in release),
            final=allrows[-1]['actual_original_predicate']),
        input_sha256={n:sha(p/n) for n in ['physics_states.npz','physics_metadata.json','result.json','trajectory.npz','destination_native_contacts.json','destination_source_observations.json','destination_strict_failure_ledger.json','task_predicate_metadata.json']},
        declaration_schema_version=r['declaration']['schema_version'],review_source_sha256=sha(__file__),limits=['Schema1 required individual positive-scalar normals; schema2 required aggregate support each intersection; schema3 records all complete spatially bounded transient observations and evaluates the unchanged terminal predicate separately. No schema promotes a diagnostic.','Source triangle-pair depth is not native penetration. Offsets are capacity arrays, not enumerated cooked shapes.','No visual or CUDA self-collision claim; no production acceptance or policy change.'])


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--episode',required=True,type=Path);p.add_argument('--neutral',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    m=read(args.episode/'physics_metadata.json');r=read(args.episode/'result.json')
    require(r['status']!='running','Do not audit a live incomplete output')
    d=dict(np.load(args.episode/'physics_states.npz',allow_pickle=False));t=dict(np.load(args.episode/'trajectory.npz',allow_pickle=False))
    g=generic(args.episode,args.neutral,m,d,t,r);write(args.output/'generic_review.json',g)
    try:s=source_and_native(args.episode,m,d,t,r)
    except Exception as exc:s=dict(accepted=False,task_success=False,independent_record_checks_passed=False,error=f'{type(exc).__name__}: {exc}',review_source_sha256=sha(__file__))
    write(args.output/'source_native_review.json',s)
    print(json.dumps(dict(output=str(args.output),generic_checks={k:v['passed'] for k,v in g['details'].items()},source_native_checks=s.get('checks'),error=s.get('error'))),flush=True)
    return 0 if g['recording_checks_passed'] and s['independent_record_checks_passed'] else 2


if __name__=='__main__':raise SystemExit(main())
