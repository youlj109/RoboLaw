"""Read-only observer validation and independent result axes for every attempt."""
from collections import Counter
import json
from pathlib import Path
import numpy as np

from kitchen_safety.risk_observer import CATEGORIES, write_json, arena_coverage_report
from kitchen_safety.risk_profile_loader import create_evaluator, digest


def read_json(path, default=None):
    return json.loads(Path(path).read_text()) if Path(path).exists() else default


def lines(path):
    with Path(path).open() as f:
        for line in f:
            yield json.loads(line)


def validate_observer(directory, *, render_root=None, replay=True):
    directory=Path(directory)
    errors=[]
    def require(ok,message):
        if not ok:errors.append(message)
    binding=read_json(directory/'safety_observer_binding.json')
    if binding is None:return dict(passed=False,errors=['Missing runtime observer binding'])
    identity=binding['identity'];dt=binding['physics_dt_s'];stride=binding['decision_stride']
    profile=binding['resolved_profile'];evaluator=create_evaluator(profile)
    decisions=list(lines(directory/'safety_decisions.jsonl'))
    previous=-1
    for j,r in enumerate(decisions):
        require(all(r.get(k)==v for k,v in identity.items()),f'observation {j}: identity differs')
        require(r['observation_id']==j and r['decision_physics_step']==j*stride,f'observation {j}: cadence gap')
        require(r['source_physics_step']==j*stride-1,f'observation {j}: future evidence')
        require(r['input_sha256']==digest(r['input']),f'observation {j}: input digest differs')
        require(r['profile_sha256']==digest(profile),f'observation {j}: profile differs')
        require(r['evaluation_complete_time_ns']>=r['decision_wall_time_ns'],f'observation {j}: completion clock')
        if replay and not r.get('error'):
            expected=evaluator.evaluate(r['input'])
            actual=r['report']
            require(actual.get('source_risk_levels',actual['risk_levels'])==expected['risk_levels'],f'observation {j}: rule replay differs')
            if 'source_risk_levels' in actual:
                enriched=arena_coverage_report(expected,r['input']['features'],binding['binding'])
                require(actual['risk_levels']==enriched['risk_levels'],f'observation {j}: Arena coverage label differs')
                for cat in CATEGORIES:
                    for field in ('level','data_complete','missing'):
                        require(actual['categories'][cat][field]==enriched['categories'][cat][field],f'observation {j}: {cat} {field} differs')
            require(actual.get('input_sha256')==expected['input_sha256'],f'observation {j}: evaluator input differs')
        previous=j
    steps=0;coverage=0;contact_count=0
    decimation=round(binding['action_dt_s']/dt)
    action_end_states={}
    source_root=f"/World/envs/env_{identity['env_id']}"
    for j,r in enumerate(lines(directory/'safety_physics.jsonl')):
        steps+=1
        if (j+1)%decimation==0:action_end_states[r['action_index']]=r.get('state',{})
        require(all(r.get(k)==v for k,v in identity.items()),f'physics {j}: identity differs')
        require(r['local_physics_step']==j,f'physics {j}: gap')
        require(r['observation_id']==j//stride and j//stride<len(decisions),f'physics {j}: bad decision reference')
        require(abs(r['start_time_s']-j*dt)<1e-8 and abs(r['end_time_s']-(j+1)*dt)<1e-8,f'physics {j}: clock mismatch')
        require(not r.get('errors'),f'physics {j}: sampling error {r.get("errors")}')
        coverage+=bool(r['contacts']['coverage'])
        for pair in r['contacts']['pairs']:
            contact_count+=1
            for key in ('bodyA','bodyB'):
                path=pair[key]
                require('/World/envs/' not in path or path.startswith(source_root+'/'),f'physics {j}: contact from another env')
            require(np.isclose(pair['force_n']*dt,pair['impulse_ns']),f'physics {j}: impulse/force units')
    commands=0;send_calls=0
    for j,r in enumerate(lines(directory/'safety_commands.jsonl')):
        commands+=1
        require(all(r.get(k)==v for k,v in identity.items()),f'command {j}: identity differs')
        require(r['local_physics_step']==j,f'command {j}: gap')
        require(r['nominal_action']==r['submitted_action'] and r['high_level_unchanged'],f'command {j}: high-level action changed')
        require(r['intervention']=={'mode':'shadow','applied':False},f'command {j}: unexpected intervention')
        events=r['targets']['writer_calls'];send_calls+=len(events)
        require(Counter(x['method'] for x in events)==Counter({k:1 for k in
            ('set_dof_actuation_forces','set_dof_position_targets','set_dof_velocity_targets')}),f'command {j}: actual sends missing or duplicated')
        for event in events:
            if binding['resolved_config']['time_schema']=='arena-causal-physics-2':
                decision=decisions[j//stride]
                require(event.get('submitted_monotonic_ns',-1)>=decision.get('evaluation_complete_monotonic_ns',0),f'command {j}: submission precedes frozen risk input')
            require(event['nominal']==event['submitted'] and event['unchanged'],f'command {j}: low-level target changed')
            if binding['binding'].get('command_index_schema') == 'physx-submitted-environment-indices-1':
                row=binding['binding']['command_backend_row']
                indices=event.get('submitted_environment_indices')
                require(event.get('environment_row')==row,f'command {j}: wrong backend row')
                require(isinstance(indices,list) and row in indices,f'command {j}: environment absent from actual submission mask')
                require(event.get('indices')==indices,f'command {j}: submission mask was rewritten')
    require(commands==steps,'Physics/command counts differ')
    require(len(decisions)==(steps+stride-1)//stride,'Missing observation or unreferenced tail')
    # Match actual high-level actions and phase to original native trajectory.
    import h5py
    with h5py.File(directory/'trajectory.h5','r') as tr:
        action_count=len(tr['action']);decimation=round(float(tr.attrs['dt'])/dt)
        require(steps==action_count*decimation,'Physical/action count differs')
        origin=np.asarray(tr.attrs.get('environment_origin_world_m',[0,0,0]),dtype=float)
        if tr.attrs.get('coordinate_frame')!='environment_local':origin=np.zeros(3)
        for index,state in action_end_states.items():
            for key in ('q','mustard','bowl'):
                if 'next_obs/'+key not in tr:continue
                expected=np.asarray(tr['next_obs/'+key][index],dtype=float).copy()
                if key!='q':expected[:3]+=origin
                actual=state.get(key)
                require(actual is not None and np.asarray(actual).shape==expected.shape and np.allclose(actual,expected,rtol=0,atol=1e-5),f'action {index}: physics/native {key} state differs')
        for r in lines(directory/'safety_commands.jsonl'):
            j=r['action_index']
            require(0<=j<action_count,'Command action reference out of bounds')
            if 0<=j<action_count:
                require(np.array_equal(np.asarray(r['submitted_action'],dtype=np.float32),tr['action'][j]),f'action {j}: native action differs')
    camera={'status':'pending_offline_render','complete':False}
    root=Path(render_root) if render_root else directory
    if (root/'safety_camera_provenance.jsonl').exists():
        rows=list(lines(root/'safety_camera_provenance.jsonl'));counts=Counter()
        seen=set();source_sequences={}
        for r in rows:
            key=(r['camera_id'],r['acquisition_sequence']);counts[r['camera_id']]+=1
            require(key not in seen,'Duplicate camera sequence');seen.add(key)
            index=r['source_action_index']
            require(type(index) is int and 0<=index<action_count,'Camera source action out of bounds')
            require(r['acquisition_sequence']==index,'Camera acquisition/source sequence mismatch')
            source_sequences.setdefault(r['camera_id'],set()).add(index)
            require(np.isclose(r.get('source_sim_time_s',float('nan')),index*binding['action_dt_s'],rtol=0,atol=1e-8),'Camera source simulation time mismatch')
            require(all(r.get(k)==v for k,v in identity.items()),'Camera from another episode/environment')
            require(r['source_physics_step']==r['source_action_index']*decimation-1,'Camera causal source mismatch')
            require(r['receive_timestamp_ns']>=r['capture_timestamp_ns'],'Camera receive precedes capture')
            require(r['status']=='available' and r['stale'] is False,'Missing/stale camera')
        complete=(set(counts)==set(binding['binding']['camera_names']) and
                  all(v==action_count for v in counts.values()) and
                  all(v==set(range(action_count)) for v in source_sequences.values()))
        require(complete,'Camera coverage differs from actions')
        camera=dict(status='verified' if complete else 'incomplete',complete=complete,counts=dict(counts),
            mode='offline_measured_state_render; not live policy observation')
    return dict(schema_version='arena-observer-validation-1',passed=not errors,errors=errors[:100],
        error_count=len(errors),physics_steps=steps,action_count=action_count,observations=len(decisions),
        physics_contact_coverage=coverage/steps if steps else None,contact_pairs=contact_count,
        submitted_calls=send_calls,action_unchanged=not any('command' in x or 'action' in x for x in errors),
        causal_replay_verified=replay and not errors,camera=camera)


def result_axes(directory, verification=None, render_root=None):
    directory=Path(directory);verification=verification or read_json(directory/'verification.json',{})
    result=read_json(directory/'result.json',{})
    has_observer=(directory/'safety_observer_binding.json').exists()
    try:
        validation=validate_observer(directory,render_root=render_root) if has_observer else dict(passed=False,errors=['Attempt ended before observer initialization'])
    except Exception as exc:validation=dict(passed=False,errors=[repr(exc)])
    risks=read_json(directory/'safety_risk_summary.json',{})
    task_evidence=read_json(directory/'safety_task_evidence.json',{})
    data_report=read_json(directory/'validation.json',[])
    data_report=data_report[0] if isinstance(data_report,list) and data_report else data_report
    legacy_passed=(data_report.get('legacy_passed',True if data_report.get('passed') is True else None)
                   if isinstance(data_report,dict) else None)
    combined_passed=verification.get('data_success')
    if combined_passed is True and (not validation['passed'] or not validation.get('camera',{}).get('complete')):
        combined_passed=False
    obstruction='confirmed' if verification.get('augmentation_eligible') is True else 'unconfirmed' if verification.get('augmentation_eligible') is False else 'unknown'
    axes=dict(schema_version='arena-result-axes-1',
        task=dict(legacy_success=result.get('success'),extended_success=task_evidence.get('extended_success'),
                  mismatch=task_evidence.get('mismatch'),failure_kind=result.get('failure_kind') if not result.get('success') else None,failure_reason=result.get('error')),
        data_integrity=dict(legacy_passed=legacy_passed,combined_passed=combined_passed,safety_extension_passed=validation['passed'],
                            safety_evidence_coverage=validation.get('physics_contact_coverage'),
                            camera_complete=validation.get('camera',{}).get('complete',False)),
        safety=dict(legacy_clearance_passed=verification.get('audit_pass'),risk=risks.get('risk'),
                    risk_complete=bool(risks.get('risk')) and all(v['complete_observations']==v['observations'] and v['observations']>0 for v in risks.get('risk',{}).values())),
        obstruction=dict(status=obstruction,source='Original independent obstruction probe; no safety-label promotion'),
        observer=dict(mode='shadow',action_unchanged=validation.get('action_unchanged',False)),
        task_evidence=task_evidence)
    axes['positive_demonstration']=bool(axes['task']['extended_success'] and verification.get('data_success') and
        verification.get('audit_pass') is True and
        validation['passed'] and axes['data_integrity']['camera_complete'] and axes['safety']['risk_complete'] and
        all(v['known_max']=='L0' for v in risks['risk'].values()))
    write_json(directory/'safety_observer_validation.json',validation)
    write_json(directory/'result_axes.json',axes)
    return axes


def write_indexes(output, attempts):
    output=Path(output);indexes=output/'indexes';indexes.mkdir(exist_ok=True)
    rows=[]
    for attempt in attempts:
        folder=output/attempt['path']
        axes=read_json(folder/'result_axes.json')
        if axes is None:
            # Sampling/planning/init failures remain visible, not fabricated successful zero-step episodes.
            axes=dict(task={'legacy_success':False},data_integrity={'safety_extension_passed':False},
                safety={'risk':None},obstruction={'status':'unknown'},positive_demonstration=False,
                failure_reason=attempt.get('error'),failure_kind=attempt.get('failure_kind'))
        rows.append(dict(attempt_path=attempt['path'],obstacle_id=attempt['obstacle_id'],axes=axes))
    selections={'all_attempts':rows,'successful_tasks':[r for r in rows if r['axes']['task']['legacy_success']],
        'positive_demonstrations':[r for r in rows if r['axes']['positive_demonstration']],
        'obstruction_confirmed':[r for r in rows if r['axes']['obstruction']['status']=='confirmed'],
        'negative_samples':[r for r in rows if r['axes']['task']['legacy_success'] is False],
        'risk_trajectories':[r for r in rows if any(v.get('known_max') in ('L1','L2','L3') for v in (r['axes'].get('safety',{}).get('risk') or {}).values())]}
    for name,values in selections.items():
        path=indexes/(name+'.jsonl');tmp=path.with_suffix('.tmp')
        tmp.write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in values));tmp.replace(path)
    return {name:len(values) for name,values in selections.items()}
