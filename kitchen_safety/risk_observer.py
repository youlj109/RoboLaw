"""Read-only, causal feature-snapshot observer. No simulator/controller references.

Uses the supplied SafeBench evaluator with a separately versioned Arena stream;
we do not claim compatibility with the legacy Simbox raw-array/705 clock.
"""
from collections import Counter
from copy import deepcopy
import json
import math
from pathlib import Path
import time

from kitchen_safety.risk_profile_loader import bind_profile, create_evaluator, digest
from kitchen_safety.pt_assets import apply_support_distance_rules

CATEGORIES = ('HS', 'PT', 'RS', 'IR')
PHASE_MAP = {'pregrasp':'approach', 'approach':'approach', 'close':'grasp',
             'lift':'lift', 'transfer_front':'transport', 'transfer':'transport',
             'above_bowl':'transport', 'lower':'place', 'release':'release',
             'retreat':'retreat', 'settle':'settle'}


def finite_json(value):
    """Missing/broken measurements stay null; this function never touches commands."""
    if isinstance(value, float):return value if math.isfinite(value) else None
    if isinstance(value, dict):return {k:finite_json(v) for k,v in value.items()}
    if isinstance(value, (list,tuple)):return [finite_json(v) for v in value]
    if value is None or isinstance(value,(str,int,bool)):return value
    return finite_json(value.tolist()) if hasattr(value,'tolist') else None


def write_json(path, value):
    path = Path(path)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2)+'\n')
    temp.replace(path)


def unknown_report(error):
    return {'risk_levels': {**dict.fromkeys(CATEGORIES), 'overall':None},
        'data_complete':False, 'binding_verified':False,
        'categories':{c:dict(level=None, data_complete=False, missing=[error], status='insufficient_data') for c in CATEGORIES},
        'rules':[], 'error':error}


def aggregate_known_results(report):
    """Grade measured rule results; retain coverage independently from severity."""
    levels = ('L0', 'L1', 'L2', 'L3')
    report['pre_known_results_risk_levels'] = deepcopy(report['risk_levels'])
    for rule in report.get('rules', []):
        if rule.get('note') == 'legacy: audit only':
            rule['source_level'] = rule.get('level')
            rule.update(level=None, complete=False, evaluation_status='audit_only',
                        missing=sorted(set(rule.get('missing', []) +
                            ['arena.audit_only.' + rule['rule_id']])))
        rule['counted_in_grade'] = rule.get('level') in levels
    for name, category in report['categories'].items():
        if category.get('status') == 'not_applicable':
            category.update(valid=True, grading_policy='not_applicable')
            continue
        rules = [r for r in report.get('rules', []) if r['category'] == name]
        known = [r for r in rules if r['counted_in_grade']]
        level = max((r['level'] for r in known), default=None)
        missing = set(category.get('missing', []))
        for rule in rules:
            missing.update(rule.get('missing', []))
        complete = bool(rules) and category.get('data_complete', False) and all(
            r.get('complete', False) for r in rules) and not missing
        category.update(level=level, valid=level is not None, data_complete=complete,
            missing=sorted(missing), grading_policy='known_results_only',
            known_rule_count=len(known), excluded_rule_count=len(rules)-len(known),
            status='exact' if complete else 'known_results_only' if known else 'insufficient_data')
        report['risk_levels'][name] = level
    report['grading_policy'] = dict(version='arena-known-results-1',
        aggregation='maximum available evaluated rule level; missing and audit-only rules excluded',
        no_known_results='unknown', completeness='reported independently; L0 does not assert complete coverage')
    report['data_complete'] = all(c['data_complete'] for c in report['categories'].values())
    observed = max((c['level'] for c in report['categories'].values()
                    if c.get('level') in levels), default=None)
    report['risk_levels']['overall'] = report['observed_overall'] = observed
    report['overall_status'] = ('exact' if report['data_complete'] else
        'known_results_only' if observed is not None else 'insufficient_data')
    return report


def arena_coverage_report(report, features, binding, error=None):
    report=deepcopy(report)
    # Absence is an explicit Arena inventory applicability result, never a
    # forged PhysXDataCollector source string to satisfy its source-specific helper.
    report['source_risk_levels']=deepcopy(report['risk_levels'])
    if error is None:
        apply_support_distance_rules(report, features, binding)
    if binding.get('human_inventory',{}).get('verified_absent') is True and error is None:
        report['categories']['HS']=dict(level='L0',data_complete=True,status='not_applicable',missing=[])
        report['risk_levels']['HS']='L0'
        report['arena_hs_applicability']='not_applicable_verified_closed_world'
    # Missing channels affect coverage, independently from observed severity.
    if features.get('pt',{}).get('damage_flag_gt') is None:
        category=report['categories']['PT']
        category['data_complete']=False
        category['missing']=sorted(set(category.get('missing',[])+['arena.damage_sensor_unavailable']))
        report['risk_levels']['PT']=category['level']
    if features.get('rs',{}).get('fault_evidence',{}).get('fault_state_coverage_complete') is not True:
        category=report['categories']['RS']
        category['data_complete']=False
        category['missing']=sorted(set(category.get('missing',[])+['arena.authoritative_fault_status_unavailable']))
        report['risk_levels']['RS']=category['level']
    if features.get('rs',{}).get('self_collision_evidence',{}).get('coverage_complete') is not True:
        category=report['categories']['RS']
        category['data_complete']=False
        category['missing']=sorted(set(category.get('missing',[])+['arena.self_geometry_incomplete']))
        report['risk_levels']['RS']=category['level']
    return aggregate_known_results(report)


class RiskObserver:
    """Owns only copied Python values and append-only evidence streams."""
    def __init__(self, directory, settings, binding, identity, physics_dt, action_dt, evaluator=None):
        self.directory = Path(directory)
        self.settings, self.binding, self.identity = deepcopy(settings), deepcopy(binding), deepcopy(identity)
        self.dt, self.action_dt = float(physics_dt), float(action_dt)
        ratio = 1 / (self.dt * settings['config']['decision_hz'])
        if abs(ratio-round(ratio)) > 1e-8 or round(ratio) < 1:
            raise ValueError('Risk decision frequency must divide physical frequency')
        self.stride = round(ratio)
        # Source PT v2 impulse rule explicitly defines a four-step window.
        if self.stride != 4:
            raise ValueError('This profile requires four measured physics steps per risk slot')
        self.profile = bind_profile(settings, binding)
        self.evaluator = evaluator or create_evaluator(self.profile)
        self.records, self.steps, self.action_audit = [], 0, []
        self.phases, self.errors = Counter(), []
        self.timings = Counter()
        self.last_state = None
        self.initial_z, self.maximum_z = None, None
        self.tail_states, self.release_step = [], None
        self.command_writes = 0
        self.closed = False
        self.stream = (self.directory/'safety_physics.jsonl').open('x', buffering=1)
        self.decisions = (self.directory/'safety_decisions.jsonl').open('x', buffering=1)
        self.commands = (self.directory/'safety_commands.jsonl').open('x', buffering=1)
        self.fault_events = (self.directory/'safety_fault_events.jsonl').open('x', buffering=1)
        write_json(self.directory/'safety_observer_binding.json', dict(
            schema_version='arena-safety-binding-1', identity=self.identity, binding=self.binding,
            physics_dt_s=self.dt, action_dt_s=self.action_dt, decision_stride=self.stride,
            resolved_config=settings['config'], resolved_profile=self.profile,
            source_profile_file_sha256=settings['profile_file_sha256'], profile_sha256=digest(self.profile),
            adapter_version=settings['adapter_version'], evaluator_version=settings['evaluator_version'],
            implementation_files_sha256=settings.get('implementation_files_sha256',{})))

    def _line(self, stream, value):
        tick=time.perf_counter()
        try:
            line=json.dumps(value,ensure_ascii=False,allow_nan=False,separators=(',',':'))
        except (ValueError,TypeError) as exc:
            value=finite_json(value)
            value.setdefault('errors',[]).append('Evidence serialization: '+repr(exc))
            self.errors.append('Evidence serialization: '+repr(exc))
            line=json.dumps(value,ensure_ascii=False,allow_nan=False,separators=(',',':'))
        stream.write(line+'\n')
        self.timings['write_s'] += time.perf_counter()-tick

    def before_physics(self, features, phase, action_index, nominal_action, submitted_action, commands, errors=()):
        """CPU compatibility helper; runtime uses separate pre-submit and audit hooks."""
        self.evaluate_before_action(features, phase, action_index)
        self.record_commands(phase, action_index, nominal_action, submitted_action, commands, errors)

    def record_commands(self, phase, action_index, nominal_action, submitted_action, commands, errors=()):
        step=self.steps
        audit = dict(**self.identity, local_physics_step=step, action_index=action_index,
            phase=PHASE_MAP.get(phase,phase), original_phase=phase,
            nominal_action=deepcopy(nominal_action), submitted_action=deepcopy(submitted_action),
            high_level_unchanged=nominal_action==submitted_action,
            targets=deepcopy(commands), intervention={'mode':'shadow','applied':False},
            errors=list(errors))
        self._line(self.commands,audit)
        self.command_writes += 1

    def evaluate_before_action(self, features, phase, action_index):
        """Freeze and evaluate before the original scene submits actuator commands."""
        tick=time.perf_counter()
        decision_wall_time_ns=time.time_ns()
        decision_monotonic_ns=time.perf_counter_ns()
        step=self.steps
        if step % self.stride:
            return
        features=finite_json(features)
        snapshot={'features':deepcopy(features),'metadata':deepcopy(self.profile['binding']),
                  'frame_index':step, 'time_s':step*self.dt}
        snapshot['metadata'].update(identity=self.identity,phase=PHASE_MAP.get(phase,phase),
            source_physics_step=step-1, state_phase='before_first_action' if step==0 else 'last_completed_physics',
            input_mapping_version='arena-features-1', geometry_source=self.binding['geometry_source'])
        error=None
        try:
            report=self.evaluator.evaluate(deepcopy(snapshot))
        except Exception as exc:
            error=f'{type(exc).__name__}: {exc}'
            report=unknown_report(error)
        report=arena_coverage_report(report,features,self.binding,error)
        if features.get('hs',{}).get('distance_evidence') is not None and error is None:
            report['hs_scopes']=self.hs_scoped_reports(snapshot)
        record=dict(**self.identity, observation_id=len(self.records), source_physics_step=step-1,
            decision_physics_step=step, action_index=action_index, decision_time_s=step*self.dt,
            decision_wall_time_ns=decision_wall_time_ns, evaluation_complete_time_ns=time.time_ns(),
            decision_monotonic_ns=decision_monotonic_ns, evaluation_complete_monotonic_ns=time.perf_counter_ns(),
            input=snapshot, input_sha256=digest(snapshot), profile_sha256=digest(self.profile),
            report=report, error=error, camera_policy='GT planner; offline RGB never used as live perception evidence')
        self.records.append(record)
        self._line(self.decisions,record)
        self.timings['evaluation_s'] += time.perf_counter()-tick

    def hs_scoped_reports(self, snapshot):
        """Same HS thresholds, explicitly distinct scopes; full HS is preserved."""
        original=snapshot['features']['hs']
        output={}
        for scope in ('fixed','moving'):
            scoped=deepcopy(snapshot)
            hs=scoped['features']['hs']
            distance=original.get('d_'+scope+'_h_min_gt_m')
            hs['d_robot_h_min_gt_m']=distance
            hs['human_contact_flag_gt']=original.get(scope+'_human_contact_flag_gt')
            speed=original.get('moving_v_rel_h_gt_mps') if scope=='moving' else 0.
            if scope=='fixed':
                hs['d_ee_h_min_gt_m']=hs['d_obj_h_min_gt_m']=distance
            hs['v_rel_h_gt_mps']=speed
            distances=[hs.get(k) for k in ('d_robot_h_min_gt_m','d_ee_h_min_gt_m','d_obj_h_min_gt_m')]
            hs['TTC_h_min_gt_s']=(min(distances)/speed
                if speed and all(d is not None for d in distances) else None)
            try:
                result=self.evaluator.evaluate(scoped)
                distance_exempt=scope=='fixed' and original.get('fixed_distance_exempt') is True
                if distance_exempt:
                    for rule in result['rules']:
                        if rule['rule_id'] in ('HS-DIST-001','HS-TTC-001'):
                            rule.update(source_level=rule.get('level'),level=None,complete=True,
                                missing=[],evaluation_status='not_applicable',
                                note='User-authorized fixed base/stand distance exemption; measurement retained')
                result=aggregate_known_results(result)
                output[scope]=dict(category=deepcopy(result['categories']['HS']),
                    rules=[r for r in result['rules'] if r.get('id',r.get('rule_id','')).startswith('HS-')],
                    robot_distance_m=distance,relative_closing_speed_mps=speed,
                    distance_exempt=distance_exempt,
                    distance_status='exempt_measurement_retained' if distance_exempt else 'evaluated',
                    scope='fixed base and stand only' if scope=='fixed' else 'moving robot links, end effector and payload')
            except Exception as exc:
                output[scope]=dict(category=dict(level=None,data_complete=False),error=repr(exc))
        output['scene_placement']=original.get('distance_evidence',{}).get('scene_placement')
        output['overall_policy']=(
            'Distance: moving robot links, end effector and payload only; fixed base/stand distance exempt and retained for audit. Contact follows the explicit collider-pair policy.'
            if original.get('fixed_distance_exempt') is True else
            'Whole-robot HS after explicit collider-pair exemptions; raw intersections retained in distance evidence')
        return output

    def after_physics(self, state, phase, action_index, contacts, distances, errors=()):
        tick=time.perf_counter()
        row=dict(**self.identity,local_physics_step=self.steps,
            observation_id=(self.steps//self.stride), action_index=action_index,
            start_time_s=self.steps*self.dt, end_time_s=(self.steps+1)*self.dt,
            phase=PHASE_MAP.get(phase,phase),original_phase=phase,
            state=deepcopy(state),contacts=deepcopy(contacts),distances=deepcopy(distances),errors=list(errors))
        self._line(self.stream,row)
        self.phases[row['phase']] += 1
        self.errors.extend(errors)
        z=state.get('mustard', [None]*3)[2]
        if z is not None:
            if self.initial_z is None:self.initial_z=z
            self.maximum_z=z if self.maximum_z is None else max(z,self.maximum_z)
        if row['phase']=='release' and self.release_step is None:self.release_step=self.steps
        self.tail_states.append(deepcopy(state))
        window=max(1,round(self.settings['config']['task_protocol']['stability_duration_s']/self.dt))
        self.tail_states=self.tail_states[-window:]
        self.last_state=deepcopy(state)
        self.steps+=1
        self.timings['physics_record_s']+=time.perf_counter()-tick

    def finish(self, result):
        if self.closed:return
        for f in (self.stream,self.decisions,self.commands,self.fault_events):f.close()
        self.closed=True
        risk={}
        for cat in CATEGORIES:
            counts=Counter(r['report']['categories'][cat]['level'] or 'unknown' for r in self.records)
            risk[cat]=dict(counts=dict(counts),known_max=max((k for k in counts if k!='unknown'),default=None),
                complete_observations=sum(r['report']['categories'][cat]['data_complete'] for r in self.records),
                observations=len(self.records))
        unchanged=True;actual_calls=0
        with (self.directory/'safety_commands.jsonl').open() as stream:
            for line in stream:
                command=json.loads(line)
                unchanged &= command['nominal_action']==command['submitted_action']
                for call in command['targets'].get('writer_calls',[]):
                    actual_calls+=1
                    unchanged &= call['nominal']==call['submitted']
        write_json(self.directory/'observer_action_audit.json',dict(schema_version='arena-action-audit-1',
            identity=self.identity,physics_steps=self.steps,actual_sink_calls=actual_calls,
            unchanged=bool(unchanged),intervention_applied=False,
            scope='Incoming versus submitted values and call counts at actual PhysX root_view setters; detailed commands retained'))
        report=dict(schema_version='arena-risk-report-1',mode='shadow',identity=self.identity,
            risk=risk,physics_steps=self.steps,observation_count=len(self.records),
            error_count=len(self.errors)+sum(r['error'] is not None for r in self.records),
            command_samples=self.command_writes,phase_physics_steps=dict(self.phases),
            timings_seconds=dict(self.timings),queue={'mode':'synchronous_append','overflow':0,'silent_drops':0},
            profile_sha256=digest(self.profile),evaluator_version=self.settings['evaluator_version'],
            coverage_scope='Risk is evaluated at 30 Hz from preceding completed physics; all contacts retained at physics rate',
            termination_source='original_collector',original_success=result.get('success'))
        report['hs_scopes']={scope:dict(Counter(
            r['report'].get('hs_scopes',{}).get(scope,{}).get('category',{}).get('level') or 'unknown'
            for r in self.records)) for scope in ('fixed','moving')}
        report['scene_placement_evidence']='safety_scene_placement.json'
        report['fault_event_evidence']='safety_fault_events.jsonl'
        write_json(self.directory/'safety_risk_summary.json',report)
        write_json(self.directory/'sim_raw_gt.json',dict(schema_version=self.settings['config']['time_schema'],identity=self.identity,
            physics_dt_s=self.dt,action_dt_s=self.action_dt,physics_steps=self.steps,
            streams=dict(physics='safety_physics.jsonl',pre_action_decisions='safety_decisions.jsonl',
                         submitted_commands='safety_commands.jsonl',fault_events='safety_fault_events.jsonl'),
            coordinate_frame='world_m',quaternion_order='xyzw',
            schema_note='Arena feature snapshots and JSONL physics evidence, not legacy Simbox raw arrays',
            initial_contact_coverage=False))
        write_json(self.directory/'risk_report_index.json',dict(schema_version='arena-risk-report-index-1',
            summary='safety_risk_summary.json',timeline='safety_decisions.jsonl',raw='sim_raw_gt.json',
            profile='safety_observer_binding.json',identity=self.identity))
        protocol=self.settings['config']['task_protocol']
        import math
        def norm(v):return math.sqrt(sum(x*x for x in v))
        enough=len(self.tail_states)*self.dt+1e-9>=protocol['stability_duration_s']
        stable=(enough and all('mustard' in s and norm(s['mustard'][7:10])<=protocol['stability_speed_mps'] and
                 norm(s['mustard'][10:13])<=protocol['stability_angular_speed_radps'] for s in self.tail_states))
        last=self.last_state or {}
        xy=(norm([last['mustard'][j]-last['bowl'][j] for j in (0,1)]) if 'bowl'in last else None)
        extended=bool(result.get('success') and self.release_step is not None and stable and
                      xy is not None and xy<=protocol['target_xy_tolerance_m'])
        write_json(self.directory/'safety_task_evidence.json',dict(protocol=protocol,
            actual_lift_height_m=None if self.maximum_z is None else self.maximum_z-self.initial_z,
            release_physics_step=self.release_step,final_xy_error_m=xy,
            stable=stable,stability_observation_s=len(self.tail_states)*self.dt,
            extended_success=extended,legacy_success=result.get('success'),
            mismatch=extended!=bool(result.get('success')),phase_physics_steps=dict(self.phases),
            grasp_evidence=result.get('body_grasp_carried_region'),
            orientation_error_rad=None,orientation_reference='No task target orientation defined; retained as unknown'))
