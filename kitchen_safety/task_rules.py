"""Compose frozen HS/PT/RS rules over task-bound relations, without new bands.

This adapter supplies source identity/applicability; numerical decisions remain
in the checked-in reference evaluator and dynamic HS implementation.
"""
from copy import deepcopy
import math
from kitchen_safety.task_contract import digest
from kitchen_safety.risk_profile_loader import create_evaluator
from kitchen_safety.risk_observer import aggregate_known_results
from safety_risk.pt_hs_inspired import evaluate_pairs


def finite_nonnegative(value):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value) and value>=0


def geometry_issues(relation, distance, geometry, targets, metric):
    """A profile name is not evidence that a distance came from that backend."""
    if not isinstance(geometry,dict):return ['geometry_evidence_missing']
    missing=[]
    if geometry.get('metric')!=metric:missing.append('geometry_metric_mismatch')
    if geometry.get('complete') is not True:missing.append('geometry_coverage_incomplete')
    if not finite_nonnegative(distance) or not finite_nonnegative(geometry.get('distance_m')):
        missing.append('geometry_distance_invalid')
    elif distance!=geometry['distance_m']:missing.append('geometry_distance_mismatch')
    if geometry.get('source_collider') not in relation['source_colliders']:
        missing.append('geometry_source_collider_unbound')
    if geometry.get('destination_collider') not in targets:missing.append('geometry_target_collider_unbound')
    effective=geometry.get('effective_target_colliders')
    if (not isinstance(effective,list) or not all(isinstance(p,str) for p in effective)
            or len(set(effective))!=len(effective) or set(effective)!=set(targets)):
        missing.append('geometry_target_coverage_mismatch')
    return missing


class TaskRules:
    def __init__(self,contract,scope,rs_scope,frozen):
        self.scope=scope;self.contract=contract
        self.profile=deepcopy(frozen['profile']);self.source_hash=frozen['profile_sha256']
        self.profile['binding'].update(scenario_id=contract['task_id'],payload_profile='task-bound-multiple-payloads',
            object_class='task-bound-asset-inventory')
        self.profile['rules'].update(disabled_categories=['IR'],allowed_self_contact_pairs=rs_scope['allowed_pairs'])
        self.profile['runtime_binding']=dict(human_inventory=scope['human_inventory'],
            hs_geometry=dict(dynamic_relation_ids=[r['relation_id'] for r in scope['relations'] if r['category']=='HS']))
        self.profile['pt_hs_inspired']['pairs']=[]
        self.base=create_evaluator(self.profile)
        self.relations={r['relation_id']:r for r in scope['relations'] if r['category']=='PT'}
        self.hs_relations={r['relation_id']:r for r in scope['relations'] if r['category']=='HS'}
        self.metric=self.profile['pt_hs_inspired']['distance_metric']
        if self.metric!='gdist_authored_collision_surface_distance':
            raise ValueError('Native task rules require the frozen gDist surface metric')
        self.payload_ids=set(contract['roles']['payload_ids'])
        self.pair_profiles={}
        prototype=frozen['profile']['pt_hs_inspired']['objects']['protected_obstacle']
        self.phase_exempt=frozen['profile']['pt_hs_inspired']['pairs'][0]['allowed_grip_phases']
        for rid,r in self.relations.items():
            actor='held_object' if r['source_role']=='payload' else 'robot' if r['source_role']=='robot' else 'manipulated_link'
            obj=r['target'];spec=dict(actor=actor,object_id=obj)
            settings=deepcopy(frozen['profile']['pt_hs_inspired'])
            settings.update(objects={obj:deepcopy(prototype)},pairs=[spec],target_object_id='__no_payload_relation__')
            settings.pop('distance_omissions',None)
            if r.get('evidence_only'):
                settings['target_object_id']=obj
                spec.update(grip_distance_exempt=True,allowed_grip_link_paths=[r['source']],allowed_grip_phases=self.phase_exempt)
            if not r['query_distance']:
                settings['distance_omissions']=dict(version='arena-pt-distance-omission-1',scope='distance_only_all_phases',
                    authorization='task-bound distance-only policy: '+r.get('reason','explicit frozen scope'),
                    selection_report_sha256=scope['scope_sha256'],relations=[dict(actor=actor,object_id=obj)])
            self.pair_profiles[rid]=dict(pt_hs_inspired=settings)
        self.profile_hash=digest(dict(source_profile_sha256=self.source_hash,task_sha256=digest(contract),
            scope_sha256=scope['scope_sha256'],bound_profile=self.profile,pair_profiles=self.pair_profiles))

    def _distance_targets(self,relation,raw):
        omission=raw.get('distance_omission')
        if omission is None:return relation['target_colliders'],False
        if not isinstance(omission,dict) or not relation['query_distance']:
            raise ValueError('Invalid runtime distance omission')
        selected=set();proof=[]
        for declaration in relation.get('operation_distance_omissions',[]):
            if (omission.get('operation_id')==declaration['operation_id']
                    and raw.get('task_phase')==omission.get('phase')
                    and raw.get('task_phase') in declaration['allowed_phases']):
                selected.update(declaration['target_colliders']);proof.append(declaration['declaration_sha256'])
        targets=set(relation['target_colliders']);selected &= targets
        remaining=sorted(targets-selected)
        if (not selected or omission.get('target_colliders')!=sorted(selected)
                or omission.get('source')!=relation['source'] or omission.get('declaration_sha256')!=proof
                or omission.get('fully_omitted') is not (not remaining)
                or omission.get('query_distance') is not False or omission.get('persist_distance') is not False
                or omission.get('force_and_impulse_retained') is not True):
            raise ValueError('Runtime distance omission is not proved by the frozen operation scope')
        return remaining,not remaining

    def evaluate(self,features):
        # Reference report supplies HS contact, dynamic HS, unchanged RS bands,
        # and audit-only support/damage. Drop is separately bound per payload.
        hs=deepcopy(features.get('hs',{}))
        for pair in hs.get('dynamic_distance',{}).get('pairs',[]):
            relation=self.hs_relations.get(pair.get('relation_id'))
            if relation is None:continue # DynamicHSEvaluator reports unknown relation IDs.
            missing=geometry_issues(relation,pair.get('distance_m'),pair.get('nearest_geometry'),relation['target_colliders'],self.metric)
            for field in ('distance_m','robot_surface_speed_mps','human_surface_speed_mps'):
                if not finite_nonnegative(pair.get(field)):
                    pair[field]=None;missing.append('HS_invalid_measurement:'+field)
            if type(pair.get('human_static')) is not bool:missing.append('HS_human_motion_status_unknown')
            if pair.get('source_actor')!=relation['source'] or pair.get('human_collider')!=relation['target']:
                missing.append('HS_source_identity_mismatch')
            if missing:
                pair.update(complete=False,level=None,missing=list(pair.get('missing',[]))+missing)
        report=self.base.evaluate(dict(features=dict(features,hs=hs,pt=dict(support_margin_gt_m=None,damage_flag_gt=None))))
        report['rules']=[r for r in report['rules'] if r['rule_id']!='PT-DROP-001']
        report['distance_omissions']={}
        payloads=features['pt']['payloads']
        if not set(payloads)<=self.payload_ids:raise ValueError('Unexpected payload feature identity')
        missing_payloads=self.payload_ids-set(payloads)
        for aid in sorted(self.payload_ids):
            payload=payloads.get(aid,dict(drop_flag_gt=None,h_drop_gt_m=None))
            partial=self.base.evaluate(dict(features=dict(pt=payload)))
            for rule in partial['rules']:
                if rule['rule_id']=='PT-DROP-001':
                    rule.update(relation_id='payload:'+aid,payload_id=aid);report['rules'].append(rule)
        pairs={}
        for raw in features['pt']['protected_pairs']:
            rid=raw['relation_id']
            if rid not in self.relations or rid in pairs:raise ValueError('Unexpected or duplicate PT relation feature')
            pairs[rid]=raw
        missing_relations=set(self.relations)-set(pairs)
        for rid,r in self.relations.items():
            raw=pairs.get(rid,dict(relation_id=rid))
            profile=self.pair_profiles[rid]
            settings=profile['pt_hs_inspired'];spec=settings['pairs'][0]
            data=deepcopy(raw);data.update(actor=spec['actor'],object_id=spec['object_id'])
            omission=raw.get('distance_omission')
            targets,fully_omitted=self._distance_targets(r,raw)
            if omission:report['distance_omissions'][rid]=deepcopy(omission)
            source='link_target_gdist_surface_distance_gt' if r.get('evidence_only') else 'pt_environment_gdist_surface_distance_gt'
            evidence_missing=[]
            if r['query_distance'] and not fully_omitted:
                geometry=raw.get('distance_geometry_evidence')
                evidence_missing=geometry_issues(r,raw.get('distance_m'),geometry,targets,self.metric)
                geometry=geometry if isinstance(geometry,dict) else {}
                covered=raw.get('distance_coverage_complete') is True and not evidence_missing
                data.update(distance_metric=geometry.get('metric'),distance_source=source,distance_coverage_complete=bool(covered),
                    link_distances_m={r['source']:raw.get('distance_m')},
                    distance_geometry_evidence=dict(metric=geometry.get('metric'),distance_source=source,
                        coverage_complete=bool(covered),nearest=geometry))
            bounds_missing=list(evidence_missing)
            if r['query_distance'] and not fully_omitted:
                params=settings['objects'][spec['object_id']]['distance']
                for field,bound in [('source_surface_speed_mps','robot_approach_bound_mps'),('target_surface_speed_mps','object_approach_bound_mps')]:
                    speed=raw.get(field)
                    if not finite_nonnegative(speed) or speed>params[bound]+1e-12:bounds_missing.append('PT distance assumption not satisfied: '+field)
                if r['source_role']=='manipulated_link':bounds_missing.append('articulation stopping/control model unbound')
            for rule_id,level,evidence,missing in evaluate_pairs([data],profile):
                if rule_id=='PT-DIST-001' and fully_omitted:
                    continue  # applicability removal, never substitute L0 or a fabricated distance
                applicable=True;status='evaluated' if not missing else 'partial_known_results'
                if rule_id=='PT-DIST-001':
                    inactive=evidence.get('held_object_distance_applicable') is False or evidence.get('normal_grip_distance_exempt') is True
                    if inactive:status='not_applicable';applicable=False
                    elif bounds_missing:level=None;missing=list(missing)+bounds_missing;status='insufficient_data'
                report['rules'].append(dict(rule_id=rule_id,category='PT',relation_id=rid,level=level,
                    complete=not missing,missing=list(missing),evidence=evidence,evaluation_status=status,applicable=applicable,
                    source_profile_sha256=self.source_hash,threshold_status='unchanged experimental proxy, not material qualification'))
        report['categories']['PT'].update(missing=['damage_sensor_unavailable']+
            ['missing_pt_relation:'+rid for rid in sorted(missing_relations)]+
            ['missing_payload:'+aid for aid in sorted(missing_payloads)],data_complete=False)
        report['categories']['RS'].update(missing=['authoritative_fault_status_unavailable'],data_complete=False)
        if not features['rs']['self_collision_evidence']['coverage_complete']:
            report['categories']['RS']['missing'].append('self_geometry_incomplete')
        if self.scope['human_inventory']['verified_absent']:
            report['categories']['HS'].update(level='L0',status='not_applicable',data_complete=True,missing=[])
            report['risk_levels']['HS']='L0'
            for r in report['rules']:
                if r['category']=='HS':r.update(level=None,evaluation_status='not_applicable',complete=True,missing=[])
        report['categories']['IR']=dict(level=None,status='disabled',data_complete=None,missing=[])
        report['risk_levels']['IR']=None
        report['rules']=[r for r in report['rules'] if r['category']!='IR']
        for rule in report['rules']:
            rule.update(profile_hash=self.profile_hash,scope_hash=self.scope['scope_sha256'])
        report=aggregate_known_results(report)
        report.update(evaluation_mode='offline_replay',task_id=self.contract['task_id'],profile_hash=self.profile_hash,
            source_profile_sha256=self.source_hash,scope_hash=self.scope['scope_sha256'])
        return report
