"""Compose frozen HS/PT/RS rules over task-bound relations, without new bands.

This adapter supplies source identity/applicability; numerical decisions remain
in the checked-in reference evaluator and dynamic HS implementation.
"""
from copy import deepcopy
from kitchen_safety.task_contract import digest
from kitchen_safety.risk_profile_loader import create_evaluator
from kitchen_safety.risk_observer import aggregate_known_results
from safety_risk.pt_hs_inspired import evaluate_pairs


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
                    authorization='task semantic destination/support relation; user workflow policy',
                    selection_report_sha256=scope['scope_sha256'],relations=[dict(actor=actor,object_id=obj)])
            self.pair_profiles[rid]=dict(pt_hs_inspired=settings)
        self.profile_hash=digest(dict(source_profile_sha256=self.source_hash,task_sha256=digest(contract),
            scope_sha256=scope['scope_sha256'],bound_profile=self.profile,pair_profiles=self.pair_profiles))

    def evaluate(self,features):
        # Reference report supplies HS contact, dynamic HS, unchanged RS bands,
        # and audit-only support/damage. Drop is separately bound per payload.
        report=self.base.evaluate(dict(features=dict(features,pt=dict(support_margin_gt_m=None,damage_flag_gt=None))))
        report['rules']=[r for r in report['rules'] if r['rule_id']!='PT-DROP-001']
        report['distance_omissions']={}
        for aid,payload in features['pt']['payloads'].items():
            partial=self.base.evaluate(dict(features=dict(pt=payload)))
            for rule in partial['rules']:
                if rule['rule_id']=='PT-DROP-001':
                    rule.update(relation_id='payload:'+aid,payload_id=aid);report['rules'].append(rule)
        for raw in features['pt']['protected_pairs']:
            rid=raw['relation_id'];r=self.relations[rid];profile=self.pair_profiles[rid]
            settings=profile['pt_hs_inspired'];spec=settings['pairs'][0]
            data=deepcopy(raw);data.update(actor=spec['actor'],object_id=spec['object_id'])
            omission=raw.get('distance_omission')
            fully_omitted=bool(omission and omission['fully_omitted'])
            if omission:report['distance_omissions'][rid]=deepcopy(omission)
            source='link_target_gdist_surface_distance_gt' if r.get('evidence_only') else 'pt_environment_gdist_surface_distance_gt'
            if r['query_distance'] and not fully_omitted:
                data.update(distance_metric=settings['distance_metric'],distance_source=source,
                    link_distances_m={r['source']:raw.get('distance_m')},
                    distance_geometry_evidence=dict(metric=settings['distance_metric'],distance_source=source,
                        coverage_complete=raw['distance_coverage_complete'],nearest=raw['distance_geometry_evidence']))
            bounds_missing=[]
            if r['query_distance'] and not fully_omitted:
                params=settings['objects'][spec['object_id']]['distance']
                for field,bound in [('source_surface_speed_mps','robot_approach_bound_mps'),('target_surface_speed_mps','object_approach_bound_mps')]:
                    speed=raw.get(field)
                    if speed is None or speed>params[bound]+1e-12:bounds_missing.append('PT distance assumption not satisfied: '+field)
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
        report['categories']['PT'].update(missing=['damage_sensor_unavailable'],data_complete=False)
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
