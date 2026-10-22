"""Per-relation experimental HS SSM; no command or simulator writes.

The ISO-inspired distance structure is separated from project severity bands
and an explicitly uncalibrated constant-deceleration candidate model.
"""
from copy import deepcopy
import math
import numpy as np
from kitchen_safety.hs_geometry import transform

VERSION='arena-hs-dynamic-ssm-1'


def validate_config(c):
    if c.get('version')!=VERSION or c.get('enabled') is not True:
        raise ValueError('Unsupported dynamic HS configuration')
    for key in ('reaction_s','warning_band_m','intrusion_m','robot_position_uncertainty_m',
                'human_position_uncertainty_m','robot_acceleration_bound_mps2',
                'robot_speed_uncertainty_mps','human_speed_uncertainty_mps','stationary_speed_tolerance_mps'):
        v=c.get(key)
        if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or v<0:
            raise ValueError('Invalid dynamic HS parameter: '+key)
    if c['reaction_s']<=0:raise ValueError('Reaction budget must be positive')
    for key in ('brake_deceleration_min_mps2',):
        if not isinstance(c.get(key),(int,float)) or isinstance(c[key],bool) or not math.isfinite(c[key]) or c[key]<=0:
            raise ValueError('Positive stopping model required: '+key)
    a=c.get('moving_human_acceleration_bound_mps2')
    if a is not None and (isinstance(a,bool) or not isinstance(a,(int,float)) or not math.isfinite(a) or a<0):
        raise ValueError('Invalid moving-human acceleration bound')
    if c.get('model_status')!='experimental_uncalibrated':
        raise ValueError('This version implements an experimental candidate, not a calibrated stop table')
    if c.get('missing_motion_policy')!='unknown':raise ValueError('Missing motion must remain unknown')


def relation(distance, robot_speed, human_speed, human_static, c, *, stop_model='robot_coupled'):
    """Conservative speed-envelope form; input speeds cover complete surfaces."""
    result=dict(distance_m=distance,robot_surface_speed_mps=robot_speed,
        human_surface_speed_mps=human_speed,human_static=human_static,stop_model=stop_model,level=None,
        complete=False,missing=[],model_calibrated=False,model_status=c['model_status'])
    for key,value in [('distance',distance),('robot_speed',robot_speed),('human_speed',human_speed)]:
        if value is None or not math.isfinite(value) or value<0:result['missing'].append(key)
    ah=0. if human_static else c.get('moving_human_acceleration_bound_mps2')
    if ah is None:result['missing'].append('moving_human_acceleration_bound')
    if stop_model not in ('robot_coupled','supported_stationary'):result['missing'].append('payload_stopping_model')
    if result['missing']:return result
    tr=c['reaction_s']
    ar=0. if stop_model=='supported_stationary' else c['robot_acceleration_bound_mps2']
    vr=robot_speed+ar*tr+c['robot_speed_uncertainty_mps']
    if stop_model=='supported_stationary':
        ts=brake=0.
    else:
        ts=vr/c['brake_deceleration_min_mps2']
        brake=vr*vr/(2*c['brake_deceleration_min_mps2'])
    vh=human_speed+ah*(tr+ts)+c['human_speed_uncertainty_mps']
    human_travel=vh*(tr+ts);reaction_travel=vr*tr
    error=c['intrusion_m']+c['robot_position_uncertainty_m']+c['human_position_uncertainty_m']
    protective=human_travel+reaction_travel+brake+error
    margin=distance-protective;warning=c['warning_band_m']
    # Boundary equality is conservative despite floating point round-off.
    level='L2' if margin<=1e-12 else 'L1' if margin<=warning+1e-12 else 'L0'
    result.update(robot_speed_bound_mps=vr,human_speed_bound_mps=vh,
        reaction_s=tr,stop_s=ts,brake_travel_m=brake,
        human_travel_m=human_travel,reaction_travel_m=reaction_travel,
        intrusion_m=c['intrusion_m'],robot_position_uncertainty_m=c['robot_position_uncertainty_m'],
        human_position_uncertainty_m=c['human_position_uncertainty_m'],
        protective_distance_m=protective,warning_outer_m=protective+warning,
        warning_band_m=warning,margin_m=margin,level=level,complete=True,
        robot_acceleration_bound_mps2=ar,human_acceleration_bound_mps2=ah,
        brake_deceleration_min_mps2=c['brake_deceleration_min_mps2'])
    return result


def features(geometry,row,measurement,grasp,c):
    """Project exact saved COM telemetry and surface vertices; no pose differencing."""
    by_path={e['path']:e for e in geometry.entries}
    body_names=geometry.metadata.get('motion_body_names',[])
    if not body_names:body_names=row.get('body_names',[])
    body_index={name:i for i,name in enumerate(body_names)}
    cache={}
    def speed(paths):
        key=tuple(paths)
        if key in cache:return cache[key]
        maximum=0.;all_static=True
        try:
            for path in paths:
                e=by_path[path];actor=e['actor']
                if actor is None:continue
                all_static=False
                if e['group']=='robot':
                    j=body_index[actor]
                    pose=row['body_pose'][j]
                    com=np.asarray(row['body_com_position_w'][j],dtype=float)
                    linear=np.asarray(row['body_com_linear_velocity_w'][j],dtype=float)
                    angular=np.asarray(row['body_com_angular_velocity_w'][j],dtype=float)
                else:
                    full=geometry.root+'/'+actor
                    motion=row['rigid_actor_motion_world'][full]
                    pose=row['rigid_actor_states_world'][full]
                    com=np.asarray(motion['com_position_world_m'],dtype=float)
                    linear=np.asarray(motion['com_linear_velocity_mps'],dtype=float)
                    angular=np.asarray(motion['angular_velocity_radps'],dtype=float)
                t=transform(pose)
                if any(v.shape!=(3,) or not np.isfinite(v).all() for v in (com,linear,angular)):
                    raise ValueError('Missing/nonfinite COM motion')
                points=e['model'].vertices@t[:3,:3].T+t[:3,3]
                velocity=linear+np.cross(angular,points-com)
                maximum=max(maximum,float(np.max(np.linalg.norm(velocity,axis=1))))
            value=(maximum,all_static,None)
        except (KeyError,TypeError,ValueError,IndexError) as exc:
            value=(None,False,type(exc).__name__+': '+str(exc))
        cache[key]=value;return value
    rows=[]
    for p in measurement.get('dynamic_relations',[]):
        vr,source_static,error=speed(p['source_colliders'])
        vh,human_static,herror=speed([p['human_collider']])
        model='robot_coupled'
        if p['source_group']=='object' and not source_static and grasp.get('held') is not True:
            model=('supported_stationary' if grasp.get('supported') is True and vr is not None
                   and vr<=c['stationary_speed_tolerance_mps'] else 'uncontrolled_payload')
        if source_static:model='supported_stationary'
        result=relation(p.get('distance_m'),vr,vh,human_static,c,stop_model=model)
        result.update({k:p[k] for k in ('relation_id','source_group','source_actor','source_colliders','human_collider')})
        result['nearest_geometry']=p.get('nearest_geometry')
        if error or herror:
            result.update(complete=False,level=None)
            result['missing']+=['robot_motion:'+error] if error else []
            result['missing']+=['human_motion:'+herror] if herror else []
        if p.get('error'):
            result.update(complete=False,level=None);result['missing'].append('geometry:'+p['error'])
        result['motion_source']='Saved COM linear/angular velocity and complete triangle vertices; no finite-difference fallback'
        rows.append(result)
    known=[p for p in rows if p['level'] is not None]
    dominant=min(known,key=lambda p:p['margin_m']) if known else None
    return dict(version=VERSION,enabled=True,pairs=rows,expected_pair_count=len(rows),
        known_pair_count=len(known),coverage_complete=bool(rows) and all(p['complete'] for p in rows),
        dominant_pair=deepcopy(dominant),level=max((p['level'] for p in known),default=None),
        model_status=c['model_status'],model_calibrated=False,warning_band_m=c['warning_band_m'],
        missing=sorted({m for p in rows for m in p['missing']}) or ([] if rows else ['dynamic_pair_geometry_unavailable']),
        aggregation='Maximum known relation level; dominant relation minimizes D-S; completeness separate',
        assumptions=deepcopy(c['assumptions']))


class DynamicHSEvaluator:
    """Retain the reference PT/RS/contact rules, replace only HS distance/TTC audit."""
    def __init__(self,base,profile):
        self.base=base;self.profile=deepcopy(profile);self.config=profile['hs_dynamic']
        validate_config(self.config)
    def __getattr__(self,name):return getattr(self.base,name)
    def evaluate(self,snapshot):
        report=self.base.evaluate(snapshot)
        hs=snapshot.get('features',snapshot).get('hs',{})
        dynamic=hs.get('dynamic_distance',{})
        absent=self.profile.get('runtime_binding',{}).get('human_inventory',{}).get('verified_absent') is True
        exempt=hs.get('dynamic_scope')=='fixed' and hs.get('fixed_distance_exempt') is True
        valid=dynamic.get('version')==VERSION and isinstance(dynamic.get('pairs'),list)
        expected=self.profile.get('runtime_binding',{}).get('hs_geometry',{}).get('dynamic_relation_ids',[])
        rows=[];missing=[];seen=set()
        if valid:
            for p in dynamic['pairs']:
                rid=p.get('relation_id')
                if rid not in expected or rid in seen:
                    missing.append('unexpected_or_duplicate_relation:'+str(rid));continue
                seen.add(rid)
                evaluated=relation(p.get('distance_m'),p.get('robot_surface_speed_mps'),
                    p.get('human_surface_speed_mps'),p.get('human_static') is True,
                    self.config,stop_model=p.get('stop_model','uncontrolled_payload'))
                if p.get('complete') is not True:
                    evaluated.update(level=None,complete=False)
                    evaluated['missing']+=p.get('missing',[]) or ['incomplete_relation_evidence']
                evaluated.update({k:p.get(k) for k in ('relation_id','source_group','source_actor','human_collider','nearest_geometry')})
                rows.append(evaluated);missing+=evaluated['missing']
        else:missing.append('hs.dynamic_distance')
        missing+=['missing_relation:'+rid for rid in expected if rid not in seen]
        if not expected:missing.append('dynamic_relation_inventory_unavailable')
        known=[p for p in rows if p['level'] is not None]
        dominant=min(known,key=lambda p:p['margin_m']) if known else {}
        level=max((p['level'] for p in known),default=None)
        complete=not missing and bool(expected) and all(p['complete'] for p in rows)
        if absent or exempt:level=None;complete=True;missing=[]
        missing=sorted(set(missing))
        derived=report['derived']
        legacy={k:derived.get(k) for k in ('D_m','S_nom_m','S_rob_m','M_nom_m','M_rob_m','S_protective_m')}
        active={} if absent or exempt else dominant
        d=active.get('distance_m');s=active.get('protective_distance_m');m=active.get('margin_m')
        which='object' if active.get('source_group')=='object' else 'robot'
        derived.update(D_m=d,S_nom_m=None,S_rob_m=s,M_nom_m=None,M_rob_m=m,
            S_protective_m=s,warning_buffer_m=self.config['warning_band_m'],
            used_distances_m={which:d},hs_distance_rule_version=VERSION,
            hs_dynamic_dominant_pair=deepcopy(active),legacy_fixed_distance_audit=legacy,
            legacy_fixed_distance_used_for_grade=False,hs_dynamic_model_calibrated=False)
        for rule in report['rules']:
            if rule['rule_id']=='HS-DIST-001':
                rule.update(level=level,complete=bool(complete),missing=list(missing),
                    evidence=dict(D_m=d,S_protective_m=s,warning_buffer_m=self.config['warning_band_m'],
                        M_rob_m=m,used_distances_m={which:d},version=VERSION,
                        dominant_pair=deepcopy(active),known_pair_count=len(known),
                        expected_pair_count=len(expected),
                        model_status=self.config['model_status'],model_calibrated=False,
                        distance_exempt=exempt,applicable=not absent and not exempt),
                    mapping_origin='arena_dynamic_hs_ssm',change_record='docs/hs_dynamic_distance.md',
                    note='Per-surface relation D-S with an experimental uncalibrated stopping envelope; severity bands are project-defined',
                    evaluation_status='not_applicable' if absent or exempt else 'evaluated' if complete else 'partial_known_results',
                    normative_clause_verified=False)
            elif rule['rule_id']=='HS-TTC-001':
                rule.update(level=None,complete=True,missing=[],evaluation_status='diagnostic_only',
                    note='Diagnostic only; never contributes a synthetic L0 to HS grading')
        rr=[r for r in report['rules'] if r['category']=='HS']
        known=max((r['level'] for r in rr if r.get('level') is not None),default=None)
        cat=report['categories']['HS']
        cat.update(level=known,data_complete=all(r['complete'] for r in rr),
            missing=sorted({x for r in rr for x in r['missing']}))
        report['risk_levels']['HS']=known
        report['warnings'].append('HS_DYNAMIC_EXPERIMENTAL_UNCALIBRATED_STOPPING_MODEL')
        return report
