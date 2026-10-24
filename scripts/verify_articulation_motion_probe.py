"""Audit measured external-joint motion and offline safety evidence.

Diagnostic actuation is not robot task completion. No physics or mesh query is
performed here, and existing raw/features/labels are never rewritten.
"""
import argparse,hashlib,json,math
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from kitchen_safety.task_contract import digest
from kitchen_safety.task_articulation_probe import validate_command


def verify(episode,risk,minimum):
    def read(n):return json.loads((episode/n).read_text())
    contract=read('task_contract.json');scope=read('relation_scope.json');schema=read('raw_capture_schema.json');summary=read('raw_capture_summary.json')
    evaluation=json.loads((risk/'evaluation_summary.json').read_text());policy=read('policy_binding.json')['config']
    assert policy['version']=='robolaw-articulation-motion-probe-1' and policy['task_sha256']==digest(contract)
    assert minimum>0 and math.isfinite(minimum)
    assert evaluation['complete'] and evaluation['raw_sha256']==summary['raw_writer']['sha256']
    assert all(schema[k]==0 for k in ('risk_queries_online','features_online','grades_online'))
    h=hashlib.sha256();steps=0;commands=0;qs=[];sources=[];dof=None;unit=None;root=None;initial=None;latest=None;max_pose_change=0.
    moving={p for paths in scope['bindings']['manipulated_actor_paths'].values() for p in paths}
    for line in (episode/'raw_evidence.jsonl').open('rb'):
        h.update(line);row=json.loads(line)
        if row['kind']=='initial_state':initial=row['state']
        if row['kind']=='physics':
            assert row['physics_step']==steps
            entries=[v for v in row['commands'] if v.get('command_scope')=='diagnostic_external_articulation']
            assert len(entries)==1
            entry=entries[0];validate_command(entry['command'],contract)
            assert entry['before_physics_step']==steps and entry['action_index']==row['action_index'] and entry['success'] is True
            assert entry['diagnostic_only'] is True and entry['operation_execution_claimed'] is False
            if dof is None:dof=entry['dof_path'];root=entry['root_path'];unit=entry['unit']
            assert dof==entry['dof_path'] and root==entry['root_path'] and unit==entry['unit']
            measured=row['state']['articulations'][root];idx=entry['dof_index']
            assert measured['dof_paths'][idx]==dof
            assert abs(entry['submitted'][0][idx]-entry['command']['effort_si'])<1e-6
            assert abs(measured['actuation_forces'][idx]-entry['submitted'][0][idx])<1e-6
            assert len(entry['submitted'][0])==len(entry['prior'][0])==len(measured['dof_paths'])
            for j in range(len(measured['dof_paths'])):
                if j!=idx:assert abs(entry['submitted'][0][j]-entry['prior'][0][j])<1e-6
            if not qs:qs.append(initial['articulations'][root]['q'][idx])
            qs.append(measured['q'][idx]);commands+=1;steps+=1;latest=row['state']
            for actor in moving:
                a=initial['actor_states_world'][actor][:7];b=latest['actor_states_world'][actor][:7]
                max_pose_change=max(max_pose_change,max(abs(x-y) for x,y in zip(a,b)))
        if row['kind'] in ('observation','terminal_observation'):sources.append(row['source_physics_step'])
    assert h.hexdigest()==summary['raw_writer']['sha256'] and steps==summary['physics_steps']
    assert commands==summary['diagnostic_external_actuation_steps']==steps
    excursion=max(qs)-min(qs);assert excursion>=minimum,(excursion,minimum)
    assert max_pose_change>1e-6
    manipulated={r['relation_id'] for r in scope['relations'] if r['category']=='PT' and r.get('source_role')=='manipulated_link'}
    assert manipulated
    hs_manipulated={r['relation_id'] for r in scope['relations'] if r['category']=='HS' and r.get('source_role')=='manipulated_link'}
    first={};max_distance_change=0.;speed=0.;frames=0;missing_stopping=0;geometry_counts=[];rh=hashlib.sha256()
    hs_first={};hs_changes=0.;hs_count=0;hs_speed=0.
    for line in (risk/'safety_decisions.jsonl').open('rb'):
        rh.update(line);row=json.loads(line);assert row['source_physics_step']==sources[frames]
        assert row['report']['scope_hash']==scope['scope_sha256'] and row['evaluation_mode']=='offline_replay'
        for rid,g in row['geometry']['relations'].items():
            assert g['metric']=='gdist_authored_collision_surface_distance' and g['complete'] is True and math.isfinite(g['distance_m']) and g['distance_m']>=0
            if rid in manipulated:
                if rid not in first:first[rid]=g['distance_m']
                max_distance_change=max(max_distance_change,abs(first[rid]-g['distance_m']))
        assert row['geometry']['self_collision'] and all(p['complete'] is True for p in row['geometry']['self_collision'])
        for p in row['features']['pt']['protected_pairs']:
            if p['relation_id'] in manipulated and p.get('source_surface_speed_mps') is not None:speed=max(speed,p['source_surface_speed_mps'])
        for rule in row['report']['rules']:
            if rule['rule_id']=='PT-DIST-001' and rule.get('relation_id') in manipulated:
                assert rule['level'] is None and 'articulation stopping/control model unbound' in rule['missing']
                missing_stopping+=1
        observed_hs=set()
        for pair in row['features']['hs']['dynamic_distance']['pairs']:
            rid=pair['relation_id']
            if rid not in hs_manipulated:continue
            observed_hs.add(rid);g=pair['nearest_geometry']
            assert g['complete'] is True and pair['level'] is None and pair['complete'] is False
            assert pair['stop_model']=='unbound_articulation_stopping_model'
            assert pair['robot_surface_speed_mps'] is not None and pair['distance_m']==g['distance_m']
            if rid not in hs_first:hs_first[rid]=pair['distance_m']
            hs_changes=max(hs_changes,abs(hs_first[rid]-pair['distance_m']))
            hs_speed=max(hs_speed,pair['robot_surface_speed_mps']);hs_count+=1
        assert observed_hs==hs_manipulated
        if hs_manipulated:assert row['report']['categories']['HS']['data_complete'] is False
        assert row['report']['categories']['IR']['status']=='disabled'
        geometry_counts.append(len(row['geometry']['relations']));frames+=1
    assert frames==len(sources)==evaluation['frames'] and frames==steps//4+1
    assert max_distance_change>1e-6 and speed>0 and missing_stopping>0
    if hs_manipulated:assert hs_changes>1e-6 and hs_speed>0
    return dict(passed=True,task_id=contract['task_id'],physics_steps=steps,frames=frames,external_command_steps=commands,
        dof_path=dof,effort_unit=unit,joint_excursion_si=excursion,minimum_displacement_si=minimum,
        maximum_pose_component_change=max_pose_change,maximum_manipulated_distance_change_m=max_distance_change,
        maximum_manipulated_surface_speed_mps=speed,unbound_stopping_distance_rules=missing_stopping,
        hs_manipulated_relations=len(hs_manipulated),hs_unbound_samples=hs_count,hs_max_distance_change_m=hs_changes,
        hs_max_surface_speed_mps=hs_speed,geometry_relation_counts=geometry_counts,raw_sha256=h.hexdigest(),risk_sha256=rh.hexdigest(),
        online_risk_calls=0,full_robot_task_execution=False,diagnostic_external_actuation=True,
        scope='Actual articulated motion and complete offline mesh/rule evidence; not robot execution or stopping calibration')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--episode',type=Path,required=True);p.add_argument('--risk',type=Path,required=True);p.add_argument('--minimum-displacement',type=float,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();assert not a.output.exists()
    result=verify(a.episode,a.risk,a.minimum_displacement);a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
if __name__=='__main__':main()
