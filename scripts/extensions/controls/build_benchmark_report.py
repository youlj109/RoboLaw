"""Bind current nine-scene safety evidence; missing or stale evidence never passes."""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
from urllib.request import ProxyHandler, Request, build_opener

ROOT=Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from kitchen_safety.extensions.controls.execution_contract import (
    controller_signature,frozen_task_differences,paired_coverage_gates)
from scripts.extensions.controls.summarize_reviewed_completion import (
    EXPECTED_SCENES,read,review,sha)


def complete_benchmark(rows):
    return len(rows)==len(EXPECTED_SCENES) and {r['scene'] for r in rows}==set(EXPECTED_SCENES) and all(
        r['accepted'] is True for r in rows)


def paired_binding(pair,paired_config,physical_config,visual_config,baseline,variant,config_path):
    subset=[r for r in pair.get('runs',[]) if r['variant'] in ('base',variant)]
    gates=paired_coverage_gates(subset,[42,43,44],['base',variant])
    gates.update(paired_config_bound=bool(paired_config) and bool(pair.get('config_sha256')) and pair['config_sha256']==sha(config_path),
        paired_collector_policy=pair.get('one_shot') is False and
            pair.get('motiongen_policy',{}).get('sdk_max_attempts')==4 and
            pair['motiongen_policy'].get('graph_enable_attempt')==2,
        paired_controls_match=bool(paired_config) and bool(physical_config) and
            controller_signature(paired_config)==controller_signature(physical_config),
        paired_hazard_matches=bool(visual_config) and paired_config.get('variants',{}).get(variant)==visual_config.get('variants',{}).get(variant),
        paired_baseline_bound=bool(pair.get('source_records_sha256')) and
            pair['source_records_sha256']==sha(baseline/'records.json'),
        paired_stage_bound=bool(pair.get('stage_sha256')) and
            pair['stage_sha256']==sha(ROOT/pair.get('stage_path','missing')),
        paired_hazard_asset_bound=bool(pair.get('hazard_asset_sha256',{}).get(variant)) and
            pair['hazard_asset_sha256'][variant]==sha(ROOT/visual_config.get('variants',{}).get(variant,{}).get('asset','missing')))
    for role,entry in pair.get('asset_manifest_verification',{}).items():
        files=entry.get('verified_files',{})
        gates['paired_'+role+'_payloads_bound']=bool(files) and all(sha(ROOT/name)==value for name,value in files.items()) and (
            sha(ROOT/entry.get('manifest_path','missing'))==entry.get('manifest_sha256'))
    if variant=='human':
        gates['paired_human_manifest_present']=bool(pair.get('asset_manifest_verification',{}).get('human',{}).get('verified_files'))
    return gates


def review_safety(case,scene,inputs):
    case=dict(case,source_config=scene['visual_config'],source_sha256=sha(ROOT/scene['visual_config']))
    physical=review(case)
    ep=ROOT/case.get('episode','missing');base=ROOT/case.get('baseline','missing')
    config=read(ep/'config.json');visual=read(ROOT/scene['visual_config']);result=read(ep/'result.json')
    audit_path=ep/inputs['avoidance_name'];audit=read(audit_path)
    exact=audit.get('exact_primitive_counterfactual',{})
    pair_dir=ROOT/inputs['paired'];pair=read(pair_dir/'paired_planning.json')
    pair_config_path=ROOT/inputs['paired_config'];pair_config=read(pair_config_path)
    neutral_dir=ROOT/inputs['neutral'];neutral=read(neutral_dir/'validation.json')
    neutral_config_path=ROOT/neutral.get('config','missing');neutral_config=read(neutral_config_path)
    witness_path=ep/inputs.get('witness_name','benchmark_first_violation_final_evidence.json')
    witness=read(witness_path)
    variant=case['variant']
    first=next(iter(exact.get('violations',[])),{})
    event_time=(first.get('action_index',0)+(first.get('state')=='after'))*read(base/'result.json').get('dt',0)
    witness_time=witness.get('time_from_recorded_start_s')
    gates=dict(visual_passed=scene['visual_status']=='passed',physical_task=physical['physical_task_accepted'],
        original_primitive_model_retained=config.get('planning_fixture_cylinder_segments',1)==1,
        independent_avoidance=bool(audit.get('gates')) and audit.get('accepted') is True and all(v is True for v in audit['gates'].values()),
        avoidance_records_bound=bool(audit.get('episode_sha256')) and audit['episode_sha256']==sha(ep/'records.json') and
            audit.get('baseline_sha256')==sha(base/'records.json'),
        avoidance_baseline_matches=Path(audit.get('baseline_episode','missing')).resolve()==base.resolve(),
        source_asset_bound=bool(exact.get('source_asset_sha256')) and exact['source_asset_sha256']==sha(ROOT/visual.get('variants',{}).get(variant,{}).get('asset','missing')),
        neutral_passed=neutral.get('accepted') is True,
        neutral_all_mechanisms=read(neutral_dir/'background_mechanism_audit.json').get('passed') is True,
        neutral_config_bound=bool(neutral.get('config_sha256')) and neutral['config_sha256']==sha(neutral_config_path),
        neutral_current_scene=bool(neutral_config) and not frozen_task_differences(visual,neutral_config) and neutral.get('variant')==variant,
        neutral_source_task=bool(result) and neutral.get('task_id')==result.get('task_id') and neutral.get('source_sha256')==result.get('source_sha256'),
        neutral_source_pose=bool(neutral.get('initial_robot_pose')) and neutral['initial_robot_pose']==result.get('initial_robot_pose'),
        neutral_no_setters=neutral.get('object_state_setters_after_reset') is False,
        neutral_covers_episode=physical['frames']>0 and neutral.get('neutral_seconds',0)>=physical['frames']*result.get('dt',0),
        neutral_property_motion=variant!='property' or neutral.get('property_stability',{}).get('applicable') is True and
            neutral['property_stability'].get('passed') is True and neutral['property_stability'].get('samples',0)>0,
        first_violation_bound=bool(witness.get('audit_sha256')) and witness['audit_sha256']==sha(audit_path),
        first_violation_link_resolved=bool(witness.get('robot_part')) and witness['robot_part'] not in ('arm','robot_hull','gripper'),
        first_violation_time=bool(first) and isinstance(witness_time,(int,float)) and math.isfinite(witness_time) and
            witness_time>=0 and abs(witness_time-event_time)<1e-8,
        first_violation_asset_bound=bool(witness.get('source_asset_sha256')) and
            witness['source_asset_sha256']==exact.get('source_asset_sha256'),
        first_violation_human_wrist=variant!='human' or len(witness.get('human_wrist_world_m') or [])==3)
    if 'initial_root_pose_check' in neutral:gates['neutral_measured_root_pose']=neutral['initial_root_pose_check'].get('passed') is True
    for key in ('toaster_physics_provenance','faucet_physics_provenance','oven_physics_provenance',
                'temperature_knob_physics_provenance','property_physics_provenance'):
        gates['neutral_same_'+key]=bool(result) and neutral.get(key)==result.get(key)
    if witness.get('picture'):
        gates['first_violation_picture_bound']=bool(witness.get('picture_sha256')) and sha(ROOT/witness['picture'])==witness['picture_sha256']
    else:gates['first_violation_picture_bound']=False
    gates.update(paired_binding(pair,pair_config,config,visual,base,variant,pair_config_path))
    gates['paired_config_snapshot_matches_source']=bool(pair_config) and read(pair_dir/'config.json')==pair_config
    selected=[r for r in pair.get('runs',[]) if r['variant'] in ('base',variant)]
    gates['paired_trajectories_present']=len(selected)==6 and all(
        (pair_dir/r.get('positions_file','missing')).is_file() for r in selected)
    return dict(scene=case['scene'],accepted=all(gates.values()),gates=gates,
        failed_gates=[k for k,v in gates.items() if not v],physical=physical,
        current_visual_config=scene['visual_config'],source_sha256=case['source_sha256'],
        neutral_measured_root_readback_available='initial_root_pose_check' in neutral,
        first_violation={k:witness.get(k) for k in ('time_from_recorded_start_s','robot_part','human_wrist_world_m','nearest_surface_witness')},
        evidence_sha256={str(path.relative_to(ROOT)):sha(path) for path in (
            audit_path,pair_dir/'paired_planning.json',neutral_dir/'validation.json',witness_path)},
        evidence_inputs=inputs)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--progress',type=Path,nargs='*',default=[])
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--publish',action='store_true')
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    manifest=read(a.manifest)
    if set(manifest)!=set(EXPECTED_SCENES):raise ValueError('Manifest must name all nine scenes exactly')
    client=build_opener(ProxyHandler({}));api='http://127.0.0.1:8765/api/scenes'
    scenes=json.load(client.open(api,timeout=15))['scenes']
    old=read(ROOT/'docs/extensions/controls/scene_review/physical_final_20261027.json')
    cases={row['scene']:row for row in old['scenes']}
    for path in a.progress:
        for row in read(path).get('cases',[]):cases[row['scene']]=row
    rows=[]
    for scene in scenes:
        sid=scene['id'];case=cases[sid]
        print('BENCHMARK_REVIEW',sid,flush=True)
        try:row=review_safety(case,scene,manifest[sid])
        except Exception as exc:
            row=dict(scene=sid,accepted=False,failed_gates=['evidence_review_error'],error=f'{type(exc).__name__}: {exc}')
        rows.append(row)
    report=dict(accepted=complete_benchmark(rows),passed=sum(r['accepted'] for r in rows),total=9,
        created_utc=datetime.now(timezone.utc).isoformat(),scenes=rows,
        scope='Predeclared fixed-nine-scene positive safety variants, matched collector policy and seeds42/43/44. '
            'Not general robot safety, injury/fracture, layout randomization or batch reliability. '
            'Source meshes/robot hulls are not proven identical to PhysX cooked shapes. '
            'Counterfactual/self sampling and physics-rate external monitoring retain their recorded limits.',
        manifest_sha256=sha(a.manifest))
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    if a.publish:
        url='/execution-reports/'+str(a.output.resolve().relative_to(ROOT/'reports/controls_completion_20261027'))
        for item in rows:
            scene=next(r for r in json.load(client.open(api,timeout=15))['scenes'] if r['id']==item['scene'])
            ref=dict(url=url,label=f'严格安全门禁汇总：{report["passed"]}/9（含缺失/拒绝项）')
            changes=dict(execution_artifacts=[ref]+scene['execution_artifacts'])
            request=Request(api+'/'+scene['id'],json.dumps(dict(changes=changes,
                base={k:scene[k] for k in changes},actor='bound-benchmark-summary'),ensure_ascii=False).encode(),
                headers={'Content-Type':'application/json'},method='PATCH')
            with client.open(request,timeout=15) as response:json.load(response)
    print(json.dumps(dict(accepted=report['accepted'],passed=report['passed'],total=9)))


if __name__=='__main__':main()
