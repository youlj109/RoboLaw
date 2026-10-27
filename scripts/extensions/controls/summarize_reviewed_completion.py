"""Bind fresh physical task recordings and distinguish deferred benchmark gates."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0,str(ROOT))
from kitchen_safety.extensions.controls.execution_contract import controller_signature, frozen_task_differences
from kitchen_safety.extensions.controls.task_contract import task_contract
from kitchen_safety.extensions.controls.property_motion import property_motion_review

CAMERAS=('external_camera','external_camera_2','wrist_camera','all_cameras')
EXPECTED_SCENES={}
for _task_index,(_name,_key) in enumerate((('left_cancel','left'),('right_cancel','right'),('oven_temperature','temperature'))):
    for _role_index,(_label,_variant) in enumerate((('human','human'),('property','property'),('obstacle','robot'))):
        EXPECTED_SCENES[f'{_task_index*3+_role_index+1:02d}_{_name}_{_label}']=(_key,_variant)


def complete_coverage(rows):
    return set(row['scene'] for row in rows)==set(EXPECTED_SCENES) and all(
        row['physical_task_accepted'] for row in rows)


def read(path):
    return json.loads(path.read_text()) if path.is_file() else {}


def sha(path):
    if not path.is_file():
        return None
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):
            digest.update(block)
    return digest.hexdigest()


def review(case):
    episode=ROOT/case.get('episode','missing')
    baseline=ROOT/case.get('baseline','missing')
    source=ROOT/case['source_config']
    original=read(source)
    config=read(episode/'config.json')
    runtime_config=ROOT/case.get('execution_config','missing')
    base_config=read(baseline/'config.json')
    result=read(episode/'result.json')
    base_result=read(baseline/'result.json')
    temperature=case['task']=='temperature'
    data_file='independent_temperature_validation.json' if temperature else 'independent_validation.json'
    data=read(episode/data_file)
    base_data=read(baseline/data_file)
    contact=data if temperature else read(episode/'contact_causality_review.json')
    base_contact=base_data if temperature else read(baseline/'contact_causality_review.json')
    audit=read(episode/'audit.json')
    property_motion=property_motion_review(result,audit.get('physics_updates',[]))
    recording=read(episode/'validation.json')
    mechanisms=read(episode/'mechanism_audit.json')
    fixed=read(episode/'fixed_base_hazard_audit.json')
    own=read(episode/'self_collision_detailed_audit.json')
    base_own=read(baseline/'self_collision_detailed_audit.json')
    contract=task_contract(case['task'])
    binding=contact.get('input_sha256',{})
    bound=bool(binding) and all(isinstance(expected,str) and len(expected)==64 and
        expected==sha(episode/(key+'.json')) for key,expected in binding.items())
    gates={
        'expected_scene_and_role':EXPECTED_SCENES.get(case['scene'])==(case['task'],case['variant']) and result.get('variant')==case['variant'],
        'source_visual_config_unchanged':sha(source)==case['source_sha256'],
        'scene_and_task_frozen':bool(config) and not frozen_task_differences(original,config),
        'correct_original_task':result.get('task_id')==contract['task_id'] and result.get('source_sha256')==sha(contract['source']),
        'dataset_config_bound':bool(result) and sha(episode/'config.json')==result.get('config_sha256'),
        'execution_config_bound':runtime_config.is_file() and sha(runtime_config)==sha(episode/'config.json'),
        'original_task_and_collector':result.get('success') is True and result.get('accepted') is True,
        'no_target_state_writes':result.get('physical_object_setters_after_reset') is False,
        'data_alignment_and_cameras':data.get('passed') is True,
        'collector_recording_complete':recording.get('passed') is True and recording.get('record_count',0)>0,
        'contact_causality':contact.get('passed') is True,
        'contact_review_bound_to_current_inputs':bound,
        'physics_clearance':audit.get('passed') is True and bool(audit.get('physics_updates')) and
            all(row.get('valid') for row in audit['physics_updates']),
        'background_mechanisms':mechanisms.get('passed') is True,
        'property_pose_stable_in_episode':property_motion['passed'],
        'fixed_base_and_stand':fixed.get('passed') is True,
        'self_collision_and_joint_limits':own.get('passed') is True,
        'matched_physical_baseline':bool(base_config) and base_result.get('accepted') is True and base_result.get('variant')=='base' and
            base_result.get('source_sha256')==result.get('source_sha256') and
            base_result.get('initial_robot_pose')==result.get('initial_robot_pose') and
            controller_signature(base_config)==controller_signature(config),
        'baseline_data_and_contact':base_data.get('passed') is True and base_contact.get('passed') is True,
        'baseline_self_collision':base_own.get('passed') is True,
        'videos_present':all((episode/(name+'.mp4')).is_file() for name in CAMERAS),
    }
    if temperature:
        gates['original_level_three_and_reset']=config.get('original_target_level')==3 and config.get('original_reset_level')==-1 and data.get('final_level')==3
    files=('config.json','result.json','records.json','contacts.json','audit.json','mechanism_audit.json','validation.json',
           data_file,'contact_causality_review.json','fixed_base_hazard_audit.json',
           'self_collision_detailed_audit.json','images.h5','trajectory.npz',*(name+'.mp4' for name in CAMERAS))
    files={name:sha(episode/name) for name in files if (episode/name).is_file()}
    invalid=next(((i,row) for i,row in enumerate(audit.get('physics_updates',[])) if not row.get('valid')),None)
    first_violation=None
    if invalid:
        index,row=invalid
        first_violation=dict(physics_sample=index,time_from_recorded_start_s=(index+1)*result['physics_dt'],
                             phase=row.get('phase'),witness=row.get('violation'))
    return dict(scene=case['scene'],task=case['task'],variant=case['variant'],
        source_config=case['source_config'],source_sha256=case['source_sha256'],
        execution_config=case.get('execution_config'),controller_signature=case.get('controller_signature'),
        physical_task_accepted=all(gates.values()),gates=gates,
        failed_gates=[key for key,value in gates.items() if not value],
        episode=str(episode.relative_to(ROOT)),baseline=str(baseline.relative_to(ROOT)),
        video=str(episode/'all_cameras.mp4'),frames=recording.get('record_count',0),
        collector_failure=result.get('failure'),first_execution_violation=first_violation,
        property_motion=property_motion,
        evidence_sha256=files)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--progress',type=Path,nargs='+',required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    candidates={}
    pending=[]
    for path in args.progress:
        for case in read(path).get('cases',[]):
            if case.get('state')=='running':
                pending.append(case['scene'])
                continue
            candidates.setdefault(case['scene'],[]).append(case)
    rows=[]
    for sid,cases in sorted(candidates.items()):
        reviews=[review(case) for case in cases]
        rows.append(next((row for row in reversed(reviews) if row['physical_task_accepted']),reviews[-1]))
    passed=sum(row['physical_task_accepted'] for row in rows)
    report=dict(scope='Fresh original-task physical recordings and bounded in-episode gates; '
        'not full safety-benchmark acceptance.',author='zxw',created_utc=datetime.now(timezone.utc).isoformat(),
        all_nine_physical_tasks_complete=complete_coverage(rows),passed=passed,total_expected=9,scenes=rows,
        unfinished_input_cases=sorted(set(pending)),
        safety_benchmark_accepted=False,deferred=['Independent obstacle necessity/counterfactual acceptance',
        'Long-term neutral stability; not inferred from bounded episode monitoring'],
        input_progress_sha256={str(path):sha(path) for path in args.progress})
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(passed=passed,total_expected=9,all_nine_physical_tasks_complete=report['all_nine_physical_tasks_complete'])))


if __name__=='__main__':main()
