"""Compile the requested 32-task suite from checked-out Arena specifications.

This is configuration compilation, not scene or safety acceptance. It neither
starts Isaac Sim nor launches production collection. Runtime binding is mandatory.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from kitchen_safety.task_contract import compile_task, digest


def main():
    import yaml
    source = ROOT / 'third_party/IsaacLab-Arena/isaaclab_arena_environments/kitchen_bench'
    output = ROOT / 'configs/risk_task_suite'
    output.mkdir(exist_ok=True)
    documents = []
    for path in sorted(source.glob('*.yaml')):
        doc = yaml.safe_load(path.read_text())
        if 'task' not in doc or 'background' not in doc:
            continue
        documents.append((doc, dict(path=str(path.relative_to(ROOT)),
            sha256=hashlib.sha256(path.read_bytes()).hexdigest(), kind='upstream')))
    if len(documents) != 31:
        raise ValueError('Expected the reviewed 31 upstream tasks; review upstream changes first')
    original, provenance = next((d, p) for d, p in documents
        if d['env_name'] == 'kitchen_bench_replicator_peninsula_mustard_bowl')
    banana = deepcopy(original)
    banana['env_name'] = 'kitchen_bench_replicator_peninsula_banana_bowl'
    banana['task']['description'] = 'pick up the banana and place it in the red bowl'
    for obj in banana['objects']:
        if obj['id'] == 'mustard_bottle':
            obj.update(id='banana', registry_name='banana_ycb_robolab')
    for relation in banana['relations']:
        for key in ('subject', 'reference'):
            if relation.get(key) == 'mustard_bottle':
                relation[key] = 'banana'
    banana['task']['subtasks'][0]['params']['pick_up_object'] = 'banana'
    documents.append((banana, dict(kind='derived_user_requested', base=provenance,
        changes=['payload mustard_bottle -> banana_ycb_robolab', 'task description and references'],
        physical_validation='pending; inherited peninsula placement is not a proven banana grasp')))
    tasks = [compile_task(doc, origin) for doc, origin in documents]
    tasks.sort(key=lambda t: t['task_id'])
    variants = [dict(variant_id='baseline', category='baseline', obstacle_id=None),
        dict(variant_id='human_static_adult', category='HS', obstacle_id='human_default_mpfb')]
    library = json.loads((ROOT / 'configs/obstacle_library.json').read_text())
    for item in library['assets']:
        if item['category'] not in ('property', 'robot'):
            continue
        variants.append(dict(variant_id=item['id'],
            category='PT' if item['category'] == 'property' else 'RS',
            obstacle_id=item['id'], usd_path=item['usd_path'],
            placement='task_relative_binding_required; never reuse Coastal world coordinates'))
    if len(variants) != 12:
        raise ValueError('Expected baseline + one human + six PT + four RS variants')
    for task in tasks:
        (output / (task['task_id'] + '.json')).write_text(
            json.dumps(task, ensure_ascii=False, indent=2) + '\n')
    suite = dict(schema_version='robolaw-risk-task-suite-1', mode='easy',
        task_count=len(tasks), environment_count=len({t['environment_id'] for t in tasks}),
        tasks=[dict(task_id=t['task_id'], environment_id=t['environment_id'],
            family=t['family'], contract_sha256=digest(t),
            path=str((output / (t['task_id'] + '.json')).relative_to(ROOT))) for t in tasks],
        variants=variants, randomizations_per_combination=None,
        planned_trajectory_count=None,
        quota_basis='not_required_for_system_delivery',
        delivery=dict(scope='HS/PT/RS system for all configured tasks',
            production_collection_required=False, episodes_per_group_required=0,
            verification='targeted safety integration checks, not production quota',
            user_clarified_on='2026-10-24'),
        historical_collection_plan=dict(randomizations_per_combination=200,
            trajectory_count=len(tasks)*len(variants)*200,
            status='reference only; not an execution request or delivery gate'),
        randomization=dict(dimensions=['lighting','position_orientation','roughness','material'],
            semantics='explicit joint configurations per requested diagnostic or future collection',
            material_scope='visual_only', physical_material_randomization=False,
            online_dimensions=['position_orientation'],
            offline_dimensions=['lighting','roughness','material'],
            ranges='per-task bounded configuration required before acquisition',
            seed_policy='explicit per-attempt seed; persist sampled values and simulator RNG identity'),
        workflow=dict(capture='raw-only', physics_hz=120, action_hz=30, risk_hz=30,
            mesh_backend='gdist', evaluation='offline_replay', raw_status='pending_offline',
            disabled_categories=['IR'], video_distribution='remote_and_local_only'),
        readiness='specifications_compiled; runtime_binding_and_end_to_end_acceptance_pending')
    (output / 'suite.json').write_text(json.dumps(suite, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(tasks=len(tasks), environments=suite['environment_count'],
        variants=len(variants), planned_trajectories=suite['planned_trajectory_count'],
        runtime_validated=0), ensure_ascii=False))


if __name__ == '__main__':
    main()
