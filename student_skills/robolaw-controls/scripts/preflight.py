#!/usr/bin/env python3
"""Read-only CPU preflight for a portable RoboLaw student skill; no GPU imports."""
import argparse
import hashlib
import json
import re
import subprocess
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo',type=Path,required=True,help='Existing RoboLaw checkout; not the skill folder')
    parser.add_argument('--task',help='Restrict selection to an exact assigned environment ID')
    parser.add_argument('--report',type=Path,help='Optional JSON report output')
    parser.add_argument('--emit-skeleton',type=Path,help='Write a planning-only task manifest, not a runnable scene')
    args=parser.parse_args()
    if args.emit_skeleton and args.emit_skeleton.exists():
        parser.error('Refusing to overwrite existing planning manifest: '+str(args.emit_skeleton))
    skill=Path(__file__).resolve().parents[1]
    assignment=json.loads((skill/'references/assignment.json').read_text())
    tasks=assignment['tasks']
    if args.task:
        tasks=[t for t in tasks if t['id']==args.task]
        if not tasks: parser.error('Task is outside this student assignment: '+args.task)
    root=args.repo.resolve()
    issues=[]; warnings=[]
    required=['README.md','configs/dependency_mirrors.json','scripts/arena_python.sh','kitchen_safety/environment.py','kitchen_safety/collision_planner.py','scripts/collect_kitchen.py']
    for rel in required:
        if not (root/rel).is_file(): issues.append({'kind':'missing_repository_file','path':rel})
    registry_assets={}
    for task in tasks:
        for role, obj in [('background',task['background'])]+[('object',x) for x in task['objects']]:
            name=obj['registry_name']
            entry=registry_assets.setdefault(name,{'registry_name':name,'roles':[],'task_ids':[],'asset_readiness':'not_checked'})
            if role not in entry['roles']: entry['roles'].append(role)
            if task['id'] not in entry['task_ids']: entry['task_ids'].append(task['id'])
    checks=[]
    for t in tasks:
        f=root/t['source_yaml']
        check={'id':t['id'],'source_yaml':t['source_yaml'],'exists':f.is_file(),'source_hash_match':False}
        if f.is_file():
            data=f.read_bytes()
            check['actual_sha256']=hashlib.sha256(data).hexdigest()
            check['source_hash_match']=check['actual_sha256']==t['source_sha256']
            if not check['source_hash_match']:
                issues.append({'kind':'source_changed','path':t['source_yaml'],'action':'Review the actual task semantics and revise assignment deliberately; never replace the source automatically.'})
            match=re.search(r'^env_name:\s*(\S+)\s*$',data.decode(),re.M)
            check['environment_id_match']=bool(match and match.group(1)==t['id'])
            if not check['environment_id_match']: issues.append({'kind':'environment_id_mismatch','path':t['source_yaml']})
        else:
            issues.append({'kind':'missing_pinned_task_yaml','path':t['source_yaml'],'action':'Restore pinned Arena source with the repository fetch_source_dependencies.py workflow; this preflight does not download.'})
        checks.append(check)
    mirrors=root/'configs/dependency_mirrors.json'
    if mirrors.is_file():
        try:
            d=json.loads(mirrors.read_text()); components=d.get('bundles',{})
            arena=components.get('arena',{}).get('revision')
            if arena!=assignment['arena_revision']: issues.append({'kind':'arena_revision_mismatch','actual':arena,'expected':assignment['arena_revision']})
        except (ValueError,TypeError): issues.append({'kind':'invalid_dependency_manifest'})
    try:
        head=subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True,stderr=subprocess.DEVNULL).strip()
        branch=subprocess.check_output(['git','-C',str(root),'branch','--show-current'],text=True,stderr=subprocess.DEVNULL).strip()
    except (OSError,subprocess.CalledProcessError): head=None; branch=None
    if head and head!=assignment['base_commit']:
        warnings.append('Checkout has advanced from the documented base; review task/source changes. No branch or file was changed.')
    catalog=[]
    for category in ('property','robot'):
        f=root/f'assets/obstacle_library/{category}/catalog.json'
        if f.is_file():
            try: catalog += [a['id'] for a in json.loads(f.read_text())['assets']]
            except (ValueError,KeyError,TypeError): issues.append({'kind':'invalid_obstacle_catalog','path':str(f.relative_to(root))})
        else: issues.append({'kind':'missing_obstacle_catalog','category':category})
    people_catalog=(root/'assets/obstacle_library/people/catalog.json').is_file()
    assets={rel:(root/rel).is_file() for rel in ['assets/kitchen/scene.usd','assets/people/mpfb_corridor/collidable.usda','assets/people/mpfb_corridor/person.usdc','assets/curobo/droid/franka_robotiq_2f_85_zero_curobo.yml']}
    if not all(assets.values()): warnings.append('Some original-example binary assets are absent. Restore them before GPU execution; static task preflight can still pass.')
    capabilities={
        'parallel_collection_module_present':(root/'kitchen_safety/parallel_collection.py').is_file(),
        'persistent_sequence_script_present':(root/'scripts/collect_kitchen_sequence.py').is_file(),
        'human_catalog_present':people_catalog,
    }
    warnings.append('Static preflight does not verify all assigned kitchen/object assets, Python runtime, CUDA, physical success, RGB output, or safety.')
    report={'schema_version':1,'group':assignment['group'],'status':'static_pass' if not issues else 'blocked','assigned_environment_count':assignment['environment_count'],'selected_environment_count':len(tasks),'selected_ids':[t['id'] for t in tasks],'representative_task':assignment['representative_task'],'branch_expected':assignment['branch'],'branch_actual':branch,'head_actual':head,'task_sources':checks,'required_registry_assets':list(registry_assets.values()),'obstacle_catalog_ids':catalog,'original_asset_presence':assets,'capabilities':capabilities,'issues':issues,'warnings':warnings,'assigned_asset_readiness':'not_checked','runtime_readiness':'not_checked','gpu_started':False,'physical_collection_verified':False}
    if args.report:
        args.report.parent.mkdir(parents=True,exist_ok=True); args.report.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    if args.emit_skeleton and not issues:
        draft={'status':'planning_only','group':assignment['group'],'owned_paths':assignment['owned_paths'],'tasks':tasks,'initial_variant_candidates':['property_vase','robot_projecting_crossbar','human_default_mpfb'],'requires_implementation':['task-specific environment','semantic obstacle placement','cuRobo executor','task-aware validation and audit'],'physical_collection_verified':False}
        args.emit_skeleton.parent.mkdir(parents=True,exist_ok=True)
        with args.emit_skeleton.open('x') as f: json.dump(draft,f,ensure_ascii=False,indent=2); f.write('\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if not issues else 1

if __name__=='__main__':
    raise SystemExit(main())
