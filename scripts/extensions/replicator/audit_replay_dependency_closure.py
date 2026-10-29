#!/usr/bin/env python3
"""Read-only reference inventory; no simulator/planner imports and no asset mutation.

This is a conservative artifact closure, not proof of an installed runtime.
It follows JSON file references and ASCII USD asset references, records binary
assets for a separate USD-aware closure, and never loads native state arrays.
"""
from __future__ import annotations
import argparse
from collections import Counter,defaultdict,deque
from datetime import datetime,timezone
import hashlib,json,os,re,subprocess
from pathlib import Path

ROOT=Path(__file__).resolve().parents[3]
ROOTS=('configs/','reports/','exports/','assets/','runtime/','third_party/','kitchen_safety/','scripts/','docs/','tests/','datasets/','.venv-arena/','student_skills/')
SUFFIXES={'.json','.npz','.npy','.usda','.usd','.usdc','.usdz','.yaml','.yml','.urdf','.obj','.stl','.dae','.mtl','.py','.sh','.md','.txt','.log','.csv','.jsonl','.png','.jpg','.jpeg','.webp','.mp4','.h5','.hdf5','.zip','.gz','.toml'}
HEX=re.compile(r'^[0-9a-f]{64}$')
MEDIA={'.png','.jpg','.jpeg','.webp','.mp4','.gif'}
CODE={'.py','.sh','.md','.toml'}


def category(path):
    p=Path(path);suffix=p.suffix.lower()
    if path.startswith('.venv-arena/'):return 'installed_runtime_reference_not_a_portable_restore_input'
    if path.startswith('runtime/') and suffix in ('.usd','.usda','.usdc'):return 'runtime_generated_usd_input'
    if path.startswith(('assets/','third_party/','runtime/')):return 'external_asset_or_runtime_source'
    if suffix in MEDIA:return 'visual_evidence_optional_for_replay'
    if suffix in ('.h5','.hdf5') or (path.startswith(('exports/','datasets/')) and suffix in ('.npz','.npy','.jsonl')) or p.name in ('grasp_contacts.json','actions.json'):
        return 'native_recording_optional_for_replay_required_for_numeric_reaudit'
    if suffix in ('.zip','.gz'):return 'visual_evidence_optional_for_replay' if 'video' in p.name else 'archive_or_runtime_snapshot'
    if path.startswith(('scripts/','kitchen_safety/','tests/','datasets/','.venv-arena/','student_skills/')) or suffix in CODE:return 'code_or_documentation'
    if suffix in ('.usd','.usda','.usdc'):return 'composed_scene_input'
    if suffix in ('.npz','.npy'):return 'planning_or_small_numerical_input'
    if path.startswith('exports/') and p.name=='result.json':return 'historical_or_latest_physical_result_json'
    if suffix=='.json':return 'configuration_planning_lineage_or_audit_json'
    return 'other_small_reference'


class Closure:
    def __init__(self,root):
        self.root=root.resolve();self.files={};self.queue=deque();self.parsed=set();self.edges=[];self.unresolved=[];self.directories=set();self.external=set();self.json_errors=[];self.skipped_data=[];self.package_members=[];self.reference_bases={}
        self.tracked=set(subprocess.check_output(['git','ls-files','-z'],cwd=root).decode().split('\0'))
    def normalize(self,value,source,explicit=False):
        if not isinstance(value,str) or not value or '\n' in value or '\0' in value:return None
        if value.startswith(('http://','https://','omniverse://','package://')):
            self.external.add(value);return None
        if value.startswith(('/World','/RootNode','/physicsScene','/Persistent','/tmp/','/proc/','/dev/')):return None
        if value.startswith('/workspace/'):value=value[len('/workspace/'):]
        elif value.startswith(str(self.root)+'/'):value=value[len(str(self.root))+1:]
        elif value.startswith(('/root/.cache/isaaclab_arena/','/home/oem/.cache/isaaclab_arena/')):
            value='runtime/home/.cache/isaaclab_arena/'+value.split('/.cache/isaaclab_arena/',1)[1]
        elif value.startswith('/'):
            if explicit:self.external.add(value)
            return None
        if value.startswith(ROOTS):target=self.root/value
        elif value.startswith(('executions/','videos/')) and source.startswith('exports/'):
            target=self.root/Path(*Path(source).parts[:2])/value
        elif value.startswith('videos/') and source=='reports/replicator_physical_execution_20261028/first_pass_before_camera_retake.json':
            target=self.root/'exports/replicator_154_physical_demos_20261028'/value
        elif explicit and Path(value).suffix.lower() in SUFFIXES:
            target=(self.root/self.reference_bases[source] if source in self.reference_bases else (self.root/source).parent)/value
        else:return None
        target=Path(os.path.normpath(target))
        if not target.is_relative_to(self.root):self.external.add(str(target));return None
        if target.is_dir():self.directories.add(str(target.relative_to(self.root)));return None
        if target.suffix.lower() not in SUFFIXES:return None
        return str(target.relative_to(self.root))
    def add(self,value,source='<seed>',key='$',expected=None,explicit=False):
        path=self.normalize(value,source,explicit)
        if path is None:return
        record=self.files.get(path)
        if record is None:
            p=self.root/path;exists=p.is_file();record=dict(path=path,exists=exists,size_bytes=p.stat().st_size if exists else None,
                tracked_in_git=path in self.tracked,category=category(path),expected_sha256=set(),reference_count=0,first_source=source,first_key=key)
            self.files[path]=record
            if exists and p.suffix.lower() in ('.json','.usda') and record['category']!='native_recording_optional_for_replay_required_for_numeric_reaudit':self.queue.append(path)
        if isinstance(expected,str) and HEX.fullmatch(expected):record['expected_sha256'].add(expected)
        record['reference_count']+=1
        self.edges.append(dict(source=source,key=key,path=path,expected_sha256=expected if isinstance(expected,str) and HEX.fullmatch(expected) else None))
    def walk(self,obj,source,key='$'):
        if isinstance(obj,dict):
            for name,value in obj.items():
                current=key+'.'+str(name)
                if isinstance(name,str) and isinstance(value,str) and HEX.fullmatch(value):self.add(name,source,current+'::<path-key>',value,True)
                expected=None
                if isinstance(value,str):
                    if name.endswith('_package_path'):
                        self.package_members.append(dict(source=source,key=current,member=value));continue
                    if name=='path':expected=obj.get('sha256')
                    elif name.endswith('_path'):expected=obj.get(name[:-5]+'_sha256')
                    explicit=name=='path' or name.endswith(('_path','_file','_filename')) or name in ('trajectory','source_scene','robot_usd','kitchen_usd','usd_path','image_path','physics_states','actions')
                    self.add(value,source,current,expected,explicit)
                else:self.walk(value,source,current)
        elif isinstance(obj,list):
            for i,value in enumerate(obj):
                if isinstance(value,str):self.add(value,source,key+f'[{i}]')
                else:self.walk(value,source,key+f'[{i}]')
    def run(self):
        while self.queue:
            source=self.queue.popleft()
            if source in self.parsed:continue
            self.parsed.add(source);p=self.root/source
            if p.suffix.lower()=='.json':
                if p.stat().st_size>20*1024**2:self.skipped_data.append(dict(path=source,reason='JSON exceeds 20MiB data-only scan cap'));continue
                try:
                    doc=json.loads(p.read_text())
                    if isinstance(doc,dict) and isinstance(doc.get('reference_episode'),str) and doc['reference_episode'].startswith(ROOTS):self.reference_bases[source]=doc['reference_episode']
                    self.walk(doc,source)
                except (ValueError,UnicodeError) as error:self.json_errors.append(dict(path=source,error=str(error)))
            else:
                try:content=p.read_text()
                except UnicodeError:continue
                for asset in re.findall(r'@([^@\n]+)@',content):self.add(asset,source,'usd.asset_reference',explicit=True)
    def finish(self):
        for row in self.files.values():
            row['expected_sha256']=sorted(row['expected_sha256'])
            if row['exists'] and row['category'] not in ('external_asset_or_runtime_source','installed_runtime_reference_not_a_portable_restore_input','runtime_generated_usd_input','native_recording_optional_for_replay_required_for_numeric_reaudit','visual_evidence_optional_for_replay') and row['size_bytes']<=32*1024**2:
                row['current_sha256']=hashlib.sha256((self.root/row['path']).read_bytes()).hexdigest()
                row['matches_at_least_one_expected_sha256']=not row['expected_sha256'] or row['current_sha256'] in row['expected_sha256']
            else:row['hash_check']='not_read_large_recording_or_external_asset'
        return sorted(self.files.values(),key=lambda x:x['path'])


def summarize(rows):
    return dict(files=len(rows),existing=sum(r['exists'] for r in rows),missing=sum(not r['exists'] for r in rows),
        existing_size_bytes=sum(r['size_bytes'] or 0 for r in rows),git_tracked=sum(r['tracked_in_git'] for r in rows),
        not_git_tracked=sum(not r['tracked_in_git'] for r in rows))


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',default='reports/replicator_dependency_closure_20261029');args=parser.parse_args()
    out=ROOT/args.output;out.mkdir(parents=True,exist_ok=True)
    latest='configs/extensions/replicator/physical_retry_results_round13_20261029.json'
    seeds=[latest,'student_skills/robolaw-replicator/references/assignment.json','configs/extensions/replicator/final_154_scenes_20261028.json','exports/replicator_154_physical_demos_20261028/manifest.json','exports/replicator_u_banana_camera_retakes_20261028/manifest.json']
    seeds += [str(p.relative_to(ROOT)) for p in sorted((ROOT/'configs/extensions/replicator').glob('physical_retry_round[0-9]*_2026102[89].json'))]
    seeds += [str(p.relative_to(ROOT)) for p in sorted((ROOT/'exports').glob('replicator_physical_retry_round[0-9]*_2026102[89]/manifest.json'))]
    closure=Closure(ROOT)
    for seed in seeds:closure.add(seed)
    closure.run();rows=closure.finish()
    summary=dict(kind='replicator_reference_dependency_closure_audit',created_at=datetime.now(timezone.utc).isoformat(),
        source_git_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),seeds=seeds,summary=summarize(rows),
        by_category={category:summarize([r for r in rows if r['category']==category]) for category in sorted({r['category'] for r in rows})},
        parsed_documents=len(closure.parsed),reference_edges=len(closure.edges),json_parse_errors=closure.json_errors,skipped_large_data_json=closure.skipped_data,package_member_refs=closure.package_members,
        excluded_directory_references=sorted(closure.directories),external_paths_or_urls=sorted(closure.external),
        limitations=['Conservative referenced-artifact closure, not a simulator launch or original runtime restoration test.',
                    'Native arrays/media/external assets were only statted, not loaded or hashed.',
                    'Binary USD/material/texture dependency closure is handled by the parallel asset audit.',
                    'Some historical/source-code SHA references intentionally differ from latest working code; report separately from missing inputs.',
                    'No exact externally supplied 91-scene list was present in the inspected local task artifacts.'])
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    (out/'files.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2)+'\n')
    (out/'references.jsonl').write_text(''.join(json.dumps(e,ensure_ascii=False)+'\n' for e in closure.edges))
    for name,selected in [('missing',[r for r in rows if not r['exists']]),('not_git_tracked',[r for r in rows if not r['tracked_in_git']]),
          ('hash_mismatches',[r for r in rows if r.get('matches_at_least_one_expected_sha256') is False]),
          ('recommended_small_restore',[r for r in rows if r['exists'] and not r['tracked_in_git'] and r['category'] not in ('external_asset_or_runtime_source','installed_runtime_reference_not_a_portable_restore_input','native_recording_optional_for_replay_required_for_numeric_reaudit','visual_evidence_optional_for_replay')])]:
        (out/(name+'.json')).write_text(json.dumps(selected,ensure_ascii=False,indent=2)+'\n')
        (out/(name+'.txt')).write_text(''.join(r['path']+'\n' for r in selected))
    snapshot=json.loads((ROOT/latest).read_text());adopted=[]
    original_rows={r['scene_id']:r for r in json.loads((ROOT/'exports/replicator_154_physical_demos_20261028/manifest.json').read_text())['entries']}
    for row in snapshot['entries']:
        chosen=row.get('current_revision_result') or row.get('latest_physical_result') or row['first_pass']['result']
        result=json.loads((ROOT/chosen['path']).read_text())
        source_scene=result.get('source_scene_path') or original_rows[row['scene_id']]['source_scene_path']
        adopted.append(dict(scene_id=row['scene_id'],success=result.get('task_success'),physical_result=chosen,
            config_path=result['config_path'],source_scene_path=source_scene,
            planning_result_path=result['planning_result_path'],trajectory_path=result['trajectory_path'],
            files=[dict(path=v,exists=(ROOT/v).is_file(),size_bytes=(ROOT/v).stat().st_size if (ROOT/v).is_file() else None,tracked_in_git=v in closure.tracked)
                for v in (chosen['path'],result['config_path'],result['planning_result_path'],result['trajectory_path'],source_scene)]))
    (out/'adopted_154_direct_inputs.json').write_text(json.dumps(adopted,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(summary,ensure_ascii=False)[:6000])
if __name__=='__main__':main()
