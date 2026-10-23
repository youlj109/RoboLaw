#!/usr/bin/env python3
"""CPU-only skill package validation; requires PyYAML, never starts simulation."""
import argparse
import hashlib
import json
import re
import subprocess
import sys
import tempfile
from pathlib import Path
import shutil
import yaml


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--repo',type=Path,required=True)
    p.add_argument('--report',type=Path,required=True)
    args=p.parse_args(); root=args.repo.resolve(); package=Path(__file__).resolve().parent
    findings=[]
    def check(name,ok,detail=None):
        findings.append({'test':name,'passed':bool(ok),'detail':detail})
    assignments=[]
    for folder in sorted(package.glob('robolaw-*')):
        a=json.loads((folder/'references/assignment.json').read_text()); assignments.append(a)
        skill=(folder/'SKILL.md').read_text()
        front=yaml.safe_load(skill.split('---',2)[1])
        check(folder.name+': frontmatter',front['name']==folder.name and bool(front['description']))
        links=re.findall(r'\[[^\]]*\]\(([^)]+)\)',skill)
        check(folder.name+': skill links resolve',all((folder/link).is_file() for link in links if '://' not in link))
        for rel in ['references/assignment.json','references/workflow.md','scripts/preflight.py','scripts/runtime_inventory.py']:
            check(folder.name+': portable reference '+rel,(folder/rel).is_file())
        matched=True
        for t in a['tasks']:
            f=root/t['source_yaml']; d=yaml.safe_load(f.read_text())
            for key in ('background','embodiment','objects','object_references','relations','task'):
                matched &= t[key]==d.get(key,[])
            matched &= t['id']==d['env_name'] and t['source_sha256']==hashlib.sha256(f.read_bytes()).hexdigest()
        check(folder.name+': metadata matches real upstream YAML',matched)
        with tempfile.TemporaryDirectory(prefix='robolaw-portable-skill-') as temp:
            temp=Path(temp); portable=temp/folder.name; shutil.copytree(folder,portable)
            script=portable/'scripts/preflight.py'
            cmd=[sys.executable,str(script),'--repo',str(root)]
            result=subprocess.run(cmd,cwd=temp,capture_output=True,text=True)
            data=json.loads(result.stdout)
            check(folder.name+': runs outside repository',result.returncode==0 and data['selected_environment_count']==a['environment_count'])
            expected_assets={obj['registry_name'] for t in a['tasks'] for obj in [t['background']]+t['objects']}
            check(folder.name+': assigned registry asset inventory', {x['registry_name'] for x in data['required_registry_assets']}==expected_assets and all(x['asset_readiness']=='not_checked' for x in data['required_registry_assets']))
            check(folder.name+': no false physics claim',data['gpu_started'] is False and data['physical_collection_verified'] is False)
            skeleton=temp/'first_task.json'
            selected=subprocess.run(cmd+['--task',a['representative_task'],'--emit-skeleton',str(skeleton)],cwd=temp,capture_output=True,text=True)
            draft=json.loads(skeleton.read_text()) if skeleton.is_file() else {}
            check(folder.name+': exact representative selection',selected.returncode==0 and len(draft.get('tasks',[]))==1 and draft['tasks'][0]['id']==a['representative_task'] and draft['status']=='planning_only')
            overwrite=subprocess.run(cmd+['--emit-skeleton',str(skeleton)],cwd=temp,capture_output=True,text=True)
            check(folder.name+': preserves existing planning output',overwrite.returncode==2 and 'Refusing to overwrite' in overwrite.stderr and json.loads(skeleton.read_text())==draft)
            inventory=subprocess.run([sys.executable,str(portable/'scripts/runtime_inventory.py')],cwd=temp,capture_output=True,text=True)
            inv=json.loads(inventory.stdout)
            check(folder.name+': readonly runtime metadata inventory',inventory.returncode==0 and inv['gpu_started'] is False and inv['runtime_verified'] is False and bool(inv['python_executable']))
            foreign=subprocess.run(cmd+['--task','kitchen_bench_lightwheel_pick_and_place'],cwd=temp,capture_output=True,text=True)
            check(folder.name+': rejects out-of-assignment task',foreign.returncode!=0)
            absent=subprocess.run([sys.executable,str(script),'--repo',str(temp/'missing-repo')],cwd=temp,capture_output=True,text=True)
            no_repo=json.loads(absent.stdout)
            check(folder.name+': missing repository blocks cleanly',absent.returncode==1 and no_repo['status']=='blocked')
            # Corrupt the expected source hash in the portable package, preserving real repo.
            acopy=json.loads((portable/'references/assignment.json').read_text())
            acopy['tasks'][0]['source_sha256']='0'*64
            (portable/'references/assignment.json').write_text(json.dumps(acopy))
            stale=subprocess.run(cmd,cwd=temp,capture_output=True,text=True)
            stale_data=json.loads(stale.stdout)
            check(folder.name+': source-version mismatch blocks',stale.returncode==1 and any(x['kind']=='source_changed' for x in stale_data['issues']))
    ids=[t['id'] for a in assignments for t in a['tasks']]
    source=root/'third_party/IsaacLab-Arena/isaaclab_arena_environments/kitchen_bench'
    expected={f.stem for f in source.glob('kitchen_bench_*.yaml')} - {'kitchen_bench_lightwheel_pick_and_place'}
    check('all 30 tasks assigned exactly once',len(ids)==30 and len(set(ids))==30 and set(ids)==expected)
    check('group counts 14 / 5 / 8 / 3',{a['group']:a['environment_count'] for a in assignments}=={'replicator':14,'lightwheel-pick-place':5,'doors':8,'controls':3})
    owned=[(a['group'],x) for a in assignments for x in a['owned_paths']]
    overlap=[(a,b,x,y) for a,x in owned for b,y in owned if a!=b and (x.startswith(y) or y.startswith(x))]
    check('ownership directories do not overlap',not overlap,overlap)
    check('personal branch names supplied by students',all(a['branch'] is None and a.get('branch_naming')=='student_name_initials' and a.get('base_branch')=='origin/ylj' for a in assignments))
    report={'passed':all(f['passed'] for f in findings),'checks':findings,'scope':'CPU static metadata, portability, scope rejection and source integrity; no physics success claim','gpu_started':False}
    args.report.parent.mkdir(parents=True,exist_ok=True); args.report.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'passed':report['passed'],'checks':len(findings),'failures':[f for f in findings if not f['passed']]},ensure_ascii=False))
    return 0 if report['passed'] else 1
if __name__=='__main__': raise SystemExit(main())
