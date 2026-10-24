"""Run short native capture/audit/offline checks for explicitly mapped assets.

Placement templates belong to their own task; never inherit old world poses or
claim a full motion envelope. No production quota is collected.
"""
import argparse,hashlib,json,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from kitchen_safety.task_contract import digest


def prepare(entry,asset,task,template):
    if asset['category'] not in ('property','robot') or type(asset['static']) is not bool:
        raise ValueError('Only declared property/robot assets with explicit physics type')
    if template.get('task_sha256')!=digest(task) or template.get('placement_mode')!='native_relations':
        raise ValueError('Placement template must be bound to this task and native relations')
    forbidden={'asset_id','usd_path','static','category','position_xyz','rotation_xyzw','hs_surface_omissions'}
    if forbidden&template.keys():raise ValueError('Template must not override asset identity/physics or reuse world poses')
    ids=set(task['asset_declarations'])|{task['roles']['robot']['id']}
    if any(r.get('reference') not in ids for r in template['placement_relations'] if 'reference' in r):
        raise ValueError('Template reference is not in the current task')
    path=(ROOT/asset['usd_path']).resolve()
    if not path.is_file():raise ValueError('Missing local risk asset: '+str(path))
    result=dict(template,asset_id=asset['id'],usd_path=str(path),static=asset['static'],category='PT' if asset['category']=='property' else 'RS')
    result['placement_evidence']=dict(template['placement_evidence'],asset_library_id=asset['id'],
        asset_usd_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),placement_template_sha256=digest(template),
        task_contract_sha256=digest(task),complete_task_motion_validated=False)
    policy=dict(version='robolaw-measured-hold-probe-1',task_sha256=digest(task),actions=3)
    return result,policy


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--mapping',type=Path,default=ROOT/'configs/risk_asset_probes/library-probes.json');p.add_argument('--library',type=Path,default=ROOT/'configs/obstacle_library.json');p.add_argument('--output',type=Path,required=True);p.add_argument('--asset-cache',type=Path,required=True);p.add_argument('--variants',nargs='+');p.add_argument('--seed',type=int,default=42);a=p.parse_args()
    config=json.loads(a.mapping.read_text());assert config['version']=='robolaw-risk-asset-probes-1'
    library={v['id']:v for v in json.loads(a.library.read_text())['assets']};ids=a.variants or sorted(config['variants'])
    if len(set(ids))!=len(ids) or any(x not in config['variants'] or x not in library for x in ids):raise ValueError('Unknown or repeated variant')
    # Preflight every input before starting any native simulation.
    prepared=[]
    for aid in ids:
        entry=config['variants'][aid];task_path=ROOT/entry['task_contract'];task=json.loads(task_path.read_text());template=json.loads((ROOT/entry['placement_template']).read_text())
        binding,policy=prepare(entry,library[aid],task,template);prepared.append((aid,task_path,binding,policy))
    a.output=a.output.resolve();a.output.mkdir(parents=True,exist_ok=False);report=dict(complete=False,variants={},purpose=config['purpose'],production_collection=False)
    def save():(a.output/'library_probe_report.json').write_text(json.dumps(report,indent=2)+'\n')
    save()
    for aid,task,binding,policy in prepared:
        bp=a.output/(aid+'.binding.json');pp=a.output/(aid+'.policy.json');bp.write_text(json.dumps(binding,indent=2)+'\n');pp.write_text(json.dumps(policy,indent=2)+'\n');run=a.output/aid
        entry=dict(stages={},passed=False,asset_sha256=binding['placement_evidence']['asset_usd_sha256'],task_sha256=policy['task_sha256'],binding_sha256=digest(binding));report['variants'][aid]=entry;save()
        for stage in ('capture','audit','evaluate'):
            command=[sys.executable,'scripts/risk_capture_workflow.py',stage,'--output',str(run)]
            if stage=='capture':command+=['--task-contract',str(task),'--asset-cache',str(a.asset_cache),'--obstacle-binding',str(bp),'--policy-factory','kitchen_safety.task_hold_probe_policy:create','--policy-config',str(pp),'--max-actions','3','--seed',str(a.seed)]
            started=time.perf_counter()
            with (a.output/(aid+'.'+stage+'.log')).open('x') as log:
                process=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
                entry['running_stage']=stage;entry['pid']=process.pid;save();code=process.wait()
            entry.pop('running_stage');entry.pop('pid');entry['stages'][stage]=dict(exit_code=code,wall_s=time.perf_counter()-started);save();print(aid,stage,entry['stages'][stage],flush=True)
            if code:break
        entry['passed']=len(entry['stages'])==3 and all(v['exit_code']==0 for v in entry['stages'].values());save()
    report['complete']=True;report['all_stages_passed']=all(v['passed'] for v in report['variants'].values());save()
    print(json.dumps(dict(complete=True,all_stages_passed=report['all_stages_passed'],variants=len(ids))),flush=True)
    raise SystemExit(not report['all_stages_passed'])
if __name__=='__main__':main()
