"""Run explicit short interface probes, never a successful-task data quota.

Each selected task gets three stationary commands, raw audit and offline risk.
Execute inside the configured Arena Python environment. Serial execution avoids
mixing GPU measurement loads; failures keep receipts and do not imply success.
"""
import argparse,json,os,signal,subprocess,sys,time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--task',action='append',required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--asset-cache',type=Path,required=True)
    p.add_argument('--timeout-s',type=float,default=180.)
    p.add_argument('--seed',type=int,default=42)
    a=p.parse_args()
    if a.timeout_s<=0:raise ValueError('timeout must be positive')
    suite=json.loads((ROOT/'configs/risk_task_suite/suite.json').read_text())
    tasks={t['task_id']:t for t in suite['tasks']}
    if len(set(a.task))!=len(a.task) or not set(a.task)<=tasks.keys():
        raise ValueError('Select unique registered task IDs')
    a.output.mkdir(parents=True,exist_ok=False)
    report=dict(kind='three_hold_action_interface_probes',full_task_acceptance=False,
        counts_toward_200_quota=False,tasks={},complete=False)
    def save():
        tmp=a.output/'probe_summary.json.part';tmp.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        tmp.replace(a.output/'probe_summary.json')
    def run(task,stage,command):
        started=time.perf_counter()
        with (a.output/(task+'.'+stage+'.log')).open('x') as log:
            child=subprocess.Popen([sys.executable]+command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,
                start_new_session=True,env=os.environ|{'PYTHONPATH':str(ROOT)})
            try:code=child.wait(timeout=a.timeout_s);timed_out=False
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGTERM)
                try:child.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
                code=child.returncode;timed_out=True
        value=dict(exit_code=code,timeout=timed_out,process_wall_s=time.perf_counter()-started)
        report['tasks'][task][stage]=value;save()
        return code==0 and not timed_out
    save()
    for name in a.task:
        target=a.output/name;report['tasks'][name]={};save()
        ok=run(name,'capture',['scripts/inspect_risk_task.py','--task-contract',tasks[name]['path'],
            '--output',str(target),'--asset-cache',str(a.asset_cache),'--capture-hold-actions','3',
            '--seed',str(a.seed),'--placement_seed',str(a.seed),'--device','cpu','--num_envs','1'])
        receipt=target/'inventory_receipt.json'
        if receipt.exists():
            row=json.loads(receipt.read_text())
            report['tasks'][name]['inventory']={k:row.get(k) for k in
                ('complete','collider_count','joint_count','missing_actor_paths','scope_unresolved','error','hold_probe')}
            ok=ok and row.get('complete') is True and row.get('hold_probe',{}).get('physics_steps')==12
        else:ok=False
        if ok:
            run(name,'audit',['scripts/audit_native_task.py','--episode',str(target/'raw_hold'),
                '--output',str(target/'audit.json')])
            run(name,'evaluate',['scripts/evaluate_native_task.py','--episode',str(target/'raw_hold'),
                '--output',str(target/'offline')])
        save();print(json.dumps(dict(task=name,stages=report['tasks'][name]),ensure_ascii=False),flush=True)
    report['complete']=True;save()
    return int(any(not all(v.get(k,{}).get('exit_code')==0 for k in ('capture','audit','evaluate'))
        for v in report['tasks'].values()))

if __name__=='__main__':raise SystemExit(main())
