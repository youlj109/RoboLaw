"""Run only the two requested C01 revisions, preserving all earlier evidence."""
import argparse
import json
import os
import subprocess
import sys
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--variant',choices=['base','human','property'],required=True)
    p.add_argument('--config',required=True)
    p.add_argument('--attempt',required=True)
    p.add_argument('--paired-report',type=Path)
    p.add_argument('--baseline',type=Path,default=Path('datasets/doors/lwx_front_20261023/base_01'))
    a=p.parse_args()
    data=Path('datasets/doors/lwx_front_20261023')
    report=Path('reports/doors/lwx_front_20261023')
    report.mkdir(parents=True,exist_ok=True)
    base=a.baseline
    ep=data/a.attempt
    neutral=data/('neutral_'+a.attempt)
    prefix=report/a.attempt
    status=Path(str(prefix)+'.pipeline.json')
    if ep.exists() or neutral.exists() or status.exists():p.error('Output already exists')
    if a.variant!='base':
        if a.paired_report is None:p.error('A complete paired planning report is required')
        plan=json.loads(a.paired_report.read_text())
        if not plan['stable_policy_necessity_in_tested_seeds'] or plan['seeds_predeclared']!=[42,43,44]:
            p.error('Full predeclared paired planning has not passed; collection refused')
        import hashlib
        if plan['config_sha256']!=hashlib.sha256(Path(a.config).read_bytes()).hexdigest():
            p.error('Paired configuration differs')
        if plan['baseline_sha256']!=hashlib.sha256((base/'trajectory.json').read_bytes()).hexdigest():
            p.error('Paired report uses a different matching baseline')
        bm=json.loads((base/'metadata.json').read_text())
        if plan['controller_parameters']!=bm['controller_parameters']:
            p.error('Planning policy differs from physical baseline')
    env=os.environ.copy();env['PYTHONPATH']='/workspace';env['HEADLESS']='1'
    for key in ['ISAACSIM_ROOT_PATH','ISAACLAB_PATH','OMNI_SERVER']:env.pop(key,None)
    steps=[]
    def run(label,args,gpu=False):
        cmd=([str(Path('/workspace/.venv-arena/bin/python'))]+list(map(str,args)))
        if gpu:cmd=['flock','/workspace/reports/skill_trials/gpu.lock']+cmd
        logfile=report/((neutral.name if label=='neutral' else a.attempt)+('.log' if label in ['neutral','collect'] else '_'+label+'.log'))
        if logfile.exists():raise FileExistsError(logfile)
        entry={'stage':label,'command':cmd,'log':str(logfile),'state':'running'}
        steps.append(entry);status.write_text(json.dumps(steps,indent=2)+'\n')
        print('START',a.attempt,label,flush=True)
        with logfile.open('x') as log:
            result=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,env=env,stdin=subprocess.DEVNULL)
        entry.update(state='passed' if result.returncode==0 else 'failed',exit_code=result.returncode)
        status.write_text(json.dumps(steps,indent=2)+'\n')
        print('END',a.attempt,label,result.returncode,flush=True)
        if result.returncode:raise RuntimeError('Stage failed: '+label)
    common=['scripts/extensions/doors/collect.py','--device','cpu','--variant',a.variant,'--adaptive-handle-height','--validated-pregrasp','--variant-config',a.config]
    run('neutral',common+['--mode','neutral','--neutral-seconds','40','--max-steps','800','--output',neutral],True)
    run('neutral_review',['scripts/extensions/doors/audit_review.py',neutral])
    run('collect',common+['--mode','collect','--max-steps','500','--output',ep],True)
    run('validation',['scripts/extensions/doors/validate_trial.py',ep])
    run('self_collision',['scripts/extensions/doors/audit_self_collision.py',ep],True)
    run('review',['scripts/extensions/doors/audit_review.py',ep,'--neutral',neutral])
    run('contact_geometry',['scripts/extensions/doors/audit_contact_geometry.py',ep])
    run('stand',['scripts/extensions/doors/audit_stand.py',ep])
    run('source_layout',['scripts/extensions/doors/audit_source_layout.py',ep,'--logs',report])
    if a.variant!='base':
        run('avoidance',['scripts/extensions/doors/audit_avoidance.py',ep,'--baseline',base])
        run('comparison',['scripts/extensions/doors/make_comparison.py',ep,'--baseline',base])
    print('FRONT_REVISION_COMPLETE',a.attempt,flush=True)


if __name__=='__main__':main()
