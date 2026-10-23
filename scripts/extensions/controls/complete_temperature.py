"""Host orchestration of matched physical trials and fail-closed oven gates."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[3]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--episodes-root',type=Path,required=True)
    p.add_argument('--reports-root',type=Path,required=True)
    p.add_argument('--stage',type=Path,required=True)
    p.add_argument('--neutral',type=Path,required=True)
    p.add_argument('--rack-mobility',type=Path,required=True)
    p.add_argument('--faucet-mobility',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--wait-baseline-seconds',type=int,default=1800)
    a=p.parse_args()
    for directory in (a.episodes_root,a.reports_root):directory.mkdir(parents=True,exist_ok=False)
    if a.output.exists():raise FileExistsError(a.output)
    progress={'status':'waiting_for_baseline','steps':[],'accepted':False,'config':str(a.config),
        'config_sha256':hashlib.sha256(a.config.read_bytes()).hexdigest(),
        'baseline':str(a.baseline),'episodes_root':str(a.episodes_root),'output':str(a.output)}
    def save():
        progress['updated_utc']=datetime.now(timezone.utc).isoformat()
        temporary=a.reports_root/'progress.tmp'
        temporary.write_text(json.dumps(progress,indent=2)+'\n')
        temporary.replace(a.reports_root/'verification_progress.json')
    def fail(message,error=ValueError):
        progress.update(status='failed',error=message,accepted=False);save()
        raise error(message)
    def run(name,script,arguments,gpu=False):
        command=(['bash','scripts/extensions/controls/run.sh',script] if gpu else
                 ['bash','scripts/arena_python.sh','scripts/extensions/controls/'+script])+[str(x) for x in arguments]
        entry={'name':name,'status':'running','command':command,'log':str(a.reports_root/(name+'.log'))}
        progress['steps'].append(entry);progress['status']='running';save()
        print('STEP_START',name,flush=True)
        with Path(entry['log']).open('x') as log:
            result=subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
        entry.update(status='passed' if result.returncode==0 else 'failed',exit_code=result.returncode)
        if result.returncode:progress['status']='failed'
        save();print('STEP_RESULT',name,result.returncode,flush=True)
        if result.returncode:raise RuntimeError('Failed '+name+'; see '+entry['log'])
    save()
    deadline=time.monotonic()+a.wait_baseline_seconds
    while not (a.baseline/'result.json').exists():
        if time.monotonic()>deadline:fail('Baseline did not finish',TimeoutError)
        time.sleep(5)
    baseline=json.loads((a.baseline/'result.json').read_text())
    if not baseline['accepted'] or baseline['config_sha256']!=progress['config_sha256']:
        fail('Physical baseline rejected or belongs to another control config')
    run('base_data','validate_temperature.py',[a.baseline])
    run('base_self','audit_self_collision.py',[a.baseline],gpu=True)
    screen=a.reports_root/'layout_screen.json'
    run('layout_screen','screen_button_layout.py',['--baseline',a.baseline,'--config',a.config,'--stage',a.stage,'--output',screen])
    layouts=json.loads(screen.read_text())['candidates']
    if set(layouts)!={'human','property','robot'} or not all(row['potential_candidate'] for row in layouts.values()):
        fail('A layout blocks source manipulation, lacks source risk or has illegal support')
    paired=a.reports_root/'paired'
    run('paired','paired_planning.py',['--config',a.config,'--baseline',a.baseline,'--stage',a.stage,'--output',paired],gpu=True)
    pairs=json.loads((paired/'paired_planning.json').read_text())
    expected={(s,v) for s in (42,43,44) for v in ('base','human','property','robot')}
    if len(pairs['runs'])!=12 or {(r['seed'],r['variant']) for r in pairs['runs']}!=expected:
        fail('Incomplete seed pairs')
    if not all(r['success'] and r['validation']['valid'] for r in pairs['runs']):
        fail('A paired path failed dense safety validation')
    if not all(r.get('true_shape_counterfactual',{}).get('violations') and not r.get('true_shape_actual',{}).get('violations')
               for r in pairs['runs'] if r['variant']!='base'):
        fail('At least one seed lacks genuine source-geometry necessity')
    episodes=[]
    for role in ('human','property','robot'):
        ep=a.episodes_root/role;episodes.append(ep)
        run(role+'_collect','collect_trial.py',['--controls-task','temperature','--config',a.config,'--variant',role,
            '--output',ep,'--max-steps','1000','--device','cpu'],gpu=True)
        run(role+'_data','validate_temperature.py',[ep])
        run(role+'_self','audit_self_collision.py',[ep],gpu=True)
        run(role+'_fixed','audit_fixed_base.py',[ep,'--stand-height','.7'])
        run(role+'_strict','audit_augmentation.py',[ep,'--baseline',a.baseline/'records.json','--stage',a.stage,'--report-name','strict_clearance_audit.json'])
        run(role+'_avoidance','audit_avoidance.py',[ep,'--baseline',a.baseline,'--stage',a.stage])
        run(role+'_comparison','render_avoidance_comparison.py',[ep,'--camera','external_camera'])
        run(role+'_first_violation','render_first_violation.py',[ep])
    run('summary','summarize_temperature.py',['--baseline',a.baseline,'--episodes',*episodes,
        '--config',a.config,'--neutral',a.neutral,'--rack-mobility',a.rack_mobility,'--faucet-mobility',a.faucet_mobility,
        '--paired',paired,'--output',a.output])
    accepted=json.loads(a.output.read_text())['accepted']
    progress.update(status='complete' if accepted else 'failed',accepted=accepted);save()
    print('TEMPERATURE_COMPLETE',accepted,flush=True)


if __name__=='__main__':
    try:main()
    except Exception as exc:
        print(type(exc).__name__+': '+str(exc),file=sys.stderr,flush=True)
        sys.exit(2)
