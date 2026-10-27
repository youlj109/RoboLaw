"""Review finished matched baselines without repeating any physical episode."""
import argparse
import json
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[3]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--progress',type=Path,nargs='+',required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    paths=set()
    for path in args.progress:
        for row in json.loads(path.read_text()).get('baselines',{}).values():
            if row['accepted']:
                paths.add(row['path'])
    report={'scope':'Independent review of existing physical baselines; no recollection','baselines':[]}
    for index,directory in enumerate(sorted(paths)):
        episode=Path(directory)
        meta=json.loads((episode/'result.json').read_text())
        if meta.get('variant')!='base':
            raise ValueError('Expected a no-hazard baseline')
        temperature='turn_oven_temperature_knob' in meta['task_id']
        checks=[('data','validate_temperature.py' if temperature else 'validate_trial.py',
                 'independent_temperature_validation.json' if temperature else 'independent_validation.json',False)]
        if not temperature:
            checks.append(('contact','review_contact_causality.py','contact_causality_review.json',False))
        checks.append(('self','audit_self_collision.py','self_collision_detailed_audit.json',True))
        row={'path':directory,'checks':{}}
        report['baselines'].append(row)
        for name,script,filename,gpu in checks:
            destination=episode/filename
            if not destination.exists():
                command=(['bash','scripts/extensions/controls/run.sh',script] if gpu else
                         ['bash','scripts/arena_python.sh','scripts/extensions/controls/'+script])+[
                             episode.resolve().relative_to(ROOT).as_posix()]
                with (args.output/f'base{index:02d}_{name}.log').open('x') as log:
                    code=subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT).returncode
                row[name+'_exit_code']=code
            row['checks'][name]=destination.exists() and json.loads(destination.read_text()).get('passed') is True
            (args.output/'review.json').write_text(json.dumps(report,indent=2)+'\n')
        row['passed']=all(row['checks'].values())
        (args.output/'review.json').write_text(json.dumps(report,indent=2)+'\n')
        print('BASELINE_REVIEW',directory,row['passed'],row['checks'],flush=True)
    if not all(row['passed'] for row in report['baselines']):
        raise SystemExit(2)


if __name__=='__main__':main()
