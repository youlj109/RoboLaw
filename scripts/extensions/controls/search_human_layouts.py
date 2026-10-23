"""Host-only bounded layout search; every candidate uses the same paired policy."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[3]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--stage',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    output=a.output.resolve()
    output.relative_to(ROOT)
    output.mkdir(parents=True,exist_ok=False)
    original=json.loads(a.config.read_text())
    positions=[[2.90,-.90,0.],[2.92,-.90,0.],[2.94,-.90,0.],
               [2.90,-.94,0.],[2.92,-.94,0.],[2.94,-.94,0.]]
    summary={'candidate_order':positions,'seeds':[42,43,44],'attempts':[],
        'selection_rule':'First declared layout with all six dense-valid paired plans, all three source-geometry baseline violations and zero source-geometry actual violations. No physical execution here; kitchen/support legality remains a separate gate.',
        'selected':None}
    for index,position in enumerate(positions):
        config=json.loads(json.dumps(original))
        config['variants']['human']['position']=position
        config['augmentation_design']='Bounded human layout candidate; unchanged baseline endpoint/control/seed policy. Acceptance requires independent physical and source-geometry gates.'
        config_path=output/f'candidate{index:02d}.json'
        config_path.write_text(json.dumps(config,indent=2)+'\n')
        paired=output/f'candidate{index:02d}'
        command=['bash','scripts/extensions/controls/run.sh','paired_planning.py',
            '--config',str(config_path.relative_to(ROOT)),
            '--baseline',str(a.baseline),'--stage',str(a.stage),
            '--output',str(paired.relative_to(ROOT)),
            '--seeds','42','43','44','--variants','base','human']
        print('HUMAN_SEARCH_START',index,position,flush=True)
        with (output/f'candidate{index:02d}.log').open('x') as log:
            process=subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
        report_path=paired/'paired_planning.json'
        report=json.loads(report_path.read_text()) if report_path.exists() else {}
        runs=report.get('runs',[])
        complete=len(runs)==6 and {(r['seed'],r['variant']) for r in runs}=={(s,v) for s in (42,43,44) for v in ('base','human')}
        good=bool(process.returncode==0 and complete and all(r.get('success') and r.get('validation',{}).get('valid') for r in runs)
            and all(r.get('true_shape_counterfactual',{}).get('violations') and not r.get('true_shape_actual',{}).get('violations') for r in runs if r['variant']=='human'))
        summary['attempts'].append({'index':index,'position':position,'exit_code':process.returncode,
            'paired_report':str(report_path.relative_to(ROOT)),'candidate_passed':good})
        if good:summary['selected']=str(config_path.relative_to(ROOT))
        (output/'search.json').write_text(json.dumps(summary,indent=2)+'\n')
        print('HUMAN_SEARCH_RESULT',index,good,flush=True)
        if good:return
    sys.exit(2)


if __name__=='__main__':main()
