"""Collect the existing 11 scenes with a read-only SafeBench observer.

Keeps the existing nonhuman presets and the separately established MPFB grasp
options. Every attempt is retained. The original obstruction quota is unchanged.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from kitchen_safety.risk_profile_loader import load_config


def commands(args):
    presets=json.loads((ROOT/'configs/reproduction_presets.json').read_text())['nonhuman10']['ids']
    ids=args.ids or presets+['human_default_mpfb']
    if len(set(ids))!=len(ids) or set(ids)-set(presets+['human_default_mpfb']):
        raise ValueError('Select unique IDs from the existing 11 scenes')
    batches=[]
    for group,selected,config in [('nonhuman10',[i for i in ids if i!='human_default_mpfb'],'configs/augmentation_nonhuman10.json'),
                                  ('human_default',[i for i in ids if i=='human_default_mpfb'],'configs/augmentation_human7.json')]:
        if not selected:continue
        output=args.output/group
        command=[sys.executable,str(ROOT/'scripts/collect_obstacle_library.py'),
            '--ids',*selected,'--randomization-config',config,'--seed',str(args.seed),
            '--episodes-per-id',str(args.episodes_per_id),'--max-attempts',str(args.max_attempts),
            '--record-rgb','--preview-per-id',str(args.preview_per_id),
            '--execution-backend','vectorized','--pipeline','--num-envs',str(args.num_envs),
            '--validation-workers',str(args.num_envs),'--device','cpu','--physics-threads','0',
            '--torch-threads','1','--camera-device','cuda:0','--background-rgb',
            '--execution-checks','deferred','--episode-length-s','180',
            '--observer-config',str(args.observer_config),'--run-id',args.run_id+'/'+group,'--output',str(output)]
        if group=='human_default':command+=['--placement-strategy','lower-first','--grasp-tilt-deg','35','--grasp-yaw-deg','0',
            '--grasp-offset-z-m','0','--grasp-offset-y-m','-0.01','--transport-bottle-height-m','1.11',
            '--supported-release','--transport-tilt-cap-deg','25']
        if args.resume and (output/'manifest.json').exists():command+=['--resume']
        batches.append((group,command))
    return batches


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--run-id',required=True)
    p.add_argument('--ids',nargs='+')
    p.add_argument('--observer-config',type=Path,default=Path('configs/safety_observer_arena.json'))
    p.add_argument('--seed',type=int,default=42)
    p.add_argument('--episodes-per-id',type=int,default=1)
    p.add_argument('--max-attempts',type=int,default=3)
    p.add_argument('--preview-per-id',type=int,default=0)
    p.add_argument('--num-envs',type=int,default=4)
    p.add_argument('--resume',action='store_true')
    p.add_argument('--dry-run',action='store_true')
    args=p.parse_args()
    if not 1<=args.episodes_per_id<=args.max_attempts:p.error('Require 1 <= episodes-per-id <= max-attempts')
    if args.num_envs<1:p.error('num-envs must be positive')
    args.output=(ROOT/args.output).resolve()
    load_config(args.observer_config)
    jobs=commands(args)
    if args.dry_run:
        import shlex
        for group,command in jobs:print(group,shlex.join(command))
        return 0
    args.output.mkdir(parents=True,exist_ok=True)
    results=[]
    for group,command in jobs:
        with (args.output/(group+'.log')).open('a') as log:
            process=subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
        results.append(dict(group=group,returncode=process.returncode,command=command))
        (args.output/'orchestration.json').write_text(json.dumps(results,indent=2)+'\n')
        if process.returncode not in (0,1):return process.returncode
    return 0 if all(r['returncode']==0 for r in results) else 1
if __name__=='__main__':raise SystemExit(main())
