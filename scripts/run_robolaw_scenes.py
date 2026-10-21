#!/usr/bin/env python3
"""Collect the ten fixed obstacle scenes and/or the default human, sequentially."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shlex
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
PRESETS=json.loads((ROOT/'configs/reproduction_presets.json').read_text())
NONHUMAN=tuple(PRESETS['nonhuman10']['ids'])
HUMAN='human_default_mpfb'

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', choices=(*NONHUMAN,HUMAN), help='Run one scene; default is all 11')
    parser.add_argument('--output', type=Path, help='New output directory')
    parser.add_argument('--dry-run', action='store_true')
    args=parser.parse_args()
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    output=(args.output or ROOT/'datasets/robolaw'/stamp).resolve()
    selected=NONHUMAN if args.scene is None else (() if args.scene==HUMAN else (args.scene,))
    commands=[]
    if selected:
        commands.append([sys.executable,str(ROOT/'scripts/collect_obstacle_library.py'),
            '--ids',*selected,'--max-attempts','1','--seed',str(PRESETS['seed']),
            '--video','--output',str(output/'nonhuman')])
    if args.scene is None or args.scene==HUMAN:
        human=output/HUMAN
        commands.append([sys.executable,str(ROOT/'scripts/collect_kitchen.py'),
            '--variant','human','--planner','curobo','--video','--seed',str(PRESETS['seed']),
            '--output',str(human)])
        commands.append([sys.executable,str(ROOT/'scripts/validate_collection.py'),str(human),
            '--require-previews','--report',str(human/'validation.json')])
        commands.append([sys.executable,str(ROOT/'scripts/audit_collection_clearance.py'),str(human),
            '--report',str(human/'audit.json')])
    if args.dry_run:
        print('OUTPUT',output)
        for command in commands: print(shlex.join(command))
        return 0
    if output.exists(): parser.error(f'Output directory already exists: {output}')
    output.mkdir(parents=True)
    for command in commands:
        print('RUN',shlex.join(command),flush=True)
        code=subprocess.run(command,cwd=ROOT).returncode
        if code:
            print(f'FAILED exit={code}; inspect {output}',file=sys.stderr)
            return code
    print('COMPLETE',output,flush=True)
    return 0
if __name__=='__main__': raise SystemExit(main())
