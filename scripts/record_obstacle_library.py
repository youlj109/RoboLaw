"""Record library entries sequentially, using one Isaac Sim process at a time."""
import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from kitchen_safety.obstacle_library import load_assets

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--ids',nargs='+')
p.add_argument('--seconds',type=float,default=1.)
p.add_argument('--output-tag',default='obstacle_library_v1')
args=p.parse_args()
assets=load_assets()
if args.ids:
    missing=set(args.ids)-{x['id'] for x in assets}
    if missing: raise ValueError(f'Unknown obstacle IDs: {sorted(missing)}')
    assets=[x for x in assets if x['id'] in args.ids]
logs=ROOT/'reports/obstacle_library';logs.mkdir(parents=True,exist_ok=True)
results=[]
for entry in assets:
    identifier=entry['id']
    command=['bash',str(ROOT/'scripts/arena_python.sh'),'scripts/record_robot_cameras.py',
             '--variant',entry['category'],'--obstacle_id',identifier,'--device','cuda:0',
             '--seconds',str(args.seconds),'--output_tag',args.output_tag+'/'+identifier]
    print('RECORDING',identifier,flush=True)
    with (logs/(identifier+'.log')).open('w') as log:
        code=subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT).returncode
    result={'id':identifier,'exit_code':code,
            'metadata':'videos/robot_cameras/'+args.output_tag+'/'+identifier+'/metadata.json'}
    results.append(result)
    (logs/(identifier+'_run.json')).write_text(json.dumps(result,indent=2)+'\n')
    print('RECORDED',identifier,'exit',code,flush=True)
sys.exit(0 if all(x['exit_code']==0 for x in results) else 1)
