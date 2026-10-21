"""Launch isolated Arena workers and retain both successful and failed attempts."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

p=argparse.ArgumentParser()
p.add_argument('--variants',nargs='+',choices=['human','property','robot','base'],default=['human','property','robot'])
p.add_argument('--episodes',type=int,default=1,help='Requested successful episodes per variant')
p.add_argument('--max-attempts',type=int,default=3,help='Attempts per requested episode')
p.add_argument('--seed',type=int,default=42)
p.add_argument('--planner',choices=['fcl','curobo'],default='curobo')
p.add_argument('--jitter',type=float,default=0.0)
p.add_argument('--video',action='store_true',help='Save all three RGB cameras, MP4 previews, and JPEG observations')
p.add_argument('--output',default='datasets/kitchen_auto/batch')
args=p.parse_args()
root=Path(__file__).resolve().parents[1]
out=(root/args.output).resolve();out.mkdir(parents=True,exist_ok=True)
manifest={'requested_per_variant':args.episodes,'variants':args.variants,'planner':args.planner,'attempts':[]}
for variant in args.variants:
    successes=0
    for attempt in range(args.episodes*args.max_attempts):
        if successes>=args.episodes:break
        seed=args.seed+attempt
        destination=out/variant/f'attempt_{attempt:04d}_seed_{seed}'
        if destination.exists():raise FileExistsError(f'Refusing to overwrite an existing collection: {destination}')
        destination.mkdir(parents=True)
        cmd=[sys.executable,str(root/'scripts/collect_kitchen.py'),'--variant',variant,'--seed',str(seed),
             '--jitter',str(args.jitter),'--planner',args.planner,'--output',str(destination)]
        if args.video:cmd+=['--video']
        with (destination/'run.log').open('w') as log:
            code=subprocess.run(cmd,cwd=root,stdout=log,stderr=subprocess.STDOUT).returncode
        result_path=destination/'result.json'
        result=json.loads(result_path.read_text()) if result_path.exists() else {'success':False,'error':f'worker exit {code}; see run.log'}
        success=code==0 and result.get('success',False)
        validation_code=None
        if success and args.video:
            # A physically successful grasp can still have empty RTX images.
            # Check the saved observations and videos before accepting RGB data.
            with (destination/'validation.log').open('w') as log:
                validation_code=subprocess.run(
                    [sys.executable,str(root/'scripts/validate_collection.py'),str(destination),
                     '--report',str(destination/'validation.json')],
                    cwd=root,stdout=log,stderr=subprocess.STDOUT).returncode
            success=validation_code==0
        successes+=int(success)
        manifest['attempts'].append({'variant':variant,'seed':seed,'success':success,'path':str(destination.relative_to(out)),
                                     'exit_code':code,'data_validation_exit_code':validation_code})
        (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
        print(variant,'attempt',attempt,'success',success,'collected',successes,flush=True)
manifest['complete']=all(sum(a['success'] and a['variant']==v for a in manifest['attempts'])>=args.episodes for v in args.variants)
(out/'manifest.json').write_text(json.dumps(manifest,indent=2))
sys.exit(0 if manifest['complete'] else 1)
