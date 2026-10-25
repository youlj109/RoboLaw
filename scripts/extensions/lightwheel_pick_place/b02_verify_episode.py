"""Complete independent checks of one B02 success without recollecting it."""
import argparse,json,subprocess,sys
from pathlib import Path
from audit_source_support import audit as support
from audit_fixed_base import audit as fixed
from audit_source_background import audit as background
from audit_avoidance import audit as avoidance
p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('--baseline',type=Path,default=Path('datasets/skill_trials/lightwheel_pick_place/base_lgd_b02_20261024T072827Z'));a=p.parse_args()
r=json.load(open(a.episode/'result.json'))
if not r.get('success'):raise SystemExit('Refusing acceptance checks of an unsuccessful episode')
def write(name,func):
 target=a.episode/name
 if target.exists():raise FileExistsError(f'Preserve {target}; choose an explicit revision for rechecking')
 result=func()
 with target.open('x') as f:json.dump(result,f,indent=2)
 print(name,result.get('passed',result.get('accepted')),flush=True)
 return result
subprocess.run([sys.executable,'scripts/extensions/lightwheel_pick_place/validate.py',str(a.episode)],check=False)
write('source_support_audit.json',lambda:support(a.episode))
write('fixed_base_hazard_audit.json',lambda:fixed(a.episode))
write('source_background_audit.json',lambda:background(a.baseline,a.episode))
subprocess.run(['bash','scripts/extensions/lightwheel_pick_place/run_host.sh','audit_self_collision.py',str(a.episode)],check=False)
result=write('avoidance_audit.json',lambda:avoidance(a.baseline,a.episode))
raise SystemExit(0 if result.get('accepted') else 2)
