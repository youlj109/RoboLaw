"""Complete independent B03 checks; pending evidence cannot be accepted."""
import argparse,json,subprocess,sys
from pathlib import Path
from audit_fixed_base import audit as fixed
from audit_source_background import audit as background
from audit_avoidance import audit as avoidance
p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('--baseline',type=Path);a=p.parse_args();r=json.loads((a.episode/'result.json').read_text())
if not r.get('success'):raise SystemExit('Original full task did not succeed; keep the failed trial')
if 'offline' in r.get('full_world_audit_schedule','') and not (a.episode/'full_world_audit.before_deferred_recheck.json').exists():
 subprocess.run([sys.executable,'scripts/extensions/lightwheel_pick_place/audit_deferred_world.py',str(a.episode),'--workers','4'],check=True)
subprocess.run([sys.executable,'scripts/extensions/lightwheel_pick_place/validate.py',str(a.episode)],check=False)

def write(name,func):
 target=a.episode/name
 report=func()
 if target.exists():
  if json.loads(target.read_text())!=report:
   from audit_source_support import save_report
   sidecar=save_report(report,target)
   raise RuntimeError('Changed independent report preserved at '+str(sidecar)+'; explicit review required')
 else:
  with target.open('x') as f:json.dump(report,f,indent=2)
 print(name,report.get('passed',report.get('accepted')),flush=True)

if r['variant']!='base':
 write('fixed_base_hazard_audit.json',lambda:fixed(a.episode))
 if a.baseline:write('source_background_audit.json',lambda:background(a.baseline,a.episode))
subprocess.run(['bash','scripts/extensions/lightwheel_pick_place/run_host.sh','audit_self_collision.py',str(a.episode)],check=False)
if r['variant']!='base' and a.baseline:
 import time
 deadline=time.monotonic()+3600
 names=('validation.json','full_world_audit.json','self_collision_audit.json','source_support_audit.json')
 while True:
  try:baseline_ready=all((a.baseline/n).exists() and json.loads((a.baseline/n).read_text()).get('passed') is True for n in names)
  except json.JSONDecodeError:baseline_ready=False
  if baseline_ready:break
  if time.monotonic()>deadline:raise SystemExit('Baseline independent gates not passed; paired acceptance remains pending')
  time.sleep(5)
 write('avoidance_audit.json',lambda:avoidance(a.baseline,a.episode))
checks={n:json.loads((a.episode/n).read_text()).get('passed') if (a.episode/n).exists() else None for n in ('validation.json','full_world_audit.json','source_support_audit.json','self_collision_audit.json')}
print('B03_EPISODE_CHECKS',json.dumps(checks),flush=True)
raise SystemExit(0 if all(v is True for v in checks.values()) else 2)
