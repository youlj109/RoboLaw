"""Finish paired evidence only after both episodes' independent gates actually pass."""
import argparse,json,time
from pathlib import Path
from audit_source_background import audit as background
from audit_avoidance import audit as avoidance
p=argparse.ArgumentParser();p.add_argument('baseline',type=Path);p.add_argument('episode',type=Path);p.add_argument('--timeout-seconds',type=float,default=3600);a=p.parse_args();required=('validation.json','full_world_audit.json','self_collision_audit.json','source_support_audit.json')
def ready(ep):
 for name in required:
  path=ep/name
  if not path.exists():return False
  try:r=json.loads(path.read_text())
  except json.JSONDecodeError:return False
  if r.get('passed') is not True:return False
 return True
start=time.monotonic()
while not all(ready(ep) for ep in (a.baseline,a.episode)):
 if time.monotonic()-start>a.timeout_seconds:raise SystemExit('Paired evidence pending or failed; no avoidance acceptance produced')
 time.sleep(5)
for name,func in [('source_background_audit.json',lambda:background(a.baseline,a.episode)),('avoidance_audit.json',lambda:avoidance(a.baseline,a.episode))]:
 path=a.episode/name
 if path.exists():raise FileExistsError(path)
 report=func()
 with path.open('x') as stream:json.dump(report,stream,indent=2)
 print(name,report.get('passed',report.get('accepted')),flush=True)
