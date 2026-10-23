"""Wait for baseline07, then test one explicitly diagnosed property retry."""
from pathlib import Path
import time,json,subprocess
r=Path('reports/doors/lwx_C02_C03_20261023');base=Path('datasets/doors/lwx_C02_C03_20261023/C03_base_07');sf=r/'C03_contact_retry03.json';state={'stage':'waiting_baseline07','complete':False};required=[('result.json','accepted'),('task_validation_full.json','accepted'),('task_review_audit.json','passed'),('source_contact_audit.json','passed'),('self_collision_audit.json','passed')]
if sf.exists():raise FileExistsError(sf)
def save():sf.write_text(json.dumps(state,indent=2))
try:
 save();deadline=time.monotonic()+18000
 while True:
  gates={f:(json.loads((base/f).read_text()).get(k,False) if (base/f).exists() else None) for f,k in required}
  if any(v is False for v in gates.values()):raise RuntimeError('Baseline07 rejected; retain raw evidence')
  if all(v is True for v in gates.values()):break
  if time.monotonic()>deadline:raise TimeoutError('Baseline07 not reviewed')
  time.sleep(20)
 state.update(stage='property04',baseline_gates=gates);save();cmd=['python3','-u','scripts/extensions/doors/supervise_validated_pair_lwx.py','--baseline',str(base),'--config','configs/extensions/doors/c03_variants_lwx_04.json','--variant','property','--label','C03','--attempt','04','--report-root',str(r),'--data-root','datasets/doors/lwx_C02_C03_20261023']
 with (r/'C03_property04_retry03.log').open('x') as f:subprocess.run(cmd,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,check=True)
 state.update(stage='ready_for_visual_review',complete=True);save()
except Exception as exc:state['error']=repr(exc);save();raise
