"""Wait for an explicitly selected offline search, then run one bounded variant campaign."""
import argparse,json,time,subprocess
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--label',required=True);p.add_argument('--variant',required=True);p.add_argument('--attempt',required=True);p.add_argument('--search',type=Path,required=True);p.add_argument('--baseline',type=Path,required=True);p.add_argument('--config',type=Path,required=True);p.add_argument('--report-root',type=Path,required=True);p.add_argument('--data-root',type=Path,required=True);p.add_argument('--after-campaign',type=Path);a=p.parse_args();sf=a.report_root/(a.label+'_'+a.variant+'_'+a.attempt+'_search_wait01.json');state={'stage':'waiting_search','complete':False}
if sf.exists():raise FileExistsError(sf)
def save():sf.write_text(json.dumps(state,indent=2))
try:
 save();deadline=time.monotonic()+18000
 if a.after_campaign:
  while True:
   prior=json.loads(a.after_campaign.read_text())
   if prior.get('error'):break
   if prior.get('complete'):state.update(stage='prior_variant_complete_no_duplicate_trial',complete=True);save();raise SystemExit(0)
   if time.monotonic()>deadline:raise TimeoutError('Prior campaign has not completed')
   time.sleep(20)
 while True:
  d=json.loads(a.search.read_text()) if a.search.exists() else {}
  if d.get('search_complete'):break
  if time.monotonic()>deadline:raise TimeoutError('Search not completed; inspect its log')
  time.sleep(20)
 if not d.get('candidates'):raise RuntimeError('No legal blocking candidates; redesign required')
 state['stage']='campaign';save();cmd=['python3','-u','scripts/extensions/doors/supervise_assigned_variant_lwx.py']
 for key in ['label','variant','attempt','search','baseline','config','report_root','data_root']:cmd+=['--'+key.replace('_','-'),str(getattr(a,key))]
 with sf.with_suffix('.log').open('x') as f:subprocess.run(cmd,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,check=True)
 state.update(stage='campaign_complete',complete=True);save()
except Exception as exc:state['error']=repr(exc);save();raise
