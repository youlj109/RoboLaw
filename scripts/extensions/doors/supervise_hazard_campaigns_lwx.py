"""Continue each completed source search with one bounded physical campaign."""
import argparse,json,subprocess,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--label',required=True);p.add_argument('--baseline',required=True);p.add_argument('--search-driver',type=Path,required=True);p.add_argument('--report-root',type=Path,required=True);p.add_argument('--data-root',type=Path,required=True);p.add_argument('--attempt',required=True);a=p.parse_args()
sf=a.report_root/(a.label+'_hazard_campaigns_'+a.attempt+'.json')
if sf.exists():raise FileExistsError(sf)
s={'stage':'waiting_completed_search_driver','complete':False,'campaigns':{}}
def save():sf.write_text(json.dumps(s,indent=2)+'\n')
try:
 save();deadline=time.monotonic()+18000
 while True:
  d=json.loads(a.search_driver.read_text()) if a.search_driver.exists() else {}
  if d.get('error'):raise RuntimeError('Search driver rejected: '+d['error'])
  if d.get('complete'):break
  if time.monotonic()>deadline:raise TimeoutError('Search driver not completed')
  time.sleep(20)
 for variant,entry in d['variants'].items():
  search=Path(entry['report']);j=json.loads(search.read_text())
  if entry['exit_code'] or not j.get('candidates'):
   s['campaigns'][variant]={'stage':'no_legal_candidates','search':str(search)};save();continue
  while True:
   prior=[]
   for path in a.report_root.glob(a.label+'_'+variant+'_*_campaign01.json'):
    prior.append((path,json.loads(path.read_text())))
   accepted=[str(path) for path,j in prior if j.get('complete') and j.get('stage')=='ready_for_visual_review']
   if accepted:break
   active=[str(path) for path,j in prior if not j.get('complete') and not j.get('error')]
   if not active:break
   if time.monotonic()>deadline:raise TimeoutError('Earlier variant campaign still active: '+str(active))
   s['stage']='waiting_prior_'+variant;s['prior_campaigns']=active;save();time.sleep(20)
  if accepted:
   s['campaigns'][variant]={'stage':'prior_variant_accepted_no_duplicate_collection','prior':accepted};save();continue
  cmd=['python3','-u','scripts/extensions/doors/supervise_assigned_variant_lwx.py','--label',a.label,'--variant',variant,'--attempt',a.attempt,'--baseline',a.baseline,'--search',str(search),'--report-root',str(a.report_root),'--data-root',str(a.data_root)]
  log=a.report_root/(a.label+'_'+variant+'_'+a.attempt+'_auto_launch01.log')
  with log.open('x') as f:proc=subprocess.Popen(cmd,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
  s['campaigns'][variant]={'stage':'launched','pid':proc.pid,'search':str(search),'log':str(log)};save()
 s.update(stage='bounded_campaigns_launched',complete=True);save()
except Exception as exc:s['error']=repr(exc);save();raise
