"""After an independently accepted baseline, search only real source obstructions."""
import argparse,json,subprocess,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('label',choices=['C01','C02','C03','C04','C05','C06','C07','C08']);p.add_argument('--attempt',default='01');p.add_argument('--run-label',default='01');p.add_argument('--report-root',type=Path,default=Path('reports/doors/lwx_C04_C08_20261024'));p.add_argument('--data-root',type=Path,default=Path('datasets/doors/lwx_C04_C08_20261024'));p.add_argument('--variants',default='property,robot,human');p.add_argument('--baseline-driver',type=Path);p.add_argument('--human-asset');p.add_argument('--search-version',choices=['15','19','21','24','25','26'],default='15');p.add_argument('--candidate-list',type=Path);p.add_argument('--hazard-asset');p.add_argument('--mid-stride',type=int);p.add_argument('--screen-approach-proxies',action='store_true');p.add_argument('--support-offset-xy',nargs=2,type=float);p.add_argument('--allow-endpoint-posture-adaptation',action='store_true');p.add_argument('--heights');p.add_argument('--support-width-m',type=float);a=p.parse_args();label=a.label
if a.allow_endpoint_posture_adaptation and a.search_version not in ['24','25','26']:p.error('Posture adaptation discovery requires v24, v25 or v26')
if (a.candidate_list or a.hazard_asset) and len(a.variants.split(','))!=1:p.error('Explicit source/grid needs exactly one variant')
if a.mid_stride is not None and not 1<=a.mid_stride<=20:p.error('Discovery stride outside [1,20]')
r=a.report_root;d=a.data_root;base=d/(label+'_base_'+a.attempt);statepath=r/(label+'_hazard_search_driver'+a.run_label+'.json')
if statepath.exists():raise FileExistsError(statepath)
state={'stage':'waiting_for_baseline_audits','complete':False,'variants':{}};deadline=time.monotonic()+18000
required=[('result.json','accepted'),('task_validation_full.json','accepted'),('task_review_audit.json','passed'),('source_contact_audit.json','passed'),('self_collision_audit.json','passed')]
def save():statepath.write_text(json.dumps(state,indent=2)+'\n')
try:
 save()
 while True:
  gates={f:(json.loads((base/f).read_text()).get(key,False) if (base/f).exists() else None) for f,key in required}
  if a.baseline_driver and a.baseline_driver.exists():
   driver=json.loads(a.baseline_driver.read_text())
   if driver.get('error'):raise RuntimeError('Baseline driver failed: '+driver['error'])
  if gates['result.json'] is False:raise RuntimeError('Collector rejected baseline; retain raw failure')
  if all(v is not None for v in gates.values()):
   state['baseline_gates']=gates;save()
   if not all(gates.values()):raise RuntimeError('Baseline rejected; inspect before hazard design')
   break
  if time.monotonic()>deadline:raise TimeoutError('Baseline audits not ready')
  time.sleep(20)
 baseline_config=json.loads((base/'metadata.json').read_text())['variant_config'];state['baseline_config']=baseline_config;save()
 for variant in a.variants.split(','):
  state['stage']='search_'+variant;save();out=r/(label+'_'+variant+'_search_'+a.run_label+'.json');cmd=['docker','exec','robolaw-wenxuanli','env','PYTHONPATH=/workspace','OPENBLAS_NUM_THREADS=1','OMP_NUM_THREADS=1','/workspace/.venv-arena/bin/python','-u','scripts/extensions/doors/search_task_hazards_v'+a.search_version+'_lwx.py','--baseline',str(base),'--config',baseline_config,'--variant',variant,'--output',str(out)]+(['--pedestal'] if variant!='human' else [])
  if variant=='human' and a.human_asset:cmd+=['--human-asset',a.human_asset]
  if a.allow_endpoint_posture_adaptation:cmd+=['--allow-endpoint-posture-adaptation']
  if a.candidate_list:cmd+=['--candidate-list',str(a.candidate_list)]
  if a.hazard_asset:cmd+=['--hazard-asset',a.hazard_asset]
  if a.heights:cmd+=['--heights',a.heights]
  if a.support_width_m is not None:cmd+=['--support-width-m',str(a.support_width_m)]
  if a.mid_stride is not None:cmd+=['--mid-stride',str(a.mid_stride)]
  if a.screen_approach_proxies:cmd+=['--screen-approach-proxies']
  if a.support_offset_xy is not None:cmd+=['--support-offset-xy',*map(str,a.support_offset_xy)]
  with out.with_suffix('.log').open('x') as f:code=subprocess.run(cmd,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT).returncode
  report=json.loads(out.read_text()) if out.exists() else {};state['variants'][variant]={'exit_code':code,'report':str(out),'candidates':len(report.get('candidates',[])),'stats':report.get('stats',{})};save()
 state['complete']=True;state['stage']='candidates_require_paired_planning';save()
except Exception as exc:state['error']=repr(exc);save();raise
