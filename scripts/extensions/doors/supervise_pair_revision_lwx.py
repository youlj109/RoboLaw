"""Recheck an existing successful physical variant against exact recorded command IK bounds."""
import argparse,json,time,subprocess,hashlib
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True);p.add_argument('--config',type=Path);p.add_argument('--campaign',type=Path);p.add_argument('--variant',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();statepath=a.output.with_suffix('.state.json');state={'stage':'waiting_config','complete':False}
if statepath.exists() or a.output.exists():raise FileExistsError(a.output)
def save():statepath.write_text(json.dumps(state,indent=2))
try:
 save();deadline=time.monotonic()+18000;config=a.config
 while config is None:
  campaign=json.loads(a.campaign.read_text()) if a.campaign.exists() else {}
  if campaign.get('selected_candidate'):config=Path(campaign['selected_candidate']['config']);break
  if campaign.get('error'):raise RuntimeError('Candidate campaign rejected: '+campaign['error'])
  if time.monotonic()>deadline:raise TimeoutError('No selected candidate')
  time.sleep(20)
 state.update(stage='paired_revision',config=str(config));save();cmd=['docker','exec','robolaw-wenxuanli','flock','/workspace/reports/skill_trials/gpu.lock','env','PYTHONPATH=/workspace','OPENBLAS_NUM_THREADS=1','OMP_NUM_THREADS=1','/workspace/.venv-arena/bin/python','-u','scripts/extensions/doors/paired_tasks_lwx.py','--baseline',str(a.baseline),'--config',str(config),'--variant',a.variant,'--output',str(a.output)]
 with a.output.with_suffix('.log').open('x') as f:subprocess.run(cmd,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,check=True)
 report=json.loads((a.output/'report.json').read_text());m=json.loads((a.baseline/'metadata.json').read_text());ratios=[];extras=[];hits=[]
 for pair in report['pairs']:
  hits.append(any(v.get('intersecting') for v in pair['off']['risk_with_added_geometry']['violations']));ratios.append(pair['on']['path_metrics']['joint_l2_length_rad']/pair['off']['path_metrics']['joint_l2_length_rad']);extras.append(pair['on']['path_metrics']['tcp_length_m']-pair['off']['path_metrics']['tcp_length_m'])
 ok=bool(report['command_ik_margin_applied_rad']==m['controller_parameters'].get('ik_limit_margin_rad',0.) and report['stable_policy_necessity_in_tested_seeds'] and report['fixed_endpoint_rerouting_supported'] and all(hits) and max(ratios)<=1.6 and max(extras)<=.15)
 state.update(stage='reviewed',complete=True,passed=ok,report=str(a.output/'report.json'),report_sha256=hashlib.sha256((a.output/'report.json').read_bytes()).hexdigest(),all_seeds_actual_intersection=all(hits),max_joint_length_ratio=max(ratios),max_added_tcp_length_m=max(extras));save()
 if not ok:raise RuntimeError('Revised exact-policy pairing rejected')
except Exception as exc:state['error']=repr(exc);save();raise
