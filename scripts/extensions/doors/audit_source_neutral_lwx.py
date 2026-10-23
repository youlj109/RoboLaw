"""Audit every native mechanism substep in source-inspection neutral trials."""
import argparse,json,hashlib
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('episodes',nargs='+',type=Path);p.add_argument('--review-config',type=Path,default=Path('configs/extensions/doors/review_gates_lwx.json'));a=p.parse_args();config=json.loads(a.review_config.read_text())
for ep in a.episodes:
 out=ep/'source_neutral_audit.json'
 if out.exists():raise FileExistsError(out)
 m=json.loads((ep/'metadata.json').read_text());samples=json.loads((ep/'physics.json').read_text());native=json.loads((ep/'result.json').read_text());joints=[]
 for inv in m['mechanism_inventory']:
  for i,(name,kind) in enumerate(zip(inv['joint_names'],inv['joint_kinds'])):
   q=[s['mechanisms'][inv['path']]['q'][i] for s in samples];dq=[abs(s['mechanisms'][inv['path']]['dq'][i]) for s in samples]
   drift=max(abs(v-q[0]) for v in q);speed=max(dq);dl=config['neutral_revolute_drift_rad' if kind=='revolute' else 'neutral_prismatic_drift_m'];vl=config['neutral_revolute_speed_rad_s' if kind=='revolute' else 'neutral_prismatic_speed_m_s']
   joints.append({'path':inv['path'],'joint':name,'kind':kind,'max_drift':drift,'max_speed':speed,'drift_limit':dl,'speed_limit':vl,'passed':drift<=dl and speed<=vl})
 continuous=bool(samples) and all(b['physics_step']==s['physics_step']+1 for s,b in zip(samples,samples[1:]));duration=(samples[-1]['physics_step']-samples[0]['physics_step'])*m['physics_dt'];rgb=json.loads((ep/'rgb_quality.json').read_text())
 passed=bool(native['completed'] and native['all_robot_clearances_valid'] and not m['mechanism_mapping_issues'] and continuous and duration>=config['neutral_seconds']-m['physics_dt'] and all(j['passed'] for j in joints) and rgb['passed'])
 report={'passed':passed,'task':m['task'],'episode':str(ep),'samples':len(samples),'duration_s':duration,'continuous_steps':continuous,'joints':joints,'mapping_issues':m['mechanism_mapping_issues'],'rgb_quality_passed':rgb['passed'],'native_result':native,'review_config':str(a.review_config),'review_config_sha256':hashlib.sha256(a.review_config.read_bytes()).hexdigest(),'physics_sha256':hashlib.sha256((ep/'physics.json').read_bytes()).hexdigest(),'scope':'All native target/background joint q/dq at 120Hz from post-reset initial sample. Robot external clearances here are action-rate diagnostics; complete operational audits remain required.'}
 out.write_text(json.dumps(report,indent=2)+'\n');print(ep.name,passed,'failed joints',[j for j in joints if not j['passed']],flush=True)
