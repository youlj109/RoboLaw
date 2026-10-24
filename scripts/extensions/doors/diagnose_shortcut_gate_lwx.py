"""Inspect original and shortened paths behind an offline final shortcut rejection."""
import argparse,json,hashlib,random
from pathlib import Path
import numpy as np,torch
from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from kitchen_safety.extensions.doors.task_planner import TaskPlanner
from kitchen_safety.extensions.doors.pregrasp_shortcuts_lwx import metrics,shortcut
p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True);p.add_argument('--config',type=Path,required=True);p.add_argument('--paired',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
if a.output.exists():p.error('Evidence exists')
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
m=json.loads((a.baseline/'metadata.json').read_text());cp=m['controller_parameters'];c=json.loads(a.config.read_text());d=json.loads(a.paired.read_text())
if d['baseline_sha256']!=sha(a.baseline/'trajectory.json') or d['config_sha256']!=sha(a.config) or d['controller_parameters']!=cp:raise ValueError('Provenance mismatch')
pair=d['pairs'][0];seed=pair['seed'];random.seed(seed);np.random.seed(seed);torch.manual_seed(seed)
stage,kin,w,g=reconstruct(m,c,c[d['variant']]);margin=cp.get('ik_limit_margin_rad',0.);kin.limits[:,0]+=margin;kin.limits[:,1]-=margin
planner=TaskPlanner(w,kin,dt=m['dt'],seed=seed,max_joint_speed=cp['max_joint_speed'],hazard_extra_padding_m=cp.get('planner_hazard_extra_padding_m',0.),pregrasp_shortcut_policy='none')
report={'scope':'Offline diagnosis only. The same MotionGen path is exposed before the original bounded_chords_v1 adapter so its original and final dense gates can be inspected; no command is executed or safety gate bypassed in production.','physics_replay':False,'unsafe_path_executed':False,'baseline':str(a.baseline),'config':str(a.config),'config_sha256':sha(a.config),'paired':str(a.paired),'paired_sha256':sha(a.paired),'implementation_sha256':sha(__file__),'seed':seed,'stored_rejection':pair['on']['failures'],'complete':False}
a.output.write_text(json.dumps(report,indent=2))
try:
 original=np.asarray(planner.plan_pose(np.asarray(pair['off']['initial_q']),np.asarray(pair['off']['target_pose']),phase='door_pregrasp')['positions']);validate=lambda path:planner.validate_path(path,phase='door_pregrasp',grip_binary=0.,resolution=.005)
 report['original_validation']=validate(original);report['original_positions']=original.tolist();report['original_metrics']=metrics(original,kin);path,trials=shortcut(original,kin,validate);report['final_validation']=validate(path);report['final_positions']=path.tolist();report['final_metrics']=metrics(path,kin);report['trials']=trials
except Exception as exc:report['diagnostic_error']=repr(exc)
report['complete']=True;a.output.write_text(json.dumps(report,indent=2)+'\n');print('SHORTCUT_GATE_DIAG',report.get('original_validation'),report.get('final_validation'),report.get('diagnostic_error'),flush=True)
