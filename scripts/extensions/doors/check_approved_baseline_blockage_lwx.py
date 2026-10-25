"""Evaluate recorded obstacle-free motion against added source geometry, offline."""
import argparse,json
from pathlib import Path
import numpy as np
from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from scripts.extensions.doors.physical_hazard_geometry import physical_hazards,physical_robot_shapes
from scripts.extensions.doors.search_avoidance_layouts import clearance
p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True);p.add_argument('--config',type=Path,required=True);p.add_argument('--variant',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();assert not a.output.exists();m=json.loads((a.baseline/'metadata.json').read_text());c=json.loads(a.config.read_text());stage,kin,w,g=reconstruct(m,c,c[a.variant]);hazard=physical_hazards(stage,w,c[a.variant]['name']);margin=w.execution_clearances[c[a.variant]['name']];records=json.loads((a.baseline/'trajectory.json').read_text());checks=[]
for i,r in enumerate(records):
 for side in ['obs','next_obs']:
  q=np.asarray(r[side]['q']);fingers={n:q[m['robot_joint_names'].index(n)] for n in w.GRIPPER_JOINT_NAMES};v=clearance(physical_robot_shapes(w.shapes(q[:7],fingers)),hazard,margin,True);checks.append(dict(action_index=i,phase=r['phase'],side=side,**v))
free=[r for r in checks if r['phase']=='pregrasp'];violations=[r for r in checks if not r['valid']];fv=[r for r in free if not r['valid']];out={'scope':'Counterfactual against source geometry at actual observed baseline states. Unsafe baseline never executed with obstacle.','baseline':str(a.baseline),'variant':a.variant,'baseline_free_pregrasp_blocked':bool(fv),'minimum':min(checks,key=lambda r:r['slack_m']),'free_minimum':min(free,key=lambda r:r['slack_m']) if free else None,'violations':violations,'samples':len(checks)};a.output.write_text(json.dumps(out,indent=2));print('BLOCKAGE',a.variant,bool(fv),out['free_minimum'],flush=True)
