"""Exact minimum source-model distance along recorded pregrasp, no search/planning."""
import argparse
import json
from pathlib import Path
import numpy as np
from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from scripts.extensions.doors.physical_hazard_geometry import physical_hazards, physical_robot_shapes
from scripts.extensions.doors.search_avoidance_layouts import clearance

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--manifest',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
report=dict(scope=__doc__,complete=False,candidates=[])
for item in json.loads(a.manifest.read_text())['candidates']:
    cfg=json.loads(Path(item['config']).read_text()); spec=cfg[item['variant']]
    ep=Path(item['baseline']); m=json.loads((ep/'metadata.json').read_text())
    rows=json.loads((ep/'trajectory.json').read_text())
    stage,kin,w,g=reconstruct(m,cfg,spec)
    hz=physical_hazards(stage,w,spec['name']); best=None; violations=[]
    for i,row in enumerate(rows):
        if row['phase']!='pregrasp':continue
        q=row['next_obs']['q']
        fingers={n:q[m['robot_joint_names'].index(n)] for n in w.GRIPPER_JOINT_NAMES}
        shapes=physical_robot_shapes(w.shapes(np.asarray(q[:7]),fingers))
        value=clearance(shapes,hz,w.execution_clearances[spec['name']],True)
        if best is None or value['distance_m']<best['distance_m']:
            best=dict(action=i,**value)
        if not value['valid']:violations.append(i)
    item_result=dict(id=item['id'],variant=item['variant'],minimum=best,
                     original_planning_clearance_m=w.clearances[spec['name']],
                     original_execution_clearance_m=w.execution_clearances[spec['name']],
                     execution_margin_violation_actions=violations)
    report['candidates'].append(item_result)
    a.output.write_text(json.dumps(report,indent=2)+'\n')
    print('BASELINE_PROXIMITY',item['id'],best,'violations',len(violations),flush=True)
report['complete']=True
a.output.write_text(json.dumps(report,indent=2)+'\n')
