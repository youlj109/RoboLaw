"""CPU discovery ranking using recorded successful baseline; no planner or physics."""
import argparse,json
from pathlib import Path
import numpy as np
from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from scripts.extensions.doors.physical_hazard_geometry import physical_hazards,physical_robot_shapes
from scripts.extensions.doors.search_avoidance_layouts import clearance
p=argparse.ArgumentParser();p.add_argument('--manifest',type=Path,required=True);p.add_argument('--static',type=Path,required=True);p.add_argument('--baseline',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();assert not a.output.exists();mfest=json.loads(a.manifest.read_text());static=json.loads(a.static.read_text());allowed={x['id'] for x in static['candidates'] if x['static_rejections_absent']};m=json.loads((a.baseline/'metadata.json').read_text());rows=json.loads((a.baseline/'trajectory.json').read_text());free=[r for r in rows if r['phase']=='pregrasp'];sampled=free[::2]+[free[-1]];report={'scope':'CPU source-geometry screening of actual baseline at subsampled recorded states; only ranking, not acceptance.','baseline':str(a.baseline),'candidates':[],'complete':False};robot_cache=None
for item in mfest['candidates']:
 if item['id'] not in allowed:continue
 c=json.loads(Path(item['config']).read_text());spec=c[item['variant']];s,kin,w,g=reconstruct(m,c,spec);hazard=physical_hazards(s,w,spec['name']);margin=w.execution_clearances[spec['name']]
 if robot_cache is None:
  robot_cache=[]
  for row in sampled:
   q=np.asarray(row['next_obs']['q']);f={n:q[m['robot_joint_names'].index(n)] for n in w.GRIPPER_JOINT_NAMES};robot_cache.append(physical_robot_shapes(w.shapes(q[:7],f)))
 checks=[clearance(shapes,hazard,margin,True) for shapes in robot_cache];endpoint=checks[-1];violations=[x for x in checks if not x['valid']];intersections=[x for x in checks if x.get('intersecting')];entry={**item,'margin_m':margin,'free_phase_samples':len(checks),'violations':len(violations),'intersections':len(intersections),'minimum':min(checks,key=lambda x:x['slack_m']),'baseline_endpoint_clear':endpoint['valid']};report['candidates'].append(entry);a.output.write_text(json.dumps(report,indent=2));print(item['id'],entry['violations'],entry['intersections'],entry['baseline_endpoint_clear'],flush=True)
report['complete']=True;a.output.write_text(json.dumps(report,indent=2))
