import json,numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
from scripts.extensions.doors.search_avoidance_layouts import context,clearance
from scripts.extensions.doors.physical_hazard_geometry import physical_robot_shapes
root=Path(__file__).resolve().parents[3];ep,m,records,specs,w,fixed,shapes=context('assets/extensions/doors/human_reach_elbow_up/collidable.usda');shapes=[physical_robot_shapes(s) for s in shapes];out=[]
for i,c in enumerate(json.loads((root/'reports/avoidance_trials/doors/human_adaptive_height_search.json').read_text())['candidates']):
 w.set_obstacle_pose('person',[*c['position'],*Rotation.from_euler('z',c['yaw_deg'],degrees=True).as_quat()]);hazards=[o for o in w.objects if w._matches(o[0],'person')];dist=[{'frame':j,'phase':records[j]['phase'],**clearance(ss,hazards,.02,True)} for j,ss in enumerate(shapes)];out.append({'candidate':i,'position':c['position'],'yaw':c['yaw_deg'],'minimum':min(dist,key=lambda x:x['slack_m']),'violations':[r for r in dist if not r['valid']]})
(root/'reports/avoidance_trials/doors/human_physical_candidates.json').write_text(json.dumps(out,indent=2));print(json.dumps([{k:v for k,v in r.items() if k!='violations'}|{'violations':len(r['violations'])} for r in out],indent=2))
