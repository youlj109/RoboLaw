import sys,json
from pathlib import Path
sys.path.insert(0,'/workspace')
from scripts.extensions.replicator.adjust_l_shape_mustard_layout import hazard_geometry
from kitchen_safety.extensions.replicator.scene_variant_builder import Hazard,InitialRobotScreen
from scripts.extensions.replicator.search_source_layout_candidates import CandidateSearch
from scripts.extensions.replicator.audit_source_layout_geometry import ray_z
p=Path('/workspace/configs/extensions/replicator/l_shape_mustard_adjust05/robot_sharp_equipment_housing.json');d=json.loads(p.read_text());l={k:v for k,v in d.items() if k!='obstacle'};h=Hazard(d['obstacle']);s=CandidateSearch(l);r=InitialRobotScreen();r.set_layout(l)
sup=[v for v in s.meshes if v[0]==s.refs['counter_top'] or v[0].startswith(s.refs['counter_top']+'/')]
for xy,yaw in [((-.112-1,1.6866),90),((.24,1.94),315),((.32,1.85),270),((.14,1.95),0),((.14,1.4),180),((-.98,1.85),45),((-.95,1.6),90)]:
 o={**d['obstacle'],'position':[*xy,.8986624325700037],'yaw_deg':yaw};out=hazard_geometry(l,s,h,r,o);sh,fp=h.at(o['position'],yaw);print(json.dumps({'xy':xy,'yaw':yaw,'result':out,'foot':[(p.tolist(),ray_z(sup,p)) for p in fp]},default=str))
