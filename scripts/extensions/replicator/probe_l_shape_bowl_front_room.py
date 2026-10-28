"""CPU candidate exploration only; freezes no replacement base or manifest."""
import copy,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
import numpy as np
from scipy.spatial.transform import Rotation
from scripts.extensions.replicator.adjust_l_shape_mustard_layout import *
from scripts.extensions.replicator.refine_l_shape_housing_goal import closest_segments
OUT=ROOT/'reports/replicator_safety_upgrade/adjust15/l_shape_mustard/base_candidates';OUT.mkdir(parents=True,exist_ok=True)
old=json.loads((ROOT/'configs/extensions/replicator/l_shape_mustard_adjust05/base_layout.json').read_text())
hc=json.loads((ROOT/'configs/extensions/replicator/l_shape_mustard_adjust08/robot_sharp_equipment_housing.json').read_text());hz=Hazard(hc['obstacle']);rob=InitialRobotScreen()
part=next(r for r in hz.rows if r[0].endswith('/LeadingEdge'));lo,hi=part[4:];edge=np.array([[lo[0],lo[1],(lo[2]+hi[2])/2],[hi[0],lo[1],(lo[2]+hi[2])/2]])
rows=[]
for dy in [-.06,-.08,-.10,-.12,-.14]:
 d=copy.deepcopy(old);d['objects']['bowl']['position'][1]+=dy;p=OUT/f'bowl_dy_{int(abs(dy)*1000):03d}mm.json';write_json(p,d)
 rels=check_layout(d);geo=diagnose(p);row=dict(config=binding(p),bowl_additional_delta_m=[0,dy,0],relations_passed=rels['initial_source_relations_passed'],source_geometry_passed=geo['initial_geometry_passed'])
 write_json(p.with_name(p.stem+'_relations.json'),rels);write_json(p.with_name(p.stem+'_geometry.json'),geo)
 if row['relations_passed'] and row['source_geometry_passed']:
  sc=CandidateSearch(d);rob.set_layout(d);row['hazard_geometry']=hazard_geometry(d,sc,hz,rob,hc['obstacle']);r=Rotation.from_euler('z',hc['obstacle']['yaw_deg'],degrees=True).as_matrix();world=edge@r.T+hc['obstacle']['position'];start,goal,*_=corridor(d,sc);measure=closest_segments(world[0,:2],world[1,:2],start[:2],goal[:2]);v=np.array(measure['guide_point_xy'])-np.array(measure['lip_point_xy']);normal=(r@np.array([0,-1,0]))[:2];row['lip']=dict(**measure,normal=normal.tolist(),cos=float(np.dot(normal,v)/max(np.linalg.norm(v),1e-12)))
 rows.append(row);write_json(OUT/'summary.json',dict(scope='CPU exploration only; no frozen replacement base',rows=rows));print(json.dumps({k:v for k,v in row.items() if k!='hazard_geometry'}|{'hazard_passed':row.get('hazard_geometry',{}).get('passed')}),flush=True)
