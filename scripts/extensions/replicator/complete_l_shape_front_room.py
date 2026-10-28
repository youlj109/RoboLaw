"""Complete two unfrozen candidates using endpoint-adjacent source-feature poses."""
import copy,itertools,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
import numpy as np
from scipy.spatial.transform import Rotation
import kitchen_safety.extensions.replicator.visual_tabletop_placement as placement
from scripts.extensions.replicator.adjust_l_shape_mustard_layout import *
OUT=ROOT/'configs/extensions/replicator/l_shape_mustard_adjust16';REPORT=ROOT/'reports/replicator_safety_upgrade/adjust16/l_shape_mustard'
def end_candidates(hazard,start,finish,z):
 d=finish[:2]-start[:2];unit=d/np.linalg.norm(d);normal=np.array([-unit[1],unit[0]]);rows=[]
 for fraction,offset,yaw,(feature,point) in itertools.product((0.,.05,.1,.9,.95,1.),[s*x for x in (.12,.16,.20,.24,.28) for s in (1,-1)],range(0,360,15),placement.feature_points(hazard)):
  r=Rotation.from_euler('z',yaw,degrees=True).as_matrix();anchor=start[:2]+fraction*d+offset*normal;pos=np.r_[anchor-(r@point)[:2],z]
  rows.append(dict(position=pos.tolist(),yaw_deg=yaw,feature=feature,feature_world=(r@point+pos).tolist(),task_fraction=fraction,side_offset_m=offset,semantic_score=abs(offset)+.01*abs(fraction-.5)))
 return sorted(rows,key=lambda c:c['semantic_score'])
placement.candidates=end_candidates
base=OUT/'base_layout.json';layout=json.loads(base.read_text());sc=CandidateSearch(layout);robot=InitialRobotScreen();robot.set_layout(layout);manifest=json.loads((OUT/'manifest.json').read_text())
for ident in ['property_tablet_stand','robot_narrow_opening']:
 p=OUT/(ident+'.json');config=json.loads(p.read_text());h=Hazard(config['obstacle']);selected,search=placement.repair(layout,sc,h,robot,12000)
 olde=REPORT/(ident+'.json');prior=REPORT/(ident+'_initial_search_failed.json');prior.write_bytes(olde.read_bytes());old=json.loads(prior.read_text())
 if selected:
  config['obstacle']=selected;write_json(p,config);after=hazard_geometry(layout,sc,h,robot,selected);assert after['passed'];old.update(config_path=rel(p),config_sha256=sha(p),passed=True,semantic_geometry_passed=True,source_geometry_passed=True,after=after,search=search,earlier_failed_evidence=binding(prior),refinement_source_code=binding(__file__),obstacle_pose_changed=True);write_json(olde,old)
  for row in manifest['entries']:
   if row['variant_id']==ident:row.update(config_sha256=sha(p),status='visual_candidate',semantic_geometry_passed=True,evidence_sha256=sha(olde),obstacle_pose_changed=True)
 else:
  write_json(REPORT/(ident+'_endpoint_refinement_failed.json'),dict(config_path=rel(p),config_sha256=sha(p),search=search,source_code=binding(__file__)))
 print(json.dumps(dict(id=ident,passed=bool(selected),selected=search['attempts'][-1] if selected else None,counts=search['rejection_counts'])),flush=True)
manifest['scenes']=manifest['entries'];manifest['counts']=dict(Counter(row['status'] for row in manifest['entries']));manifest['stage']='complete';write_json(OUT/'manifest.json',manifest);write_json(REPORT/'progress.json',manifest)
