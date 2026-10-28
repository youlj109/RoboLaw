#!/usr/bin/env python3
"""Bounded 5mm/5-degree refinement of the source-valid L mustard lip."""
import copy,itertools,json
from collections import Counter
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
import numpy as np
from scipy.spatial.transform import Rotation
from scripts.extensions.replicator.refine_l_shape_housing_goal import closest_segments
from scripts.extensions.replicator.adjust_l_shape_mustard_layout import hazard_geometry,sha,rel,binding,SCOPE
from kitchen_safety.extensions.replicator.scene_variant_builder import Hazard,InitialRobotScreen,corridor,separated,write_json
from kitchen_safety.extensions.replicator.visual_tabletop_placement import protected_columns,protected_overlap
from scripts.extensions.replicator.search_source_layout_candidates import CandidateSearch
from scripts.extensions.replicator.audit_source_layout_geometry import ray_z

def main():
    source=ROOT/'configs/extensions/replicator/l_shape_mustard_adjust07/robot_sharp_equipment_housing.json'
    previous=ROOT/'reports/replicator_safety_upgrade/adjust07/l_shape_mustard/robot_sharp_equipment_housing.json'
    config=json.loads(source.read_text());old=config['obstacle'];layout={k:v for k,v in config.items() if k!='obstacle'}
    prior=json.loads(previous.read_text());edge=np.asarray(prior['source_lip_local_endpoints']);guide=prior['guide'];a=np.asarray(guide['start']);b=np.asarray(guide['finish'])
    scene=CandidateSearch(layout);hazard=Hazard(old);robot=InitialRobotScreen();robot.set_layout(layout)
    start,finish,*_=corridor(layout,scene);guards=protected_columns(layout,scene,start,finish)
    sp=scene.refs['counter_top'];support=[r for r in scene.meshes if r[0]==sp or r[0].startswith(sp+'/')]
    objects=[r for n in layout['objects'] for r in scene.shapes[n].at([0,0,0])]+scene.shapes['droid'].at([0,0,0])
    pool=[]
    for dx,dy,dyaw in itertools.product(np.arange(-.025,.0251,.005),np.arange(-.025,.0251,.005),range(-15,16,5)):
        pos=np.asarray(old['position'])+[dx,dy,0.];yaw=old['yaw_deg']+dyaw;r=Rotation.from_euler('z',yaw,degrees=True).as_matrix();world=edge@r.T+pos
        measure=closest_segments(world[0,:2],world[1,:2],a[:2],b[:2]);pool.append(dict(position=pos.tolist(),yaw_deg=float(yaw),lip_world_endpoints=world.tolist(),source_lip_to_guide=measure))
    pool.sort(key=lambda c:c['source_lip_to_guide']['horizontal_distance_m']);attempts=[];selected=None
    for index,c in enumerate(pool):
        row=dict(index=index,**c);attempts.append(row);shapes,footprint=hazard.at(c['position'],c['yaw_deg'])
        overlap=protected_overlap(shapes,guards)
        if overlap:row.update(rejected='endpoint_access',measurement=overlap);continue
        hits=[ray_z(support,p) for p in footprint]
        if not hits or any(h is None or not .0019<=h['vertical_gap_m']<=.007 for h in hits):row['rejected']='support';continue
        ok,measurement=separated(shapes,objects,.015)
        if not ok:row.update(rejected='task_objects_or_stand',measurement=measurement);continue
        ok,measurement=separated(shapes,scene.world,.002)
        if not ok:row.update(rejected='original_kitchen',measurement=measurement);continue
        initial=robot.check(shapes)
        if not initial['passed']:row.update(rejected='initial_robot',measurement=initial);continue
        row.update(passed=True,source_initial_robot=initial,support_ray_count=len(hits));selected=copy.deepcopy(old);selected.update(position=c['position'],yaw_deg=c['yaw_deg']);break
    assert selected is not None,'Even source coarse candidate should remain valid'
    out=ROOT/'configs/extensions/replicator/l_shape_mustard_adjust08';reports=ROOT/'reports/replicator_safety_upgrade/adjust08/l_shape_mustard';config['obstacle']=selected;cp=out/'robot_sharp_equipment_housing.json';write_json(cp,config)
    after=hazard_geometry(layout,scene,hazard,robot,selected);assert after['passed']
    evidence=dict(schema_version=1,scope=SCOPE,source_config=binding(source),coarse_search=binding(previous),config_path=rel(cp),config_sha256=sha(cp),source_lip_prim=prior['source_lip_prim'],source_lip_local_endpoints=edge.tolist(),source_lip_definition=prior['source_lip_definition'],guide=guide,search_declaration=dict(xy_range_m=.025,xy_step_m=.005,yaw_range_deg=15,yaw_step_deg=5,selection='Lowest exact source-lip segment to nominal goal-approach segment distance among the declared finite grid; source checks unchanged.'),total_candidates=len(pool),attempts=attempts,selected_index=len(attempts)-1,rejection_counts=dict(Counter(x.get('rejected','passed') for x in attempts)),after=after,passed=True,semantic_geometry_passed=True,source_geometry_passed=True,risk_verified=False,native_stability_verified=False,safe_execution_verified=False,source_code=binding(__file__))
    ep=reports/'robot_sharp_equipment_housing.json';write_json(ep,evidence)
    manifest=json.loads((ROOT/'configs/extensions/replicator/l_shape_mustard_adjust07/manifest.json').read_text());entry=manifest['entries'][0];entry.update(config_path=rel(cp),config_sha256=sha(cp),evidence_path=rel(ep),evidence_sha256=sha(ep));manifest['entries']=[entry];manifest['scenes']=[entry];write_json(out/'manifest.json',manifest)
    print(json.dumps(dict(passed=True,visited=len(attempts),selected=attempts[-1],manifest=rel(out/'manifest.json'))),flush=True)

if __name__=='__main__':main()
