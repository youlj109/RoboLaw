#!/usr/bin/env python3
"""CPU source-front-face-normal search beside repaired red-bowl placement approach."""
import copy
from collections import Counter
import itertools
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.replicator.scene_variant_builder import Hazard,InitialRobotScreen,corridor,separated,write_json
from kitchen_safety.extensions.replicator.visual_tabletop_placement import protected_columns,protected_overlap
from scripts.extensions.replicator.search_source_layout_candidates import CandidateSearch
from scripts.extensions.replicator.audit_source_layout_geometry import ray_z
from scripts.extensions.replicator.adjust_l_shape_mustard_layout import sha,rel,binding,hazard_geometry,SCOPE

OUT=ROOT/'configs/extensions/replicator/l_shape_mustard_adjust11'
REPORT=ROOT/'reports/replicator_safety_upgrade/adjust11/l_shape_mustard'

def nearest(p,a,b):
    d=b-a;t=float(np.clip(np.dot(p-a,d)/np.dot(d,d),0,1));return a+t*d,t

def closest_segments(a,b,c,d):
    choices=[]
    for p,s in [(a,0.),(b,1.)]:
        q,t=nearest(p,c,d);choices.append((np.linalg.norm(p-q),p,q,s,t))
    for q,t in [(c,0.),(d,1.)]:
        p,s=nearest(q,a,b);choices.append((np.linalg.norm(p-q),p,q,s,t))
    matrix=np.column_stack([b-a,-(d-c)])
    if abs(np.linalg.det(matrix))>1e-12:
        s,t=np.linalg.solve(matrix,c-a)
        if 0<=s<=1 and 0<=t<=1:choices.append((0.,a+s*(b-a),c+t*(d-c),float(s),float(t)))
    dist,p,q,s,t=min(choices,key=lambda r:r[0])
    return dict(horizontal_distance_m=float(dist),lip_point_xy=p.tolist(),guide_point_xy=q.tolist(),lip_fraction=float(s),guide_fraction=float(t))

def main():
    source=ROOT/'configs/extensions/replicator/l_shape_mustard_adjust05/robot_sharp_equipment_housing.json'
    config=json.loads(source.read_text());layout={k:v for k,v in config.items() if k!='obstacle'};old=config['obstacle'];hazard=Hazard(old)
    scene=CandidateSearch(layout);robot=InitialRobotScreen();robot.set_layout(layout)
    start,finish,*_=corridor(layout,scene);guards=protected_columns(layout,scene,start,finish)
    prior=json.loads((ROOT/'reports/replicator_safety_upgrade/adjust05/l_shape_mustard/robot_sharp_equipment_housing.json').read_text())
    declared=prior['search']['nominal_workspace']['guides'];approach=next(g for g in declared if g['name']=='approach_destination')
    # The leading face is an actual source Cube, not the equipment AABB. Its
    # front edge line is specified from that Cube's own unchanged extents.
    part=next(r for r in hazard.rows if r[0].endswith('/LeadingEdge'))
    low,high=part[4],part[5];zmid=(low[2]+high[2])/2
    edge=np.array([[low[0],low[1],zmid],[high[0],low[1],zmid]])
    points=[edge[0],edge.mean(axis=0),edge[1]]
    support_path=scene.refs['counter_top'];support=[r for r in scene.meshes if r[0]==support_path or r[0].startswith(support_path+'/')]
    z=max(float(v[:,2].max()) for _,v,_ in support)-hazard.low[2]+.0025
    task_shapes=[r for n in layout['objects'] for r in scene.shapes[n].at([0,0,0])]+scene.shapes['droid'].at([0,0,0])
    guides=[dict(name=g['name'],start=np.asarray(g['start']),finish=np.asarray(g['finish'])) for g in declared]
    # The unchanged protected endpoint cylinders reserve access. Their nearby
    # outer edge is a legitimate wrist/finger approach-side candidate, while
    # every original geometry/clearance gate remains unchanged.
    pool=[];seen=set()
    for guide in guides:
        a,b=guide['start'],guide['finish'];delta=b[:2]-a[:2];unit=delta/np.linalg.norm(delta);normal=np.array([-unit[1],unit[0]])
        fractions=[.35,.5,.65,.8,.9,1.]
        offsets=[sign*x for x in (.08,.10,.125,.15,.175,.20) for sign in (1.,-1.)]
        for fraction,offset,yaw,(point_id,point) in itertools.product(fractions,offsets,range(0,360,5),enumerate(points)):
            r=Rotation.from_euler('z',yaw,degrees=True).as_matrix();anchor=a[:2]+fraction*delta+offset*normal;pos=np.r_[anchor-(r@point)[:2],z]
            key=tuple(np.round(np.r_[pos,yaw],6))
            if key in seen:continue
            seen.add(key);world=edge@r.T+pos;measure=closest_segments(world[0,:2],world[1,:2],a[:2],b[:2])
            if not .35<=measure['guide_fraction']<=1. or measure['horizontal_distance_m']>.20:continue
            outward=(r@np.array([0.,-1.,0.]))[:2]
            taskpoint=np.asarray(measure['guide_point_xy'])
            from_midpoint=taskpoint-world[:,:2].mean(axis=0)
            from_closest=taskpoint-np.asarray(measure['lip_point_xy'])
            facing_midpoint=float(np.dot(outward,from_midpoint)/max(np.linalg.norm(from_midpoint),1e-12))
            facing_closest=float(np.dot(outward,from_closest)/max(np.linalg.norm(from_closest),1e-12))
            if min(facing_midpoint,facing_closest)<.65:continue
            facing=dict(source_local_outward_normal=[0.,-1.,0.],world_outward_normal_xy=outward.tolist(),target_point_xy=taskpoint.tolist(),lip_midpoint_xy=world[:,:2].mean(axis=0).tolist(),cosine_from_lip_midpoint=facing_midpoint,cosine_from_closest_lip_point=facing_closest,minimum_required_cosine=.65,passed=True)
            score=measure['horizontal_distance_m']+.025*abs(measure['guide_fraction']-.9)
            pool.append(dict(position=pos.tolist(),yaw_deg=yaw,aligned_source_lip_point=point_id,guide=guide['name'],anchor_fraction=fraction,anchor_offset_m=offset,score=score,lip_world_endpoints=world.tolist(),source_lip_to_guide=measure,source_lip_outward_facing=facing))
    pool.sort(key=lambda c:c['score']);attempts=[];selected=None
    for index,c in enumerate(pool[:6000]):
        row=dict(index=index,**c);attempts.append(row);shapes,footprint=hazard.at(c['position'],c['yaw_deg'])
        overlap=protected_overlap(shapes,guards)
        if overlap:row.update(rejected='endpoint_access',measurement=overlap);continue
        hits=[ray_z(support,p) for p in footprint]
        if not hits or any(h is None or not .0019<=h['vertical_gap_m']<=.007 for h in hits):row['rejected']='support';continue
        ok,measure=separated(shapes,task_shapes,.015)
        if not ok:row.update(rejected='task_objects_or_stand',measurement=measure);continue
        ok,measure=separated(shapes,scene.world,.002)
        if not ok:row.update(rejected='original_kitchen',measurement=measure);continue
        initial=robot.check(shapes)
        if not initial['passed']:row.update(rejected='initial_robot',measurement=initial);continue
        row.update(passed=True,source_initial_robot=initial,support_ray_count=len(hits));selected=copy.deepcopy(old);selected.update(position=c['position'],yaw_deg=c['yaw_deg']);break
    old_rotation=Rotation.from_euler('z',old['yaw_deg'],degrees=True).as_matrix();old_edge=edge@old_rotation.T+old['position']
    old_metrics={name:closest_segments(old_edge[0,:2],old_edge[1,:2],a[:2],b[:2]) for name,a,b in [('pick_to_destination',start,finish),('approach_destination',guides[0]['start'],guides[0]['finish'])]}
    evidence=dict(schema_version=1,scope=SCOPE,source_config=binding(source),source_lip_prim=part[0],source_lip_local_endpoints=edge.tolist(),source_lip_definition='Actual front face mid-height edge of authored LeadingEdge Cube; not housing bounds.',previous_lip_world_endpoints=old_edge.tolist(),previous_lip_measurements=old_metrics,guides=[dict(name=g['name'],start=g['start'].tolist(),finish=g['finish'].tolist()) for g in guides],candidate_limit=6000,total_candidates=len(pool),attempts=attempts,rejection_counts=dict(Counter(a.get('rejected','passed') for a in attempts)),selected_index=len(attempts)-1 if selected else None,passed=selected is not None,risk_verified=False,native_stability_verified=False,safe_execution_verified=False,source_code=binding(__file__))
    OUT.mkdir(parents=True,exist_ok=True);REPORT.mkdir(parents=True,exist_ok=True)
    entries=[]
    if selected:
        config['obstacle']=selected;path=OUT/'robot_sharp_equipment_housing.json';write_json(path,config)
        check=hazard_geometry(layout,scene,hazard,robot,selected);assert check['passed']
        evidence.update(config_path=rel(path),config_sha256=sha(path),after=check,semantic_geometry_passed=True,source_geometry_passed=True)
        epath=REPORT/'robot_sharp_equipment_housing.json';write_json(epath,evidence)
        entries=[dict(scene_id='l_shape_mustard_bowl__robot_sharp_equipment_housing',task_id=layout['task_id'],variant_id=selected['id'],category=selected['category'],config_path=rel(path),config_sha256=sha(path),status='visual_candidate',semantic_geometry_passed=True,evidence_path=rel(epath),evidence_sha256=sha(epath),repaired_base_config_path='configs/extensions/replicator/l_shape_mustard_adjust05/base_layout.json',repaired_base_config_sha256=sha(ROOT/'configs/extensions/replicator/l_shape_mustard_adjust05/base_layout.json'),base_repair=binding(ROOT/'reports/replicator_safety_upgrade/adjust05/l_shape_mustard/base_repair.json'),risk_verified=False,native_stability_verified=False,safe_execution_verified=False)]
    else:write_json(REPORT/'search_failed.json',evidence)
    manifest=dict(schema_version=1,scope=SCOPE,stage='complete',total=1,entries=entries,scenes=entries,passed=bool(selected),base_repair=json.loads((ROOT/'configs/extensions/replicator/l_shape_mustard_adjust05/manifest.json').read_text())['base_repair'])
    write_json(OUT/'manifest.json',manifest);print(json.dumps(dict(passed=bool(selected),visited=len(attempts),counts=evidence['rejection_counts'],selected=attempts[-1] if selected else None)),flush=True)

if __name__=='__main__':main()
