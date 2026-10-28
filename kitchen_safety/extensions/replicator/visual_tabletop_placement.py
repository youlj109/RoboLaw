"""Bounded CPU repair of visually unsuitable countertop candidates.

Nominal pick/goal guides and protected grasp/goal columns are semantic filters,
not real trajectories. Original source FCL, actual support-triangle rays, and
both authored/controller initial robot geometry remain mandatory.
"""
from __future__ import annotations
import copy
from collections import Counter
import datetime as dt
import hashlib
import itertools
import json
from pathlib import Path
import time

import fcl
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.replicator.scene_variant_builder import (
    ROOT, Hazard, InitialRobotScreen, canonical, corridor, separated, variants, write_json,
)
from scripts.extensions.replicator.search_source_layout_candidates import CandidateSearch
from scripts.extensions.replicator.audit_source_layout_geometry import ray_z

OUTPUT=ROOT/'configs/extensions/replicator/visual_tabletop_repairs'
REPORTS=ROOT/'reports/replicator_safety_upgrade/visual_tabletop_repairs'
SCOPE='Visual-semantic candidate and initial source-geometry checks only; no trajectory risk, native stability or task-success certification.'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def relative(path):return str(Path(path).resolve().relative_to(ROOT))


def targets():
    reviewed={}
    for file in (ROOT/'reports/replicator_safety_upgrade/visual_triage').glob('review_*.json'):
        for row in json.loads(file.read_text()).get('entries',[]):
            if row.get('visual_status')=='adjust' and row['category']!='human' and row['variant_id']!='robot_steel_clamp':
                reviewed[row['scene_id']]=row
    # Preserve already source-risk-screened entries; they have a separate owner.
    risk=set()
    for file in (ROOT/'reports/replicator_safety_upgrade/tasks').glob('*/result.json'):
        for row in json.loads(file.read_text()).get('entries',[]):
            if row.get('risk_verified') and row.get('config_path') and row.get('evidence_path'):
                evidence=json.loads((ROOT/row['evidence_path']).read_text())
                if evidence.get('config_sha256')==sha(ROOT/row['config_path']):risk.add(row['scene_id'])
    return [reviewed[key] for key in sorted(set(reviewed)-risk)]


def feature_points(hazard):
    by_name={r[0].rsplit('/',1)[-1]:r for r in hazard.rows}
    name=hazard.asset['id']
    if name=='robot_projecting_crossbar':
        r=by_name['ProjectingTube'];mid=(r[4]+r[5])/2;tip=mid.copy();tip[1]=r[4][1]
        return [('tube_tip',tip),('tube_midpoint',mid)]
    if name=='robot_narrow_opening':
        return [(n,(by_name[n][4]+by_name[n][5])/2) for n in ['InsideWearPlate0','InsideWearPlate1']]
    if name=='robot_sharp_equipment_housing':
        r=by_name['LeadingEdge'];return [('leading_edge',(r[4]+r[5])/2)]
    # Property body centre is only an alignment guide; the full source compound
    # collider, including stems/handles/feet, is checked below.
    return [('property_body_midheight',(hazard.low+hazard.high)/2)]


def point_segment(point,start,finish):
    delta=finish[:2]-start[:2]
    t=float(np.clip(np.dot(point-start[:2],delta)/max(np.dot(delta,delta),1e-12),0.,1.))
    return float(np.linalg.norm(point-(start[:2]+delta*t))),t


def candidates(hazard,start,finish,z):
    delta=finish[:2]-start[:2];direction=delta/max(np.linalg.norm(delta),1e-12)
    if np.linalg.norm(direction)<.5:raise ValueError('Pick and goal need a meaningful task-relative direction')
    normal=np.array([-direction[1],direction[0]])
    heading=float(np.rad2deg(np.arctan2(direction[1],direction[0])))
    # Portal X is the plane normal. Keep its orientation tied to task direction,
    # while the side post can create a bypass candidate; no forced-fit claim.
    yaw_values=[heading+angle for angle in (0.,30.,-30.,60.,-60.,180.,210.,150.,240.,120.)] if hazard.asset['id']=='robot_narrow_opening' else sorted(set([float(y) for y in range(0,360,30)]+[heading,heading+90.,heading+180.,heading+270.]))
    features=feature_points(hazard);rows=[];seen=set()
    # Large fixtures need their body outside the endpoint access columns while
    # their source projecting feature remains beside the middle workspace.
    # Wider offsets are considered after all closer alternatives; source
    # geometry/support/initial robot checks below never change with offset.
    offsets=tuple(sign*distance for distance in (.04,.08,.12,.16,.20,.24,.28) for sign in (1.,-1.))
    for fraction,offset,yaw,(feature,point) in itertools.product((.5,.35,.65,.25,.75,.15,.85),offsets,yaw_values,features):
        rotation=Rotation.from_euler('z',yaw,degrees=True).as_matrix()
        anchor=start[:2]+fraction*delta+offset*normal
        xyz=np.r_[anchor-(rotation@point)[:2],z]
        key=tuple(round(float(x),5) for x in [*xyz,yaw%360])
        if key in seen:continue
        seen.add(key)
        score=abs(abs(offset)-.08)+.06*abs(fraction-.5)
        rows.append(dict(position=xyz.tolist(),yaw_deg=yaw,feature=feature,
                         feature_world=(rotation@point+xyz).tolist(),task_fraction=fraction,
                         side_offset_m=offset,semantic_score=float(score)))
    return sorted(rows,key=lambda r:(round(r['semantic_score'],8),abs(r['task_fraction']-.5),abs(r['side_offset_m']),r['yaw_deg']))


def protected_columns(layout,scene,start,finish):
    params=scene.spec['task']['subtasks'][0]['params'];pick=params['pick_up_object'];dest=params['destination_location']
    vertices=scene.shapes[pick].vertices
    pick_radius=float(np.clip(np.max(np.ptp(vertices[:,:2],axis=0))/2+.025,.08,.13))
    if dest in scene.shapes:
        goal_radius=float(np.clip(np.max(np.ptp(scene.shapes[dest].vertices[:,:2],axis=0))/2+.025,.08,.13))
    else:goal_radius=.16
    return [dict(role='initial_grasp',center=start.tolist(),radius_m=pick_radius,z_range_m=[float(start[2]-.08),float(start[2]+.13)]),
            dict(role='goal_opening',center=finish.tolist(),radius_m=goal_radius,z_range_m=[float(finish[2]-.08),float(max(start[2],finish[2])+.16)])]


def protected_overlap(shapes,columns):
    for guard in columns:
        center=np.asarray(guard['center'])
        zlow,zhigh=guard['z_range_m']
        column=fcl.CollisionObject(fcl.Cylinder(guard['radius_m'],zhigh-zlow),
                                   fcl.Transform(np.r_[center[:2],(zlow+zhigh)/2]))
        for name,source_shape,low,high in shapes:
            if high[2]<guard['z_range_m'][0] or low[2]>guard['z_range_m'][1]:continue
            gap=np.linalg.norm(np.maximum(np.maximum(low[:2]-center[:2],center[:2]-high[:2]),0.))
            if gap<guard['radius_m'] and fcl.collide(source_shape,column,fcl.CollisionRequest(),fcl.CollisionResult()):
                return dict(role=guard['role'],hazard_part=name,horizontal_bbox_gap_m=float(gap),
                            required_semantic_radius_m=guard['radius_m'],check='source_shape_vs_reserved_access_cylinder')
    return None


def repair(layout,scene,hazard,robot,max_candidates):
    start,finish,*_=corridor(layout,scene)
    pick=scene.spec['task']['subtasks'][0]['params']['pick_up_object']
    support_id=next(r['reference'] for r in scene.spec['relations'] if r['kind']=='on' and r['subject']==pick)
    support_path=scene.refs[support_id]
    support=[r for r in scene.meshes if r[0]==support_path or r[0].startswith(support_path+'/')]
    if not support:raise ValueError('Missing source support surface')
    surface_top=max(float(v[:,2].max()) for _,v,_ in support)
    z=surface_top-hazard.low[2]+.0025
    objects=[r for name in layout['objects'] for r in scene.shapes[name].at([0,0,0])]+scene.shapes['droid'].at([0,0,0])
    guards=protected_columns(layout,scene,start,finish)
    guides=[dict(name='pick_to_destination',start=start,finish=finish,penalty=0.)]
    # A transfer can be shorter than the fixture. Approaching a task endpoint
    # from the robot is another meaningful manipulation corridor, without
    # forcing a large frame into the pick-to-goal gap. Cut the guide outside
    # the real source stand bounding box; final source checks remain mandatory.
    base=np.asarray(layout['robot_position'],dtype=float)
    stand_vertices=scene.shapes['droid'].vertices
    stand_low=stand_vertices.min(axis=0);stand_high=stand_vertices.max(axis=0)
    for name,target in [('approach_pick',start),('approach_destination',finish)]:
        delta=target[:2]-base[:2];length=float(np.linalg.norm(delta))
        if length<1e-8:continue
        unit=delta/length
        exits=[((stand_high[k] if unit[k]>0 else stand_low[k])-base[k])/unit[k]
               for k in (0,1) if abs(unit[k])>1e-8]
        exit_distance=min(v for v in exits if v>=0.)
        if length-exit_distance<.15:continue
        begin=np.r_[base[:2]+unit*(exit_distance+.03),start[2]]
        guides.append(dict(name=name,start=begin,finish=target,penalty=.015))
    pool=[]
    for guide in guides:
        for row in candidates(hazard,guide['start'],guide['finish'],z):
            row['guide']=guide['name'];row['semantic_score']+=guide['penalty'];pool.append(row)
    pool.sort(key=lambda row:round(row['semantic_score'],8))
    attempts=[];selected=None
    for index,candidate in enumerate(pool[:max_candidates]):
        row=dict(index=index,**candidate);attempts.append(row)
        shapes,footprint=hazard.at(candidate['position'],candidate['yaw_deg'])
        overlap=protected_overlap(shapes,guards)
        if overlap:
            row.update(rejected='protected_initial_grasp_or_goal_column',measurement=overlap);continue
        hits=[ray_z(support,p) for p in footprint]
        if not hits or any(h is None or not .0019<=h['vertical_gap_m']<=.007 for h in hits):
            row['rejected']='support_footprint';continue
        passed,measurement=separated(shapes,objects,.015)
        if not passed:row.update(rejected='task_objects_or_stand_source_geometry',measurement=measurement);continue
        passed,measurement=separated(shapes,scene.world,.002)
        if not passed:row.update(rejected='kitchen_source_geometry',measurement=measurement);continue
        initial=robot.check(shapes)
        if not initial['passed']:
            row.update(rejected='source_initial_robot',measurement=initial);continue
        row.update(passed=True,support_ray_count=len(hits),source_initial_robot=initial,
                   kitchen_source_check=measurement)
        selected=copy.deepcopy(hazard.asset);selected.update(position=candidate['position'],yaw_deg=candidate['yaw_deg']);break
    return selected,dict(scope=SCOPE,semantic_geometry_passed=selected is not None,
        nominal_workspace=dict(pick=start.tolist(),destination=finish.tolist(),protected_columns=guards,
            guides=[dict(name=g['name'],start=g['start'].tolist(),finish=g['finish'].tolist()) for g in guides],
            protected_columns_scope='Conservative visual-semantic space reservations, not certified grasp configurations or swept trajectories.'),
        feature_definitions=[dict(name=n,local_point=p.tolist()) for n,p in feature_points(hazard)],
        support_id=support_id,support_path=support_path,support_surface_z_m=surface_top,
        source_geometry_clearance_m=.002,task_object_and_stand_clearance_m=.015,
        selection_rule='First initial-geometry-valid candidate after sorting task-relative feature side distance and middle-segment fraction; all visited failures retained.',
        pool_size=len(pool),candidate_limit=max_candidates,visited=len(attempts),selected_candidate=len(attempts)-1 if selected else None,
        attempts=attempts,rejection_counts=dict(Counter(a.get('rejected','passed') for a in attempts)),
        risk_verified=False,native_stability_verified=False,safe_execution_verified=False)


def build(*,max_candidates=3000,output=OUTPUT,reports=REPORTS,task_filter=None):
    output=Path(output);reports=Path(reports);started=time.monotonic()
    todo=targets()
    if task_filter:todo=[r for r in todo if r['task_id'] in task_filter]
    original=json.loads((ROOT/'configs/extensions/replicator/scene_variants/manifest.json').read_text())
    originals={r['scene_id']:r for r in original['scenes']}
    assets={a['id']:Hazard(a) for a in variants() if a['id'] in {r['variant_id'] for r in todo}}
    robot=InitialRobotScreen();entries=[]
    manifest=dict(schema_version=1,scope=SCOPE,total=len(todo),stage='running',entries=entries,scenes=entries,
                  task_list=sorted({r['task_id'] for r in todo}),source_code_sha256=sha(__file__))
    def save():
        manifest.update(updated_at=dt.datetime.now(dt.timezone.utc).isoformat(),elapsed_seconds=time.monotonic()-started,
                        counts=dict(Counter(e['status'] for e in entries)))
        write_json(output/'manifest.json',manifest);write_json(reports/'progress.json',manifest)
    save()
    for task in manifest['task_list']:
        base_path,layout=canonical(task);scene=CandidateSearch(layout);robot.set_layout(layout)
        for review in [r for r in todo if r['task_id']==task]:
            old=originals[review['scene_id']];source=json.loads((ROOT/old['config_path']).read_text());short=task.removeprefix('kitchen_bench_replicator_')
            entry={k:review[k] for k in ['scene_id','task_id','variant_id','category']}
            entry.update(original_config_path=old['config_path'],original_config_sha256=sha(ROOT/old['config_path']),
                         visual_review=review,status='needs_repair',config_path=None,risk_verified=False,
                         native_stability_verified=False,safe_execution_verified=False)
            try:
                selected,report=repair(layout,scene,assets[review['variant_id']],robot,max_candidates)
                report.update(scene_id=review['scene_id'],task_id=task,variant_id=review['variant_id'],original_obstacle=source['obstacle'],
                              source_layout_path=relative(base_path),source_layout_sha256=sha(base_path),
                              original_config_sha256=entry['original_config_sha256'],hazard_dependencies=assets[review['variant_id']].dependencies,
                              code_sha256={relative(__file__):sha(__file__), 'kitchen_safety/extensions/replicator/scene_variant_builder.py':sha(ROOT/'kitchen_safety/extensions/replicator/scene_variant_builder.py')})
                evidence=reports/short/(review['variant_id']+'.json');write_json(evidence,report)
                entry.update(evidence_path=relative(evidence),evidence_sha256=sha(evidence),semantic_geometry_passed=selected is not None)
                if selected:
                    config=copy.deepcopy(source);config['obstacle']=selected
                    path=output/short/(review['variant_id']+'.json');write_json(path,config)
                    entry.update(status='visual_candidate',config_path=relative(path),config_sha256=sha(path),
                        placement=selected,selected_candidate=report['selected_candidate'],nominal_workspace=report['nominal_workspace'])
                else:entry['reason']='No placement passed the bounded semantic/support/source-geometry search.'
            except Exception as exc:entry['reason']=repr(exc)
            entries.append(entry);save()
            print(json.dumps(dict(scene_id=entry['scene_id'],status=entry['status'],reason=entry.get('reason'))),flush=True)
    manifest['stage']='complete';save();return manifest
