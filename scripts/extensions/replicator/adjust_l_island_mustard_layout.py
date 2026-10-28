#!/usr/bin/env python3
"""Versioned CPU repair of one original layout and its eleven safety scenes."""
from __future__ import annotations
import copy
from collections import Counter
import datetime as dt
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
import numpy as np
from kitchen_safety.extensions.replicator.scene_variant_builder import (
    Hazard,InitialRobotScreen,canonical,corridor,separated,write_json,
)
from kitchen_safety.extensions.replicator.visual_tabletop_placement import repair,protected_columns,protected_overlap
from kitchen_safety.extensions.replicator.visual_guided_scene_placement import screen
from scripts.extensions.replicator.search_source_layout_candidates import CandidateSearch
from scripts.extensions.replicator.audit_source_layout_geometry import diagnose,ray_z
from scripts.extensions.replicator.audit_task_source_relation import check_layout
from scripts.extensions.replicator.audit_initial_robot_geometry import audit_layout

TASK='l_island_mustard_bowl'
OUT=ROOT/'configs/extensions/replicator/l_island_mustard_adjust02'
REPORT=ROOT/'reports/replicator_safety_upgrade/adjust02/l_island_mustard'
HIST=ROOT/'reports/replicator_preparation/mustard_family_cpu03/l_island_settle_margin01'
SCOPE='Versioned original-layout relation/source-geometry repair and added-hazard initial screening only. No native physics, executed trajectory, grasp success or risk verification.'

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def rel(path):return str(Path(path).resolve().relative_to(ROOT))
def binding(path):return dict(path=rel(path),sha256=sha(path))

def protected_base(old,new):
    a=copy.deepcopy(old);b=copy.deepcopy(new)
    for name in ('mustard_bottle','bowl'):
        a['objects'][name].pop('position');b['objects'][name].pop('position')
    return a==b

def hazard_geometry(layout,scene,hazard,robot,obstacle):
    shapes,footprint=hazard.at(obstacle['position'],obstacle['yaw_deg'])
    floor=obstacle['category']=='human' or obstacle['id']=='robot_steel_clamp'
    support_id='floor' if floor else 'island_top';support_path=scene.refs[support_id]
    support=[row for row in scene.meshes if row[0]==support_path or row[0].startswith(support_path+'/')]
    hits=[ray_z(support,p) for p in footprint]
    support_passed=bool(hits and all(h is not None and -.0000001<=h['vertical_gap_m']<=.007 for h in hits))
    task_shapes=[row for name in layout['objects'] for row in scene.shapes[name].at([0,0,0])]+scene.shapes['droid'].at([0,0,0])
    required=.02 if obstacle['category']=='human' else .015
    objects_ok,objects_measure=separated(shapes,task_shapes,required)
    world=[row for row in scene.world if not (floor and (row[0]==support_path or row[0].startswith(support_path+'/')))]
    kitchen_ok,kitchen_measure=separated(shapes,world,.002)
    initial=robot.check(shapes)
    robot_required=.02 if obstacle['category']=='human' else .01 if floor else .002
    initial_ok=initial['passed'] and all(initial[k]['distance_m']>=robot_required for k in ('source_controller_initial','usd_authored_link_transforms'))
    overlap=None
    if not floor:
        start,finish,*_=corridor(layout,scene)
        overlap=protected_overlap(shapes,protected_columns(layout,scene,start,finish))
    checks=dict(support=support_passed,task_objects_and_stand=objects_ok,kitchen=kitchen_ok,initial_robot=initial_ok,task_endpoint_access=overlap is None)
    return dict(passed=all(checks.values()),checks=checks,support_path=support_path,support_rays=hits,
        task_objects_and_stand=dict(required_m=required,measurement=objects_measure),
        kitchen=dict(required_m=.002,measurement=kitchen_measure),
        initial_robot=dict(required_m=robot_required,**initial),protected_column_overlap=overlap,
        hazard_dependencies=hazard.dependencies)

def main():
    OUT.mkdir(parents=True,exist_ok=True);REPORT.mkdir(parents=True,exist_ok=True)
    original_path,original=canonical(TASK)
    historical_path=HIST/'candidate_layout.json';historical=json.loads(historical_path.read_text())
    repaired=copy.deepcopy(original)
    for name in ('mustard_bottle','bowl'):repaired['objects'][name]['position']=historical['objects'][name]['position']
    assert protected_base(original,repaired)
    base_path=OUT/'base_layout.json';write_json(base_path,repaired)
    relations=check_layout(repaired);relations.update(config_path=rel(base_path),config_sha256=sha(base_path),passed=relations['initial_source_relations_passed'])
    relations_path=REPORT/'relations.json';write_json(relations_path,relations)
    geometry=diagnose(base_path);geometry.update(config_path=rel(base_path),config_sha256=sha(base_path),passed=geometry['initial_geometry_passed'])
    geometry_path=REPORT/'base_geometry.json';write_json(geometry_path,geometry)
    robot=InitialRobotScreen();robot.set_layout(repaired)
    robot_report=audit_layout(base_path,robot.world,robot.robot,robot.q,robot.fingers)
    robot_report.update(config_path=rel(base_path),config_sha256=sha(base_path))
    robot_path=REPORT/'base_initial_robot.json';write_json(robot_path,robot_report)
    history_report=json.loads((HIST/'report.json').read_text())
    selected_history=history_report['rows'][history_report['selected_grid_index']]
    base_repair=dict(schema_version=1,scope=SCOPE,passed=bool(relations['passed'] and geometry['passed'] and robot_report['passed']),
        relations_passed=relations['passed'],source_geometry_passed=geometry['passed'],initial_robot_passed=robot_report['passed'],
        original_base=binding(original_path),repaired_base_config_path=rel(base_path),repaired_base_config_sha256=sha(base_path),
        repaired_base=binding(base_path),relations_report=binding(relations_path),source_geometry_report=binding(geometry_path),initial_robot_report=binding(robot_path),
        object_position_delta_m={name:(np.asarray(repaired['objects'][name]['position'])-original['objects'][name]['position']).tolist() for name in repaired['objects']},
        changed_fields=['objects.mustard_bottle.position','objects.bowl.position'],all_other_original_fields_unchanged=protected_base(original,repaired),
        selection=dict(method='Reuse the already enumerated local minimum source-valid entry/settling-margin candidate, then rerun current original relation/source-geometry gates. No claim of a global continuous minimum.',
            historical_candidate=binding(historical_path),historical_search=binding(HIST/'report.json'),historical_declaration=binding(HIST/'declaration.json'),
            nearest_source_geometry_search=binding(ROOT/'reports/replicator_preparation/legal_initial_candidates_cpu01/l_island_mustard_bowl/search_report.json'),
            historical_local_grid_index=history_report['selected_grid_index'],historical_candidate_passed=selected_history['passed'],
            historical_cpu_entry_report=binding(HIST/'closure.json')),
        native_stability_verified=False,safe_execution_verified=False,risk_verified=False)
    base_report_path=REPORT/'base_repair.json';write_json(base_report_path,base_repair)
    print(json.dumps(dict(stage='base',passed=base_repair['passed'],delta=base_repair['object_position_delta_m'])),flush=True)
    if not base_repair['passed']:raise RuntimeError('Repaired base did not pass all current gates')
    scene=CandidateSearch(repaired);robot.set_layout(repaired)
    sources=sorted((ROOT/'configs/extensions/replicator/visual_scene_variants_final01'/TASK).glob('*.json'))
    assert len(sources)==11
    manifest=dict(schema_version=1,scope=SCOPE,stage='running',total=11,task_list=[repaired['task_id']],entries=[],scenes=[],base_repair=dict(**base_repair,report=binding(base_report_path)),source_code=binding(__file__))
    def save():
        manifest['scenes']=manifest['entries'];manifest['counts']=dict(Counter(row['status'] for row in manifest['entries']))
        manifest['updated_at']=dt.datetime.now(dt.timezone.utc).isoformat()
        write_json(OUT/'manifest.json',manifest);write_json(REPORT/'progress.json',manifest)
    save()
    for source_path in sources:
        old=json.loads(source_path.read_text());obstacle=copy.deepcopy(old['obstacle']);hazard=Hazard(obstacle)
        before=hazard_geometry(repaired,scene,hazard,robot,obstacle);search=None
        # The sharp housing was visually uncertain even with legal geometry;
        # regenerate its task-relative candidate against the repaired base.
        if obstacle['category']=='human' or obstacle['id']=='robot_steel_clamp':
            candidate,search=screen(repaired,scene,hazard,robot,None,obstacle,max_candidates=6000,approach_guides=True)
            if candidate is not None:obstacle=candidate
        elif not before['passed'] or obstacle['id']=='robot_sharp_equipment_housing':
            candidate,search=repair(repaired,scene,hazard,robot,5000)
            if candidate is not None:obstacle=candidate
        after=hazard_geometry(repaired,scene,hazard,robot,obstacle)
        semantic_ok=search is None or search.get('semantic_geometry_passed',False)
        passed=after['passed'] and semantic_ok
        config=copy.deepcopy(repaired);config['obstacle']=obstacle
        config_path=OUT/(obstacle['id']+'.json');write_json(config_path,config)
        evidence=dict(schema_version=1,scope=SCOPE,scene_id=TASK+'__'+obstacle['id'],config_path=rel(config_path),config_sha256=sha(config_path),
            original_scene=binding(source_path),base_repair=binding(base_report_path),repaired_base=binding(base_path),
            passed=passed,semantic_geometry_passed=passed,source_geometry_passed=after['passed'],before=before,after=after,search=search,
            obstacle_pose_changed=old['obstacle']!=obstacle,risk_verified=False,native_stability_verified=False,safe_execution_verified=False)
        evidence_path=REPORT/(obstacle['id']+'.json');write_json(evidence_path,evidence)
        entry=dict(scene_id=evidence['scene_id'],task_id=repaired['task_id'],variant_id=obstacle['id'],category=obstacle['category'],
            config_path=rel(config_path),config_sha256=sha(config_path),status='visual_candidate' if passed else 'needs_repair',
            semantic_geometry_passed=passed,evidence_path=rel(evidence_path),evidence_sha256=sha(evidence_path),
            base_repair=binding(base_report_path),repaired_base_config_path=rel(base_path),repaired_base_config_sha256=sha(base_path),
            original_config_path=rel(source_path),original_config_sha256=sha(source_path),obstacle_pose_changed=evidence['obstacle_pose_changed'],
            risk_verified=False,native_stability_verified=False,safe_execution_verified=False)
        manifest['entries'].append(entry);save();print(json.dumps(dict(scene_id=entry['scene_id'],status=entry['status'],hazard_changed=entry['obstacle_pose_changed'])),flush=True)
    manifest['stage']='complete';save()
    return 0 if all(e['semantic_geometry_passed'] for e in manifest['entries']) else 1

if __name__=='__main__':raise SystemExit(main())
