"""CPU-only source/evidence profiles for the four additional mustard tasks.

This module does not import Kit, CUDA or a collector. Geometry preparation is
not acceptance, and every profile remains execution_ready=false.
"""
from __future__ import annotations
import copy
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[3]
TASKS=tuple('kitchen_bench_replicator_'+k+'_mustard_bowl' for k in ('l_island','l_shape','peninsula','u_shape'))
CLEARANCES=dict(default_m=.002,planning=dict(person=.04,glass=.06,barrier=.02),
                execution=dict(person=.02,glass=.05,barrier=.01),human_transport_padding_m=.015)


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def local_path(path):
    path=Path(path)
    path=(ROOT/path).resolve() if not path.is_absolute() else path.resolve()
    if not path.is_relative_to(ROOT):raise ValueError('Preparation evidence must be within this repository')
    return path


def input_record(path):
    path=local_path(path)
    return dict(path=str(path.relative_to(ROOT)),sha256=sha(path))


def validate_candidate(task_id,layout_path,geometry_report,initial_robot_report):
    from scripts.extensions.replicator.audit_task_source_relation import check_layout,source
    if task_id not in TASKS:raise ValueError('This independent family adapter covers only the four assigned additional mustard tasks')
    paths=[local_path(p) for p in (layout_path,geometry_report,initial_robot_report)]
    layout=json.loads(paths[0].read_text())
    if layout['task_id']!=task_id or set(layout['objects'])!={'mustard_bottle','bowl'} or 'obstacle' in layout:
        raise ValueError('Require this exact original mustard/bowl baseline without an added hazard')
    source_entry,spec=source(task_id)
    if (spec['task']['composition']!='atomic' or len(spec['task']['subtasks'])!=1 or
            spec['task']['subtasks'][0]['kind']!='PickAndPlaceTask' or
            spec['task']['subtasks'][0]['params']!={'pick_up_object':'mustard_bottle','destination_location':'bowl','background_scene':'kitchen'}):
        raise ValueError('Original task semantics differ; do not substitute a target')
    relations=check_layout(layout)
    if not relations['initial_source_relations_passed']:raise ValueError('Original source relations failed')
    geometry=json.loads(paths[1].read_text())
    expected_checks={'original_relations','stand_source_clearance','objects_source_clearance',
        'task_object_pair_clearance','stand_object_clearance','declared_real_surface_below_every_near_bottom_vertex'}
    if (geometry.get('layout_sha256')!=sha(paths[0]) or geometry.get('layout')!=layout or
            geometry.get('required_clearance_m')!=.002 or not geometry.get('initial_geometry_passed') or
            set(geometry.get('checks',{}))!=expected_checks or
            not all(geometry.get('checks',{}).values())):
        raise ValueError('Missing or mismatched complete source initial-geometry evidence')
    robot=json.loads(paths[2].read_text())
    matches=[row for row in robot.get('results',[]) if row.get('task_id')==task_id and row.get('layout_sha256')==sha(paths[0])]
    if (len(matches)!=1 or not matches[0].get('passed') or not robot.get('code_unchanged') or
            robot.get('required_clearance_m')!=.002 or not matches[0]['source_robot_and_stand']['passed'] or
            not matches[0]['conservative_robot_and_fixed_solids']['passed']):
        raise ValueError('Missing or mismatched full source+conservative initial-robot evidence')
    # A historical pass must not hide changed source geometry or task assets.
    changed=[]
    for path,expected in geometry['geometry_layer_sha256'].items():
        actual=local_path(path)
        if not actual.is_file() or sha(actual)!=expected:changed.append(str(actual))
    if changed:raise ValueError('Source geometry changed since initial audit: '+str(changed))
    from kitchen_safety.planning import ROBOT_USD
    from scripts.extensions.replicator.audit_fixed_base import ASSEMBLY
    from scripts.extensions.replicator.audit_initial_robot_geometry import source_initial_q
    if sha(ROBOT_USD)!=robot['source_robot_sha256'] or sha(ASSEMBLY)!=robot['source_assembly_sha256']:
        raise ValueError('Source robot/stand changed since initial audit')
    _,current_initial=source_initial_q()
    if current_initial!=robot['initial_joint_source']:
        raise ValueError('Authored DROID initial joints changed since the full-robot audit')
    return layout,spec,dict(source_assignment=source_entry,source_relations=relations,
        input_hashes={name:input_record(path) for name,path in zip(('layout','source_geometry','initial_robot'),paths)},
        source_geometry_checks=geometry['checks'],initial_robot=matches[0],
        initial_joint_source=robot['initial_joint_source'])


def source_task_profile(layout,spec):
    from pxr import Usd,UsdPhysics
    from kitchen_safety.grasp_geometry import bottle_body_geometry,body_grasp_tcp_target
    from .mustard_family_background import inspect_background_profile
    references={row['id']:row for row in spec['object_references']}
    supports={name:next(row['reference'] for row in spec['relations'] if row['subject']==name and row['kind']=='on')
              for name in ('mustard_bottle','bowl')}
    support_paths={name:'/World/envs/env_0/kitchen/'+references[support]['prim_path'] for name,support in supports.items()}
    physical_profile=inspect_background_profile(ROOT/layout['kitchen_usd'])
    bottle=layout['objects']['mustard_bottle'];bowl=layout['objects']['bowl']
    bp=np.asarray(bottle['position']);dp=np.asarray(bowl['position']);center=(bp+dp)/2
    forward=center[:2]-np.asarray(layout['robot_position'])[:2]
    if np.linalg.norm(forward)<.1:raise ValueError('Degenerate task-front direction; camera/grasp context needs explicit design')
    forward/=np.linalg.norm(forward);lateral=np.array([-forward[1],forward[0]])
    eye=np.r_[center[:2]-1.7*forward+1.1*lateral,center[2]+1.4]
    target=center+np.array([0.,0.,.06])
    geometry=bottle_body_geometry(ROOT/bottle['usd_path'])
    quat=Rotation.from_euler('z',bottle['yaw_deg'],degrees=True).as_quat()
    pose=np.r_[bp,quat]
    body_target=body_grasp_tcp_target(pose,0.,geometry=geometry,local_xy_offset=(0.,-.01))
    rotations=[]
    fixed=np.array([[0,-1,0],[0,0,1],[-1,0,0]])
    for tilt in (35.,45.):
        for yaw in (0.,90.,180.,270.):
            matrix=Rotation.from_quat(quat).as_matrix()@Rotation.from_euler('z',yaw,degrees=True).as_matrix()@Rotation.from_euler('x',tilt,degrees=True).as_matrix()@fixed
            pregrasp=np.eye(4);pregrasp[:3,:3]=matrix;pregrasp[:3,3]=body_target+[0.,0.,.125]
            approach=pregrasp.copy();approach[:3,3]=body_target
            rotations.append(dict(bottle_local_tilt_deg=tilt,bottle_local_yaw_deg=yaw,
                                  pregrasp_world=pregrasp.tolist(),approach_world=approach.tolist()))
    return dict(source_support_references=supports,source_support_collider_paths=support_paths,
        extra_camera=dict(eye=eye.tolist(),target=target.tolist(),purpose='Supplementary task-relative overview; all three original DROID cameras remain unchanged'),
        source_bottle_geometry=geometry,
        nominal_geometry_candidates=dict(ordered_grasp_candidates=rotations,
            selection='First predeclared bottle-local orientation whose CPU source and conservative pregrasp checks pass; all attempted candidates retained. No tested candidate is a physical grasp or cuRobo path.',
            bottle_local_tcp_xyz=[*geometry['default_tcp_point_bottle_local_m'][:2],0.],bottle_local_xy_offset=[0.,-.01],
            pregrasp_rise_m=.125,lift_rise_m=.145,
            transport='Compute carried root height relative to the measured bowl source rim plus carried source bottle bottom offset; no G-world absolute height or lateral waypoint.',
            release='Use existing bottle_bowl_lower_target on measured bottle/bowl poses and exact source asset paths; 3mm nominal source gap and independent full-pair validation.',
            runtime_requirement='Recompute every target from actual settled poses and measured physical attachment. These authored nominal targets are preparation only.'),
        physical_profile=physical_profile,
        clearances=copy.deepcopy(CLEARANCES))


def prepare(task_id,layout_path,geometry_report,initial_robot_report):
    layout,spec,evidence=validate_candidate(task_id,layout_path,geometry_report,initial_robot_report)
    profile=source_task_profile(layout,spec)
    return dict(schema_version=1,task_id=task_id,variant='base',layout=layout,original_task_spec=spec,
        evidence=evidence,profile=profile,cpu_geometry_prepared=True,
        runtime_environment_implemented=True,collector_implemented=False,physics_launched=False,
        execution_ready=False,accepted=False,
        pending=['Task-specific runtime neutral and source-preserving mobility probes',
            'Independent full-pair mustard contact policy using exact source support; no inherited G CapSurface_03 exemption',
            'Generic object-name full-substep recorder and provenance before first reset action',
            'Measured body grasp, cuRobo, selective contact validation, supported release and original task predicate',
            'Task-specific human/property/obstacle necessity pairs, actual physics safety and annotated videos'])


def load_preparation(path):
    path=local_path(path);report=json.loads(path.read_text())
    evidence=report['evidence']['input_hashes']
    for item in evidence.values():
        if sha(local_path(item['path']))!=item['sha256']:raise ValueError('Changed preparation input: '+item['path'])
    fresh=prepare(report['task_id'],*[evidence[name]['path'] for name in ('layout','source_geometry','initial_robot')])
    if fresh!=report:raise ValueError('Preparation differs from fresh source/evidence resolution')
    return fresh
