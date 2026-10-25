"""Targeted offline source checks for visually reviewed exact layouts, no search/planner."""
import argparse
import hashlib
import json
import math
import itertools
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from kitchen_safety.extensions.doors.task_model import joint_position_from_openness
from kitchen_safety.planning import transform
from scripts.extensions.doors.physical_hazard_geometry import physical_hazards, physical_robot_shapes
from scripts.extensions.doors.search_avoidance_layouts import clearance, separation

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--manifest',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
if a.output.exists():
    raise FileExistsError(a.output)
manifest=json.loads(a.manifest.read_text())
report=dict(scope='Targeted OFFLINE source diagnostic only. No planner, no physical execution, no final acceptance. Goal arm checks constrain the recorded posture only; gripper checks constrain the recorded Cartesian contact policy.',
            manifest=str(a.manifest),trajectory_planning_invoked=False,complete=False,candidates=[])

def clean(value):
    if isinstance(value,dict):return {k:clean(v) for k,v in value.items()}
    if isinstance(value,list):return [clean(v) for v in value]
    if isinstance(value,float) and not math.isfinite(value):return None
    return value

for item in manifest['candidates']:
    cfg=json.loads(Path(item['config']).read_text());spec=cfg[item['variant']]
    ep=Path(item['baseline']);m=json.loads((ep/'metadata.json').read_text())
    rows=json.loads((ep/'trajectory.json').read_text())
    stage,kin,w,g=reconstruct(m,cfg,spec)
    hazard=physical_hazards(stage,w,spec['name'])
    support=physical_hazards(stage,w,spec['support']['name']) if spec.get('support') else []
    if not hazard or (spec.get('support') and not support):
        raise RuntimeError('Missing actual source collider for '+item['id'])
    keys=[spec['name']]+([spec['support']['name']] if support else [])
    fixed=[o for o in w.objects if not any(w._matches(o[0],key) for key in keys)]
    margin=w.clearances[spec['name']]
    door_initial=[o for o in w.objects if any(o[0].startswith(body+'/') for body in g['moving_bodies'])]
    hinge=np.asarray(g['hinge']);axis=np.asarray(g['axis']);axis=axis/np.linalg.norm(axis)
    door_corners=np.concatenate([np.asarray(list(itertools.product(*zip(o[2],o[3])))) for o in door_initial])
    radial=door_corners-hinge
    rotation_radius=float(np.linalg.norm(radial-np.outer(radial@axis,axis),axis=1).max())
    def shapes(q):
        fingers={n:q[m['robot_joint_names'].index(n)] for n in w.GRIPPER_JOINT_NAMES}
        return w.shapes(np.asarray(q[:7]),fingers)
    def both(shape_set,hz,required):
        return dict(source=clearance(physical_robot_shapes(shape_set),hz,required),
                    conservative=clearance(shape_set,hz,required))
    initial=shapes(m['initial_q'])
    pre=[i for i,row in enumerate(rows) if row['phase']=='pregrasp']
    result=dict(id=item['id'],variant=item['variant'],config=item['config'],
                config_sha256=hashlib.sha256(Path(item['config']).read_bytes()).hexdigest(),
                original_planning_clearance_m=margin,
                original_execution_clearance_m=w.execution_clearances[spec['name']],
                source_hazard_colliders=len(hazard),source_support_colliders=len(support),
                initial_hazard=both(initial,hazard,margin),
                source_intersection_actions=[],source_clearance_risk_actions=[],
                scene_intersections=[],source_floor_contacts=[],
                measured_door_sweep_failures=[],contact_gripper_failures=[])
    if support:
        result['initial_support']=both(initial,support,.002)
    for i in pre:
        physical=physical_robot_shapes(shapes(rows[i]['next_obs']['q']))
        source=clearance(physical,hazard,0.)
        if source.get('intersecting',False):
            result['source_intersection_actions'].append(i)
        risk=clearance(physical,hazard,margin)
        if not risk['valid']:
            result['source_clearance_risk_actions'].append(i)
    result['recorded_pregrasp_goal']=both(shapes(rows[pre[-1]]['next_obs']['q']),hazard,margin)
    for hn,ho,hl,hh in hazard+support:
        for n,o,lo,hi in fixed:
            if np.linalg.norm(np.maximum(np.maximum(hl-hi,lo-hh),0.))>.008:continue
            distance,hit=separation(ho,o)
            supported=abs(hl[2]-hi[2])<.008 and any(s in n.lower() for s in ['floor','counter','stand_instanceable'])
            if supported and distance<.008:
                result['source_floor_contacts'].append(dict(added=hn,scene=n,distance_m=distance))
            elif hit:
                result['scene_intersections'].append(dict(added=hn,scene=n,distance_m=distance))
    for i in range(pre[-1],len(rows)):
        source=physical_robot_shapes(shapes(rows[i]['next_obs']['q']))
        gripper=[s for s in source if s[0].startswith('gripper/')]
        ch=clearance(gripper,hazard,margin)
        cs=clearance(gripper,support,.002) if support else dict(valid=True)
        if not ch['valid'] or not cs['valid']:
            result['contact_gripper_failures'].append(dict(action=i,phase=rows[i]['phase'],hazard=ch,support=cs))
    initial_door=transform(m['initial_door_pose'][:3],m['initial_door_pose'][3:])
    for i,row in enumerate(rows):
        pose=row['next_obs']['door_body_pose']
        delta=transform(pose[:3],pose[3:])@np.linalg.inv(initial_door)
        for body in g['moving_bodies']:w.move_body(body,delta)
        door=[o for o in w.objects if any(o[0].startswith(body+'/') for body in g['moving_bodies'])]
        ch=clearance(door,hazard,margin)
        cs=clearance(door,support,.002) if support else dict(valid=True)
        if not ch['valid'] or not cs['valid']:
            result['measured_door_sweep_failures'].append(dict(action=i,hazard=ch,support=cs))
    result['door_sampling_scope']=dict(recorded_action_states=len(rows),all_checked=True,
                                       continuous_interpolation_certified=False,
                                       physical_substeps_not_checked_in_this_diagnostic=True)
    # Entire source joint range, including angles beyond the collection threshold.
    # A rigid point between two samples is at most 2*r*sin(delta_angle/4)
    # from its nearest sample. Add this Hausdorff bound to original clearance;
    # every sample passing then certifies intervening angles in this FCL model.
    lower,upper=g['limits_rad']
    rest=joint_position_from_openness(g['reset_openness'],lower,upper)
    if abs(rest)>1e-7:
        raise ValueError('Full-range diagnostic currently requires source zero-reset geometry')
    angles=np.linspace(lower,upper,int(np.ceil((upper-lower)/np.deg2rad(.5)))+1)
    step=float(angles[1]-angles[0]);between_bound=2.*rotation_radius*math.sin(step/4.)
    sweep=dict(joint_limits_rad=[lower,upper],angles_checked=len(angles),
               max_step_rad=step,source_radius_bound_m=rotation_radius,
               between_sample_displacement_bound_m=between_bound,
               original_hazard_margin_m=margin,required_sample_hazard_margin_m=margin+between_bound,
               original_support_margin_m=.002,required_sample_support_margin_m=.002+between_bound,
               failed_angle_count=0,first_failures=[],source_model_full_range_clear=False,
               scope='Conservative continuous-angle check for the reconstructed rigid source FCL door model only; not PhysX cooked geometry or physical execution acceptance.')
    for angle in angles:
        delta=np.eye(4);delta[:3,:3]=Rotation.from_rotvec(axis*(angle-rest)).as_matrix()
        delta[:3,3]=hinge-delta[:3,:3]@hinge
        for body in g['moving_bodies']:w.move_body(body,delta)
        door=[o for o in w.objects if any(o[0].startswith(body+'/') for body in g['moving_bodies'])]
        ch=clearance(door,hazard,margin+between_bound)
        cs=clearance(door,support,.002+between_bound) if support else dict(valid=True)
        if not ch['valid'] or not cs['valid']:
            sweep['failed_angle_count']+=1
            if len(sweep['first_failures'])<3:
                sweep['first_failures'].append(dict(angle_rad=float(angle),hazard=ch,support=cs))
    sweep['source_model_full_range_clear']=sweep['failed_angle_count']==0
    result['full_joint_range_door_sweep']=sweep
    # These are rejection diagnostics, deliberately not accepted=true gates.
    report['candidates'].append(clean(result))
    a.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('LAYOUT_SOURCE',item['id'],'hits',len(result['source_intersection_actions']),
          'initial',result['initial_hazard']['conservative']['valid'],
          'scene',len(result['scene_intersections']),
          'door',len(result['measured_door_sweep_failures']),
          'contact',len(result['contact_gripper_failures']),
          'full_range',sweep['source_model_full_range_clear'],flush=True)
report['complete']=True
a.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
