"""Translate existing C01 hazards toward the robot front; CPU candidate screening only."""
import argparse
import copy
import hashlib
import json
from collections import Counter
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.doors.reconstruction import reconstruct, DOOR
from kitchen_safety.planning import transform
from scripts.extensions.doors.physical_hazard_geometry import physical_hazards, physical_robot_shapes
from scripts.extensions.doors.search_avoidance_layouts import clearance, separation


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--variant', choices=['human', 'property'], required=True)
    p.add_argument('--baseline', type=Path, default=Path('datasets/doors/lwx_20261023/base_03'))
    p.add_argument('--config', type=Path, default=Path('configs/extensions/doors/avoidance_variants.json'))
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--dx-min', type=float, default=.27)
    p.add_argument('--dx-max', type=float, default=.35)
    p.add_argument('--dy-min', type=float, default=-.09)
    p.add_argument('--dy-max', type=float, default=-.045)
    p.add_argument('--step', type=float, default=.01)
    a = p.parse_args()
    if a.output.exists():
        p.error('Refusing to overwrite search evidence')
    m = json.loads((a.baseline/'metadata.json').read_text())
    rows = json.loads((a.baseline/'trajectory.json').read_text())
    original = json.loads(a.config.read_text())[a.variant]
    stage, kin, world = reconstruct(m, original)
    name = original['name']
    margin = world.execution_clearances[name]
    def shapes(state):
        angles = dict(zip(m['robot_joint_names'], state['q']))
        return world.shapes(np.asarray(state['q'][:7]), {n: angles[n] for n in world.GRIPPER_JOINT_NAMES})
    start = shapes(rows[0]['obs'])
    # Real measured pregrasp states, not synthetic unsafe physical execution.
    free_rows = [(i,r) for i,r in enumerate(rows) if r['phase']=='pregrasp']
    free = [(i,physical_robot_shapes(shapes(r['next_obs']))) for i,r in free_rows[::3]]
    # Saved successful high-grasp path screens usability; fresh planning/physics still required.
    old = a.baseline.parent/(a.variant+'_01')
    old_rows = json.loads((old/'trajectory.json').read_text())
    endpoint = shapes([r for r in old_rows if r['phase']=='pregrasp'][-1]['next_obs'])
    # Store poses, not live FCL objects whose transforms are later mutated.
    reference = transform(m['initial_door_pose'][:3], m['initial_door_pose'][3:])
    swept_poses = [r['next_obs']['door_body_pose'] for r in rows[::4]+[rows[-1]]]
    static = [o for o in world.objects if '/kitchen/' in o[0] or '/stand_instanceable/' in o[0]]
    old_hazard = physical_hazards(stage,world,name)
    old_lo=np.min([x[2] for x in old_hazard],axis=0)
    old_hi=np.max([x[3] for x in old_hazard],axis=0)
    # For human, the raised hand occupies the rightmost 10cm of the source body mesh.
    anchor = np.array([old_hi[0]-.05, (old_lo[1]+old_hi[1])/2, 1.25]) if a.variant=='human' else (old_lo+old_hi)/2
    base=np.asarray(m['robot_base'])
    yaw=np.deg2rad(m['robot_yaw_deg'])
    forward=np.array([np.cos(yaw),np.sin(yaw)])
    lateral=np.array([-forward[1],forward[0]])
    shifts = [(float(dx),float(dy)) for dx in np.arange(a.dx_min,a.dx_max+1e-6,a.step) for dy in np.arange(a.dy_min,a.dy_max+1e-6,a.step)]
    shifts.sort(key=lambda d: (abs((anchor[:2]+d-base[:2])@lateral),np.linalg.norm(d)))
    report={'variant':a.variant,'baseline':str(a.baseline),'baseline_sha256':hashlib.sha256((a.baseline/'trajectory.json').read_bytes()).hexdigest(),
            'source_config_sha256':hashlib.sha256(a.config.read_bytes()).hexdigest(),
            'scope':'CPU translated-source geometry screening; not physical success or final dense audit. Same assets, yaw, height and controller; robot group untouched.',
            'original_spec':original,'front_metric':'Lateral distance of bottle center / rightmost raised-hand bounding region from actual robot forward axis; visual confirmation still required.',
            'original_front_lateral_m':float(abs((anchor[:2]-base[:2])@lateral)), 'candidates':[],'rejected':[]}
    for dx,dy in shifts:
        spec=copy.deepcopy(original)
        delta=np.array([dx,dy,0.])
        spec['position']=(np.asarray(original['position'])+delta).tolist()
        if spec.get('support'):
            spec['support']['position']=(np.asarray(original['support']['position'])+delta).tolist()
        pose=[*spec['position'],*Rotation.from_euler('z',spec.get('yaw_deg',0),degrees=True).as_quat()]
        world.set_obstacle_pose(name,pose)
        if spec.get('support'):
            ob=spec['support'];world.set_obstacle_pose(ob['name'],[*ob['position'],0.,0.,0.,1.])
        hazards=physical_hazards(stage,world,name,pose)
        proxy=[o for o in world.objects if world._matches(o[0],name)]
        support=[o for o in world.objects if spec.get('support') and world._matches(o[0],spec['support']['name'])]
        row={'translation_m':[dx,dy,0.],'spec':spec,'front_lateral_m':float(abs((anchor[:2]+delta[:2]-base[:2])@lateral)),
             'front_distance_m':float((anchor[:2]+delta[:2]-base[:2])@forward)}
        reason=None
        for geom in hazards+support:
            hn,ho,hl,hh=geom
            for on,oo,ol,oh in static:
                if np.linalg.norm(np.maximum(np.maximum(hl-oh,ol-hh),0))>.007:continue
                d,hit=separation(ho,oo)
                resting=hl[2]>=oh[2]-.006 and ('/floor' in on.lower() or '/counter_' in on or 'stand' in on.lower())
                if hit and not (resting and d<.007):
                    reason='source_or_stand_overlap';row['pair']=[hn,on];break
            if reason:break
        if reason is None and (not clearance(start,proxy,world.clearances[name])['valid'] or (support and not clearance(start,support,.002)['valid'])):
            reason='initial_robot_clearance'
        if reason is None and (not clearance(endpoint,proxy,margin)['valid'] or (support and not clearance(endpoint,support,.002)['valid'])):
            reason='existing_high_grasp_endpoint_blocked'
        if reason is None:
            door_minimum = None
            for door_pose in swept_poses:
                world.move_body(DOOR, transform(door_pose[:3],door_pose[3:])@np.linalg.inv(reference))
                moving = [o for o in world.objects if o[0].startswith(DOOR+'/')]
                check = clearance(moving,hazards,margin,True)
                if door_minimum is None or check['distance_m'] < door_minimum['distance_m']:door_minimum=check
                if not check['valid'] or (support and not clearance(moving,support,.002)['valid']):
                    reason='baseline_door_sweep_blocked'
                    break
            world.move_body(DOOR,np.eye(4))
            row['door_sweep_minimum']=door_minimum
        if reason is None:
            risks=[{'frame':i,**clearance(rs,hazards,margin,True)} for i,rs in free]
            row['baseline_minimum']=min(risks,key=lambda x:x['slack_m'])
            row['violating_samples']=sum(not x['valid'] for x in risks)
            if not row['violating_samples']:reason='baseline_not_blocked'
        if reason:
            row['reason']=reason;report['rejected'].append(row)
        else:
            report['candidates'].append(row)
            print('CANDIDATE',a.variant,row['translation_m'],row['front_lateral_m'],row['baseline_minimum']['distance_m'],flush=True)
    report['candidates'].sort(key=lambda x:(x['front_lateral_m'],x['baseline_minimum']['distance_m']))
    report['rejection_counts']=dict(Counter(r['reason'] for r in report['rejected']))
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(report,indent=2)+'\n')
    print('SEARCH_DONE',a.variant,len(report['candidates']),report['rejection_counts'],flush=True)


if __name__=='__main__':
    main()
