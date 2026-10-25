"""Finite source geometry diagnostics only. Does not import or run a planner."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from scripts.extensions.doors.physical_hazard_geometry import physical_hazards, physical_robot_shapes
from scripts.extensions.doors.search_avoidance_layouts import clearance, separation

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--manifest', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
if a.output.exists():
    raise FileExistsError(a.output)
mfest = json.loads(a.manifest.read_text())
meta = json.loads((Path(mfest['baseline'])/'metadata.json').read_text())
if mfest.get('proposed_robot_base_world_m'):
    meta['robot_base'] = mfest['proposed_robot_base_world_m']
report = dict(scope='CPU sampled source geometry diagnostic. Not planning/physical acceptance.',
              curobo_invoked=False, ik_invoked=False, candidates=[], complete=False)

def clean(x):
    if isinstance(x, dict):
        return {k:clean(v) for k,v in x.items()}
    if isinstance(x, list):
        return [clean(v) for v in x]
    if isinstance(x, float) and not math.isfinite(x):
        return None
    return x

for item in mfest['candidates']:
    cfg = json.loads(Path(item['config']).read_text())
    obstacle = cfg[item['variant']]
    stage, kin, world, geometry = reconstruct(meta, cfg, obstacle)
    hazard = physical_hazards(stage, world, obstacle['name'])
    support = physical_hazards(stage, world, obstacle['support']['name']) if obstacle.get('support') else []
    if obstacle.get('support') and not support:
        raise RuntimeError('Configured support has no resolved colliders: '+obstacle['support']['asset'])
    if not hazard:
        raise RuntimeError('No source obstacle colliders')
    q = mfest.get('preview_initial_q', meta['initial_q'])
    fingers = {n:q[meta['robot_joint_names'].index(n)] for n in world.GRIPPER_JOINT_NAMES}
    robot = physical_robot_shapes(world.shapes(np.array(q[:7]), fingers))
    margin = world.clearances[obstacle['name']]
    source_initial_valid = bool(world.valid(np.asarray(q[:7]), gripper_q=fingers))
    initial_world_collision = world.last_collision
    joint_limits_valid = bool(np.all(np.asarray(q[:7]) >= kin.limits[:,0]) and np.all(np.asarray(q[:7]) <= kin.limits[:,1]))
    row = dict(id=item['id'], variant=item['variant'], source_initial=clearance(robot, hazard, margin),
               support_initial=clearance(robot, support, .002) if support else None,
               initial_world_valid=source_initial_valid, initial_world_collision=initial_world_collision, initial_joint_limits_valid=joint_limits_valid,
               scene_intersections=[], floor_support_contacts=[], door_sweep_failures=[],
               door_sweep_sampling_step_deg=2, actual_physics='not_run', planning='not_run')
    keys = [obstacle['name']]+([obstacle['support']['name']] if support else [])
    fixed = [o for o in world.objects if not any(world._matches(o[0], k) for k in keys)]
    for hn, ho, lo, hi in hazard+support:
        for n, obj, other_lo, other_hi in fixed:
            if np.linalg.norm(np.maximum(np.maximum(lo-other_hi, other_lo-hi), 0.)) > .008:
                continue
            distance, hit = separation(ho, obj)
            supported = abs(lo[2]-other_hi[2]) < .008 and any(s in n.lower() for s in ['floor','counter','stand_instanceable'])
            if supported and distance < .008:
                row['floor_support_contacts'].append(dict(added=hn, scene=n, distance_m=distance))
            elif hit:
                row['scene_intersections'].append(dict(added=hn, scene=n, distance_m=distance))
    hinge, axis = np.array(meta['hinge']), np.array(meta['axis'])
    for deg in range(0, 91, 2):
        rot = Rotation.from_rotvec(axis*math.radians(deg)).as_matrix()
        delta = np.eye(4)
        delta[:3,:3] = rot
        delta[:3,3] = hinge-rot@hinge
        for body in geometry['moving_bodies']:
            world.move_body(body, delta)
        door = [o for o in world.objects if any(o[0].startswith(body+'/') for body in geometry['moving_bodies'])]
        h = clearance(door, hazard, margin)
        s = clearance(door, support, .002) if support else dict(valid=True)
        if not h['valid'] or not s['valid']:
            row['door_sweep_failures'].append(dict(deg=deg, hazard=h, support=s))
    row['static_rejections_absent'] = bool(source_initial_valid and joint_limits_valid and row['source_initial']['valid'] and
        (not row['support_initial'] or row['support_initial']['valid']) and
        not row['scene_intersections'] and not row['door_sweep_failures'])
    report['candidates'].append(clean(row))
    a.output.write_text(json.dumps(report, indent=2, allow_nan=False))
    print(item['id'], 'initial', row['source_initial']['valid'], 'scene',len(row['scene_intersections']),
          'full90',len(row['door_sweep_failures'])==0, flush=True)
report['complete'] = True
a.output.write_text(json.dumps(report, indent=2, allow_nan=False))
