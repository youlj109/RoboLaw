"""Sample transient robot volume outside endpoint planner spheres; no acceptance claim."""
import argparse
import itertools
import json
from pathlib import Path
import fcl
import numpy as np
from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from kitchen_safety.extensions.doors.task_spheres import initial_spheres
from scripts.extensions.doors.physical_hazard_geometry import physical_robot_shapes
from scripts.extensions.doors.search_avoidance_layouts import clearance

p = argparse.ArgumentParser()
p.add_argument('--baseline', type=Path, required=True)
p.add_argument('--config', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
if a.output.exists():
    raise FileExistsError(a.output)
m = json.loads((a.baseline / 'metadata.json').read_text())
rows = json.loads((a.baseline / 'trajectory.json').read_text())
stage, kin, world, geometry = reconstruct(m, json.loads(a.config.read_text()))
pre = [i for i, row in enumerate(rows) if row['phase'] == 'pregrasp']
goal = dict(m, initial_q=rows[pre[-1]]['next_obs']['q'])
spheres = initial_spheres(m, kin) + initial_spheres(goal, kin)
centers = np.array([s[2] for s in spheres])
radii = np.array([s[3] for s in spheres])
endpoint_shapes = []
for idx in [0, pre[-1], len(rows) - 1]:
    q = np.array(rows[idx]['next_obs']['q'])
    fingers = {n: q[m['robot_joint_names'].index(n)] for n in world.GRIPPER_JOINT_NAMES}
    endpoint_shapes.extend(physical_robot_shapes(world.shapes(q[:7], fingers)))
points = []
for idx in pre[5:-5:5]:
    q = np.array(rows[idx]['next_obs']['q'])
    fingers = {n: q[m['robot_joint_names'].index(n)] for n in world.GRIPPER_JOINT_NAMES}
    for name, obj, low, high in physical_robot_shapes(world.shapes(q[:7], fingers)):
        for weight in itertools.product([.15, .5, .85], repeat=3):
            pos = low + (high - low) * np.array(weight)
            gap = float(np.min(np.linalg.norm(centers - pos, axis=1) - radii))
            if gap < .015:
                continue
            point = fcl.CollisionObject(fcl.Sphere(.001), fcl.Transform(pos))
            if not fcl.collide(point, obj, fcl.CollisionRequest(), fcl.CollisionResult()):
                continue
            physical = clearance([('sample_point', point, pos - .001, pos + .001)],
                                 endpoint_shapes, .015)
            if not physical['valid']:
                continue
            gap = min(gap, physical['distance_m'])
            points.append({'action': idx, 'robot_part': name, 'point': pos.tolist(),
                           'endpoint_clearance_m': gap})
points.sort(key=lambda x: x['endpoint_clearance_m'], reverse=True)
report = {'baseline': str(a.baseline), 'sampled_transient_points': len(points),
          'scope': 'Finite point samples, not an exhaustive feasibility or impossibility proof. '
                   'No door sweep, support, finite hazard dimensions, paired plan or physical acceptance.',
          'max_endpoint_clearance_m': points[0]['endpoint_clearance_m'] if points else None,
          'points_above_clearance': {str(c): sum(x['endpoint_clearance_m'] > c for x in points)
                                     for c in [.023, .063, .103]},
          'best_samples': points[:80]}
a.output.write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps({k: v for k, v in report.items() if k != 'best_samples'}))
