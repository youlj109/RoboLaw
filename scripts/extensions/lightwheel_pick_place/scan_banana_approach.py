"""Bounded banana_1 pregrasp+approach MotionGen feasibility scan (no PhysX exec).

Sweeps grasp yaw/tilt and local-approach standoff around a saved episode's
measured layout. Reports planner-reachable (pregrasp, approach) pairs only.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import h5py
import numpy as np
from pxr import Usd
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
from kitchen_safety.extensions.lightwheel_pick_place.planner import LimePlanner
from kitchen_safety.planning import DroidKinematics, transform

parser = argparse.ArgumentParser()
parser.add_argument('--episode', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--pick-object', default='banana_1')
parser.add_argument('--destination-object', default='wooden_bowl')
parser.add_argument('--yaw-deg', default='0,-45,-90,-135,90,45,180')
parser.add_argument('--tilt-deg', default='0,10,20')
parser.add_argument('--standoff-m', default='0.20,0.17,0.14,0.12')
parser.add_argument('--height-offset-m', type=float, default=0.01)
parser.add_argument('--contact-remain-m', default='0.02', help='comma list: stop this far short of center along -TCP_Z')
parser.add_argument('--max-candidates', type=int, default=48)
args = parser.parse_args()


def csv_floats(s: str):
    return [float(x) for x in s.split(',') if x.strip() != '']


def short_err(exc: BaseException) -> str:
    msg = str(exc)
    if 'IK_FAIL' in msg:
        return 'IK_FAIL'
    if 'FINETUNE_TRAJOPT_FAIL' in msg:
        return 'FINETUNE_TRAJOPT_FAIL'
    if 'pose residual' in msg:
        return 'LOCAL_IK_RESIDUAL'
    return msg[:160]


ep = args.episode
result = json.loads((ep / 'result.json').read_text())
base = np.asarray(result['robot_base_pose'], dtype=float)
kin = DroidKinematics(base[:3], Rotation.from_quat(base[3:7]).as_euler('xyz', degrees=True)[2])
stage = Usd.Stage.Open(str(ep / 'scene.usda'))
pick = args.pick_object
dest = args.destination_object
rigid = ('banana_1', 'banana_2', 'wooden_bowl', 'plate', 'apple', 'orange')
movable = tuple(n for n in rigid if n != pick) + ('glass',)
world = CollisionWorld(
    stage, kin,
    pick_object=pick,
    destination_object=dest,
    source_surface_suffix='/island_island_group/top_geometry_right',
    movable_objects=movable,
)

with h5py.File(ep / 'trajectory.h5') as f:
    q0 = np.asarray(f['obs/q'][0, :7], dtype=float)
    # Prefer measured initial object roots from h5 when present.
    initial_poses = {}
    for name in rigid:
        key = 'obs/' + name
        if key in f:
            initial_poses[name] = np.asarray(f[key][0, :7], dtype=float)

# Fall back to result.json initial states / layout poses.
if not initial_poses:
    states = result.get('initial_object_states') or {}
    for name in rigid:
        if name in states:
            initial_poses[name] = np.asarray(states[name][:7], dtype=float)

for name, pose in initial_poses.items():
    if name == pick:
        continue
    try:
        world.set_obstacle_pose(name, pose)
    except Exception as exc:
        print('WARN set_obstacle', name, exc, flush=True)

pick_pose = initial_poses[pick]
# Align hull to measured pick root for object_center.
world.bind_pick(pick)
# Recompute center from current hull + pick pose.
center = (transform(pick_pose[:3], pick_pose[3:7]) @ np.r_[world.object_center, 1])[:3]
center = center + np.array([0.0, 0.0, float(args.height_offset_m)])

planner = LimePlanner(world, kin, dt=float(result.get('dt') or 1 / 30), seed=int(result.get('seed') or 43), max_joint_speed=0.4)
print('SCAN_READY', 'center', center.tolist(), 'q0', q0.tolist(), flush=True)

yaws = csv_floats(args.yaw_deg)
tilts = csv_floats(args.tilt_deg)
standoffs = csv_floats(args.standoff_m)
candidates = []
for yaw in yaws:
    for tilt in tilts:
        for st in standoffs:
            candidates.append((tilt, yaw, st))
            if len(candidates) >= args.max_candidates:
                break
        if len(candidates) >= args.max_candidates:
            break
    if len(candidates) >= args.max_candidates:
        break

rows = []
best = None
t0 = time.time()
R0 = np.array([[0, -1, 0], [0, 0, 1], [-1, 0, 0]], dtype=float)
for i, (tilt, yaw, st) in enumerate(candidates):
    R = Rotation.from_euler('z', yaw, degrees=True).as_matrix() @ Rotation.from_euler('x', tilt, degrees=True).as_matrix() @ R0
    approach = R[:, 2]
    approach = approach / max(float(np.linalg.norm(approach)), 1e-6)
    hover = center - float(st) * approach
    remain = float(csv_floats(args.contact_remain_m)[0])
    contact = center - remain * approach
    row = {
        'index': i,
        'tilt_deg': tilt,
        'yaw_deg': yaw,
        'standoff_m': st,
        'contact_remain_m': float(csv_floats(args.contact_remain_m)[0]),
        'hover_xyz': hover.tolist(),
        'contact_xyz': contact.tolist(),
        'tcp_z_world': approach.tolist(),
        'pregrasp_ok': False,
        'approach_ok': False,
    }
    try:
        T_h = np.eye(4)
        T_h[:3, :3] = R
        T_h[:3, 3] = hover
        pre = planner.plan_pose(q0, T_h, (pick,), 0.0, phase='transfer')
        q_pre = np.asarray(pre['positions'][-1], dtype=float)
        row['pregrasp_ok'] = True
        row['pregrasp_meta'] = pre.get('metadata')
        T_c = np.eye(4)
        T_c[:3, :3] = R
        T_c[:3, 3] = contact
        ap = planner.plan_pose(q_pre, T_c, (pick,), 0.0, phase='transfer')
        q_ap = np.asarray(ap['positions'][-1], dtype=float)
        row['approach_ok'] = True
        row['approach_meta'] = ap.get('metadata')
        # FCL validate endpoints
        row['fcl_pre'] = world.clearance(q_pre, (pick,))
        row['fcl_ap'] = world.clearance(q_ap, (pick,))
        row['passed'] = bool(row['fcl_pre'].get('valid') and row['fcl_ap'].get('valid'))
        if row['passed'] and best is None:
            best = row
            print('BEST', row['tilt_deg'], row['yaw_deg'], row['standoff_m'], flush=True)
    except Exception as exc:
        row['error'] = short_err(exc)
        row['passed'] = False
    rows.append(row)
    print(
        f"CAND {i}/{len(candidates)} tilt={tilt} yaw={yaw} st={st} "
        f"pre={row['pregrasp_ok']} ap={row['approach_ok']} pass={row.get('passed')} "
        f"err={row.get('error')}",
        flush=True,
    )
    if best is not None:
        # Keep scanning a few more for alternatives, but stop early once we have 3 passes.
        if sum(1 for r in rows if r.get('passed')) >= 3:
            break

report = {
    'episode': str(ep),
    'pick_object': pick,
    'center_xyz': center.tolist(),
    'robot_base_pose': base.tolist(),
    'q0': q0.tolist(),
    'elapsed_s': time.time() - t0,
    'n_candidates': len(rows),
    'n_passed': sum(1 for r in rows if r.get('passed')),
    'best': best,
    'rows': rows,
    'scope': 'Offline MotionGen pregrasp+approach pair scan; no PhysX execution.',
}
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(report, indent=2) + '\n')
print('COMPLETE', 'passed', report['n_passed'], 'best', best, flush=True)
