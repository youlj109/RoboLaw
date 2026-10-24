"""Bounded WP0 feasibility scan from a saved bilateral-close episode.

Mirrors collect shelf-lift planning: MotionGen phase='transfer' for free
Cartesian deltas; local cartesian phase='lift' for pure +Z. Independently
validates each candidate with FCL phase='lift'. No PhysX execution.
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
from kitchen_safety.planning import DroidKinematics

parser = argparse.ArgumentParser()
parser.add_argument('--episode', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--pick-object', default='lemon')
parser.add_argument('--dx-mm', default='0', help='comma mm list')
parser.add_argument('--dy-mm', default='0,-40,-80,-120,-160,-200')
parser.add_argument('--dz-mm', default='0,20,40,60,80,100,120,160')
parser.add_argument('--max-candidates', type=int, default=80)
args = parser.parse_args()


def mm_list(s: str):
    return [float(x) / 1000.0 for x in s.split(',') if x.strip() != '']


def short_err(exc: BaseException) -> str:
    msg = str(exc)
    if 'FINETUNE_TRAJOPT_FAIL' in msg:
        return 'FINETUNE_TRAJOPT_FAIL'
    if 'IK_FAIL' in msg:
        return 'IK_FAIL'
    if 'Cartesian' in msg and 'vertical column' in msg:
        return 'VERTICAL_COLUMN'
    if 'collision/joint constraints' in msg:
        return 'CUROBO_CART_COLLISION'
    if 'Path validation failed' in msg or 'invalid_waypoints' in msg:
        return 'FCL_INVALID'
    return msg[:180]


ep = args.episode
result = json.loads((ep / 'result.json').read_text())
base = result['robot_base_pose']
kin = DroidKinematics(base[:3], Rotation.from_quat(base[3:7]).as_euler('xyz', degrees=True)[2])
stage = Usd.Stage.Open(str(ep / 'scene.usda'))
pick = args.pick_object
dest = result.get('destination_object') or 'plate'
support = (result.get('source_support_contact_model') or {})
surface = support.get('surface') or ''
# Suffix after /kitchen
suffix = '/shelves_main_group/corpus/board_0'
if '/kitchen/' in surface:
    suffix = '/' + surface.split('/kitchen/', 1)[1]
movable = tuple(n for n in ('plate', 'bowl', 'orange', 'apple', 'lime', 'wooden_bowl') if n != pick) + ('glass',)
world = CollisionWorld(
    stage, kin,
    pick_object=pick,
    destination_object=dest,
    source_surface_suffix=suffix,
    movable_objects=movable,
)

rigid = ('lemon', 'plate', 'bowl', 'orange', 'apple', 'lime', 'wooden_bowl')
with h5py.File(ep / 'trajectory.h5') as f:
    q_full = np.asarray(f['next_obs/q'][-1])
    for name in rigid:
        if name in f['next_obs']:
            world.set_obstacle_pose(name, np.asarray(f[f'next_obs/{name}'][-1, :7]))
    tcp0 = np.asarray(f['next_obs/tcp'][-1])

q = q_full[:7].copy()
names = result['robot_joint_names']
world.attachment = np.asarray(result['planning_attachment_tcp_to_lime'])
world.gripper_joint_position = float(q_full[names.index('finger_joint')])
world.allowed_phase = 'lift'
# Restore lift-floor semantics from the recorded close state.
if 'source_resting_min_z_m' in support:
    world.source_resting_min_z = float(support['source_resting_min_z_m'])
    world.source_resting_xy = np.asarray(support.get('source_resting_xy_m') or tcp0[:2], dtype=float)
    world.source_support_mode = support.get('mode') or 'native_rest_until_detached'
    world.source_detached = False
    world.source_lift_floor_z = float(support['source_resting_min_z_m']) - 0.0005
    if 'source_resting_root_m' in support:
        world.source_resting_root = np.asarray(support['source_resting_root_m'], dtype=float)

planner = LimePlanner(world, kin, dt=float(result['dt']), seed=int(result.get('seed') or 43), max_joint_speed=0.4)
tcp_T = kin.fk(q)
# Prefer measured TCP translation if present.
if tcp0 is not None and len(tcp0) >= 3:
    # Keep orientation from FK; translation from measured tcp if close.
    if float(np.linalg.norm(tcp_T[:3, 3] - tcp0[:3])) < 0.05:
        tcp_T = tcp_T.copy()
        tcp_T[:3, 3] = tcp0[:3]

dxs, dys, dzs = mm_list(args.dx_mm), mm_list(args.dy_mm), mm_list(args.dz_mm)
candidates = []
for dx in dxs:
    for dy in dys:
        for dz in dzs:
            if abs(dx) < 1e-9 and abs(dy) < 1e-9 and abs(dz) < 1e-9:
                continue  # skip identity
            candidates.append((dx, dy, dz))

# Prefer: larger |dy| out of shelf, moderate dz; pure-Z first band for cartesian.
def rank(c):
    dx, dy, dz = c
    pure_z = abs(dx) < 1e-9 and abs(dy) < 1e-9
    # score: prefer out-of-shelf (-y) with some z, penalize huge z for board_1 risk
    return (0 if pure_z else 1, abs(dy) < 0.04, -abs(dy), abs(dz - 0.06), abs(dx), dz)

candidates = sorted(candidates, key=rank)[: max(1, args.max_candidates)]

ignore = (args.pick_object,)
rows = []
ok_rows = []
t0 = time.perf_counter()
for i, (dx, dy, dz) in enumerate(candidates):
    delta = np.array([dx, dy, dz], dtype=float)
    target = tcp_T.copy()
    target[:3, 3] = tcp_T[:3, 3] + delta
    pure_z = abs(dx) < 1e-9 and abs(dy) < 1e-9
    # Match collect: shelf free motion uses transfer; pure Z can use lift cartesian.
    plan_phase = 'lift' if pure_z else 'transfer'
    world.allowed_phase = 'lift'
    row = {
        'index': i,
        'delta_m': delta.tolist(),
        'delta_mm': [round(1000 * x) for x in delta],
        'plan_phase': plan_phase,
        'ok': False,
    }
    started = time.perf_counter()
    try:
        path = planner.plan_pose(q, target, ignore, 1, phase=plan_phase)
        row['plan_status'] = (path.get('metadata') or {}).get('status')
        row['plan_method'] = (path.get('metadata') or {}).get('method')
        row['plan_samples'] = int(len(path['positions']))
        validation = planner.validate_path(path['positions'], ignore, 1, phase='lift')
        row['validation_valid'] = bool(validation.get('valid'))
        if not validation.get('valid'):
            colliders = validation.get('colliders') or []
            # validation structure may nest differently
            detail = validation
            if 'colliders' not in detail and isinstance(detail.get('measurement'), dict):
                detail = detail['measurement']
            row['validation'] = {
                'valid': False,
                'colliders': colliders or detail.get('colliders'),
                'invalid_waypoints': detail.get('invalid_waypoints'),
                'slack_m': detail.get('slack_m'),
                'obstacle': detail.get('obstacle'),
                'robot_part': detail.get('robot_part'),
            }
            # compact collider names
            raw = str(validation)
            boards = sorted({b for b in ('board_0', 'board_1', 'board_2', 'shelves') if b in raw})
            row['board_hits'] = boards
            row['error'] = 'FCL_INVALID'
        else:
            row['ok'] = True
            row['validation_valid'] = True
            ok_rows.append(row)
    except RuntimeError as exc:
        row['error'] = short_err(exc)
        raw = str(exc)
        row['board_hits'] = sorted({b for b in ('board_0', 'board_1', 'board_2', 'shelves') if b in raw})
    row['seconds'] = round(time.perf_counter() - started, 3)
    rows.append(row)
    print(
        f"CAND {i:02d} dmm={row['delta_mm']} phase={plan_phase} ok={row['ok']} "
        f"err={row.get('error')} boards={row.get('board_hits')} t={row['seconds']}",
        flush=True,
    )
    if len(ok_rows) >= 5:
        print('EARLY_STOP enough reachable candidates', flush=True)
        break

# Choose: prefer max |dy| among ok with dz<=0.10; else any ok with smallest board risk
def choose(oks):
    if not oks:
        return None
    scored = sorted(
        oks,
        key=lambda r: (
            -abs(r['delta_m'][1]),  # more retract
            abs(r['delta_m'][2] - 0.06),
            abs(r['delta_m'][0]),
            r['delta_m'][2],
        ),
    )
    return scored[0]


chosen = choose(ok_rows)
# Build minimal lift template from chosen WP0: WP0 -> further retract at same Z if needed -> rise
template = None
if chosen is not None:
    dx, dy, dz = chosen['delta_m']
    # If WP0 already cleared laterally (|dy|>=0.10), rise from there; else extend -Y at hold_z then rise.
    hold_z = max(dz, 0.04)
    y_final = dy if abs(dy) >= 0.10 else min(dy, -0.12)  # more negative
    waypoints = [[dx, dy, dz]]
    if abs(y_final - dy) > 1e-6:
        waypoints.append([dx, y_final, hold_z if dz < hold_z else dz])
    waypoints.append([dx, y_final if abs(y_final) >= abs(dy) else dy, max(0.12, hold_z + 0.08)])
    template = {
        'wp0_delta_m': [dx, dy, dz],
        'waypoints_delta_m': waypoints,
        'shelf_retract_y': float(y_final if abs(y_final) >= abs(dy) else dy),
        'shelf_preclear_z': float(dz if abs(dy) < 1e-9 else 0.0),
        'shelf_retract_clear_z': float(max(dz, 0.0)),
        'shelf_rise_z': float(waypoints[-1][2]),
        'shelf_retract_steps': 1 if abs(dy) >= 0.05 else 3,
        'plan_phase_wp0': chosen['plan_phase'],
        'rationale': (
            'WP0 from feasibility scan (plan+FCL). '
            'Minimal template: keep WP0, optional extend -Y at hold Z, then rise clear of board_1.'
        ),
    }

out = {
    'source_episode': str(ep),
    'pick_object': args.pick_object,
    'tcp0_m': tcp_T[:3, 3].tolist(),
    'q0': q.tolist(),
    'grid': {'dx_m': dxs, 'dy_m': dys, 'dz_m': dzs, 'tested': len(rows)},
    'method': (
        'Offline scan from saved bilateral-close state. '
        'Pure +Z → plan_pose(phase=lift) cartesian column; '
        'lateral/diagonal → plan_pose(phase=transfer) MotionGen (matches collect shelf remap). '
        'All candidates validate_path(phase=lift). No PhysX.'
    ),
    'reachable_count': len(ok_rows),
    'chosen': chosen,
    'lift_template': template,
    'candidates': rows,
    'elapsed_s': round(time.perf_counter() - t0, 2),
}
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(out, indent=2))
print('CHOSEN', json.dumps(chosen, indent=2) if chosen else None, flush=True)
print('TEMPLATE', json.dumps(template, indent=2) if template else None, flush=True)
print('WROTE', args.output, 'reachable', len(ok_rows), '/', len(rows), flush=True)
raise SystemExit(0 if chosen is not None else 3)
