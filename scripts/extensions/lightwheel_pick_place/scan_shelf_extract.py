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
movable = tuple(n for n in ('plate', 'bowl', 'orange', 'apple') if n != pick) + ('glass',)
world = CollisionWorld(
    stage, kin,
    pick_object=pick,
    destination_object=dest,
    source_surface_suffix=suffix,
    movable_objects=movable,
)

rigid = ('lemon', 'plate', 'bowl', 'orange', 'apple')
with h5py.File(ep / 'trajectory.h5') as f:
    q_full = np.asarray(f['next_obs/q'][-1])
    for name in rigid:
        if name == pick:
            continue  # carried via attachment
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

# Stage-2: if WP0 is pure +Z, advance to that end-q and scan lateral deltas from there.
stage2_rows=[]; stage2_ok=[]; stage2_q=None; stage2_tcp=None
if chosen is not None:
    dx0,dy0,dz0=chosen['delta_m']
    if abs(dx0)<1e-9 and abs(dy0)<1e-9 and dz0>0:
        print('STAGE2 from WP0', chosen['delta_mm'], flush=True)
        target0=tcp_T.copy(); target0[:3,3]=tcp_T[:3,3]+np.array(chosen['delta_m'])
        path0=planner.plan_pose(q, target0, ignore, 1, phase='lift')
        stage2_q=np.asarray(path0['positions'][-1]).copy()
        stage2_tcp=kin.fk(stage2_q)
        # Prefer small laterals; try cartesian_line (zero-padding lift phase) before MotionGen.
        lat_dy=[-0.02,-0.03,-0.04,-0.05,-0.06,-0.08,-0.10,-0.12,-0.16]
        lat_dz=[0.0,0.01,0.02,0.04]
        lat_dx=[0.0]
        s2=[]
        for dy in lat_dy:
            for dz in lat_dz:
                for dx in lat_dx:
                    s2.append((dx,dy,dz))
        # small |dy| first (chainable micro steps), then deeper retracts
        s2=sorted(s2, key=lambda c: (abs(c[1]), abs(c[2]), abs(c[0])))[:40]
        for i,(dx,dy,dz) in enumerate(s2):
            delta=np.array([dx,dy,dz],dtype=float)
            target=stage2_tcp.copy(); target[:3,3]=stage2_tcp[:3,3]+delta
            row={'index':i,'delta_m':delta.tolist(),'delta_mm':[round(1000*x) for x in delta],
                 'ok':False,'relative_to':'wp0'}
            started=time.perf_counter()
            path=None; used=None
            for method in ('cartesian_line','transfer'):
                try:
                    if method=='cartesian_line':
                        path=planner.plan_cartesian_line(stage2_q, target, ignore, 1, phase='lift')
                    else:
                        path=planner.plan_pose(stage2_q, target, ignore, 1, phase='transfer')
                    validation=planner.validate_path(path['positions'], ignore, 1, phase='lift')
                    if validation.get('valid'):
                        row['ok']=True; row['plan_phase']=method; used=method
                        row['plan_status']=(path.get('metadata') or {}).get('status')
                        stage2_ok.append(row); break
                    else:
                        raw=str(validation)
                        row['error']='FCL_INVALID'
                        row['board_hits']=sorted({b for b in ('board_0','board_1','board_2','shelves') if b in raw})
                        row['plan_phase']=method
                except RuntimeError as exc:
                    row['error']=short_err(exc); row['plan_phase']=method
                    raw=str(exc)
                    row['board_hits']=sorted({b for b in ('board_0','board_1','board_2','shelves') if b in raw})
                    path=None
            row['seconds']=round(time.perf_counter()-started,3)
            stage2_rows.append(row)
            print(f"S2 {i:02d} dmm={row['delta_mm']} ok={row['ok']} phase={row.get('plan_phase')} err={row.get('error')} t={row['seconds']}", flush=True)
            if len(stage2_ok)>=5:
                print('STAGE2_EARLY_STOP', flush=True); break

# Build minimal lift template from chosen WP0
# Build minimal lift template from chosen WP0.
template = None
stage2_chosen = choose(stage2_ok) if stage2_ok else None
if chosen is not None:
    dx, dy, dz = chosen['delta_m']
    waypoints = [[dx, dy, dz]]
    if stage2_chosen is not None:
        dx2, dy2, dz2 = stage2_chosen['delta_m']
        # Absolute from original TCP: WP0 + stage2 relative
        abs2 = [dx+dx2, dy+dy2, dz+dz2]
        waypoints.append(abs2)
        # Final rise at retracted Y
        waypoints.append([abs2[0], abs2[1], max(abs2[2]+0.08, 0.12)])
        template = {
            'wp0_delta_m': [dx, dy, dz],
            'wp1_delta_m': abs2,
            'wp1_relative_to_wp0_m': [dx2, dy2, dz2],
            'waypoints_delta_m': waypoints,
            'shelf_preclear_z': float(dz),
            'shelf_retract_y': float(abs2[1]),
            'shelf_retract_clear_z': float(abs2[2]),
            'shelf_rise_z': float(waypoints[-1][2]),
            'shelf_retract_steps': 1,
            'plan_phase_wp0': chosen['plan_phase'],
            'plan_phase_wp1': stage2_chosen.get('plan_phase') or 'cartesian_line',
            'rationale': (
                'Stage1 WP0 pure-Z cartesian lift+FCL; stage2 lateral from WP0 end-q via transfer+FCL; '
                'then rise at cleared Y.'
            ),
        }
    else:
        # Fallback: WP0 only + hopeful retract (may still fail)
        waypoints.append([0.0, -0.12, max(dz, 0.04)])
        waypoints.append([0.0, -0.12, 0.12])
        template = {
            'wp0_delta_m': [dx, dy, dz],
            'waypoints_delta_m': waypoints,
            'shelf_preclear_z': float(dz),
            'shelf_retract_y': -0.12,
            'shelf_retract_clear_z': float(max(dz, 0.04)),
            'shelf_rise_z': 0.12,
            'shelf_retract_steps': 1,
            'plan_phase_wp0': chosen['plan_phase'],
            'rationale': 'Only stage1 WP0 reachable; stage2 lateral unresolved.',
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
    'stage2_chosen': stage2_chosen if chosen is not None else None,
    'stage2_reachable_count': len(stage2_ok),
    'stage2_candidates': stage2_rows,
    'lift_template': template,
    'candidates': rows,
    'elapsed_s': round(time.perf_counter() - t0, 2),
}
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(out, indent=2))
print('CHOSEN', json.dumps(chosen, indent=2) if chosen else None, flush=True)
print('TEMPLATE', json.dumps(template, indent=2) if template else None, flush=True)
print('WROTE', args.output, 'reachable', len(ok_rows), '/', len(rows), 'stage2', len(stage2_ok), flush=True)
raise SystemExit(0 if (chosen is not None and (stage2_chosen is not None or abs(chosen['delta_m'][1])>=0.05)) else (0 if chosen is not None else 3))
