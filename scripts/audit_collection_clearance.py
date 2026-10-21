"""Recheck recorded robot/object collision envelopes against each safety obstacle.

Run with the Arena Python environment; Isaac Sim itself is not started::

    .venv-arena/bin/python scripts/audit_collection_clearance.py \
        datasets/kitchen_auto/human/episode_002 --report reports/clearance.json

This checks saved action-rate states, including the final next_obs. The collector's
physics-rate checks cover the intervening states. Geometry comes from the saved
asset reference (or legacy layout), while placements and margins come from that
episode. Recorded hazard poses override the initial placement for moving props.
An envelope intersection does not by itself prove that visible meshes penetrate.
"""
import argparse
import json
import sys
from pathlib import Path

import h5py
import numpy as np
from pxr import Gf, Usd, UsdGeom

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from kitchen_safety.collision_planner import CollisionWorld
from kitchen_safety.planning import DroidKinematics, transform


CARRY_PHASES = {'lift', 'transfer_front', 'transfer', 'above_bowl', 'lower'}
HAZARDS = {'human': 'person', 'property': 'glass', 'robot': 'barrier'}


def hazard_stage(layout, variant, obstacle=None):
    """Compose only the requested obstacle, without unrelated kitchen geometry."""
    if obstacle is not None:
        if obstacle.get('category', variant) != variant:
            raise ValueError('Saved obstacle category does not match the episode variant')
        asset = obstacle['usd_path']
        position = obstacle['position']
        yaw = obstacle.get('yaw_deg', 0)
    elif variant == 'human':
        asset = layout['human'].get('asset', 'assets/people/mpfb/collidable.usda')
        position = layout['human']['position']
        yaw = layout['human'].get('yaw_deg', 0)
    elif variant == 'property':
        asset = layout['property'].get('glass_asset', 'assets/safety/glass.usda')
        position = layout['property']['glass_position']
        yaw = layout['property'].get('glass_yaw_deg', 0)
    elif variant == 'robot':
        asset = layout['robot'].get('barrier_asset', 'assets/safety/barrier.usda')
        position = layout['robot']['barrier_position']
        yaw = layout['robot'].get('barrier_yaw_deg', 0)
    else:
        raise ValueError(f'No safety obstacle is defined for variant {variant!r}')
    asset_path = ROOT / asset
    if not asset_path.is_file():
        raise FileNotFoundError(asset_path)
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, 'Z')
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    prim = UsdGeom.Xform.Define(stage, '/World/' + HAZARDS[variant].title())
    prim.GetPrim().GetReferences().AddReference(str(asset_path))
    prim.ClearXformOpOrder()
    prim.AddTranslateOp().Set(Gf.Vec3d(*position))
    if yaw:
        prim.AddRotateZOp().Set(yaw)
    return stage, {'asset': asset, 'position': position, 'yaw_deg': yaw,
                   'source': 'saved obstacle entry' if obstacle is not None else 'saved layout'}


def audit_episode(directory):
    directory = Path(directory).resolve()
    result = json.loads((directory / 'result.json').read_text())
    layout, variant = result['layout'], result['variant']
    hazard = HAZARDS[variant]
    obstacle = result.get('obstacle')
    stage, hazard_info = hazard_stage(layout, variant, obstacle)
    base = np.array(layout['robot_position'], dtype=float)
    base[2] += layout['stand_height']
    kin = DroidKinematics(base, layout['robot_yaw_deg'])
    settings = layout.get('planning', {})
    world = CollisionWorld(
        stage, kin,
        clearances=settings.get('clearances_m', {}),
        execution_clearances=settings.get('execution_clearances_m', {}),
        default_clearance=settings.get('default_clearance_m', .002))
    if not world.objects:
        raise RuntimeError(f'No obstacle collision geometry loaded for {hazard_info}')

    with h5py.File(directory / 'trajectory.h5') as data:
        joint_names = json.loads(data.attrs['joint_names'])
        arm_indices = [joint_names.index(f'panda_joint{i}') for i in range(1, 8)]
        gripper_indices = {name: joint_names.index(name)
                           for name in CollisionWorld.GRIPPER_JOINT_NAMES}
        n = len(data['obs/q'])
        if n == 0:
            raise ValueError('The episode has no recorded states')
        phases = data['phase'].asstr()[:].tolist()
        if len(phases) != n:
            raise ValueError('Phase/state counts differ')
        dt = float(data.attrs['dt'])
        times = data['time'][:].tolist() if 'time' in data else (np.arange(n) * dt).tolist()
        fields = ('q', 'mustard', 'tcp', 'tcp_quat_xyzw', 'hazard_pose')
        states = {}
        for field in fields:
            if field not in data['obs']:
                if field in ('tcp_quat_xyzw', 'hazard_pose'):
                    continue
                raise ValueError(f'Missing recorded observation {field!r}')
            values = data['obs/' + field][:]
            final = data['next_obs/' + field][-1:]
            if len(values) != n or not np.isfinite(values).all() or not np.isfinite(final).all():
                raise ValueError(f'Invalid recorded observation {field!r}')
            states[field] = np.concatenate([values, final], axis=0)
        if 'hazard_pose' in states and states['hazard_pose'].shape != (n + 1, 7):
            raise ValueError('hazard_pose must contain one xyz + xyzw pose per state')
        if (obstacle is not None and not obstacle.get('static', True)
                and 'hazard_pose' not in states):
            raise ValueError('Dynamic obstacle episode lacks recorded hazard_pose; '
                             'initial placement cannot establish execution clearance')
        phases.append(phases[-1])
        times.append(times[-1] + dt)

    rows, phase_minima = [], {}
    for index in range(n + 1):
        if 'hazard_pose' in states:
            world.set_obstacle_pose(hazard, states['hazard_pose'][index])
        actual_q = states['q'][index]
        q_arm = actual_q[arm_indices]
        gripper_q = {name: float(actual_q[i]) for name, i in gripper_indices.items()}
        bottle_pose = states['mustard'][index, :7]
        if phases[index] in CARRY_PHASES:
            if 'tcp_quat_xyzw' in states:
                tcp_pose = transform(states['tcp'][index], states['tcp_quat_xyzw'][index])
            else:
                tcp_pose = kin.fk(q_arm)
                tcp_pose[:3, 3] = states['tcp'][index]
            world.attachment = np.linalg.inv(tcp_pose) @ transform(bottle_pose[:3], bottle_pose[3:])
        else:
            world.attachment = None
        # Always use the observed bottle pose, including closing and release,
        # so a slip cannot be hidden by a perfect virtual attachment.
        measurement = world.clearance(
            q_arm, mode='execution', gripper_q=gripper_q,
            object_pose=bottle_pose, obstacle_filter=(hazard,))
        if measurement['obstacle'] is None or not np.isfinite(measurement['distance_m']):
            raise RuntimeError(f'No finite clearance measurement for {hazard!r}')
        row = dict(sample_index=index, frame=index if index < n else None,
                   state_source='obs' if index < n else 'final_next_obs',
                   time_s=float(times[index]), phase=phases[index], **measurement)
        rows.append(row)
        current = phase_minima.get(phases[index])
        if current is None or row['slack_m'] < current['slack_m'] or (
                row['intersecting'] and not current['intersecting']):
            phase_minima[phases[index]] = row

    minimum = min(rows, key=lambda row: (row['slack_m'], not row['intersecting']))
    failures = [row for row in rows if not row['valid']]
    intersections = [row['sample_index'] for row in rows if row['intersecting']]
    return dict(
        path=str(directory), variant=variant, obstacle_id=result.get('obstacle_id'),
        recorded_success=result.get('success'),
        clearance_passed=not failures, hazard=hazard, hazard_geometry=hazard_info,
        robot_base_position=base.tolist(), robot_base_yaw_deg=layout['robot_yaw_deg'],
        frames=n, states_checked=n + 1, sampling_dt_s=dt,
        gripper_joint_angles='recorded, mapped by joint_names',
        carried_object_pose='recorded xyz + xyzw, including before grasp and after release',
        hazard_pose='recorded xyz + xyzw at each state' if 'hazard_pose' in states else 'fixed episode placement (legacy/static)',
        attachment_tcp_orientation='recorded' if 'tcp_quat_xyzw' in states else 'USD FK fallback',
        minimum_clearance_m=minimum['distance_m'], minimum_clearance_sample=minimum,
        phase_minima=phase_minima, failed_sample_count=len(failures),
        intersecting_sample_count=len(intersections), intersecting_samples=intersections,
        failed_samples=failures, samples=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episodes', nargs='+', help='Episode directories containing result.json and trajectory.h5')
    parser.add_argument('--report', default='reports/collection_clearance_audit.json')
    args = parser.parse_args()
    episodes = []
    for directory in args.episodes:
        try:
            item = audit_episode(directory)
        except Exception as error:
            item = dict(path=str(Path(directory).resolve()), clearance_passed=False,
                        error=f'{type(error).__name__}: {error}')
        episodes.append(item)
        summary = {key: item[key] for key in (
            'path', 'clearance_passed', 'states_checked', 'minimum_clearance_m',
            'failed_sample_count', 'intersecting_sample_count', 'error') if key in item}
        print(json.dumps(summary), flush=True)
    report = dict(
        passed=all(item['clearance_passed'] for item in episodes),
        scope='Recorded action-rate states plus the final next_obs; current conservative collision envelopes; '
              'saved obstacle entry/layout and execution margins; recorded hazard poses when available; '
              'person/glass/barrier only. '
              'Intersections report zero separation, not penetration depth. '
              'Envelope intersection does not by itself prove visible mesh penetration. '
              'This audit does not replace physics-rate execution checks or visual review.',
        episodes=episodes)
    path = Path(args.report)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
