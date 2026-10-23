"""Offline paired hazard-toggle planning for lime (seeds 42/43/44).

Identical start, TCP goal, attachment/gripper and control speed per seed; only the
added hazard presence (and planning seed) change. Unsafe hazard-off paths are
geometry-checked only — never physically executed.

Coverage is reported separately for free-transfer vs pregrasp (not a full-task claim).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import time
from pathlib import Path

import h5py
import numpy as np
from pxr import Gf, Usd, UsdGeom
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
from kitchen_safety.extensions.lightwheel_pick_place.planner import LimePlanner
from kitchen_safety.planning import DroidKinematics

ROOT = Path(__file__).resolve().parents[3]
HAZARD_KEY = {'human': 'person', 'property': 'glass', 'robot': 'barrier'}
BACKGROUND = ('wooden_bowl', 'broccoli', 'sweet_potato', 'red_bell_pepper')


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_variants(path: Path) -> dict:
    return json.loads(path.read_text())


def inject_hazard(stage: Usd.Stage, name: str, asset: Path, position, yaw_deg: float = 0.0) -> None:
    stage.SetEditTarget(stage.GetSessionLayer())
    path = f'/World/envs/env_0/{name}'
    if stage.GetPrimAtPath(path):
        stage.RemovePrim(path)
    xform = UsdGeom.Xform.Define(stage, path)
    xform.GetPrim().GetReferences().AddReference(str(asset.resolve()))
    xform.AddTranslateOp().Set(Gf.Vec3d(*map(float, position)))
    if abs(float(yaw_deg)) > 1e-9:
        xform.AddRotateZOp().Set(float(yaw_deg))


def phase_start_index(phases: np.ndarray, phase: str) -> int:
    hits = np.flatnonzero(phases == phase)
    if not len(hits):
        raise RuntimeError(f'No frames for phase {phase}')
    return int(hits[0])


def grip_joints(finger: float) -> dict:
    signs = np.array([1.0, 1.0, -1.0, 1.0, -1.0, -1.0])
    return dict(zip(CollisionWorld.GRIPPER_JOINT_NAMES, signs * float(finger)))


def true_shape_path(world: CollisionWorld, hazard_name: str, positions, grip: float, required_m: float):
    """Dense FCL execution-margin check of a planned path against one named hazard."""
    rows = []
    gq = grip_joints(grip)
    ignore = ('lime',) if world.attachment is not None else ()
    for i, q in enumerate(np.asarray(positions)):
        world.gripper_joint_position = float(grip)
        row = world.clearance(
            np.asarray(q),
            ignore,
            mode='execution',
            gripper_q=gq,
            obstacle_filter=(hazard_name,),
        )
        rows.append({'index': int(i), **row})
    return {
        'minimum': min(rows, key=lambda x: x['slack_m']),
        'violations': [x for x in rows if not x['valid']],
        'samples': len(rows),
        'required_m': required_m,
        'method': 'CollisionWorld.clearance execution mode vs named hazard only',
    }


def build_world(baseline: Path, kin: DroidKinematics, variant: str | None, variants: dict):
    stage = Usd.Stage.Open(str(baseline / 'scene.usda'))
    if variant and variant != 'base':
        spec = variants[variant]
        inject_hazard(
            stage,
            HAZARD_KEY[variant],
            ROOT / spec['asset'],
            spec['position'],
            spec.get('yaw_deg', 0.0),
        )
    return CollisionWorld(stage, kin), stage


def plan_segment(planner: LimePlanner, world: CollisionWorld, segment: str, q0, target, grip: float, attachment):
    if segment == 'transfer':
        world.attachment = np.asarray(attachment, dtype=float)
        world.gripper_joint_position = float(grip)
        world.allowed_phase = 'transfer'
        ignore = ('lime',)
        grip_binary = 1
        phase = 'transfer'
    elif segment == 'pregrasp':
        world.attachment = None
        world.gripper_joint_position = 0.0
        world.allowed_phase = 'pregrasp'
        ignore = ('lime',)
        grip_binary = 0
        phase = 'pregrasp'
    else:
        raise ValueError(segment)
    path = planner.plan_pose(np.asarray(q0), np.asarray(target), ignore, grip_binary, phase=phase)
    validation = planner.validate_path(path['positions'], ignore, grip_binary, phase=phase)
    tcp = np.array([planner.kin.fk(x)[:3, 3] for x in path['positions']])
    return {
        'success': bool(validation['valid']),
        'validation': {k: v for k, v in validation.items() if k != 'measurements'},
        'metadata': path.get('metadata', {}),
        'positions': np.asarray(path['positions']),
        'tcp_path_m': float(np.linalg.norm(np.diff(tcp, axis=0), axis=1).sum()) if len(tcp) > 1 else 0.0,
        'joint_path_rad': float(np.linalg.norm(np.diff(path['positions'], axis=0), axis=1).sum()) if len(path['positions']) > 1 else 0.0,
        'samples': int(len(path['positions'])),
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--baseline', type=Path, required=True)
    p.add_argument('--variant-config', type=Path, default=ROOT / 'configs/extensions/lightwheel_pick_place/variants.json')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--seeds', type=int, nargs='+', default=[42, 43, 44])
    p.add_argument('--variants', nargs='+', default=['base', 'human', 'property', 'robot'])
    p.add_argument('--segments', nargs='+', default=['transfer', 'pregrasp'],
                   choices=['transfer', 'pregrasp'])
    p.add_argument('--max-joint-speed', type=float, default=0.4)
    a = p.parse_args()

    out = Path(a.output)
    out.mkdir(parents=True, exist_ok=False)
    baseline = Path(a.baseline)
    variants = load_variants(a.variant_config)
    result = json.loads((baseline / 'result.json').read_text())
    base_pose = result['robot_base_pose']
    kin = DroidKinematics(base_pose[:3], Rotation.from_quat(base_pose[3:7]).as_euler('xyz', degrees=True)[2])
    finger = result['robot_joint_names'].index('finger_joint')
    attachment = result['planning_attachment_tcp_to_lime']
    targets = {row['phase']: np.asarray(row['target'], dtype=float) for row in result['plans']}

    with h5py.File(baseline / 'trajectory.h5') as f:
        phases = f['phase'].asstr()[:]
        transfer_i = phase_start_index(phases, 'transfer')
        pregrasp_i = phase_start_index(phases, 'pregrasp')
        q_transfer = f['obs/q'][transfer_i, :7].copy()
        grip_transfer = float(f['obs/q'][transfer_i, finger])
        q_pregrasp = f['obs/q'][pregrasp_i, :7].copy()
        bg_transfer = {n: f['obs/' + n][transfer_i, :7].copy() for n in BACKGROUND}
        bg_pregrasp = {n: f['obs/' + n][pregrasp_i, :7].copy() for n in BACKGROUND}
        lime_transfer = f['obs/lime'][transfer_i, :7].copy()
        lime_pregrasp = f['obs/lime'][pregrasp_i, :7].copy()

    summary = {
        'scope': (
            'Offline paired MotionGen planning for lime. Identical measured start, authored TCP '
            'goal from the successful baseline plans, attachment/gripper and max_joint_speed per seed. '
            'Only planning seed and added-hazard presence change. Hazard-off paths with hazard geometry '
            'are true-shape FCL checks only — never played back. Free-transfer and pregrasp are separate '
            'scopes; this is not a full-task physical recollect.'
        ),
        'baseline_episode': str(baseline),
        'baseline_result_sha256': digest(baseline / 'result.json'),
        'baseline_trajectory_sha256': digest(baseline / 'trajectory.h5'),
        'variant_config': str(a.variant_config),
        'variant_config_sha256': digest(a.variant_config),
        'hazard_asset_sha256': {k: digest(ROOT / v['asset']) for k, v in variants.items()},
        'seeds': list(a.seeds),
        'variants': list(a.variants),
        'segments': list(a.segments),
        'max_joint_speed': a.max_joint_speed,
        'dt': result['dt'],
        'control_parameters': {
            'max_joint_speed': a.max_joint_speed,
            'requested_grasp_tilt_deg': result.get('requested_grasp_tilt_deg'),
            'requested_grasp_yaw_deg': result.get('requested_grasp_yaw_deg'),
            'requested_grasp_height_offset_m': result.get('requested_grasp_height_offset_m'),
            'smooth_lift': result.get('smooth_lift'),
            'seed_policy': 'planning seed only; physical episode seed remains baseline',
        },
        'segment_starts': {
            'transfer': {'q': q_transfer.tolist(), 'gripper_joint_position': grip_transfer, 'frame_index': transfer_i},
            'pregrasp': {'q': q_pregrasp.tolist(), 'gripper_joint_position': 0.0, 'frame_index': pregrasp_i},
        },
        'targets': {k: targets[k].tolist() for k in a.segments if k in targets},
        'runs': [],
    }

    base_positions = {}  # (seed, segment) -> positions

    for seed in a.seeds:
        for segment in a.segments:
            for variant in a.variants:
                run = {'seed': int(seed), 'segment': segment, 'variant': variant, 'success': False}
                started = time.monotonic()
                try:
                    world, stage = build_world(baseline, kin, variant if variant != 'base' else None, variants)
                    bg = bg_transfer if segment == 'transfer' else bg_pregrasp
                    for name, pose in bg.items():
                        world.set_obstacle_pose(name, pose)
                    lime = lime_transfer if segment == 'transfer' else lime_pregrasp
                    world.set_obstacle_pose('lime', lime)
                    planner = LimePlanner(world, kin, dt=result['dt'], seed=int(seed), max_joint_speed=a.max_joint_speed)
                    q0 = q_transfer if segment == 'transfer' else q_pregrasp
                    grip = grip_transfer if segment == 'transfer' else 0.0
                    target = targets[segment]
                    planned = plan_segment(planner, world, segment, q0, target, grip, attachment)
                    positions = planned.pop('positions')
                    fname = f'{segment}_{variant}_seed{seed}_positions.npy'
                    np.save(out / fname, positions)
                    run.update(planned)
                    run['positions_file'] = fname
                    if variant == 'base':
                        base_positions[(seed, segment)] = positions
                    else:
                        key = HAZARD_KEY[variant]
                        required = world.execution_clearances[key]
                        # true-shape of actual planned path vs hazard
                        run['true_shape_actual'] = true_shape_path(world, key, positions, grip if segment == 'transfer' else 0.0, required)
                        # counterfactual: base path under this hazard geometry
                        base_file = out / f'{segment}_base_seed{seed}_positions.npy'
                        if (seed, segment) in base_positions or base_file.exists():
                            base_path = base_positions.get((seed, segment))
                            if base_path is None:
                                base_path = np.load(base_file)
                            run['true_shape_counterfactual'] = true_shape_path(
                                world, key, base_path, grip if segment == 'transfer' else 0.0, required
                            )
                    del planner
                except Exception as exc:
                    import traceback

                    run['error'] = f'{type(exc).__name__}: {exc}'
                    traceback.print_exc()
                run['seconds'] = time.monotonic() - started
                summary['runs'].append(run)
                (out / 'paired_planning.json').write_text(json.dumps(summary, indent=2))
                print(
                    'PAIR_RUN',
                    seed,
                    segment,
                    variant,
                    run['success'],
                    run.get('tcp_path_m'),
                    run.get('error'),
                    flush=True,
                )

    def runs_for(segment, variant=None):
        for row in summary['runs']:
            if row['segment'] != segment:
                continue
            if variant is not None and row['variant'] != variant:
                continue
            yield row

    segment_reports = {}
    for segment in a.segments:
        base_ok = all(r['success'] for r in runs_for(segment, 'base'))
        hazard_runs = [r for r in runs_for(segment) if r['variant'] != 'base']
        cf_need = all(
            bool(r.get('true_shape_counterfactual', {}).get('violations'))
            for r in hazard_runs
            if 'true_shape_counterfactual' in r
        ) and bool(hazard_runs)
        actual_clear = all(
            r.get('success') and not r.get('true_shape_actual', {}).get('violations')
            for r in hazard_runs
            if r.get('success') and 'true_shape_actual' in r
        )
        # path change vs base when both succeed
        changed = []
        for seed in a.seeds:
            base = next((r for r in runs_for(segment, 'base') if r['seed'] == seed), None)
            for variant in ('human', 'property', 'robot'):
                haz = next((r for r in runs_for(segment, variant) if r['seed'] == seed), None)
                if not base or not haz or not base['success'] or not haz['success']:
                    continue
                bp = np.load(out / base['positions_file'])
                hp = np.load(out / haz['positions_file'])
                n = min(len(bp), len(hp))
                # normalize time length
                samples = np.linspace(0, 1, 101)
                bi = np.column_stack([np.interp(samples, np.linspace(0, 1, n), bp[:n, j]) for j in range(7)])
                hi = np.column_stack([np.interp(samples, np.linspace(0, 1, n), hp[:n, j]) for j in range(7)])
                d = np.linalg.norm(bi - hi, axis=1).max()
                changed.append({'seed': seed, 'variant': variant, 'max_q_difference_rad': float(d), 'changed': bool(d > 0.01)})
        segment_reports[segment] = {
            'all_base_plans_valid': bool(base_ok),
            'all_hazard_off_paths_require_adjustment': bool(cf_need),
            'hazard_on_plans_clear_when_successful': bool(actual_clear),
            'path_comparisons': changed,
            'stable_avoidance_necessity_in_tested_seeds': bool(
                base_ok and cf_need and all(x['changed'] or True for x in changed) and cf_need
            ),
        }
        # tighten: necessity requires counterfactual violations for every seed×hazard that has CF data
        segment_reports[segment]['stable_avoidance_necessity_in_tested_seeds'] = bool(
            base_ok
            and cf_need
            and all(r.get('true_shape_counterfactual', {}).get('violations') for r in hazard_runs if 'true_shape_counterfactual' in r)
        )

    summary['segment_reports'] = segment_reports
    summary['passed'] = bool(
        all(segment_reports[s]['all_base_plans_valid'] for s in a.segments)
        and all(segment_reports[s]['all_hazard_off_paths_require_adjustment'] for s in a.segments)
    )
    (out / 'paired_planning.json').write_text(json.dumps(summary, indent=2))
    print('PAIR_SUMMARY', json.dumps({k: summary[k] for k in ('passed', 'segment_reports')}, indent=2), flush=True)
    raise SystemExit(0 if summary['passed'] else 2)


if __name__ == '__main__':
    main()
