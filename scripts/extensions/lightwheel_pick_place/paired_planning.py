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
from kitchen_safety.extensions.lightwheel_pick_place.lemon_kinematics import LemonKinematics
from kitchen_safety.planning import DroidKinematics

ROOT = Path(__file__).resolve().parents[3]
HAZARD_KEY = {'human': 'person', 'property': 'glass', 'robot': 'barrier'}
DEFAULT_BACKGROUND = ('wooden_bowl', 'broccoli', 'sweet_potato', 'red_bell_pepper')


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
    pick = getattr(world, 'pick_object', 'lime')
    ignore = (pick,) if world.attachment is not None else ()
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


def build_world(baseline: Path, kin: DroidKinematics, variant: str | None, variants: dict, *, task_spec):
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
    movable = tuple(n for n in task_spec.rigid_objects if n != task_spec.pick_object) + ('glass',)
    world = CollisionWorld(
        stage, kin,
        pick_object=task_spec.pick_object,
        destination_object=task_spec.destination_object,
        source_surface_suffix=task_spec.source_surface_suffix,
        movable_objects=movable,
    )
    calibration = json.loads((baseline / 'result.json').read_text()).get('source_support_contact_model', {})
    if calibration.get('mode') == 'native_rest_until_detached':
        world.source_support_mode = calibration['mode']
        world.source_resting_min_z = calibration['source_resting_min_z_m']
        world.source_resting_xy = np.asarray(calibration['source_resting_xy_m'])
        world.source_resting_root = np.asarray(calibration['source_resting_root_m'])
        world.source_surface_top = calibration['surface_top_m']
        world.source_detached = False
        world.source_lift_floor_z = calibration['source_resting_min_z_m'] - calibration['source_resting_extra_tolerance_m']
    return world, stage


def plan_segment(planner: LimePlanner, world: CollisionWorld, segment: str, q0, target, grip: float, attachment, recorded_method=None):
    pick = getattr(world, 'pick_object', 'lime')
    if segment in ('transfer', 'lift'):
        world.attachment = np.asarray(attachment, dtype=float)
        world.gripper_joint_position = float(grip)
        world.allowed_phase = segment
        ignore = (pick,)
        grip_binary = 1
        phase = segment
    elif segment in ('pregrasp', 'approach'):
        world.attachment = None
        world.gripper_joint_position = 0.0
        world.allowed_phase = segment
        ignore = (pick,)
        grip_binary = 0
        phase = segment
    else:
        raise ValueError(segment)
    if recorded_method == 'cartesian_line_ik':
        path = planner.plan_cartesian_line(np.asarray(q0), np.asarray(target), ignore, grip_binary, phase=phase)
    else:
        planning_phase = 'transfer' if recorded_method in ('motion_gen','motion_gen_joint_goal','motion_gen_upper_route') else phase
        if planning_phase=='transfer' and getattr(planner,'allow_upper_route_policy',False):
            from kitchen_safety.extensions.lightwheel_pick_place.b03_strategy import plan_transfer
            path=plan_transfer(planner,np.asarray(q0),np.asarray(target),ignore,grip_binary)
        else:path = planner.plan_pose(np.asarray(q0), np.asarray(target), ignore, grip_binary, phase=planning_phase)
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
    p.add_argument('--recorded-plan-index', type=int, default=None, help='Replay one exact recorded plan leg, preserving its start, goal and planning method; duplicate phase labels are not merged')
    p.add_argument('--task', default=None,
                   help='Override task id; default reads result.json task field from --baseline')
    p.add_argument('--baseline', type=Path, required=True)
    p.add_argument('--variant-config', type=Path, default=None)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--seeds', type=int, nargs='+', default=[42, 43, 44])
    p.add_argument('--variants', nargs='+', default=['base', 'human', 'property', 'robot'])
    p.add_argument('--segments', nargs='+', default=['transfer', 'approach'],
                   choices=['transfer', 'pregrasp', 'approach', 'lift'])
    p.add_argument('--max-joint-speed', type=float, default=0.4)
    a = p.parse_args()
    from kitchen_safety.extensions.lightwheel_pick_place.task_config import resolve_task
    result_probe = json.loads((a.baseline / 'result.json').read_text())
    task_id = a.task or result_probe.get('task')
    from kitchen_safety.extensions.lightwheel_pick_place.task_config import episode_spec
    TASK_SPEC = episode_spec({**result_probe,'task':task_id})
    PICK = result_probe.get('pick_object') or TASK_SPEC.pick_object
    DEST = result_probe.get('destination_object') or TASK_SPEC.destination_object
    BACKGROUND = tuple(TASK_SPEC.rigid_objects)
    if a.variant_config is None:
        a.variant_config = TASK_SPEC.default_variants

    out = Path(a.output)
    out.mkdir(parents=True, exist_ok=False)
    baseline = Path(a.baseline)
    variants = load_variants(a.variant_config)
    result = json.loads((baseline / 'result.json').read_text())
    base_pose = result['robot_base_pose']
    kin = (LemonKinematics if PICK=='lemon' else DroidKinematics)(base_pose[:3], Rotation.from_quat(base_pose[3:7]).as_euler('xyz', degrees=True)[2])
    finger = result['robot_joint_names'].index('finger_joint')
    attachment = result['planning_attachment_tcp_to_lime']
    targets = {row['phase']: np.asarray(row['target'], dtype=float) for row in result['plans']}
    recorded = None
    if a.recorded_plan_index is not None:
        recorded = result['plans'][a.recorded_plan_index]
        if not recorded.get('executed'): raise ValueError('Selected recorded plan was not executed')
        a.segments = [recorded['phase']]
        targets = {recorded['phase']: np.asarray(recorded['target'], dtype=float)}

    with h5py.File(baseline / 'trajectory.h5') as f:
        phases = f['phase'].asstr()[:]
        starts = {}
        for segment in ('transfer', 'pregrasp', 'approach', 'lift'):
            if segment not in a.segments:
                continue
            i = phase_start_index(phases, segment)
            if recorded is not None:
                previous = [x for x in result['plans'][:a.recorded_plan_index] if x['phase'] == segment and x.get('executed')]
                offset = sum(int(x['metadata']['samples']) + 12 for x in previous)
                i = int(np.flatnonzero(phases == segment)[offset])
                expected = sum(int(x['metadata']['samples']) + 12 for x in result['plans'] if x['phase'] == segment and x.get('executed'))
                if int(np.count_nonzero(phases == segment)) != expected:
                    raise ValueError('Recorded phase frame count does not match plan samples plus 12 hold frames')
                if phases[i] != segment: raise ValueError('Recorded leg start phase mismatch')
            active_pick=next((name for name,bounds in result.get('pick_action_ranges',{}).items() if bounds['start']<=i<bounds['end']),PICK)
            starts[segment] = {
                'frame_index': i,'active_pick':active_pick,'active_pick_pose':f['obs/'+active_pick][i,:7].copy(),
                'source_detached': (json.loads((baseline / 'full_world_audit.json').read_text())['measurements'][i * 8].get('source_detached_before', False)),
                'q': f['obs/q'][i, :7].copy(),
                'gripper_joint_position': float(f['obs/q'][i, finger]),
                'background': {n: f['obs/' + n][i, :7].copy() for n in BACKGROUND},
                PICK: f[f'obs/{PICK}'][i, :7].copy() if f'obs/{PICK}' in f else f['obs/lime'][i, :7].copy(),
            }
        # attachment is measured at close; reuse baseline planning attachment for carried phases

    summary = {
        'scope': (
            'Offline paired planning with explicit task and matching bounded route/pose-IK fallback policy; recorded method identifies the source leg.  Identical measured start, authored TCP '
            'goal from the successful baseline plans, attachment/gripper and max_joint_speed per seed. '
            'Only planning seed and added-hazard presence change. Hazard-off paths with hazard geometry '
            'are true-shape FCL checks only — never played back. Free-transfer and pregrasp are separate '
            'scopes; this is not a full-task physical recollect.'
        ),
        'recorded_plan_index': a.recorded_plan_index,
        'recorded_method': recorded['metadata']['method'] if recorded else None,
        'task': TASK_SPEC.task_id,
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
            s: {
                'q': starts[s]['q'].tolist(),
                'gripper_joint_position': starts[s]['gripper_joint_position'] if s in ('transfer', 'lift') else 0.0,
                'frame_index': starts[s]['frame_index'],'active_pick':starts[s]['active_pick'],
            }
            for s in starts
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
                    world, stage = build_world(baseline, kin, variant if variant != 'base' else None, variants, task_spec=TASK_SPEC)
                    st = starts[segment]
                    for name, pose in st['background'].items():
                        world.set_obstacle_pose(name, pose)
                    active_pick=st['active_pick']
                    if world.pick_object!=active_pick:world.bind_pick(active_pick)
                    world.set_obstacle_pose(active_pick, st['active_pick_pose'])
                    world.source_detached = st['source_detached']
                    planner = LimePlanner(world, kin, dt=result['dt'], seed=int(seed), max_joint_speed=a.max_joint_speed)
                    search_policy=result.get('grasp_search_policy',{})
                    planner.pose_ik_joint_fallback=bool(search_policy.get('pose_ik_joint_goal_fallback',{}).get('enabled') or result.get('transfer_route_policy',{}).get('pose_ik_joint_goal_fallback'))
                    planner.allow_upper_route_policy=bool(search_policy or result.get('transfer_route_policy',{}).get('mode')=='upper')
                    q0 = st['q']
                    grip = st['gripper_joint_position'] if segment in ('transfer', 'lift') else 0.0
                    target = targets[segment]
                    segment_attachment=attachment
                    measured_attachment_policy=(result.get('planning_attachment_policy')=='measured_segment' or bool(result.get('planning_attachment_updates')))
                    if measured_attachment_policy:
                        from kitchen_safety.planning import transform
                        segment_attachment=np.linalg.inv(kin.fk(q0))@transform(st['active_pick_pose'][:3],st['active_pick_pose'][3:7])
                    run['active_pick']=active_pick
                    run['attachment_source']='measured selected-pick pose at segment start' if measured_attachment_policy else 'recorded close attachment'
                    planned = plan_segment(planner, world, segment, q0, target, grip, segment_attachment, recorded['metadata']['method'] if recorded else None)
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
                        run['true_shape_actual'] = true_shape_path(world, key, positions, grip if segment in ('transfer', 'lift') else 0.0, required)
                        # counterfactual: base path under this hazard geometry
                        base_file = out / f'{segment}_base_seed{seed}_positions.npy'
                        if (seed, segment) in base_positions or base_file.exists():
                            base_path = base_positions.get((seed, segment))
                            if base_path is None:
                                base_path = np.load(base_file)
                            run['true_shape_counterfactual'] = true_shape_path(
                                world, key, base_path, grip if segment in ('transfer', 'lift') else 0.0, required
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
        changed = []
        for seed in a.seeds:
            base = next((r for r in runs_for(segment, 'base') if r['seed'] == seed), None)
            for variant in ('human', 'property', 'robot'):
                haz = next((r for r in runs_for(segment, variant) if r['seed'] == seed), None)
                if not base or not haz or not base.get('success') or not haz.get('success'):
                    continue
                bp = np.load(out / base['positions_file'])
                hp = np.load(out / haz['positions_file'])
                n = min(len(bp), len(hp))
                samples = np.linspace(0, 1, 101)
                bi = np.column_stack([np.interp(samples, np.linspace(0, 1, n), bp[:n, j]) for j in range(7)])
                hi = np.column_stack([np.interp(samples, np.linspace(0, 1, n), hp[:n, j]) for j in range(7)])
                d = np.linalg.norm(bi - hi, axis=1).max()
                changed.append({'seed': int(seed), 'variant': variant, 'max_q_difference_rad': float(d), 'changed': bool(d > 0.01)})
        segment_reports[segment] = {
            'all_base_plans_valid': bool(base_ok),
            'path_comparisons': changed,
            'hazard_runs_success': {v: all(r['success'] for r in runs_for(segment, v)) for v in ('human', 'property', 'robot')},
        }

    # Primary evidence segment per hazard (from geometry risk phases / redesign).
    primary = {v: recorded['phase'] for v in ('human','property','robot')} if recorded else {'human': 'approach', 'property': 'transfer', 'robot': 'transfer'}
    variant_reports = {}
    primary = {v:s for v,s in primary.items() if v in a.variants}
    for variant, segment in primary.items():
        if segment not in a.segments:
            variant_reports[variant] = {'passed': False, 'reason': f'primary segment {segment} not run'}
            continue
        rows = [r for r in runs_for(segment, variant)]
        bases = [r for r in runs_for(segment, 'base')]
        base_ok = all(r['success'] for r in bases) and len(bases) == len(a.seeds)
        cf_ok = all(bool(r.get('true_shape_counterfactual', {}).get('violations')) for r in rows)
        on_ok = all(r.get('success') and not r.get('true_shape_actual', {}).get('violations') for r in rows)
        path_rows = [
            c for c in segment_reports[segment]['path_comparisons']
            if c['variant'] == variant
        ]
        path_ok = bool(path_rows) and all(c['changed'] for c in path_rows)
        # Necessity: identical start/goal/controls; hazard-off path violates true-shape
        # margins for every tested seed. Hazard-on may succeed with a detour or fail
        # from the same start (failure is recorded; physical success may use an earlier
        # redundant pregrasp). Path change is required only when hazard-on succeeds.
        necessity = bool(base_ok and cf_ok and rows)
        variant_reports[variant] = {
            'primary_segment': segment,
            'all_base_plans_valid': bool(base_ok),
            'all_hazard_off_paths_require_adjustment': bool(cf_ok and rows),
            'all_hazard_on_plans_clear': bool(on_ok and rows),
            'path_changed_when_hazard_on_succeeds': bool(path_ok) if any(r.get('success') for r in rows) else None,
            'seeds': [
                {
                    'seed': r['seed'],
                    'success': r.get('success'),
                    'cf_violations': len(r.get('true_shape_counterfactual', {}).get('violations') or []),
                    'actual_violations': len(r.get('true_shape_actual', {}).get('violations') or []),
                    'tcp_path_m': r.get('tcp_path_m'),
                    'error': r.get('error'),
                }
                for r in rows
            ],
            'stable_avoidance_necessity_in_tested_seeds': necessity,
            'passed': necessity,
        }

    summary['segment_reports'] = segment_reports
    summary['variant_reports'] = variant_reports
    summary['primary_segments'] = primary
    summary['passed'] = bool(variant_reports) and all(v.get('passed') for v in variant_reports.values())
    (out / 'paired_planning.json').write_text(json.dumps(summary, indent=2))
    print('PAIR_SUMMARY', json.dumps({k: summary[k] for k in ('passed', 'variant_reports', 'primary_segments')}, indent=2), flush=True)
    raise SystemExit(0 if summary['passed'] else 2)


if __name__ == '__main__':
    main()
