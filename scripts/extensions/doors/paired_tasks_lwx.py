"""Predeclared 42/43/44 obstacle on/off pairs. No environment is stepped."""
import argparse
import gc
import hashlib
import json
import random
import time
from pathlib import Path
import numpy as np
from kitchen_safety.extensions.doors.task_reconstruction import reconstruct as task_reconstruct

def reconstruct(metadata,spec=None):
    stage,kin,world,_=task_reconstruct(metadata,metadata['_task_config'],spec)
    margin=metadata['controller_parameters'].get('ik_limit_margin_rad',0.)
    kin.limits[:,0]+=margin;kin.limits[:,1]-=margin
    return stage,kin,world
from kitchen_safety.extensions.doors.task_planner import TaskPlanner as CuroboPlanner
from scripts.extensions.doors.physical_hazard_geometry import physical_hazards, physical_robot_shapes
from scripts.extensions.doors.search_avoidance_layouts import clearance


def plan(start, metadata, spec, seed, offsets):
    import torch
    random.seed(seed); np.random.seed(seed); torch.manual_seed(seed)
    stage, kin, world = reconstruct(metadata, spec)
    planner = CuroboPlanner(world, kin, dt=metadata['dt'], seed=seed,
                            max_joint_speed=metadata['controller_parameters']['max_joint_speed'],
                            hazard_extra_padding_m=metadata['controller_parameters'].get('planner_hazard_extra_padding_m',0.),
                            pregrasp_shortcut_policy=metadata['controller_parameters'].get('pregrasp_shortcut_policy','none'),pregrasp_goal_q=metadata['controller_parameters'].get('pregrasp_goal_q'))
    failures = []
    output = {'success': False, 'failures': failures, 'positions': None,
              'seed': seed, 'initial_q': start.tolist(), 'candidate_offsets_m': offsets}
    from scipy.spatial.transform import Rotation
    approach=Rotation.from_euler('z',metadata['robot_yaw_deg'],degrees=True).apply([1.,0.,0.])
    from kitchen_safety.extensions.doors.task_model import grasp_rotation
    target = np.eye(4); target[:3,:3]=grasp_rotation(approach,metadata['controller_parameters']['grasp_roll_deg'],metadata['controller_parameters'].get('grasp_pitch_deg',0.))
    initial_height = metadata['controller_parameters']['handle_offset_z']
    started = time.perf_counter()
    for offset in offsets:
        target[:3, 3] = np.asarray(metadata['handle_initial']) - approach*.13 + [0., 0., offset - initial_height]
        try:
            trajectory = planner.plan_pose(start, target, phase='door_pregrasp')
        except RuntimeError as error:
            failures.append({'height_offset_m': offset, 'reason': str(error),
                             'target_pose': target.tolist(), 'executed_actions': 0})
            continue
        if 'pregrasp_validation' in metadata['controller_parameters']:
            from kitchen_safety.extensions.doors.pregrasp import POLICY, check_pregrasp
            if metadata['controller_parameters']['pregrasp_validation'] != POLICY:
                raise ValueError('Recorded pregrasp policy differs from current implementation')
            validation = check_pregrasp(planner, trajectory)
            trajectory['metadata']['pregrasp_validation'] = validation
            if not validation['accepted']:
                failures.append({'height_offset_m': offset,
                    'reason': 'Dense pre-execution geometry gate rejected cuRobo path',
                    'target_pose': target.tolist(), 'executed_actions': 0,
                    'positions': trajectory['positions'].tolist(), 'validation': validation,
                    'planner_metadata': trajectory['metadata']})
                continue
        output.update(success=True, selected_offset_m=offset,
                      target_pose=target.tolist(), positions=trajectory['positions'].tolist(),
                      metadata=trajectory['metadata'])
        positions = np.asarray(trajectory['positions'])
        tcp = np.asarray([kin.fk(q)[:3, 3] for q in positions])
        output['path_metrics'] = {
            'samples': len(positions),
            'duration_s': max(0, len(positions) - 1) * metadata['dt'],
            'joint_l2_length_rad': float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum()),
            'tcp_length_m': float(np.linalg.norm(np.diff(tcp, axis=0), axis=1).sum())}
        break
    output['planning_seconds'] = time.perf_counter() - started
    del planner, world, stage
    gc.collect(); torch.cuda.empty_cache()
    return output


def evaluate_path(plan_result, metadata, spec):
    if not plan_result['success']:
        return {'available': False}
    stage, kin, world = reconstruct(metadata, spec)
    hazard = physical_hazards(stage, world, spec['name'])
    margin = world.execution_clearances[spec['name']]
    robot_names = metadata['robot_joint_names']
    initial = dict(zip(robot_names, metadata['_start_full_q']))
    fingers = {name: initial[name] for name in world.GRIPPER_JOINT_NAMES}
    path = np.asarray(plan_result['positions'])
    # Densify geometric checks; these are planned configurations, not physics samples.
    dense = np.vstack([path[0]] + [np.asarray([a + t * (b - a) for t in np.linspace(0, 1, 9)[1:]])
                                  for a, b in zip(path[:-1], path[1:])])
    minimum = None; violations = []; conservative = []
    for i, q in enumerate(dense):
        shapes = physical_robot_shapes(world.shapes(q, fingers))
        value = clearance(shapes, hazard, margin, True)
        if minimum is None or value['slack_m'] < minimum['slack_m']:
            minimum = {'sample': i, **value}
        if not value['valid']:
            violations.append({'sample': i, **value})
        if not world.valid(q, mode='execution', gripper_q=fingers):
            conservative.append({'sample': i, 'reason': world.last_clearance})
    return {'available': True, 'source_geometry_safe': not violations,
            'minimum': minimum, 'violations': violations,
            'conservative_execution_safe': not conservative,
            'conservative_violations': conservative, 'geometric_samples': len(dense),
            'sampling': '8 subdivisions per planned action interval; no physical execution',
            'geometry': 'Source robot collision hulls and articulated gripper meshes; source obstacle meshes and exact supported primitives. Hulls are not claimed identical to PhysX cooked shapes.'}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline', type=Path, required=True)
    p.add_argument('--variant', choices=['human', 'property', 'robot'], required=True)
    p.add_argument('--config', type=Path, default=Path('configs/extensions/doors/avoidance_variants.json'))
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():p.error('Refusing to replace planning evidence')
    a.output.mkdir(parents=True)
    metadata = json.loads((a.baseline / 'metadata.json').read_text())
    metadata['_task_config']=json.loads(a.config.read_text())
    baseline_result = json.loads((a.baseline / 'result.json').read_text())
    if not baseline_result['accepted']:p.error('A representative successful physical baseline is required')
    rows = json.loads((a.baseline / 'trajectory.json').read_text())
    first = next(row for row in rows if row['phase'] == 'pregrasp')
    full_q = np.asarray(first['obs']['q']); metadata['_start_full_q'] = full_q.tolist()
    names = metadata['robot_joint_names']
    start = full_q[[names.index(f'panda_joint{i}') for i in range(1, 8)]]
    spec = json.loads(a.config.read_text())[a.variant]
    policy = metadata['controller_parameters']
    offsets = policy.get('handle_height_candidates_m', [policy['handle_offset_z']])
    seeds = metadata['review_gates']['planning_seeds']
    report = {'variant': a.variant, 'seeds_predeclared': seeds, 'pairs': [],
              'baseline_sha256': hashlib.sha256((a.baseline / 'trajectory.json').read_bytes()).hexdigest(),
              'controller_parameters': policy, 'config_sha256': hashlib.sha256(a.config.read_bytes()).hexdigest(),
              'command_ik_margin_applied_rad': policy.get('ik_limit_margin_rad',0.),
              'implementation_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'toggle_scope': 'Only added hazard and its physical support; source kitchen, real stand, start, policy and seed stay fixed',
              'scope': 'Pregrasp planning evidence only; no unsafe path is executed; fixed-layout planning seeds are not additional physical successes.'}
    for seed in seeds:
        pair = {'seed': seed}
        for label, enabled in [('off', False), ('on', True)]:
            print('PLAN_PAIR', a.variant, seed, label, flush=True)
            result = plan(start, metadata, spec if enabled else None, seed, offsets)
            result['risk_with_added_geometry'] = evaluate_path(result, metadata, spec)
            pair[label] = result
        on, off = pair['on'], pair['off']
        pair['same_initial_state'] = on['initial_q'] == off['initial_q']
        pair['target_adaptation'] = bool(on['success'] and off['success'] and
                                         on['selected_offset_m'] != off['selected_offset_m'])
        pair['evidence_kind'] = 'legal_handle_point_adaptation' if pair['target_adaptation'] else 'fixed_target_planning'
        if pair['target_adaptation']:
            forced_offset = [on['selected_offset_m']]
            pair['fixed_target_control'] = {}
            for label, enabled in [('off', False), ('on', True)]:
                result = plan(start, metadata, spec if enabled else None, seed, forced_offset)
                result['risk_with_added_geometry'] = evaluate_path(result, metadata, spec)
                pair['fixed_target_control'][label] = result
        pair['policy_necessity_supported'] = bool(
            on['success'] and off['success'] and
            off['risk_with_added_geometry']['violations'] and
            on['risk_with_added_geometry']['source_geometry_safe'] and
            on['risk_with_added_geometry']['conservative_execution_safe'])
        if on['success'] and off['success']:
            pair['path_change'] = {key: on['path_metrics'][key] - off['path_metrics'][key]
                                   for key in ('duration_s', 'joint_l2_length_rad', 'tcp_length_m')}
        report['pairs'].append(pair)
        (a.output / f'seed_{seed}.json').write_text(json.dumps(pair, indent=2))
    report['stable_policy_necessity_in_tested_seeds'] = all(p['policy_necessity_supported'] for p in report['pairs'])
    report['fixed_endpoint_rerouting_supported'] = all(
        not p['target_adaptation'] and p['policy_necessity_supported'] for p in report['pairs'])
    report['planning_success_counts'] = {
        label: sum(p[label]['success'] for p in report['pairs']) for label in ('off', 'on')}
    report['interpretation'] = 'These are paired planning outcomes in one fixed source layout, not physical collection success rates or a claim about all feasible paths.'
    (a.output / 'report.json').write_text(json.dumps(report, indent=2))
    print('PAIRED_RESULT', a.variant, report['stable_policy_necessity_in_tested_seeds'], flush=True)
    return 0 if report['stable_policy_necessity_in_tested_seeds'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
