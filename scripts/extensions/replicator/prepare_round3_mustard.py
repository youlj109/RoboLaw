#!/usr/bin/env python3
"""Select one fixed-layout mustard candidate per round2 failure, CPU only.

The finite screen includes *all* open fingers against the payload, then the
source closing sweep, and unchanged conservative endpoint collision spheres.
It never launches cuRobo, Kit, or physics and does not promise grasp success.
"""
import argparse
from copy import deepcopy
import importlib.util
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.geometry_audit import minimum_distance
from kitchen_safety.extensions.replicator.physical_retry_round2_mustard import PHASES, make_targets
from scripts.extensions.replicator.prepare_round2_mustard import Gripper, read, save, sha, scene_shapes

SNAPSHOT = ROOT/'configs/extensions/replicator/physical_retry_results_round2_20261028.json'
MANIFEST = ROOT/'exports/replicator_physical_retry_round2_20261028/manifest.json'
DIAGNOSTIC = ROOT/'reports/replicator_physical_retry_round2_20261028/diagnose_planning_failures_cpu.py'
REPORT = ROOT/'reports/replicator_physical_retry_round3_20261029/mustard_cpu_selection.json'
CONFIG = ROOT/'configs/extensions/replicator/physical_retry_round3_mustard_recipes_20261029.json'


def ref(path):
    return dict(path=str(Path(path).relative_to(ROOT)), sha256=sha(path))


def declarations(sid):
    # Source-frame x/y are bottle axes, not kitchen axes. The rejected bounded
    # upright scan showed palm/bottle interference, so retain side clearance.
    fixed = np.array([[0., -1., 0.], [0., 0., 1.], [-1., 0., 0.]])
    yaws = (90., 270., 45., 135., 0., 180.)
    z_values = (0., .010)
    rows = []
    for tilt in (35., 45., 55., 65.):
        for yaw in yaws:
            rz = Rotation.from_euler('z', yaw, degrees=True).as_matrix()
            for z in z_values:
                local = np.eye(4)
                local[:3, :3] = rz @ Rotation.from_euler('x', tilt, degrees=True).as_matrix() @ fixed
                local[:3, 3] = rz @ np.array([0., -.010, z])
                rows.append(dict(name=f'side_tilt{tilt:g}_yaw{yaw:g}_z{z:g}',
                    bottle_to_tcp_local=local.tolist(), tilt_deg=tilt,
                    rotation_about_bottle_z_deg=yaw, body_local_tcp_z_m=z, lateral_offset_m=.010,
                    planning_gripper_joint_rad=.37, execution_gripper_joint_rad=.70))
    return rows


def source_screen(recipe, shapes, gripper):
    target = np.asarray(recipe['target_poses_world']['approach'])
    pre = np.asarray(recipe['target_poses_world']['pregrasp'])
    bottle = [x for x in shapes if '/objects/mustard_bottle/' in x[0]]
    environment = [x for x in shapes if '/objects/mustard_bottle/' not in x[0]]
    if not bottle: raise ValueError('Missing source bottle geometry')
    evidence = []
    for fraction in [1., *np.linspace(0., 1., 17)[:-1]]:
        pose = target.copy()
        pose[:3, 3] = pre[:3, 3] + fraction*(target[:3, 3]-pre[:3, 3])
        own = minimum_distance(gripper.shapes(pose, 0.), bottle, .002)
        env = minimum_distance(gripper.shapes(pose, 0.), environment, .002)
        evidence.append(dict(phase='open_approach', fraction=float(fraction), bottle=own, environment=env))
        if not own['valid'] or not env['valid']:
            return dict(passed=False, stage='open_source_sweep', samples=evidence)
    contact = {}; closure = []
    for angle in np.arange(.02, .701, .02):
        parts = gripper.shapes(target, float(angle))
        sides = {}
        for side in ('left', 'right'):
            pad = [x for x in parts if f'/{side}_inner_finger/' in x[0]]
            query = minimum_distance(pad, bottle, .001)
            sides[side] = query
            if not query['valid'] and side not in contact: contact[side] = float(angle)
        own = minimum_distance(gripper.shapes(target, float(angle), nonpad=True), bottle, .002)
        env = minimum_distance(gripper.shapes(target, float(angle)), environment, .002)
        closure.append(dict(angle_rad=float(angle), nonpad_bottle=own, environment=env, sides=sides))
        if not own['valid'] or not env['valid']:
            return dict(passed=False, stage='source_closure_sweep', samples=evidence, closure=closure, first_contact=contact)
        if len(contact) == 2:
            balanced = abs(contact['left']-contact['right']) <= .040001
            return dict(passed=balanced, stage='source_bilateral_estimate' if not balanced else 'source_pass',
                samples=evidence, closure=closure, first_contact=contact,
                estimated_contact_angle_rad=float(angle), source_contact_not_physical=True)
    return dict(passed=False, stage='no_source_bilateral_contact', samples=evidence, closure=closure, first_contact=contact)


def endpoint_screen(recipe, previous, probe):
    q = np.asarray(previous['start_q'])
    rows = []
    masks = {p['phase']: p['contact_masks'] for p in previous['phases']}
    ancestor = previous
    while set(masks) != set(PHASES):
        path = ROOT/ancestor['prior_result_path']
        if sha(path) != ancestor['prior_result_sha256']: raise ValueError('Changed planning history')
        ancestor = read(path)
        masks.update({p['phase']: p['contact_masks'] for p in ancestor['phases'] if p['phase'] not in masks})
    # Approach first rejects the previous failed endpoint cheaply. Then solve
    # all six endpoints in their execution order from source-authored q.
    checks = ('approach', *PHASES)
    for index, phase in enumerate(checks):
        if index == 1: q = np.asarray(previous['start_q'])
        target = np.asarray(recipe['target_poses_world'][phase])
        try: q = probe.kin.ik(target, q, attempts=2)
        except RuntimeError as error:
            return dict(passed=False, stage=phase+'_ik', error=str(error), endpoints=rows)
        actual = probe.kin.fk(q)
        pe = float(np.linalg.norm(actual[:3, 3]-target[:3, 3]))
        re = float(Rotation.from_matrix(actual[:3, :3]@target[:3, :3].T).magnitude())
        probe.phase = phase; probe.carrying = phase in ('lift', 'transfer', 'lower')
        probe.grip = recipe['planning_gripper_joint_rad'] if probe.carrying else 0.
        probe.attachment = np.asarray(recipe['nominal_attachment']); probe.masks = masks[phase]
        collision = probe.check(q)
        clear = not collision['world_pairs']
        valid = bool(pe < .002 and re < np.deg2rad(2.) and clear)
        rows.append(dict(phase=phase, q_rad=q.tolist(), position_error_m=pe,
            orientation_error_rad=re, passed=valid, collision=collision))
        if not valid: return dict(passed=False, stage=phase+'_endpoint', endpoints=rows)
    return dict(passed=True, stage='endpoint_pass', endpoints=rows,
        self_collision_scope='Recorded only; full planner self-collision rules remain unchanged.')


def rationale(sid, previous_status):
    if 'peninsula' in sid:
        return ('上轮接近期间瓶体偏移/倾倒或单指持续顶住，未形成可搬运双侧夹持。'
            '改变瓶身中段抓取入侧，并对接近/抬升增加原接口的竖直运动代价；完整检查张爪双指扫掠与闭合对称性，保持布局与目标瓶位。')
    if sid.startswith('l_shape'):
        return '上轮approach的开放指尖保守包络与托盘冲突；改变抓取入侧及腕部俯仰，使张爪包络离开托盘，保持全部场景物体位置。'
    if 'steel_clamp' in sid:
        return '上轮approach的指爪或前臂保守包络与钢夹梁冲突；改变瓶身抓取入侧和腕部姿态，并使用原5mm球包络筛选端点，保留钢夹及其碰撞。'
    return '上轮approach开放指尖保守包络与背景碗冲突；改变瓶身抓取方位与腕部姿态，保留尖角外壳及背景碗布局、全部碰撞规则。'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', action='append', default=[])
    parser.add_argument('--max-candidates', type=int, default=48)
    parser.add_argument('--report', type=Path, default=REPORT)
    parser.add_argument('--config', type=Path, default=CONFIG)
    args = parser.parse_args()
    if os.environ.get('CUDA_VISIBLE_DEVICES') != '': raise RuntimeError('Hide CUDA for this CPU preparation')
    if args.report.exists() or args.config.exists(): raise FileExistsError('Round3 candidate output already exists')
    spec = importlib.util.spec_from_file_location('round2_bound_cpu_diagnostic', DIAGNOSTIC)
    diag = importlib.util.module_from_spec(spec); spec.loader.exec_module(diag)
    snapshot = read(SNAPSHOT); manifest = {r['scene_id']:r for r in read(MANIFEST)['entries']}
    cohort = [e for e in snapshot['entries'] if not e['latest_success'] and 'mustard' in e['scene_id']]
    if len(cohort) != 8: raise ValueError('Expected eight remaining mustard scenes')
    if args.scene: cohort = [e for e in cohort if e['scene_id'] in args.scene]
    gripper = Gripper()
    output = dict(kind='physical_retry_round3_mustard_cpu_selection', source_snapshot=ref(SNAPSHOT),
        source_manifest=ref(MANIFEST), diagnostic_implementation=ref(DIAGNOSTIC),
        source_script=ref(Path(__file__)), rejected_upright_scan=ref(REPORT.parent/'mustard_upright_screen_rejected.json'),
        layout_changed=False, physics_launched=False,
        kit_started=False, curobo_called=False, candidate_count_per_scene=1,
        scope='Finite CPU source surface sweep and conservative endpoint screening only; no continuous planning, cooked contact, stability, avoidance, physical success or scene-infeasibility claim.', entries=[])
    recipes = {}
    for entry in cohort:
        begin = time.monotonic(); sid = entry['scene_id']; row = manifest[sid]
        previous_ref = row['candidate_planning_result']; previous_path = ROOT/previous_ref['path']
        if sha(previous_path) != previous_ref['sha256']: raise ValueError('Changed round2 candidate')
        previous = read(previous_path); fake = dict(previous, failed_phase='approach')
        probe = diag.Probe(row, fake); shapes = scene_shapes(ROOT/row['source_scene_path'])
        trials = []; selected = None; best = None; declarations_for_scene = declarations(sid)[:args.max_candidates]
        for candidate in declarations_for_scene:
            recipe = make_targets(previous, candidate)
            # Gripper x is its approach direction; keep a source-independent
            # vertical approach for these upright/shallow-tilt candidates.
            approach = np.asarray(recipe['target_poses_world']['approach']); pre = approach.copy(); pre[2, 3] += .125
            recipe['target_poses_world']['pregrasp'] = pre.tolist()
            recipe['linear_motion_phases'] = ['approach', 'lift']
            source = source_screen(recipe, shapes, gripper)
            trial = dict(candidate=candidate, source=source)
            if source['passed']:
                recipe['planning_gripper_joint_rad'] = source['estimated_contact_angle_rad']
                endpoint = endpoint_screen(recipe, previous, probe); trial['endpoint'] = endpoint
                if endpoint['passed']: selected = recipe
                progress = sum(e['passed'] for e in endpoint.get('endpoints', []))
                hit_count = len(endpoint.get('endpoints', [{}])[-1].get('collision', {}).get('world_pairs', []))
                score = (progress, -hit_count)
                if best is None or score > best[0]: best = (score, deepcopy(recipe), len(trials))
            trials.append(trial)
            print(json.dumps(dict(scene_id=sid, candidate=candidate['name'], source_pass=source['passed'],
                endpoint_pass=trial.get('endpoint',{}).get('passed'), stage=trial.get('endpoint',source)['stage'])), flush=True)
            if selected is not None: break
        fully_passed = selected is not None
        selected_index = len(trials)-1
        if selected is None and best is not None:
            selected = best[1]; selected_index = best[2]
        if selected is None:
            raise RuntimeError('No source-gripper candidate; retain evidence before freezing '+sid)
        record = dict(scene_id=sid, task_id=entry['task_id'], previous_status=row['execution_status'],
            previous_candidate=previous_ref, latest_actual_result=entry['latest_physical_result'],
            source_scene=dict(path=row['source_scene_path'],sha256=row['source_scene_sha256']),
            config=dict(path=row['config_path'],sha256=row['config_sha256']),
            rationale_zh=rationale(sid,row['execution_status']), trials=trials, selected=selected,
            selected_index=selected_index, cpu_screen_passed=fully_passed,
            selection_rule='First complete source/endpoint pass; otherwise maximum passed endpoints then minimum blocking pairs among source-passing candidates.',
            elapsed_seconds=time.monotonic()-begin)
        if selected is not None:
            targets = {p:np.asarray(selected['target_poses_world'][p]) for p in PHASES}
            pred = dict(zip(PHASES[1:],PHASES[:-1]))
            selected['hold_orientation_phases'] = [p for p in ('approach','lift','transfer','lower')
                if np.allclose(targets[p][:3,:3],targets[pred[p]][:3,:3],atol=1e-7,rtol=0)]
            selected['grasp_info'].update(round3_rule='all_open_fingers_source_sweep_and_symmetric_body_closure',
                selected_candidate=trials[selected_index]['candidate'], source_estimated_contact_angle_rad=selected['planning_gripper_joint_rad'],
                closure_semantics='Execution target remains .70 rad. Planning angle is a nominal source-surface estimate, not measured physical grasp.',
                prior_physical_result=entry['latest_physical_result'])
            selected.update(scene_id=sid,task_id=entry['task_id'],payload_name='mustard_bottle',layout_fixed=True,
                orientation_hold_endpoint_consistency_verified=True,cpu_screen_passed=fully_passed,
                description=record['rationale_zh']+' 选中'+trials[selected_index]['candidate']['name']+
                    f'；实际闭合指令0.70rad，源几何预计接触角{selected["planning_gripper_joint_rad"]:.2f}rad，仅为规划估计。')
            if not fully_passed:
                selected['description'] += ' 有限CPU筛选未通过全部端点，具体剩余约束见绑定报告；保留为单次cuRobo候选，不宣称可行。'
            recipes[sid]=selected
        output['entries'].append(record)
        save(args.report.with_suffix('.partial.json'),output)
        del probe, shapes
    output['summary']=dict(scenes=len(cohort),selected=len(recipes),rejected=len(cohort)-len(recipes))
    save(args.report,output)
    evidence=ref(args.report)
    for recipe in recipes.values(): recipe['evidence']=evidence
    save(args.config,dict(kind='physical_retry_round3_mustard_recipes',recipes=recipes,
        layout_changed=False,physics_launched=False,candidate_count_per_scene=1,evidence=evidence))
    print(json.dumps(dict(output=str(args.config),**output['summary'])),flush=True)


if __name__=='__main__': main()
