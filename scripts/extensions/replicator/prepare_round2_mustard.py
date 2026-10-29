#!/usr/bin/env python3
"""Prepare one fixed-layout mustard recipe per remaining failure, on CPU.

Finite reference-derived candidates, source non-pad gripper/bottle separation,
open approach and closing-sweep environment checks, then six endpoint IK.
No GPU, Kit, physics, layout mutation, or claim of full-path clearance/success.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
import fcl
import numpy as np
from pxr import Usd
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics, ROBOT_USD
from kitchen_safety.geometry_audit import minimum_distance
from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld
from kitchen_safety.extensions.replicator.quick_trajectory import PlanningWorld
from kitchen_safety.extensions.replicator.physical_retry_round2_mustard import PHASES, candidates, make_targets
from scripts.extensions.replicator.variant_geometry import SourceRobotGeometry


def read(path): return json.loads(Path(path).read_text())


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


def save(path, data):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    temp = Path(str(path)+'.tmp')
    temp.write_text(json.dumps(data, indent=2, allow_nan=False)+'\n')
    temp.replace(path)


class Gripper:
    def __init__(self):
        self.world = CollisionWorld.__new__(CollisionWorld)
        self.world.fcl = fcl
        self.world._gripper_geometry()
        source = SourceRobotGeometry(self.world)
        self.parts = [p for p in source.parts if '/Gripper/' in p[1]]
        self.base_from_tcp = np.eye(4)
        self.base_from_tcp[0, 3] = -.131

    def shapes(self, tcp, angle, *, nonpad=False):
        angles = dict(zip(self.world.GRIPPER_JOINT_NAMES, np.array([1,1,-1,1,-1,-1])*angle))
        links = self.world._gripper_links(angles)
        base = tcp @ self.base_from_tcp
        out = []
        for name, body, obj, inverse, rotation, position, lo, hi in self.parts:
            if nonpad and body.rsplit('/', 1)[-1] in ('left_inner_finger', 'right_inner_finger'):
                continue
            delta = base @ links[body] @ inverse
            r, t = delta[:3, :3], delta[:3, 3]
            obj.setTransform(fcl.Transform(r @ rotation, r @ position+t))
            center, extent = r @ ((lo+hi)/2)+t, np.abs(r) @ ((hi-lo)/2)
            out.append((name, obj, center-extent, center+extent))
        return out


def scene_shapes(path):
    world = PlanningWorld(Usd.Stage.Open(str(path)))
    rows = []
    for name, _, lo, hi in world.objects:
        if name in world.meshes:
            vertices, faces = world.meshes[name]
            model = fcl.BVHModel(); model.beginModel(len(vertices), len(faces))
            model.addSubModel(vertices, faces); model.endModel()
            obj = fcl.CollisionObject(model)
        else:
            obj = fcl.CollisionObject(fcl.Box(*np.maximum(hi-lo, 1e-4)), fcl.Transform((lo+hi)/2))
        rows.append((name, obj, lo, hi))
    return rows


def cpu_check(recipe, old, layout, shapes, gripper):
    targets = {p: np.asarray(recipe['target_poses_world'][p]) for p in PHASES}
    bottle = [r for r in shapes if '/objects/mustard_bottle/' in r[0]]
    environment = [r for r in shapes if '/objects/mustard_bottle/' not in r[0]]
    checks = []
    # Endpoint first, then a sampled vertical approach, followed by closure.
    samples = [('approach', targets['approach'], 0.)]
    for rise in np.linspace(.125, 0., 17)[:-1]:
        tcp = targets['approach'].copy(); tcp[2, 3] += rise
        samples.append(('approach_vertical_sample', tcp, 0.))
    samples += [('close_sweep', targets['approach'], float(a)) for a in np.linspace(0., recipe['planning_gripper_joint_rad'], 9)[1:]]
    for label, tcp, angle in samples:
        own = minimum_distance(gripper.shapes(tcp, angle, nonpad=True), bottle, .002)
        env = minimum_distance(gripper.shapes(tcp, angle), environment, .002)
        checks.append(dict(phase=label, nominal_finger_joint_rad=angle,
            source_nonpad_bottle=own, source_gripper_environment=env))
        if not own['valid'] or not env['valid']:
            return dict(passed=False, rejected_stage='gripper_source_geometry', checks=checks)
    kin = DroidKinematics(np.asarray(layout['robot_position'])+[0,0,layout['stand_height']], layout['robot_yaw_deg'])
    q = np.asarray(old['start_q']); ik = []
    for phase in PHASES:
        try:
            q = kin.ik(targets[phase], q, attempts=4)
        except RuntimeError as error:
            return dict(passed=False, rejected_stage=phase+'_ik', reason=str(error), checks=checks, ik=ik)
        actual = kin.fk(q)
        pe = float(np.linalg.norm(actual[:3,3]-targets[phase][:3,3]))
        re = float(np.rad2deg(Rotation.from_matrix(actual[:3,:3] @ targets[phase][:3,:3].T).magnitude()))
        ok = bool(pe <= .002 and re <= 2. and np.all(q >= kin.limits[:,0]) and np.all(q <= kin.limits[:,1]))
        ik.append(dict(phase=phase, q_rad=q.tolist(), position_error_m=pe, orientation_error_deg=re, passed=ok))
        if not ok:
            return dict(passed=False, rejected_stage=phase+'_ik_error', checks=checks, ik=ik)
    return dict(passed=True, checks=checks, ik=ik)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, default=ROOT/'reports/replicator_physical_retry_round2_20261028/mustard_recipes.json')
    args = p.parse_args()
    if args.output.exists(): raise FileExistsError(args.output)
    plan_path = ROOT/'configs/extensions/replicator/physical_retry_round1_20261028.json'
    failures_path = ROOT/'reports/replicator_physical_retry_round1_20261028/remaining_failures_grasp_analysis.json'
    visual_path = ROOT/'reports/replicator_physical_retry_round1_20261028/visual_review_mustard.json'
    reference_path = ROOT/'reports/replicator_preparation/mustard_family_cpu02/g_grasp_reference01.json'
    reference = read(reference_path)
    if not reference['source_physical_success'] or sha(ROBOT_USD) != reference['source_robot_sha256']:
        raise ValueError('Measured reference provenance mismatch')
    failed = {r['scene_id']:r for r in read(failures_path)['entries'] if 'mustard' in r['scene_id']}
    rows = [r for r in read(plan_path)['entries'] if r['scene_id'] in failed]
    if len(rows) != 14: raise ValueError('Expected exactly fourteen remaining mustard failures')
    visuals = {r['scene_id']:r for r in read(visual_path)['entries']}
    declaration = candidates(reference)
    gripper = Gripper()
    output = dict(kind='physical_retry_round2_mustard_recipes', candidate_count_per_scene=1,
        layout_changed=False, physics_launched=False, curobo_called=False,
        planning_gripper_is_measured=False, closed_command_rad=.70,
        scope='CPU source surfaces and sampled endpoint IK only; no full-arm/path/self-collision, cooked PhysX, contact-force, avoidance or physical success proof.',
        inputs={str(x.relative_to(ROOT)):sha(x) for x in (plan_path, failures_path, visual_path, reference_path)},
        declarations=declaration, entries=[])
    for row in rows:
        begin = time.monotonic(); sid = row['scene_id']; previous = read(ROOT/row['result_path']); layout = read(ROOT/row['config_path'])
        for path, key in (('result_path','result_sha256'),('config_path','config_sha256'),('source_scene_path','source_scene_sha256')):
            if sha(ROOT/row[path]) != row[key]: raise ValueError('Frozen input changed '+path)
        shapes = scene_shapes(ROOT/row['source_scene_path']); trials = []; selected = None
        # L tall-glass had a real lift followed by loss; reserve more transport
        # clearance while keeping its historical final bottle pose unchanged.
        rise = .04 if sid == 'l_shape_mustard_bowl__property_tall_glass' else 0.
        for candidate in declaration:
            recipe = make_targets(previous, candidate, transport_raise_m=rise)
            check = cpu_check(recipe, previous, layout, shapes, gripper)
            trials.append(dict(candidate=candidate['name'], **check))
            print(json.dumps(dict(scene_id=sid, candidate=candidate['name'], cpu_pass=check['passed'], rejected_stage=check.get('rejected_stage'))), flush=True)
            if check['passed']:
                selected = recipe; break
        # A failed finite CPU screen is retained explicitly. The first declared
        # candidate remains a proposal only; the parent must record rejection
        # or decide whether its single full-path planner call is still useful.
        if selected is None:
            selected = make_targets(previous, declaration[0], transport_raise_m=rise)
        selected.update(scene_id=sid, task_id=row['task_id'], payload_name='mustard_bottle', layout_fixed=True,
            description_zh='移到宽瓶身倾斜抓取；绝对闭合目标改为0.70 rad，规划预计夹持角0.37 rad；携物保持姿态，保留历史放置瓶位。'+(' 搬运端点提高40 mm，减少低位扫掠；本轮仍需实测是否失持。' if rise else ''),
            evidence=dict(previous_result_path=row['result_path'], previous_result_sha256=row['result_sha256'],
                layout_path=row['config_path'], layout_sha256=row['config_sha256'],
                execution_result_sha256=visuals[sid]['execution_result_sha256'],
                visual_observation=visuals[sid]['observed_visual_facts_zh'],
                reference_episode=reference['reference_episode'], reference_sha256=sha(reference_path)),
            cpu_screen_passed=any(x['passed'] for x in trials), cpu_trials=trials,
            evaluated_cpu_candidates=len(trials), elapsed_cpu_seconds=time.monotonic()-begin)
        output['entries'].append(selected)
        save(args.output.with_suffix('.partial.json'), output)
        del shapes
    output['summary'] = dict(scenes=len(output['entries']), cpu_screen_passed=sum(x['cpu_screen_passed'] for x in output['entries']),
        cpu_screen_failed=sum(not x['cpu_screen_passed'] for x in output['entries']))
    output['implementation_sha256'] = {str(x.relative_to(ROOT)):sha(x) for x in (Path(__file__), ROOT/'kitchen_safety/extensions/replicator/physical_retry_round2_mustard.py')}
    save(args.output, output)
    print(json.dumps(dict(output=str(args.output), **output['summary'])), flush=True)


if __name__ == '__main__': main()
