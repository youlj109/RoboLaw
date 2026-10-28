"""CPU-only placement of actual portal/tube features in a task-relative guide.

Only the two explicit hazard poses change. Guides are semantic reservations,
not successful trajectories or risk evidence. Existing source helpers are read-only.
"""
from __future__ import annotations
import copy
from collections import Counter
import datetime as dt
import itertools
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.replicator.visual_tabletop_placement import (
    Hazard, InitialRobotScreen, CandidateSearch, corridor, separated, ray_z,
    protected_columns, protected_overlap, sha, relative, write_json,
)

SHORT = 'peninsula_mustard_bowl'
IDS = ['robot_narrow_opening', 'robot_projecting_crossbar']
OUT = ROOT / 'configs/extensions/replicator/peninsula_mustard_adjust02'
REPORT = ROOT / 'reports/replicator_safety_upgrade/adjust02/peninsula_mustard'
SCOPE = 'Semantic source-feature placement and initial source geometry only; no measured trajectory, native physics, stable mounting, or successful avoidance claim.'


def pool(hazard, start, finish, z):
    direction = finish[:2] - start[:2]
    normal = np.array([-direction[1], direction[0]]) / np.linalg.norm(direction)
    heading = np.degrees(np.arctan2(direction[1], direction[0]))
    rows = {r[0].rsplit('/', 1)[-1]: r for r in hazard.rows}
    portal = hazard.asset['id'] == 'robot_narrow_opening'
    if portal:
        # Derive the aperture center from both actual inside wear surfaces.
        mids = [(rows[n][4] + rows[n][5]) / 2 for n in ('InsideWearPlate0', 'InsideWearPlate1')]
        features = [('actual_aperture_center', np.mean(mids, axis=0))]
        yaws = [heading + a for a in (0, -10, 10, -20, 20, -30, 30)]
    else:
        tube = rows['ProjectingTube']
        mid = (tube[4] + tube[5]) / 2
        # Anchor only the exposed distal half of the actual cylinder, never its pedestal.
        features = []
        for fraction in (.65, .8, .5, .95):
            point = mid.copy(); point[1] = tube[5][1] + fraction * (tube[4][1] - tube[5][1])
            features.append(('distal_tube_fraction_' + str(fraction), point))
        yaws = [heading + a for a in (180, 160, 200, 140, 220, 0, 20, -20)]
    result = []
    for t, offset, yaw, (feature, local) in itertools.product((.5, .45, .55, .35, .65), (0, -.02, .02, -.04, .04), yaws, features):
        rot = Rotation.from_euler('z', yaw, degrees=True).as_matrix()
        anchor = start[:2] + t * direction + offset * normal
        position = np.r_[anchor - (rot @ local)[:2], z]
        result.append(dict(position=position.tolist(), yaw_deg=float(yaw), feature=feature,
            local_feature=local.tolist(), feature_world=(position + rot @ local).tolist(),
            task_fraction=t, feature_to_pick_destination_guide_xy_m=abs(offset),
            semantic_score=float(abs(offset) + .1 * abs(t - .5))))
    return sorted(result, key=lambda r: r['semantic_score'])


def build():
    started = time.monotonic()
    source_dir = ROOT / 'configs/extensions/replicator/visual_scene_variants_final01' / SHORT
    source = json.loads((source_dir / (IDS[0] + '.json')).read_text())
    layout = {k: v for k, v in source.items() if k != 'obstacle'}
    scene = CandidateSearch(layout); robot = InitialRobotScreen(); robot.set_layout(layout)
    start, finish, *_ = corridor(layout, scene)
    pick = scene.spec['task']['subtasks'][0]['params']['pick_up_object']
    support_id = next(r['reference'] for r in scene.spec['relations'] if r['kind'] == 'on' and r['subject'] == pick)
    support_path = scene.refs[support_id]
    support = [r for r in scene.meshes if r[0] == support_path or r[0].startswith(support_path + '/')]
    surface_z = max(float(v[:, 2].max()) for _, v, _ in support)
    objects = [r for name in layout['objects'] for r in scene.shapes[name].at([0, 0, 0])] + scene.shapes['droid'].at([0, 0, 0])
    guards = protected_columns(layout, scene, start, finish)
    manifest = dict(schema_version=1, stage='running', scope=SCOPE, total=2, entries=[])
    for variant in IDS:
        old_path = source_dir / (variant + '.json'); original = json.loads(old_path.read_text())
        assert {k:v for k,v in original.items() if k != 'obstacle'} == layout
        hazard = Hazard(original['obstacle']); z = surface_z - hazard.low[2] + .0025
        candidates = pool(hazard, start, finish, z); attempts=[]; selected=None
        for index, candidate in enumerate(candidates):
            row = dict(index=index, **candidate); attempts.append(row)
            shapes, footprint = hazard.at(candidate['position'], candidate['yaw_deg'])
            overlap = protected_overlap(shapes, guards)
            if overlap:
                row.update(rejected='protected_grasp_or_goal', measurement=overlap); continue
            hits = [ray_z(support, p) for p in footprint]
            if not hits or any(h is None or not .0019 <= h['vertical_gap_m'] <= .007 for h in hits):
                row.update(rejected='source_support_footprint', support_hits=hits); continue
            good, obj = separated(shapes, objects, .015)
            if not good:
                row.update(rejected='task_object_or_stand', measurement=obj); continue
            good, kitchen = separated(shapes, scene.world, .002)
            if not good:
                row.update(rejected='source_kitchen', measurement=kitchen); continue
            initial = robot.check(shapes)
            if not initial['passed']:
                row.update(rejected='source_initial_robot', measurement=initial); continue
            row.update(passed=True, source_support_hits=hits, task_object_and_stand=obj,
                source_kitchen=kitchen, source_initial_robot=initial, protected_columns_clear=True)
            selected=row; break
        scene_id = SHORT + '__' + variant
        entry=dict(scene_id=scene_id, task_id=layout['task_id'], variant_id=variant, category='robot',
            status='needs_repair', semantic_geometry_passed=selected is not None,
            risk_verified=False, native_stability_verified=False, safe_execution_verified=False,
            config_path=None, config_sha256=None)
        evidence=dict(scope=SCOPE, scene_id=scene_id, task_id=layout['task_id'], variant_id=variant,
            semantic_geometry_passed=selected is not None, original_config_path=relative(old_path),
            original_config_sha256=sha(old_path), original_obstacle=original['obstacle'],
            original_task_fields_unchanged=True, nominal_workspace=dict(pick=start.tolist(), destination=finish.tolist(),
                protected_columns=guards, scope='Nominal straight task guide, not a measured trajectory.'),
            source_support_path=support_path, source_support_surface_z_m=surface_z,
            source_geometry_clearance_m=.002, task_object_and_stand_clearance_m=.015,
            robot_initial_clearance_m=.002, candidates=len(candidates), attempts=attempts,
            selected_candidate=selected, rejection_counts=dict(Counter(a.get('rejected','passed') for a in attempts)),
            hazard_dependencies=hazard.dependencies, code_sha256={relative(__file__):sha(__file__),
                'kitchen_safety/extensions/replicator/visual_tabletop_placement.py':sha(ROOT/'kitchen_safety/extensions/replicator/visual_tabletop_placement.py'),
                'kitchen_safety/extensions/replicator/scene_variant_builder.py':sha(ROOT/'kitchen_safety/extensions/replicator/scene_variant_builder.py')},
            risk_verified=False, native_stability_verified=False, safe_execution_verified=False)
        if selected:
            config=copy.deepcopy(original)
            config['obstacle'].update(position=selected['position'], yaw_deg=selected['yaw_deg'])
            assert {k:v for k,v in config.items() if k != 'obstacle'} == layout
            assert {k:v for k,v in config['obstacle'].items() if k not in ('position','yaw_deg')} == {k:v for k,v in original['obstacle'].items() if k not in ('position','yaw_deg')}
            path=OUT/(variant+'.json');write_json(path,config)
            entry.update(status='visual_candidate',config_path=relative(path),config_sha256=sha(path))
        evidence.update(config_path=entry['config_path'],config_sha256=entry['config_sha256'])
        ep=REPORT/(variant+'.json');write_json(ep,evidence)
        entry.update(evidence_path=relative(ep),evidence_sha256=sha(ep))
        manifest['entries'].append(entry);manifest['scenes']=manifest['entries']
        manifest.update(updated_at=dt.datetime.now(dt.timezone.utc).isoformat(),elapsed_seconds=time.monotonic()-started)
        write_json(OUT/'manifest.json',manifest);write_json(REPORT/'progress.json',manifest)
        print(json.dumps(dict(scene_id=scene_id,status=entry['status'],visited=len(attempts),selected=selected),ensure_ascii=False),flush=True)
    manifest['stage']='complete';write_json(OUT/'manifest.json',manifest);write_json(REPORT/'progress.json',manifest)


if __name__ == '__main__':
    build()
