"""Build the 14 × 11 first-stage scenes without claiming collection acceptance.

The original task objects, robot and YAML relations are immutable. Added assets
are placed in a task-relative corridor and screened against source collision
surfaces and actual support triangles. This does not certify trajectory
avoidance, native contact stability or task success.
"""
from __future__ import annotations

import copy
import datetime
import hashlib
import itertools
import json
from pathlib import Path

import fcl
import numpy as np
from pxr import Usd, UsdGeom
from scipy.spatial.transform import Rotation

from kitchen_safety.geometry_audit import static_collision_shapes, minimum_distance
from kitchen_safety.obstacle_library import get_asset
from scripts.extensions.replicator.audit_source_layout_geometry import triangles, ray_z
from scripts.extensions.replicator.audit_task_source_relation import check_layout
from scripts.extensions.replicator.search_source_layout_candidates import CandidateSearch

ROOT = Path(__file__).resolve().parents[3]
OUTPUT = ROOT / 'configs/extensions/replicator/scene_variants'
REPORTS = ROOT / 'reports/replicator_scene_build'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def variants():
    names = ['human_default_mpfb'] + json.loads((ROOT/'configs/reproduction_presets.json').read_text())['nonhuman10']['ids']
    human = json.loads((ROOT/'configs/layout.json').read_text())['human']
    result = []
    for name in names:
        if name == 'human_default_mpfb':
            result.append(dict(id=name, category='human', label='MPFB 人体', static=True,
                usd_path=human['asset'], position=human['position'], yaw_deg=human['yaw_deg'],
                provenance=dict(source='configs/layout.json', field='human.asset')))
        else:
            result.append(copy.deepcopy(get_asset(name)))
    if len(result) != 11 or len({x['id'] for x in result}) != 11:
        raise ValueError('Expected the original eleven unique variants')
    return result


def canonical(task_id):
    short = task_id.removeprefix('kitchen_bench_replicator_')
    path = ROOT/'configs/extensions/replicator'/f'{short}_source_layout.json'
    if short != 'g_shape_mustard_bowl':
        return path, json.loads(path.read_text())
    path = ROOT/'configs/extensions/replicator/g_shape_mustard.json'
    raw = json.loads(path.read_text())
    # A schema conversion only: all source object/base pose values are retained.
    result = {key: copy.deepcopy(raw[key]) for key in
        ('task_id', 'task', 'kitchen_usd', 'stand_height', 'robot_position', 'robot_yaw_deg')}
    result['objects'] = {
        'mustard_bottle': dict(usd_path='assets/robolab/objects/hope/mustard_bottle.usd',
            position=raw['mustard_position'], yaw_deg=raw.get('mustard_yaw_deg', 0)),
        'bowl': dict(usd_path='assets/robolab/objects/ycb/bowl.usd',
            position=raw['bowl_position'], yaw_deg=raw.get('bowl_yaw_deg', 0))}
    return path, result


class Hazard:
    def __init__(self, asset):
        self.asset = asset
        self.stage = Usd.Stage.Open(str(ROOT/asset['usd_path']))
        if not self.stage or not self.stage.GetDefaultPrim() or self.stage.GetCompositionErrors():
            raise ValueError('Unresolved hazard USD: ' + asset['usd_path'])
        root = self.stage.GetDefaultPrim()
        self.rows = []
        # A reference replaces the root's rigid pose as the runtime does. All
        # source assets here have an identity rigid root; fail instead of guessing.
        root_matrix = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(root)).T
        if not np.allclose(root_matrix, np.eye(4), atol=1e-7):
            raise ValueError('Hazard has a nonidentity root; explicit adapter needed')
        for name, obj, lo, hi in static_collision_shapes(self.stage, str(root.GetPath())):
            self.rows.append((name, obj, np.array(obj.getRotation()), np.array(obj.getTranslation()), lo, hi))
        self.low = np.min([r[4] for r in self.rows], axis=0)
        self.high = np.max([r[5] for r in self.rows], axis=0)
        self.dependencies = {str(Path(layer.realPath).relative_to(ROOT)): sha(layer.realPath)
            for layer in self.stage.GetUsedLayers() if Path(layer.realPath).is_file()}

    def at(self, position, yaw):
        rotation = Rotation.from_euler('z', yaw, degrees=True).as_matrix()
        position = np.asarray(position)
        rows = []
        support_points = []
        for name, obj, prior_rotation, prior_translation, lo, hi in self.rows:
            obj.setTransform(fcl.Transform(rotation @ prior_rotation, rotation @ prior_translation + position))
            center = rotation @ ((lo+hi)/2) + position
            radius = np.abs(rotation) @ ((hi-lo)/2)
            rows.append((name, obj, center-radius, center+radius))
            if lo[2] <= self.low[2] + .002:
                for x, y in itertools.product((lo[0], hi[0]), (lo[1], hi[1])):
                    support_points.append(rotation @ np.array([x, y, lo[2]]) + position)
        return rows, np.asarray(support_points)


def separated(left, right, clearance=.002):
    """AABB pruning, then unchanged source FCL triangle/primitive query."""
    lows = np.array([row[2] for row in right])
    highs = np.array([row[3] for row in right])
    checked = 0
    for name, obj, low, high in left:
        gaps = np.linalg.norm(np.maximum(np.maximum(lows-high, low-highs), 0), axis=1)
        for index in np.flatnonzero(gaps <= clearance):
            other = right[index]
            checked += 1
            distance = float(fcl.distance(obj, other[1], fcl.DistanceRequest(), fcl.DistanceResult()))
            if distance < clearance or fcl.collide(obj, other[1], fcl.CollisionRequest(), fcl.CollisionResult()):
                return False, dict(hazard=name, other=other[0], raw_distance_m=distance, checked_pairs=checked)
    return True, dict(checked_pairs=checked, required_clearance_m=clearance)


def corridor(layout, scene):
    params = scene.spec['task']['subtasks'][0]['params']
    start = np.asarray(layout['objects'][params['pick_up_object']]['position'], dtype=float)
    dest = params['destination_location']
    if dest in layout['objects']:
        finish = np.asarray(layout['objects'][dest]['position'], dtype=float)
    else:
        box = scene.bounds[scene.assets[dest]]
        finish = (box.min_point.numpy()[0] + box.max_point.numpy()[0]) / 2
    delta = finish[:2]-start[:2]
    direction = delta/max(np.linalg.norm(delta), 1e-12)
    normal = np.array([-direction[1], direction[0]])
    front = (start[:2]+finish[:2])/2 - np.asarray(layout['robot_position'])[:2]
    front /= np.linalg.norm(front)
    return start, finish, direction, normal, front


class InitialRobotScreen:
    """Both source controller q and the USD pose displayed before physics."""
    def __init__(self):
        from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld
        from kitchen_safety.planning import DroidKinematics
        from scripts.extensions.replicator.variant_geometry import SourceRobotGeometry
        from scripts.extensions.replicator.audit_initial_robot_geometry import source_initial_q
        values, self.provenance = source_initial_q()
        self.q = np.array([values[f'panda_joint{i}'] for i in range(1,8)])
        self.fingers = {name: values[name] for name in CollisionWorld.GRIPPER_JOINT_NAMES}
        self.world = CollisionWorld(Usd.Stage.CreateInMemory(), DroidKinematics([0,0,.8],0))
        self.robot = SourceRobotGeometry(self.world)

    def set_layout(self, layout):
        from kitchen_safety.planning import DroidKinematics
        self.world.kin = DroidKinematics(np.asarray(layout['robot_position'])+[0,0,layout['stand_height']],layout['robot_yaw_deg'])

    def check(self, shapes):
        initial = minimum_distance(self.robot.shapes(self.q,self.fingers),shapes,.002)
        from scripts.extensions.replicator.audit_scene_initial_robot import authored_rows
        authored = minimum_distance(authored_rows(self.robot,self.world.kin.base),shapes,.002)
        return dict(passed=bool(initial['valid'] and authored['valid']),
            source_controller_initial=initial, usd_authored_link_transforms=authored)


def place(layout, scene, hazard, robot_screen=None):
    start, finish, direction, normal, front = corridor(layout, scene)
    center = (start[:2]+finish[:2])/2
    floor_mounted = hazard.asset['category'] == 'human' or hazard.asset['id'] == 'robot_steel_clamp'
    pick = scene.spec['task']['subtasks'][0]['params']['pick_up_object']
    support_id = 'floor' if floor_mounted else next(row['reference'] for row in scene.spec['relations']
        if row['kind'] == 'on' and row['subject'] == pick)
    support_path = scene.refs[support_id]
    support_meshes = [row for row in scene.meshes if row[0] == support_path or row[0].startswith(support_path+'/')]
    if not support_meshes:
        raise ValueError('Missing actual support triangle surface: '+support_path)
    surface_top = max(float(vertices[:, 2].max()) for _, vertices, _ in support_meshes)
    z = surface_top - hazard.low[2] + .0025
    yaw_forward = np.degrees(np.arctan2(front[1], front[0]))
    # A projecting asset's local -Y axis faces the operation corridor.
    yaw = yaw_forward + 90
    candidates = []
    if floor_mounted:
        floor_lateral = np.array([-front[1], front[0]])
        for offset in (.45, .60, .75, .90, 1.05, 1.20):
            for back in (.10, -.10, -.30, .30, -.50, -.70):
                for sign in (1, -1):
                    xy = np.asarray(layout['robot_position'])[:2] + sign*floor_lateral*offset + front*back
                    face = center-xy
                    for extra_yaw in (0, 90, -90, 180):
                        candidates.append((xy, np.degrees(np.arctan2(face[1],face[0]))+90+extra_yaw))
    else:
        for offset in (.08, .14, .20, .26, .32, .40, .50, .60):
            for along in (0., -.12, .12, -.24, .24):
                for sign in (1, -1):
                    candidates.append((center + normal*offset*sign + direction*along, yaw))
        # Local support search for narrow sink counters/inside corners. These
        # coordinates are derived anew from each original support, never copied.
        vertices = np.concatenate([vertices for _, vertices, _ in support_meshes])
        low, high = vertices[:, :2].min(0), vertices[:, :2].max(0)
        x_values = np.arange(low[0]+.04, high[0]-.039, .08)
        y_values = np.arange(low[1]+.04, high[1]-.039, .08)
        grid = [np.array([x,y]) for x,y in itertools.product(x_values,y_values)
                if np.linalg.norm(np.array([x,y])-center) <= 1.]
        grid.sort(key=lambda xy: float(np.linalg.norm(xy-center)))
        for xy in grid:
            for extra_yaw in (0, 90, -90, 180):
                candidates.append((xy, yaw+extra_yaw))
    objects = [row for name in layout['objects'] for row in scene.shapes[name].at([0,0,0])]
    stand = scene.shapes['droid'].at([0,0,0])
    attempts = []
    for index, (xy, angle) in enumerate(candidates):
        position = [float(xy[0]),float(xy[1]),float(z)]
        rows, footprint = hazard.at(position, angle)
        attempt = dict(index=index, position=position, yaw_deg=float(angle))
        hits = [ray_z(support_meshes, point) for point in footprint]
        if not hits or any(hit is None or not .0019 <= hit['vertical_gap_m'] <= .007 for hit in hits):
            attempt['rejected'] = 'support_footprint'; attempts.append(attempt); continue
        valid, measure = separated(rows, objects + stand, .015)
        if not valid:
            attempt.update(rejected='task_objects_or_stand', measurement=measure); attempts.append(attempt); continue
        valid, measure = separated(rows, scene.world, .002)
        if not valid:
            attempt.update(rejected='kitchen_source', measurement=measure); attempts.append(attempt); continue
        robot_measure = robot_screen.check(rows) if robot_screen else None
        if robot_measure is not None and not robot_measure['passed']:
            attempt.update(rejected='source_initial_robot', measurement=robot_measure); attempts.append(attempt); continue
        attempt.update(passed=True, support_ray_count=len(hits), source_pair_check=measure)
        attempts.append(attempt)
        entry = copy.deepcopy(hazard.asset)
        entry.update(position=position, yaw_deg=float(angle))
        return entry, dict(passed=True, support_id=support_id, support_path=support_path,
            support_surface_z_m=surface_top, footprint_method='Conservative bottom-collider AABB corners checked against actual source support triangles',
            nominal_task_corridor=dict(pick=start.tolist(), destination=finish.tolist(), direction_xy=direction.tolist()),
            selected_candidate=index, attempts=attempts,
            source_initial_robot=robot_measure,
            source_geometry_clearance_m=.002, task_object_and_stand_clearance_m=.015,
            note='Task-relative near-corridor placement; trajectory avoidance necessity, full moving-arm collision and native settling are not certified.')
    error = ValueError(f"No supported source-clear placement for {hazard.asset['id']}; {len(attempts)} candidates")
    error.attempts = attempts
    raise error


def build(task_filter=None, repair_initial_robot=None):
    manifest_path = OUTPUT/'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    progress = json.loads((REPORTS/'progress.json').read_text())
    entries = {row['scene_id']:row for row in progress['entries']}
    assets = {asset['id']: Hazard(asset) for asset in variants()}
    repair_ids = set(json.loads(Path(repair_initial_robot).read_text())['failed_scenes']) if repair_initial_robot else set()
    robot_screen = InitialRobotScreen()
    total = len(manifest['scenes'])
    for task_id in manifest['tasks']:
        if task_filter and task_id not in task_filter:
            continue
        base_path, base = canonical(task_id)
        relation = check_layout(base)
        if not relation['initial_source_relations_passed']:
            raise ValueError('Canonical source relations failed: '+task_id)
        scene = CandidateSearch(base)
        robot_screen.set_layout(base)
        relation_path = REPORTS/'cpu'/task_id.removeprefix('kitchen_bench_replicator_')/'source_relations.json'
        write_json(relation_path, relation)
        for row in manifest['scenes']:
            if row['task_id'] != task_id:
                continue
            if row['checks']['config_generated'] and row['scene_id'] not in repair_ids:
                continue
            try:
                obstacle, placement = place(base, scene, assets[row['variant_id']], robot_screen)
                layout = copy.deepcopy(base); layout['obstacle'] = obstacle
                target = ROOT/row['config_path']
                write_json(target, layout)
                evidence_path = REPORTS/'cpu'/row['scene_id']/'geometry.json'
                evidence = dict(schema_version=1, scene_id=row['scene_id'], task_id=task_id,
                    canonical_layout_path=str(base_path.relative_to(ROOT)), canonical_layout_sha256=sha(base_path),
                    config_path=str(target.relative_to(ROOT)), config_sha256=sha(target),
                    original_task_poses_preserved={k:v for k,v in layout.items() if k!='obstacle'} == base,
                    source_relations_path=str(relation_path.relative_to(ROOT)), source_relations_sha256=sha(relation_path),
                    hazard_asset_sha256=assets[row['variant_id']].dependencies,
                    hazard_source_collision_parts=len(assets[row['variant_id']].rows),
                    placement=placement, simulation_loaded=False, avoidance_verified=False,
                    task_success_verified=False, physical_stability_verified=False)
                write_json(evidence_path, evidence)
                row.update(status='validated', cpu_evidence_path=str(evidence_path.relative_to(ROOT)))
                row.pop('error', None)
                row['checks'].update(config_generated=True, source_relations=True, assets_resolved=True, placement_geometry=True)
                row['checks']['initial_robot_geometry'] = True
            except Exception as error:
                row.update(status='failed', error=str(error))
                if hasattr(error, 'attempts'):
                    failed_path = REPORTS/'cpu'/row['scene_id']/'failed_placement.json'
                    write_json(failed_path, dict(error=str(error), attempts=error.attempts,
                        task_id=task_id, variant_id=row['variant_id'], source_layout=base))
                    row['failed_cpu_evidence_path'] = str(failed_path.relative_to(ROOT))
            entries[row['scene_id']].update(copy.deepcopy(row))
            progress.update(stage='generating_layouts', updated_at=datetime.datetime.now(datetime.timezone.utc).isoformat())
            write_json(manifest_path, manifest); write_json(REPORTS/'progress.json', progress)
            print(json.dumps(dict(scene=row['scene_id'], status=row['status'], configured=sum(x['checks']['config_generated'] for x in manifest['scenes']), total=total, error=row.get('error')),ensure_ascii=False), flush=True)
    progress.update(stage='layouts_generated', updated_at=datetime.datetime.now(datetime.timezone.utc).isoformat())
    write_json(REPORTS/'progress.json', progress)
    return manifest
