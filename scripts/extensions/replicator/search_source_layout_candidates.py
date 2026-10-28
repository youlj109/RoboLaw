#!/usr/bin/env python3
"""CPU nearest-grid candidates preserving original A-task assets and relations.

The predeclared search is sequential (stand, then YAML object order), not a
claim of globally optimal reachability. Every visited candidate is retained.
The original layout is never overwritten. Successful output remains
execution_ready=false until independent robot/physics/task checks are done.
"""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
from types import SimpleNamespace
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
import fcl
import numpy as np
from pxr import Usd
from isaaclab_arena.relations.relations import On, NextTo, RotateAroundSolution
from isaaclab_arena.relations.validation.pre_physics import OnRelationValidator, NextToValidator
from scripts.extensions.replicator.audit_task_source_relation import geometry, check_layout, STAND
from scripts.extensions.replicator.audit_source_layout_geometry import (
    REQUIRED_M, diagnose, minimum_distance, pose_rows, ray_z, sha, triangles,
    static_collision_shapes, RAY_BOUNDS,
)


def nearest_grid(radius, step, z_values):
    count = round(radius / step)
    if abs(count * step - radius) > 1e-10:
        raise ValueError('Grid radius must be an exact multiple of its step')
    rows = [(x * step, y * step, z) for x in range(-count, count + 1)
            for y in range(-count, count + 1) if x * x + y * y <= count * count
            for z in z_values]
    # Rounded integer-grid distance avoids floating-point asymmetry in ties.
    return sorted(rows, key=lambda p: (round(sum(v*v for v in p), 12),
        round(abs(p[0]) + abs(p[1]), 12), p[2], -p[0], -p[1]))


class TranslatedShapes:
    def __init__(self, rows, vertices):
        self.vertices = vertices.copy()
        self.rows = [(name, obj, np.asarray(obj.getRotation()).copy(),
                      np.asarray(obj.getTranslation()).copy(), lo.copy(), hi.copy())
                     for name, obj, lo, hi in rows]

    def at(self, delta):
        delta = np.asarray(delta)
        result = []
        for name, obj, rotation, position, lo, hi in self.rows:
            obj.setTransform(fcl.Transform(rotation, position + delta))
            result.append((name, obj, lo + delta, hi + delta))
        return result


class CandidateSearch:
    def __init__(self, layout):
        RAY_BOUNDS.clear()
        self.layout = copy.deepcopy(layout)
        self.entry, self.spec, self.assets, self.bounds, _ = geometry(layout)
        if not check_layout(layout)['initial_source_relations_passed']:
            raise ValueError('Initial input must preserve original task relations')
        for row in self.spec['relations']:
            cls = {'on': On, 'next_to': NextTo, 'rotate_around_solution': RotateAroundSolution}.get(row['kind'])
            if cls:
                args = [self.assets[row['reference']]] if 'reference' in row else []
                self.assets[row['subject']].relations.append(cls(*args, **row['params']))
        params = SimpleNamespace(verbose=False, on_relation_z_tolerance_m=.005)
        self.validators = [OnRelationValidator(params), NextToValidator(params)]
        self.kitchen = Usd.Stage.Open(str(ROOT / layout['kitchen_usd']))
        root = str(self.kitchen.GetDefaultPrim().GetPath())
        self.world = static_collision_shapes(self.kitchen, root)
        self.meshes = triangles(self.kitchen, root)
        self.refs = {row['id']: root + '/' + row['prim_path'] for row in self.spec['object_references']}
        floor = self.refs['floor']
        self.nonfloor = [row for row in self.world if not (row[0] == floor or row[0].startswith(floor + '/'))]
        self.stages = [self.kitchen]
        self.shapes = {}
        stand = Usd.Stage.Open(str(ROOT / STAND)); self.stages.append(stand)
        rows, verts = pose_rows(stand, '/panda', np.asarray(layout['robot_position']) + [0, 0, layout['stand_height']],
                                layout['robot_yaw_deg'], '/panda/panda_link0/stand_instanceable')
        self.shapes['droid'] = TranslatedShapes(rows, verts)
        for name, cfg in layout['objects'].items():
            stage = Usd.Stage.Open(str(ROOT / cfg['usd_path'])); self.stages.append(stage)
            rows, verts = pose_rows(stage, str(stage.GetDefaultPrim().GetPath()), cfg['position'], cfg['yaw_deg'])
            self.shapes[name] = TranslatedShapes(rows, verts)
        self.offsets = {name: np.zeros(3) for name in self.shapes}

    def positions(self, subject, delta):
        positions = {self.assets[row['id']]: (0., 0., 0.) for row in self.spec['object_references']}
        for name in self.shapes:
            offset = np.asarray(delta) if name == subject else self.offsets[name]
            base = (np.asarray(self.layout['robot_position']) + [0, 0, self.layout['stand_height']]
                    if name == 'droid' else np.asarray(self.layout['objects'][name]['position']))
            positions[self.assets[name]] = tuple(base + offset)
        return positions

    def evaluate(self, subject, delta):
        positions = self.positions(subject, delta)
        passed = all(validator._validate(positions, self.bounds) for validator in self.validators)
        result = dict(offset_xyz_m=list(delta), original_relations_passed=bool(passed), passed=False)
        if not passed:
            result['not_evaluated'] = 'Rejected by unchanged original On/NextTo relations; yaw is fixed'
            return result
        subject_rows = self.shapes[subject].at(delta)
        measure = minimum_distance(subject_rows, self.nonfloor if subject == 'droid' else self.world, REQUIRED_M)
        result['source_surface_to_kitchen'] = measure
        if not measure['valid']:
            return result
        others = {}
        for other in self.shapes:
            if other == subject:
                continue
            others[other] = minimum_distance(subject_rows, self.shapes[other].at(self.offsets[other]), REQUIRED_M)
        result['source_surface_to_other_task_bodies'] = others
        if not all(row['valid'] for row in others.values()):
            return result
        if subject != 'droid':
            support_id = next(row['reference'] for row in self.spec['relations']
                              if row['kind'] == 'on' and row['subject'] == subject)
            support_path = self.refs[support_id]
            meshes = [row for row in self.meshes if row[0] == support_path or row[0].startswith(support_path + '/')]
            vertices = self.shapes[subject].vertices + delta
            bottom = vertices[vertices[:, 2] <= vertices[:, 2].min() + .002]
            rays = [ray_z(meshes, point) for point in bottom]
            # Existing On 5 mm height band, plus the recorded 2 mm vertex band.
            valid = len(rays) > 0 and all(row is not None and 0 <= row['vertical_gap_m'] <= .007 for row in rays)
            result['declared_support_rays'] = dict(path=support_path, count=len(rays),
                hit_count=sum(row is not None for row in rays), passed=bool(valid),
                min_gap_m=min((row['vertical_gap_m'] for row in rays if row is not None), default=None),
                max_gap_m=max((row['vertical_gap_m'] for row in rays if row is not None), default=None))
            if not valid:
                return result
        result['passed'] = True
        return result

    def search(self, grids):
        results = []
        for subject in self.shapes:
            grid = grids['stand' if subject == 'droid' else 'objects']
            rows = []
            chosen = None
            for index, delta in enumerate(grid):
                row = self.evaluate(subject, delta)
                row['grid_index'] = index
                rows.append(row)
                if row['passed']:
                    chosen = index
                    self.offsets[subject] = np.asarray(delta)
                    break
            results.append(dict(subject=subject, total_predeclared_grid_points=len(grid),
                visited_count=len(rows), unvisited_count=len(grid)-len(rows), selected_grid_index=chosen, rows=rows))
            print(json.dumps(dict(subject=subject, visited=len(rows), selected=chosen,
                                  offset=self.offsets[subject].tolist())), flush=True)
            if chosen is None:
                return None, results
        candidate = copy.deepcopy(self.layout)
        candidate['robot_position'] = (np.asarray(candidate['robot_position']) + self.offsets['droid']).tolist()
        for name, cfg in candidate['objects'].items():
            cfg['position'] = (np.asarray(cfg['position']) + self.offsets[name]).tolist()
        candidate['execution_ready'] = False
        return candidate, results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--layout', type=Path, required=True)
    parser.add_argument('--initial-diagnostic', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        raise FileExistsError(args.output_dir)
    args.output_dir.mkdir(parents=True)
    layout = json.loads(args.layout.read_text())
    initial = json.loads(args.initial_diagnostic.read_text())
    if initial['layout_sha256'] != sha(args.layout) or initial['layout'] != layout:
        raise ValueError('Initial failure evidence must bind exactly this input layout')
    grids = dict(stand=nearest_grid(.8, .025, [0.]), objects=nearest_grid(.5, .025, [0., .001]))
    policy = dict(required_clearance_m=REQUIRED_M,
        stand=dict(xy_radius_m=.8, xy_step_m=.025, z_offsets_m=[0.]),
        objects=dict(xy_radius_m=.5, xy_step_m=.025, z_offsets_m=[0., .001]),
        selection='Stand then source YAML object order; first passing point in Euclidean displacement order. Ties: L1 XY, Z, positive X, positive Y. Stop each subject at its first passing point; preserve every visited row and unvisited count.',
        fixed=['source assets and dependencies', 'kitchen and background poses', 'yaw', 'stand height', 'original relation parameters', 'task and destination identity'],
        gates=['unchanged original On/NextTo/yaw', 'raw source FCL separation >=2mm from kitchen and other task bodies', 'actual declared support triangle ray for every near-bottom source vertex'],
        caveat='Sequential bounded grid; not a global optimum or proof no feasible pose exists outside the grid. Raw source triangle surfaces do not certify cooked physics/solid containment. No full robot/IK/stability/task rollout.')
    declaration = dict(layout_path=str(args.layout), layout_sha256=sha(args.layout),
        initial_diagnostic_path=str(args.initial_diagnostic), initial_diagnostic_sha256=sha(args.initial_diagnostic), policy=policy,
        code_sha256={str(path.relative_to(ROOT)): sha(path) for path in [Path(__file__),
            ROOT/'scripts/extensions/replicator/audit_source_layout_geometry.py',
            ROOT/'scripts/extensions/replicator/audit_task_source_relation.py', ROOT/'kitchen_safety/geometry_audit.py']})
    # Written before evaluating any candidate; later evidence never overwrites it.
    (args.output_dir/'search_declaration.json').write_text(json.dumps(declaration, indent=2)+'\n')
    search = CandidateSearch(layout)
    candidate, rows = search.search(grids)
    report = dict(**declaration, task_id=layout['task_id'], subjects=rows, candidate_found=candidate is not None,
                  initial_geometry_passed=False, execution_ready=False)
    if candidate is not None:
        path = args.output_dir/'candidate_layout.json'
        path.write_text(json.dumps(candidate, indent=2)+'\n')
        final = diagnose(path)
        final_path = args.output_dir/'candidate_geometry.json'
        final_path.write_text(json.dumps(final, indent=2, allow_nan=False)+'\n')
        report.update(candidate_path=str(path), candidate_sha256=sha(path),
                      candidate_geometry_path=str(final_path), candidate_geometry_sha256=sha(final_path),
                      initial_geometry_passed=final['initial_geometry_passed'], original_source_relations=final['original_source_relations'])
    report['geometry_layer_sha256'] = {str(Path(layer.realPath)): sha(layer.realPath)
        for stage in search.stages for layer in stage.GetUsedLayers() if Path(layer.realPath).is_file()}
    report['code_unchanged'] = all(sha(ROOT/path) == expected for path, expected in declaration['code_sha256'].items())
    report['passed'] = bool(report['initial_geometry_passed'] and report['code_unchanged'])
    (args.output_dir/'search_report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(args.output_dir), passed=report['passed'], execution_ready=False)), flush=True)
    return int(not report['passed'])


if __name__ == '__main__':
    raise SystemExit(main())
