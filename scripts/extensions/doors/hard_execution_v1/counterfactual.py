#!/usr/bin/env python3
"""CPU geometry replay: retain a reaching person during recorded later motion.

This never imports Isaac, starts physics, drives a door, or synthesizes robot
commands. A counterfactual collision is evidence from recorded poses, not a
claim that a dangerous physical rollout was performed.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))

import fcl
import numpy as np
from pxr import Usd, UsdGeom

from kitchen_safety.extensions.doors.collision import DoorCollisionWorld
from kitchen_safety.extensions.doors.hard_execution_v1.geometry import MeasuredDoorGeometry
from kitchen_safety.extensions.doors.hard_execution_v1.person import PersonAsset
from kitchen_safety.geometry_audit import minimum_distance
from kitchen_safety.planning import DroidKinematics, ROBOT_USD, transform

DOOR = '/World/envs/env_0/kitchen/fridge_main_group/Refrigerator032_fridge_door'
KITCHEN = ROOT / 'assets/kitchen/scene.usd'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def pose_matrix(value):
    value = np.asarray(value, dtype=float)
    if value.shape != (7,) or not np.isfinite(value).all():
        raise ValueError('Expected finite xyz+xyzw measured rigid-body pose')
    return transform(value[:3], value[3:])


def robot_shapes(world, q, joint_names):
    """Same source-derived convex geometry as live audit, with link labels."""
    joints = dict(zip(joint_names, q))
    links = world.kin.links([joints[f'panda_joint{i}'] for i in range(1, 8)])
    rows = []
    local_gripper = world._gripper_links({name: joints[name] for name in world.GRIPPER_JOINT_NAMES})
    entries = [(body, model, vertices, links[body]) for body, model, vertices in world.robot_hulls]
    entries.extend((body, model, vertices, links[world.gripper_root] @ local_gripper[body])
                   for body, model, vertices in world.gripper_models)
    for body, model, vertices, matrix in entries:
        points = vertices @ matrix[:3, :3].T + matrix[:3, 3]
        rows.append((body, fcl.CollisionObject(model, fcl.Transform(matrix[:3, :3], matrix[:3, 3])),
                     points.min(0), points.max(0)))
    return rows


def detail(left, right, margin):
    row = minimum_distance(left, right, margin)
    a = next(shape for shape in left if shape[0] == row['robot_part'])
    b = next(shape for shape in right if shape[0] == row['obstacle'])
    distances = fcl.DistanceResult()
    fcl.distance(a[1], b[1], fcl.DistanceRequest(enable_nearest_points=True), distances)
    if not row['intersecting']:
        row['nearest_points_world_m'] = np.asarray(distances.nearest_points).tolist()
    else:
        contacts = fcl.CollisionResult()
        fcl.collide(a[1], b[1], fcl.CollisionRequest(num_max_contacts=4, enable_contact=True), contacts)
        row['intersection_contact_points_world_m'] = [np.asarray(c.pos).tolist() for c in contacts.contacts]
    row['human_mesh'] = row.pop('robot_part')
    return row


def replay(episode, human_frames=(72, 80), window='resume', stride=1):
    metadata = json.loads((episode / 'metadata.json').read_text())
    audit = json.loads((episode / 'audit.json').read_text())['checks']
    result = json.loads((episode / 'result.json').read_text())
    spec = metadata['hard_execution']['spec']
    if sha(ROOT / spec['human_asset']) != spec['human_asset_sha256']:
        raise ValueError('Frozen human source differs from recorded execution metadata')
    physics = [r for r in audit if r.get('sample_kind') == 'physics_substep']
    if window == 'resume':
        selected = [r for r in physics if r['intrusion']['completed']]
    elif window == 'all':
        selected = physics
    else:
        raise ValueError(window)
    if not selected:
        raise ValueError('No recorded physics samples in requested window; no hypothetical motion will be invented')
    sampled = selected[::stride]
    if sampled[-1] is not selected[-1]:
        sampled.append(selected[-1])

    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim('/World/envs/env_0/kitchen').GetReferences().AddReference(str(KITCHEN))
    if stage.GetCompositionErrors():
        raise ValueError(str(stage.GetCompositionErrors()))
    source_matrix = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(DOOR))
    # Keep the source nonuniform scale in the cuboid vertices. Only strip scale
    # from the BODY pose used to map those vertices to measured rigid readbacks.
    source_rigid = np.asarray(source_matrix.RemoveScaleShear()).T
    recorded_initial = pose_matrix(metadata['initial_door_pose'])
    alignment = recorded_initial @ np.linalg.inv(source_rigid)
    alignment_translation = float(np.linalg.norm(alignment[:3, 3]))
    alignment_rotation = float(np.arccos(np.clip((np.trace(alignment[:3, :3]) - 1.) / 2., -1., 1.)))
    if alignment_translation > 1e-4 or alignment_rotation > 1e-4:
        raise ValueError('Source closed door does not match native reset; explicit new source calibration is required')
    door = MeasuredDoorGeometry(stage, DOOR)
    # No kitchen collision ingestion is needed for robot-vs-person pairs.
    # The unchanged class still reads every original arm/gripper source mesh.
    robot_stage = Usd.Stage.CreateInMemory()
    robot_stage.DefinePrim('/World/envs/env_0')
    kin = DroidKinematics(metadata['robot_base'], metadata['robot_yaw_deg'])
    world = DoorCollisionWorld(robot_stage, kin, env_root='/World/envs/env_0')
    people = {}
    for frame in human_frames:
        person = PersonAsset(spec)
        person._source_shapes(frame)
        people[frame] = person
    margin = spec['source_mesh_human_clearance_m']
    paused = [r for r in physics if 66 <= r['human_frame'] <= 90 and r['intrusion']['hold_requested']]
    pause_checks = {}
    if paused:
        for frame, person in people.items():
            state = min(paused, key=lambda r: abs(r['human_frame'] - frame))
            measured_door = door.measured(pose_matrix(state['door_body_pose']) @ np.linalg.inv(source_rigid))
            robot = robot_shapes(world, state['q'], metadata['robot_joint_names'])
            pause_checks[str(frame)] = {
                'physics_step': state['physics_step'], 'recorded_human_frame': state['human_frame'],
                'counterfactual_human_frame': frame, 'measured_door_deg': float(np.degrees(state['door_q'])),
                'human_to_door': detail(person.shapes, measured_door, margin),
                'human_to_robot': detail(person.shapes, robot, margin),
            }
    records = {frame: [] for frame in human_frames}
    for state in sampled:
        moving_door = door.measured(pose_matrix(state['door_body_pose']) @ np.linalg.inv(source_rigid))
        robot = robot_shapes(world, state['q'], metadata['robot_joint_names'])
        for frame, person in people.items():
            records[frame].append({
                'physics_step': state['physics_step'], 'action_index': state['action_index'],
                'simulation_seconds': state['simulation_seconds'],
                'recorded_human_frame': state['human_frame'], 'counterfactual_human_frame': frame,
                'measured_door_rad': state['door_q'], 'measured_door_deg': float(np.degrees(state['door_q'])),
                'human_to_door': detail(person.shapes, moving_door, margin),
                'human_to_robot': detail(person.shapes, robot, margin),
            })
    summaries = {}
    for frame, rows in records.items():
        summary = {}
        for kind in ('human_to_door', 'human_to_robot'):
            violating = [r for r in rows if not r[kind]['valid']]
            hits = [r for r in rows if r[kind]['intersecting']]
            worst = min(rows, key=lambda r: r[kind]['distance_m'])
            summary[kind] = {'minimum_distance_m': worst[kind]['distance_m'],
                             'violating_samples': len(violating), 'intersecting_samples': len(hits),
                             'first_below_margin': violating[0] if violating else None,
                             'first_intersection': hits[0] if hits else None,
                             'worst_sample': worst}
        summaries[str(frame)] = summary
    dependencies = {KITCHEN, ROBOT_USD, ROOT / spec['human_asset'], ROOT / spec['human_animation'],
                    Path(__file__).resolve(), ROOT / 'kitchen_safety/planning.py',
                    ROOT / 'kitchen_safety/collision_planner.py', ROOT / 'kitchen_safety/geometry_audit.py',
                    ROOT / 'kitchen_safety/extensions/doors/collision.py',
                    ROOT / 'kitchen_safety/extensions/doors/hard_execution_v1/person.py',
                    ROOT / 'kitchen_safety/extensions/doors/hard_execution_v1/geometry.py'}
    dependencies.update(Path(layer.realPath) for layer in stage.GetUsedLayers() if layer.realPath)
    required_end = float(spec.get('hard_completion_door_rad', 0.))
    reached_end = bool(required_end > 0 and physics[-1]['door_q'] >= required_end)
    breach = any(r[k]['violating_samples'] for r in summaries.values() for k in ('human_to_door', 'human_to_robot'))
    safe_pause = bool(len(pause_checks) == len(people) and all(
        row[k]['valid'] for row in pause_checks.values() for k in ('human_to_door', 'human_to_robot')))
    return {
        'scope': 'CPU-only counterfactual source geometry replay of recorded actual physics poses; no dangerous physical replay.',
        'episode': str(episode), 'window': window, 'stride': stride,
        'all_selected_physics_samples_checked': stride == 1,
        'selected_physics_samples': len(selected), 'evaluated_physics_samples': len(sampled),
        'required_clearance_m': margin, 'human_position_m': spec['human_position_m'],
        'recorded_final_door_deg': float(np.degrees(physics[-1]['door_q'])),
        'recorded_selected_door_range_deg': [float(np.degrees(min(r['door_q'] for r in selected))),
                                           float(np.degrees(max(r['door_q'] for r in selected)))],
        'additional_measured_opening_required_deg': float(np.degrees(required_end)),
        'additional_end_angle_reached_by_recorded_substep': reached_end,
        'native_task_success_reported_separately': bool(result['task_success']),
        'actual_demonstration_completed_reported': bool(result.get('demonstration_completed')),
        'human_yaw_deg': spec['human_yaw_deg'], 'fixed_human_frames': list(human_frames),
        'geometry': {'human': 'Frozen six source triangle meshes, evaluated at fixed reaching frame.',
                     'door': 'All enabled source affine Cube colliders represented by exact corner triangles; measured xyz+xyzw rigid pose.',
                     'robot': 'Source-derived per-link convex hulls with measured arm and six gripper joint positions.',
                     'door_colliders': len(door.rows), 'source_to_native_initial_translation_m': alignment_translation,
                     'source_to_native_initial_rotation_rad': alignment_rotation},
        'recorded_motion_requires_human_clearance': breach,
        'recorded_pause_is_safe_for_frozen_reaching_person': safe_pause,
        'waiting_necessity_supported_by_actual_later_motion': bool(
            window == 'resume' and stride == 1 and reached_end and safe_pause and breach),
        'physical_counterfactual_executed': False, 'accepted': False,
        'limitations': ['The person is deliberately frozen at a reaching pose while recorded later robot/door states are replayed geometrically.',
                       'No trajectory, contacts or force response of a hypothetical collision is simulated.',
                       'No unrecorded later opening angle or robot pose is synthesized.',
                       'Robot convex hulls conservatively cover source mesh surfaces; door and human use source triangles.',
                       'Discrete recorded physics samples do not prove continuous swept-volume clearance between samples.'],
        'input_sha256': {name: sha(episode / name) for name in ('metadata.json', 'audit.json', 'result.json')},
        'source_dependency_sha256': {str(p.relative_to(ROOT)): sha(p) for p in sorted(dependencies)},
        'actual_pause_checks': pause_checks,
        'summary': summaries, 'samples': {str(k): v for k, v in records.items()},
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episode', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--human-frames', nargs='+', type=int, default=[72, 80])
    parser.add_argument('--window', choices=['resume', 'all'], default='resume')
    parser.add_argument('--stride', type=int, default=1, help='Default 1 checks every recorded physics substep; >1 is exploratory only')
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    if args.stride < 1 or any(frame < 0 or frame > 167 for frame in args.human_frames):
        raise ValueError('Invalid frame or stride')
    report = replay(args.episode, tuple(args.human_frames), args.window, args.stride)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k: report[k] for k in ('scope', 'evaluated_physics_samples', 'recorded_motion_requires_human_clearance', 'summary')}, indent=2))


if __name__ == '__main__':
    main()
