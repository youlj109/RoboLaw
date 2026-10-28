#!/usr/bin/env python3
"""Prepare or run one original-cup/original-counter passive support observation.

Default: CPU preparation only. --run invokes the existing native neutral
collector, including its shared GPU lock, then joins each recorded physics step
to CPU source geometry. There are no runtime object/joint state setters here.
The only changed initial condition is the cup reset pose. This is deliberately
not the accepted glass-in-bowl episode and never establishes a general allowed
penetration depth or a task-success result.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))
DEFAULT_CONFIG = ROOT / 'configs/extensions/replicator/hard_execution_v1/property_yaw10.json'
COUNTER = '/World/envs/env_0/kitchen/CapSurface_02'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    def convert(item):
        if hasattr(item, 'tolist'):
            return item.tolist()
        if isinstance(item, Path):
            return str(item)
        raise TypeError(type(item).__name__)
    Path(path).write_text(json.dumps(value, indent=2, default=convert, allow_nan=False) + '\n')


def source_pair(world, pose):
    """Retain signed vertical separation and both source FCL representations."""
    import numpy as np
    counter = next(item for item in world.objects if item[0] == COUNTER)
    vertices = world.payload_geometry.vertices @ pose[:3, :3].T + pose[:3, 3]
    result = dict(source_bottom_minus_countertop_m=float(vertices[:, 2].min() - counter[3][2]),
                  counter_bounds_world_m=[counter[2], counter[3]])
    for label, model in (('source_triangles', world.payload_triangle_model),
                         ('source_convex_envelope', world.payload_model)):
        body = world.fcl.CollisionObject(model, world.fcl.Transform(pose[:3, :3], pose[:3, 3]))
        raw = float(world.fcl.distance(body, counter[1], world.fcl.DistanceRequest(), world.fcl.DistanceResult()))
        intersecting = bool(world.fcl.collide(body, counter[1], world.fcl.CollisionRequest(), world.fcl.CollisionResult()))
        if not np.isfinite(raw):
            raise ValueError('Nonfinite source FCL result')
        result[label] = dict(raw_distance_m=raw, separation_m=max(0., raw), intersecting=intersecting or raw < 0.,
                             negative_fcl_value_is_collision_sentinel_not_depth=True)
    return result, vertices


def prepare(args):
    import numpy as np
    from pxr import Usd, UsdPhysics
    from scipy.spatial.transform import Rotation
    from kitchen_safety.extensions.replicator.hard_execution_v1.property_environment import read_config, execution_layout
    from kitchen_safety.extensions.replicator.hard_execution_v1.property_geometry import build_world, source_initial_q
    from kitchen_safety.extensions.replicator.task_motion import pose_matrix

    original = read_config(args.config)
    config = copy.deepcopy(original)
    # Determine the counter height and all cup source vertices from the exact
    # source world, rather than relying on a guessed root-to-bottom distance.
    stage, world, kin, old_pose = build_world(original)
    counter = next(item for item in world.objects if item[0] == COUNTER)
    yaw = args.yaw_deg
    rotation = Rotation.from_euler('z', yaw, degrees=True)
    local_rotated = rotation.apply(world.payload_geometry.vertices)
    target = np.r_[original['safe_placement']['glass_target_position_m'][:2],
                   counter[3][2] + args.gap_m - float(local_rotated[:, 2].min())]
    config['glass']['position'] = target.tolist()
    config['glass']['yaw_deg'] = yaw
    config['calibration_scope'] = dict(kind='original_cup_counter_passive_reset_observation',
        changed_from_accepted_layout=['fragile_glass initial position and yaw only'],
        initial_bottom_gap_m=args.gap_m, upright_reset=True, runtime_pose_writes=False,
        physical_asset_or_actuator_changes=False, accepted_scene_is_reference_only=True,
        no_task_execution_or_acceptance_claim=True, no_general_penetration_tolerance=True)
    glass_stage = Usd.Stage.Open(str(ROOT / config['glass']['usd_path']))
    glass_root = glass_stage.GetDefaultPrim()
    mass = float(UsdPhysics.MassAPI(glass_root).GetMassAttr().Get())
    source_mass_matches = mass == float(np.float32(.56)) and float(config['glass']['mass_kg']) == .56
    if not source_mass_matches:
        raise ValueError('Original 0.56 kg source mass changed')
    body = UsdPhysics.RigidBodyAPI(glass_root)
    dynamic = glass_root.HasAPI(UsdPhysics.RigidBodyAPI) and body.GetRigidBodyEnabledAttr().Get() is not False and body.GetKinematicEnabledAttr().Get() is not True
    pose = np.r_[target, rotation.as_quat()]
    pair, vertices = source_pair(world, pose_matrix(pose))
    world.payload_pose = pose_matrix(pose)
    values, initial = source_initial_q()
    q = np.asarray([values[f'panda_joint{i}'] for i in range(1, 8)])
    clear = bool(world.valid(q, gripper_q=np.zeros(6), object_pose=pose))
    layout = execution_layout(config)
    before = execution_layout(original)
    unchanged = copy.deepcopy(layout)
    unchanged['objects']['fragile_glass'] = before['objects']['fragile_glass']
    radius = float(np.linalg.norm(vertices[:, :2] - target[:2], axis=1).max())
    checks = dict(original_input_hashes_match=True, original_dynamic_glass=bool(dynamic),
        original_glass_mass_kg_056=source_mass_matches, all_41_source_colliders=len(world.payload_geometry.provenance['colliders']) == 41,
        kitchen_robot_bowl_layout_unchanged=unchanged == before,
        source_counter_height_matches_config=abs(float(counter[3][2]) - original['safe_placement']['countertop_z_m']) < 1e-7,
        initial_positive_gap_matches=abs(pair['source_bottom_minus_countertop_m'] - args.gap_m) < 1e-8,
        initial_source_counter_nonintersecting=not pair['source_triangles']['intersecting'],
        complete_cup_inside_original_safe_circle=radius <= original['safe_placement']['region_radius_m'],
        complete_cup_over_original_counter=bool(np.all(vertices[:, :2].min(0) >= counter[2][:2]) and np.all(vertices[:, :2].max(0) <= counter[3][:2])),
        original_initial_robot_and_full_world_clear=clear)
    args.output.mkdir(parents=True, exist_ok=False)
    derived = args.output / 'calibration_config.json'
    write(derived, config)
    sources = [Path(__file__).resolve(), ROOT / 'scripts/extensions/replicator/hard_execution_v1/property_collect.py']
    sources += [ROOT / ('kitchen_safety/extensions/replicator/hard_execution_v1/' + name + '.py')
                for name in ('property_environment', 'property_runtime', 'property_geometry')]
    sources += [ROOT / 'kitchen_safety/extensions/replicator/task_recorder.py',
                ROOT / 'kitchen_safety/extensions/replicator/task_contacts.py']
    snapshots = args.output / 'source_snapshot'
    snapshots.mkdir()
    captured = {}
    for path in sources:
        name = path.relative_to(ROOT).as_posix()
        captured[name] = dict(sha256=sha(path), text=path.read_text())
    write(snapshots / 'runtime_sources.json', captured)
    command = [sys.executable, str(ROOT / 'scripts/extensions/replicator/hard_execution_v1/property_collect.py'),
               '--config', str(derived), '--output', str(args.output / 'native'), '--stage', 'neutral',
               '--duration-seconds', str(args.duration_seconds), '--seed', str(args.seed),
               '--device', 'cpu', '--physics-threads', str(args.physics_threads), '--torch-threads', str(args.torch_threads)]
    report = dict(scope=config['calibration_scope'], checks=checks, cpu_preparation_passed=all(checks.values()),
        config_source=str(args.config), config_source_sha256=sha(args.config), config_sha256=sha(derived),
        input_sha256=config['input_sha256'], reset_pose_xyz_xyzw=pose, source_mass_kg=mass,
        full_source_geometry=world.payload_geometry.provenance, source_pair=pair,
        maximum_source_radius_from_safe_center_m=radius, initial_robot=initial, initial_world_failure=world.last_clearance,
        source_layout=layout, native_command=command,
        native_lock='Existing property_collect.py acquires /workspace/reports/skill_trials/gpu.lock internally, nonblocking; wrapper does not hold a second lock.',
        requested_duration_seconds=args.duration_seconds, source_fcl_geometry_is_physx_cooked_shape=False,
        requested_stability=dict(start_seconds=5., minimum_window_seconds=3., linear_speed_m_s=.005,
            angular_speed_rad_s=.05, translation_m=.0005, rotation_deg=.2, minimum_weight_fraction=.8),
        physics_launched=False, accepted=False)
    write(args.output / 'preparation.json', report)
    if not report['cpu_preparation_passed']:
        raise RuntimeError('Calibration CPU preparation failed; see preparation.json')
    return command


def review(episode, config_path, output):
    """Read archived native observations; this function cannot launch physics."""
    import numpy as np
    from scipy.spatial.transform import Rotation
    from kitchen_safety.extensions.replicator.hard_execution_v1.property_environment import read_config
    from kitchen_safety.extensions.replicator.hard_execution_v1.property_geometry import build_world
    from kitchen_safety.extensions.replicator.task_motion import pose_matrix

    episode, output = Path(episode), Path(output)
    if output.exists():
        raise FileExistsError(output)
    config = read_config(config_path)
    stage, world, kin, pose = build_world(config)
    meta = json.loads((episode / 'physics_metadata.json').read_text())
    states = np.load(episode / 'physics_states.npz')
    native = json.loads((episode / 'native_glass_support_contacts.json').read_text())
    result = json.loads((episode / 'result.json').read_text())
    actions = json.loads((episode / 'actions.json').read_text())
    expected = dict(subject='/World/envs/env_0/fragile_glass',
                    filters=['/World/envs/env_0/bowl', COUNTER])
    if native['identity'] != expected:
        raise ValueError('Native contact identities differ from the original cup/counter pair')
    steps = states['physics_step'].astype(int)
    if [r['physics_step'] for r in native['rows']] != steps.tolist():
        raise ValueError('Native contacts are not aligned one-to-one with actual physics samples')
    gi = meta['task_object_names'].index('fragile_glass')
    elapsed = (steps - steps[0]) * meta['physics_dt_s']
    cup = states['task_object_states'][:, gi]
    counter = next(item for item in world.objects if item[0] == COUNTER)
    counter_top = float(counter[3][2])
    safe_xy = np.asarray(config['safe_placement']['glass_target_position_m'][:2])
    records = []
    for index, raw in enumerate(native['rows']):
        current = cup[index]
        pose = pose_matrix(current[:7])
        pair, points = source_pair(world, pose)
        counts = np.asarray(raw['counts']).reshape(-1)
        starts = np.asarray(raw['starts']).reshape(-1)
        first, count = int(starts[1]), int(counts[1])
        select = slice(first, first + count)
        separations = np.asarray(raw['separations_m']).reshape(-1)[select]
        forces = np.asarray(raw['normal_forces_n']).reshape(-1)[select]
        locations = np.asarray(raw['points_world_m']).reshape(-1, 3)[select]
        normals = np.asarray(raw['normals_world']).reshape(-1, 3)[select]
        filtered = np.asarray(raw['filtered_force_matrix_n']).reshape(-1, 2, 3)[0]
        local = (locations - pose[:3, 3]) @ pose[:3, :3]
        summed = np.sum(forces[:, None] * normals, axis=0)
        loaded = forces > .01
        bottom = (np.abs(local[:, 2]) <= .006) & (np.linalg.norm(local[:, :2], axis=1) <= .040)
        # This bounds contact-point location using its own signed separation;
        # it is not a global allowable source/body penetration depth.
        numeric = float(4 * np.max(np.abs(np.spacing(np.asarray(np.r_[pose[:3, 3], counter[3]], dtype=np.float32)))))
        delta_z = locations[:, 2] - counter_top
        surface = (delta_z >= np.minimum(0., separations * .5) - numeric) & (delta_z <= np.maximum(0., separations * .5) + numeric)
        upward = normals[:, 2] >= np.cos(np.deg2rad(20))
        on_counter = np.all((locations[:, :2] >= counter[2][:2]) & (locations[:, :2] <= counter[3][:2]), axis=1)
        all_patch = bool(count and np.all(bottom & surface & upward & on_counter))
        force_match = float(np.linalg.norm(summed - filtered[1]))
        record = dict(physics_step=int(steps[index]), elapsed_seconds=float(elapsed[index]), phase=str(states['phase'][index]),
            actual_pose_xyz_xyzw=current[:7], actual_linear_velocity_m_s=current[7:10], actual_angular_velocity_rad_s=current[10:13],
            linear_speed_m_s=float(np.linalg.norm(current[7:10])), angular_speed_rad_s=float(np.linalg.norm(current[10:13])),
            tilt_deg=float(np.degrees(np.arccos(np.clip(pose[2, 2], -1., 1.)))),
            source_pair=pair, all_source_footprint_radius_from_safe_center_m=float(np.linalg.norm(points[:, :2] - safe_xy, axis=1).max()),
            native_points_world_m=locations, native_points_cup_local_m=local, native_normals_world=normals,
            native_normal_forces_n=forces, native_signed_separations_m=separations,
            native_minimum_signed_separation_m=float(separations.min()) if count else None,
            native_filtered_counter_force_n=filtered[1], native_filtered_bowl_force_n=filtered[0],
            native_raw_summed_counter_force_n=summed, raw_filtered_force_residual_n=force_match,
            counter_upward_weight_fraction=float(filtered[1, 2] / (.56 * 9.81)),
            original_bottom_counter_upward_patch=all_patch,
            loaded_counter_support_observed=bool(loaded.any() and all_patch and force_match < .1 and
                filtered[1, 2] >= .8 * .56 * 9.81 and np.linalg.norm(filtered[0]) < .01),
            numeric_coordinate_bound_m=numeric, static_counter_contact_offset_verified=False)
        records.append(record)
    stable = [r for r in records if r['elapsed_seconds'] >= 5. - 1e-8]
    first = next((i for i, r in enumerate(records) if r['elapsed_seconds'] >= 5. - 1e-8), None)
    stable_pose_motion = None
    if first is not None:
        positions = cup[first:, :3]
        rotations = Rotation.from_quat(cup[first:, 3:7])
        stable_pose_motion = dict(max_translation_m=float(np.linalg.norm(positions - positions[0], axis=1).max()),
            max_rotation_deg=float(np.degrees((rotations * rotations[0].inv()).magnitude()).max()))
    summary = dict(samples=len(stable), duration_seconds=0. if not stable else stable[-1]['elapsed_seconds'] - stable[0]['elapsed_seconds'],
        pose_motion=stable_pose_motion)
    for key, values in {
        'source_bottom_minus_countertop_m': [r['source_pair']['source_bottom_minus_countertop_m'] for r in stable],
        'native_signed_minimum_m': [r['native_minimum_signed_separation_m'] for r in stable if r['native_minimum_signed_separation_m'] is not None],
        'counter_upward_weight_fraction': [r['counter_upward_weight_fraction'] for r in stable],
        'linear_speed_m_s': [r['linear_speed_m_s'] for r in stable],
        'angular_speed_rad_s': [r['angular_speed_rad_s'] for r in stable],
    }.items():
        summary[key] = None if not values else dict(minimum=min(values), maximum=max(values))
    commands = np.asarray([row['action'] for row in actions])
    checks = dict(continuous_post_reset_substeps=bool(len(steps) > 1 and np.all(np.diff(steps) == 1)),
        same_step_contact_state_join=True, all_phases_passive_neutral=bool(np.isin(states['phase'], ['reset', 'initial_settle', 'neutral_hold']).all()),
        no_planning_attachment=not bool(states['has_planning_attachment'].any()),
        constant_arm_hold_and_open_gripper=bool(len(commands) and np.all(commands[:, :7] == commands[0, :7]) and np.all(commands[:, 7] == 0.)),
        stable_observation_window_at_least_3_seconds=summary['duration_seconds'] >= 3. - 1e-7,
        every_stable_sample_correct_native_support=bool(stable and all(r['loaded_counter_support_observed'] for r in stable)),
        stable_linear_speed_below_5mm_s=bool(stable and max(r['linear_speed_m_s'] for r in stable) <= .005),
        stable_angular_speed_below_005rad_s=bool(stable and max(r['angular_speed_rad_s'] for r in stable) <= .05),
        stable_translation_below_05mm=bool(stable_pose_motion and stable_pose_motion['max_translation_m'] <= .0005),
        stable_rotation_below_02deg=bool(stable_pose_motion and stable_pose_motion['max_rotation_deg'] <= .2),
        every_stable_cup_upright_inside_safe_circle=bool(stable and all(r['tilt_deg'] <= 3. and r['all_source_footprint_radius_from_safe_center_m'] <= .052 for r in stable)))
    report = dict(scope='Independent same-step CPU source geometry joined to passive native reset observation; no task-success or general penetration tolerance',
        episode=str(episode), config=str(config_path), config_sha256=sha(config_path),
        input_sha256={name: sha(episode / name) for name in ('physics_metadata.json', 'physics_states.npz', 'native_glass_support_contacts.json', 'actions.json', 'result.json')},
        source_geometry=world.payload_geometry.provenance, contact_identity=expected, checks=checks,
        stable_native_support_observed=all(checks.values()), accepted=False, physical_nonpenetration_claim=False,
        static_counter_contact_offset_verified=False, general_penetration_allowance_generated=False,
        source_fcl_representation_is_physx_cooked_shape=False, actual_signed_residuals_preserved=True,
        native_collector_status=result.get('status'), native_physical_observation_audit=result.get('physical_observation_audit'),
        stable_window=summary, rows=records)
    write(output, report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--output', type=Path, required=True, help='Fresh calibration directory, or existing prepared directory with --review-only')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--run', action='store_true', help='Explicitly invoke the existing native neutral collector after CPU preparation')
    mode.add_argument('--review-only', action='store_true', help='Read existing output/native records; does not launch physics')
    parser.add_argument('--gap-m', type=float, default=.003)
    parser.add_argument('--yaw-deg', type=float, default=-170.)
    parser.add_argument('--duration-seconds', type=float, default=8.)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--physics-threads', type=int, default=2)
    parser.add_argument('--torch-threads', type=int, default=2)
    args = parser.parse_args()
    args.config = args.config.resolve()
    args.output = args.output.resolve()
    if not .0001 <= args.gap_m <= .004:
        parser.error('Initial source bottom gap must be in [0.1,4] mm')
    if not 8 <= args.duration_seconds <= 20:
        parser.error('Passive calibration duration must be in [8,20] seconds')
    if args.review_only:
        report = review(args.output / 'native', args.output / 'calibration_config.json', args.output / 'source_contact_review.json')
        print(json.dumps(dict(stable_native_support_observed=report['stable_native_support_observed'], output=str(args.output))))
        return 0
    command = prepare(args)
    print(json.dumps(dict(cpu_preparation_passed=True, physics_launched=False, output=str(args.output)), indent=2), flush=True)
    if not args.run:
        return 0
    if ROOT != Path('/workspace'):
        raise RuntimeError('Native execution requires the verified /workspace runtime; no host simulation launch')
    completed = subprocess.run(command, cwd=ROOT, check=False)
    write(args.output / 'launch_result.json', dict(native_returncode=completed.returncode, command=command,
        physics_requested=True, scope='Passive cup/counter calibration only; no task acceptance'))
    if (args.output / 'native/physics_states.npz').exists():
        report = review(args.output / 'native', args.output / 'calibration_config.json', args.output / 'source_contact_review.json')
        print(json.dumps(dict(stable_native_support_observed=report['stable_native_support_observed'], native_returncode=completed.returncode)))
    return completed.returncode


if __name__ == '__main__':
    raise SystemExit(main())
