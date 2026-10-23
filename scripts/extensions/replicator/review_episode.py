"""CPU review of recorded mustard-to-bowl contact, release and background state.

This is an independent numeric check of raw action-rate records. It does not
replace physics-substep collision, neutral-scene, source-geometry or visual
reviews, and never marks a new episode as repair-certified.
"""
import argparse
import ast
import hashlib
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import h5py
import numpy as np
from scipy.spatial.transform import Rotation
from scripts.extensions.replicator.review_neutral_evidence import load_neutral

ROOT = Path(__file__).resolve().parents[3]
SOURCE_TASK = ROOT / 'third_party/IsaacLab-Arena/isaaclab_arena/tasks/pick_and_place_task.py'
SOURCE_SPATIAL = ROOT / 'third_party/IsaacLab-Arena/isaaclab_arena/tasks/predicates/spatial.py'


@dataclass(frozen=True)
class SourceThresholds:
    force_threshold: float
    velocity_threshold: float
    support_cone_half_angle_rad: float


def source_thresholds(path=SOURCE_TASK):
    """Read literal upstream constructor defaults without importing Kit."""
    tree = ast.parse(Path(path).read_text())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'PickAndPlaceTask')
    init = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == '__init__')
    defaults = dict(zip([arg.arg for arg in init.args.args][-len(init.args.defaults):], init.args.defaults))

    def number(node):
        if isinstance(node, ast.Constant) and type(node.value) in [int, float]:
            return float(node.value)
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id == 'math' and node.attr == 'pi':
            return math.pi
        if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Div):
            return number(node.left) / number(node.right)
        raise ValueError('Unsupported source predicate default; inspect upstream explicitly')

    return SourceThresholds(**{name: number(defaults[name]) for name in SourceThresholds.__dataclass_fields__})


@dataclass(frozen=True)
class Limits:
    loaded_force_n: float = .01
    support_force_n: float = .01
    open_finger_rad: float = .01
    object_speed_m_s: float = .01
    object_drift_m: float = .001
    stability_window_s: float = 1.
    lift_height_m: float = .01


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def require(condition, message):
    if not condition:
        raise ValueError(message)


def background_samples(episode, result, action_background, action_dt, settling_s):
    """Use the true reset reference when complete prefix/suffix evidence exists.

    Legacy recordings retain their documented action-rate reference. A partial
    new evidence set fails rather than silently falling back to that reference.
    """
    evidence_names = ['startup_audit.json', 'startup_physics_states.npz',
                      'motion_physics_index.npz', 'motion_physics_alignment.json', 'collection_provenance.json']
    if not any((episode / name).exists() for name in evidence_names):
        return action_background, np.arange(len(action_background))*action_dt, [], 'first_recorded_action_after_warmup'
    from kitchen_safety.extensions.replicator.collection_evidence import audit_saved_collection_evidence
    from kitchen_safety.extensions.replicator.startup_recorder import audit_startup
    complete = audit_saved_collection_evidence(episode)
    require(complete['passed'], 'Incomplete absolute background physics/provenance evidence')
    prefix_report = read(episode / 'startup_audit.json')
    with np.load(episode / 'startup_physics_states.npz', allow_pickle=False) as stored:
        prefix = {name: stored[name] for name in stored.files}
    meta = prefix_report['metadata']
    require(prefix_report == result['startup_recording'] and prefix_report['passed'] is True and
            meta['background_physics'] == result['background_physics'], 'Startup/result physical profile mismatch')
    require(audit_startup(prefix, meta)['passed'], 'Recomputed startup physics failed')
    require(meta['background_paths'] == result['background_rigid_paths'] == prefix['background_paths'].tolist(),
            'Reset background identities differ')
    with np.load(episode / 'background_physics_states.npz', allow_pickle=False) as suffix, \
         np.load(episode / 'motion_physics_index.npz', allow_pickle=False) as index:
        require(suffix['paths'].tolist() == meta['background_paths'] and
                len(suffix['poses']) == len(index['physics_step']), 'Suffix background sample mapping differs')
        values = np.concatenate([prefix['background_rigid_poses'], suffix['poses']])
        steps = np.concatenate([prefix['physics_step'], index['physics_step']])
    require(np.all(np.diff(steps) == 1) and steps[0] == meta['reset_physics_step'], 'Background physics has gaps')
    require(np.isfinite(values).all(), 'Nonfinite complete background physics')
    decimation = round(action_dt/meta['physics_dt_s'])
    require(abs(decimation*meta['physics_dt_s']-action_dt) < 1e-9 and decimation > 0,
            'Background physics/action dt differs')
    action_indices = (8+np.arange(len(action_background)))*decimation
    require(action_indices[-1] == len(values)-1 and np.array_equal(values[action_indices], action_background),
            'Full background physics does not match every recorded action boundary')
    time = (steps-steps[0])*meta['physics_dt_s']
    require(time[-1] >= settling_s, 'Complete physics is too short for settling')
    names = list(complete['input_sha256']) + ['startup_audit.json', 'background_physics_states.npz']
    return values, time, [episode / name for name in set(names)], 'reset_initial_full_physics'


def review(episode, neutral_dir, body_z_range, limits=Limits(), source=None):
    """Return the measured review; malformed/missing evidence raises an error."""
    episode, neutral_dir = Path(episode), Path(neutral_dir)
    result = read(episode / 'result.json')
    contacts = read(episode / 'grasp_contacts.json')
    relation = read(episode / 'source_relation_audit.json')
    neutral, states = load_neutral(neutral_dir)
    source = source_thresholds() if source is None else source
    require(source.force_threshold >= 0 and source.velocity_threshold >= 0 and
            0 <= source.support_cone_half_angle_rad < math.pi / 2,
            'Invalid source predicate thresholds')
    neutral_frames = len(states['q'])
    require(all(np.isfinite(value).all() for value in states.values() if value.dtype.kind in 'biufc'), 'Nonfinite neutral states')
    require(states['background'].shape == (neutral_frames, len(neutral['background_paths']), 7),
            'Missing or mismatched raw neutral background states')
    require(states['background_velocity'].shape == (neutral_frames, len(neutral['background_paths']), 6),
            'Missing or mismatched raw neutral background velocities')
    low, high = map(float, body_z_range)
    require(np.isfinite([low, high]).all() and low < high, 'Invalid source contact band')
    require(all(np.isfinite(v) and v > 0 for v in asdict(limits).values()), 'Limits must be positive and finite')
    with h5py.File(episode / 'trajectory.h5') as h5:
        phases = h5['phase'].asstr()[:]
        dt = float(h5.attrs['dt'])
        names = json.loads(h5.attrs['joint_names'])
        require(len(names) == len(set(names)), 'Duplicate joint names')
        finger_index = names.index('finger_joint')
        n = len(phases)
        require(n > 0 and np.isfinite(dt) and dt > 0, 'Invalid frame count or dt')
        values = {}
        for name in ['mustard', 'bowl', 'q', 'background_rigid_poses']:
            before, after = h5['obs/' + name][:], h5['next_obs/' + name][:]
            require(len(before) == n and before.shape == after.shape, f'Misaligned {name}')
            require(np.isfinite(before).all() and np.isfinite(after).all(), f'Nonfinite {name}')
            require(np.allclose(before[1:], after[:-1], rtol=0, atol=1e-6), f'Discontinuous {name} records')
            values[name] = np.concatenate([before[:1], after])
    obj, q, background = values['mustard'], values['q'], values['background_rigid_poses']
    require(obj.shape == (n + 1, 13) and q.shape == (n + 1, len(names)), 'Invalid object/joint dimensions')
    indices = {phase: np.flatnonzero(phases == phase) for phase in dict.fromkeys(phases)}
    for phase in ['close', 'lift', 'above_bowl', 'lower', 'release']:
        require(phase in indices, 'Missing task phase: ' + phase)
    phase_order = ['close', 'lift', 'above_bowl', 'lower', 'release']
    require(all(indices[x][-1] < indices[y][0] for x, y in zip(phase_order, phase_order[1:])),
            'Task phases are out of order or interleaved')

    loaded, seen = [], set()
    for row in contacts:
        action = int(row['action_index'])
        require(action not in seen and 0 <= action < n, 'Duplicate or invalid contact action index')
        seen.add(action)
        require(row['phase'] == phases[action], 'Contact phase does not match HDF5 action')
        require(set(row['fingers']) == {'left_inner_finger', 'right_inner_finger'}, 'Missing bilateral finger data')
        flags, body_flags, loaded_z = [], [], []
        for finger in row['fingers'].values():
            force = np.asarray(finger['normal_forces_n'], dtype=float).reshape(-1)
            points = np.asarray(finger['points_bottle_local_m'], dtype=float).reshape(-1, 3)
            require(len(force) == len(points), 'Contact forces/points have different lengths')
            require(np.isfinite(force).all() and np.isfinite(points).all(), 'Nonfinite contact record')
            mask = np.abs(force) > limits.loaded_force_n
            flags.append(bool(mask.any()))
            body_flags.append(bool(mask.any() and np.all((points[mask, 2] >= low) & (points[mask, 2] <= high))))
            loaded_z.extend(points[mask, 2].tolist())
        loaded.append(dict(action=action, phase=row['phase'], bilateral_loaded=all(flags),
                           bilateral_loaded_body_region=all(body_flags),
                           loaded_z_range=[min(loaded_z), max(loaded_z)] if loaded_z else None))
    first = next((row for row in sorted(loaded, key=lambda x: x['action']) if row['bilateral_loaded_body_region']), None)
    carried = [row for row in loaded if row['phase'] in ['lift', 'above_bowl', 'lower']]
    expected_carried = set(np.concatenate([indices[p] for p in ['lift', 'above_bowl', 'lower']]).tolist())

    neutral_dt = float(neutral['dt'])
    require(np.isfinite(neutral_dt) and neutral_dt > 0, 'Invalid neutral dt')
    settling_s = float(neutral['background_settling_seconds'])
    settling_frames = round(settling_s / dt)
    require(0 <= settling_frames < n and abs(settling_frames * dt - settling_s) < 1e-6, 'Invalid settling duration')
    require(neutral_frames == neutral['frames'] and neutral_frames > 0, 'Neutral NPZ/report frame count differs')
    paths = result['background_rigid_paths']
    require(bool(paths) and len(paths) == len(set(paths)), 'Empty or duplicate background body paths')
    require(background.shape == (n + 1, len(paths), 7), 'Background dimensions do not match body paths')
    linked = {neutral['articulation_root_aliases'].get(path, {}).get('native_path', path)
              for path in neutral['joint_connected_bodies']}
    joint_targets = neutral['joint_targets']
    dynamic_targets = {target.get('native_rigid_body', target['rigid_body']) for target in joint_targets
                       if target['kind'] == 'dynamic_rigid_body'}
    alias_valid = all(alias['native_path'] in neutral['background_paths'] and
                      alias['unique_joint_graph_root'] == [authored]
                      for authored, alias in neutral['articulation_root_aliases'].items())
    tolerance = neutral['background_tolerances']
    translation_limit, rotation_limit = float(tolerance['translation_m']), float(tolerance['rotation_deg'])
    require(np.isfinite([translation_limit, rotation_limit]).all() and min(translation_limit, rotation_limit) > 0,
            'Invalid predeclared background tolerances')
    body_background, body_time, background_files, background_reference = background_samples(
        episode, result, background, dt, settling_s)
    body_rows = []
    for i, path in enumerate(paths):
        start = 0 if path in linked else int(np.searchsorted(body_time, settling_s-1e-8))
        body = body_background[start:, i]
        angle = np.rad2deg((Rotation.from_quat(body[:, 3:7]) * Rotation.from_quat(body[0, 3:7]).inv()).magnitude())
        translation = np.linalg.norm(body[:, :3] - body[0, :3], axis=1)
        body_rows.append(dict(path=path, joint_connected=path in linked, start_time_s=float(body_time[start]),
                              translation_m=float(translation.max()), rotation_deg=float(angle.max()),
                              passed=bool(translation.max() <= translation_limit and angle.max() <= rotation_limit)))
    profile_keys = ['recipe', 'source_sha256', 'recipe_sha256', 'overlay_sha256']
    profile_same = all(result['background_physics'][key] == neutral['background_physics'][key] for key in profile_keys)
    if neutral['raw_schema'] == 'full_post_reset_physics':
        configuration_keys = ['simulation_device', 'physics_threads', 'torch_threads', 'physics_dt', 'action_dt', 'episode_length_s']
        profile_same = profile_same and result['task_id'] == neutral['task_id'] and result['seed'] == neutral['seed'] and all(
            result['execution_configuration'][key] == neutral['execution_configuration'][key] for key in configuration_keys)
    force_matrix = np.asarray(result['final_destination_contact_force_matrix_w_n'], dtype=float)
    require(force_matrix.shape == (1, 1, 1, 3), 'Source predicate requires one sensed body and one filtered destination')
    force = force_matrix[0, 0, 0]
    require(np.isfinite(force).all(), 'Nonfinite destination support force')
    final_speed = float(np.linalg.norm(obj[-1, 7:10]))
    final_open_angle = float(q[-1, finger_index])
    stability_samples = int(np.ceil(limits.stability_window_s / dt)) + 1
    require(stability_samples <= len(obj), 'Insufficient recording for final stability window')
    drift = float(np.linalg.norm(np.ptp(obj[-stability_samples:, :3], axis=0)))
    followup_s = (n - int(indices['release'][-1]) - 1) * dt
    lift_height = float(obj[indices['lift'][-1] + 1, 2] - obj[0, 2])
    controls = result['effective_controls']
    checks = {
        'source_task_predicate_and_final_destination': all(result[key] is True for key in ['success', 'arena_success', 'final_on_destination']),
        'source_relations': relation['passed'] is True,
        'no_physical_failure_reasons': result['physical_failure_reasons'] == [],
        'neutral_probe_passed': neutral['passed'] is True,
        'same_background_physics_as_neutral': profile_same,
        'neutral_duration_covers_episode': max(n * dt, float(body_time[-1])) <= neutral['duration_seconds'] + 1e-6,
        'same_initial_wait': abs(controls['initial_settling_seconds'] - settling_s) < 1e-6 and controls['initial_settling_actions'] == settling_frames and bool(np.all(phases[:settling_frames] == 'settle')),
        'background_paths_complete': set(paths) == set(neutral['background_paths']) and bool(linked) and linked <= set(paths),
        'joint_target_native_mapping_complete': bool(dynamic_targets) and linked == dynamic_targets and alias_valid,
        'all_background_stable': all(row['passed'] for row in body_rows),
        'bilateral_body_load_before_lift': first is not None and first['action'] < indices['lift'][0],
        'carried_contact_records_complete': {row['action'] for row in carried} == expected_carried,
        'bilateral_body_load_through_recorded_carry': bool(carried) and all(row['bilateral_loaded_body_region'] for row in carried),
        'object_actually_lifted': lift_height >= limits.lift_height_m,
        'gripper_released': abs(final_open_angle) < limits.open_finger_rad,
        'destination_support_force': float(np.linalg.norm(force)) > limits.support_force_n,
        'original_support_force_magnitude': float(np.linalg.norm(force)) >= source.force_threshold,
        'original_upward_support_cone': force[2] > 0 and force[2] >= np.linalg.norm(force) * math.cos(source.support_cone_half_angle_rad),
        'original_object_speed': final_speed < source.velocity_threshold,
        'release_followup_covers_stability_window': followup_s >= limits.stability_window_s,
        'released_object_still': final_speed < limits.object_speed_m_s and drift <= limits.object_drift_m,
    }
    checks = {key: bool(value) for key, value in checks.items()}
    paths_to_hash = [episode / name for name in ['result.json', 'trajectory.h5', 'grasp_contacts.json', 'source_relation_audit.json']]
    paths_to_hash += [neutral_dir / name for name in ['neutral_probe.json', 'neutral_states.npz']]
    paths_to_hash += background_files
    return dict(schema_version=1, passed=all(checks.values()), checks=checks, episode=str(episode), neutral=str(neutral_dir),
                scope='Independent raw contact/state review for mustard-to-bowl; complete reset-based physical background when available, otherwise explicitly labelled legacy action-rate reference. No visual or multi-seed certification.',
                frames=n, dt=dt, duration_s=n * dt, neutral_duration_s=neutral['duration_seconds'],
                neutral_raw_schema=neutral['raw_schema'],
                background_reference=background_reference, background_physics_samples=len(body_background),
                background_observation_duration_s=float(body_time[-1]),
                initial_settling_s=settling_s, thresholds=asdict(limits), background_tolerances=tolerance,
                original_predicate_thresholds=asdict(source),
                source_contact_z_range_m=[low, high], first_bilateral_body_loaded_contact=first,
                first_lift_action=int(indices['lift'][0]), lift_height_m=lift_height,
                final_gripper_joint_rad=final_open_angle, final_object_linear_speed_m_s=final_speed,
                final_destination_contact_force_n=force.tolist(), final_stability_window_position_diameter_bound_m=drift,
                release_followup_seconds=followup_s, background_bodies=body_rows,
                loaded_actions_by_phase={phase: dict(recorded=sum(row['phase'] == phase for row in loaded),
                    bilateral=sum(row['phase'] == phase and row['bilateral_loaded'] for row in loaded),
                    bilateral_body_region=sum(row['phase'] == phase and row['bilateral_loaded_body_region'] for row in loaded))
                    for phase in phase_order}, background_physics=result['background_physics'],
                input_sha256={str(path): sha(path) for path in paths_to_hash},
                review_code_sha256={str(path): sha(path) for path in [Path(__file__),
                    Path(__file__).with_name('review_neutral_evidence.py'), Path(__file__).with_name('neutral_task_probe.py')]},
                visual_review='Pending separate inspection of original camera frames and videos.',
                limitations=['Action-rate contacts do not establish substep causal ordering.',
                    'Schema-2 neutral physics, topology and source relations are independently recomputed; legacy neutral support remains historical only. Visual inspection is separate.',
                    'Original speed and upward support-force components are independently recomputed from raw final state; source spatial-containment and predicate-sequence flags still come from the simulator and need their separate checks.'])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episode', type=Path)
    parser.add_argument('--neutral', type=Path, required=True, help='Directory with this run\'s neutral_probe.json and neutral_states.npz')
    parser.add_argument('--output', type=Path, required=True, help='New JSON file; existing evidence is never overwritten')
    parser.add_argument('--bottle-usd', type=Path, required=True, help='Original bottle asset used to independently derive the contact region')
    parser.add_argument('--source-task', type=Path, default=SOURCE_TASK, help='Pinned upstream PickAndPlaceTask source; reads its original thresholds without Kit')
    for name, default in asdict(Limits()).items():
        parser.add_argument('--' + name.replace('_', '-'), type=float, default=default)
    args = parser.parse_args(argv)
    if args.output.exists():
        raise FileExistsError(args.output)
    try:
        from kitchen_safety.grasp_geometry import bottle_body_geometry
        geometry = bottle_body_geometry(args.bottle_usd)
        limits = Limits(**{name: getattr(args, name) for name in asdict(Limits())})
        report = review(args.episode, args.neutral, geometry['safe_contact_z_range_m'], limits, source_thresholds(args.source_task))
        report['source_bottle'] = dict(path=str(args.bottle_usd), sha256=sha(args.bottle_usd),
                                     contact_band=geometry['safe_contact_z_range_m'])
        report['predicate_source_sha256'] = {str(path): sha(path) for path in [args.source_task, SOURCE_SPATIAL]}
    except (OSError, ValueError, KeyError, TypeError, IndexError, StopIteration, SyntaxError) as error:
        report = dict(schema_version=1, passed=False, episode=str(args.episode), neutral=str(args.neutral),
                      checks={'required_evidence_readable_and_valid': False}, error=f'{type(error).__name__}: {error}')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({key: value for key, value in report.items() if key not in ['background_bodies', 'input_sha256']}, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
