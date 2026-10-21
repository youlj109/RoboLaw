"""Check state/action alignment and fully decode all recorded RGB and previews."""
import argparse
import io
import json
from pathlib import Path
import sys
import time

import h5py
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from kitchen_safety.dataset import (CAMERA_NAMES, RGB_ALIGNMENT, camera_pose_errors,
                                   camera_poses_from_body_states, decode_video)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def require_visible_motion(result, observer_recorded, first_q, middle_q, first_bodies, middle_bodies):
    # A failed attempt may stop in initial settle. Requiring moving pixels for
    # physically stationary evidence rejects valid negative data. Moving episodes
    # and all legacy/successful episodes retain the original stale-image check.
    stationary = (np.max(np.abs(np.asarray(first_q)-np.asarray(middle_q))) <= .001 and
                  np.max(np.abs(np.asarray(first_bodies)[..., :3]-np.asarray(middle_bodies)[..., :3])) <= .001)
    return not (observer_recorded and result.get('success') is False and stationary)


def attach_safety_validation(item, root, image_root, require_rgb):
    """Retain successful legacy checks even when the new evidence fails."""
    from kitchen_safety.risk_reporting import validate_observer
    try:
        item['safety_extension'] = validate_observer(root, render_root=image_root)
        require(item['safety_extension']['passed'], 'Safety extension invalid: '+str(item['safety_extension']['errors']))
        if require_rgb:
            require(item['safety_extension']['camera']['complete'], 'Missing safety camera provenance')
    except Exception as error:
        item.setdefault('safety_extension', {'passed': False, 'errors': [repr(error)]})
        item.update(passed=False, error=f'{type(error).__name__}: {error}')
        return False
    return True


def validate_episode(root, require_rgb=False, require_previews=False, *, render_root=None):
    root = Path(root)
    image_root = Path(render_root) if render_root is not None else root
    started = time.perf_counter()
    result = json.loads((root / 'result.json').read_text())
    item = {'path': str(root), 'success': result['success'], 'passed': False}
    item.update(collection_purpose=result.get('collection_purpose', 'diagnostic'),
                collection_source=result.get('collection_source'), collection_job=result.get('collection_job'))
    with h5py.File(root / 'trajectory.h5', 'r') as trajectory:
        n = len(trajectory['action'])
        item['frames'] = n
        require(n > 0 and n == result['steps'], 'Trajectory must have a nonzero count matching result.steps')
        require(trajectory['action'].shape == (n, 8), 'Action shape must be (frames, 8)')
        require(np.isfinite(trajectory['action'][:]).all(), 'Nonfinite action')
        dt = float(trajectory.attrs['dt'])
        require(np.isfinite(dt) and dt > 0, 'Trajectory dt must be positive and finite')
        require(len(trajectory['phase']) == n, 'Phase and action counts differ')
        np.testing.assert_allclose(trajectory['time'][:], np.arange(n) * dt, rtol=0, atol=1e-6)
        require(set(trajectory['obs']) == set(trajectory['next_obs']), 'obs/next_obs fields differ')
        require(bool(trajectory['obs'].keys()), 'No observations recorded')
        for key in trajectory['obs']:
            obs, next_obs = trajectory['obs'][key][:], trajectory['next_obs'][key][:]
            require(len(obs) == n and next_obs.shape == obs.shape, f'{key}: observation/action counts differ')
            require(np.isfinite(obs).all() and np.isfinite(next_obs).all(), f'{key}: nonfinite observation')
            np.testing.assert_allclose(next_obs[:-1], obs[1:], rtol=0, atol=1e-6)
        item['state_chain_continuous'] = True

    if render_root is not None:
        rendered = json.loads((image_root / 'render_report.json').read_text())
        require(Path(rendered['source_episode']).resolve() == root.resolve(),
                'Rendered images reference a different source episode')
        require(rendered.get('completed') is True and rendered.get('complete_trajectory') is True,
                'Offline rendering must have completed the entire measured trajectory')
        require(rendered.get('source_obs_indices') == list(range(n)),
                'Offline RGB must retain every source obs index in action order')
        # Physical success belongs to the source. Only the independently
        # measured rendering preparation belongs to this image output.
        result = {**result, 'appearance_mode': 'randomized_episode',
                  'appearance_warmup': rendered.get('appearance_warmup', {}),
                  'production_quota_eligible': False}
        item.update(render_output=str(image_root), production_quota_eligible=False,
                    render_collection_purpose=rendered.get('collection_purpose', 'diagnostic'),
                    render_collection_source=rendered.get('collection_source'),
                    acceptance_authority='verify_attempt; this result validates data, not production quota')
        require_rgb = True

    has_images = (image_root / 'images.h5').is_file()
    if require_rgb or require_previews:
        require(has_images, 'Required images.h5 is missing')
    if has_images:
        if result.get('appearance_mode') == 'authored_fixed_benchmark':
            require(result.get('production_quota_eligible') is False,
                    'Fixed-appearance throughput benchmarks cannot contribute randomized production quota')
            warmup = result.get('appearance_warmup', {})
            require(warmup.get('physics_advanced') is False and
                    warmup.get('convergence_checked') is False and warmup.get('passed') is None,
                    'Fixed-appearance benchmark must disclose unmeasured convergence without advancing physics')
            item['appearance_validation_scope'] = (
                'Authored fixed USD appearance performance experiment. Randomized appearance convergence '
                'was not measured; RGB structure and camera alignment are checked below.')
            item['production_quota_eligible'] = False
        elif result.get('episode_spec') is not None or (root / 'episode_spec.json').is_file():
            warmup = result.get('appearance_warmup', {})
            if warmup.get('mode') == 'fixed_preparation':
                require(result.get('production_quota_eligible') is False,
                        'Experimental fixed preparation cannot contribute production quota')
                require(warmup.get('preparation_passed') is True and
                        warmup.get('convergence_required') is False and
                        warmup.get('convergence_checked') is True and
                        type(warmup.get('passed')) is bool,
                        'Fixed preparation must retain the measured pixel convergence result separately')
                require(warmup.get('physics_advanced') is False and
                        isinstance(warmup.get('physics_step_count_before'), int) and
                        warmup.get('physics_step_count_before') == warmup.get('physics_step_count_after'),
                        'Fixed preparation must not advance physics')
                require(warmup.get('render_frames') == warmup.get('requested_render_frames') == 128 and
                        isinstance(warmup.get('render_generation_before'), int) and
                        isinstance(warmup.get('render_generation_after'), int) and
                        warmup['render_generation_after'] - warmup['render_generation_before'] >= 128,
                        'Fixed preparation requires 128 fresh render generations')
                item['appearance_validation_scope'] = (
                    'Randomized appearance with 128 pure-render preparation frames. Pixel convergence '
                    'is retained as a diagnostic and is not required; all RGB/state checks below remain active.')
                item['production_quota_eligible'] = False
            else:
                require(warmup.get('passed') is True and warmup.get('physics_advanced') is False,
                        'Randomized RGB requires appearance_warmup.passed=True without advancing physics')
            item['appearance_warmup'] = {
                key: warmup[key] for key in ('passed', 'physics_advanced', 'render_frames', 'elapsed_seconds',
                    'mode', 'preparation_passed', 'convergence_required', 'convergence_checked',
                    'requested_render_frames', 'physics_step_count_before', 'physics_step_count_after',
                    'render_generation_before', 'render_generation_after')
                if key in warmup}
        metadata = json.loads((image_root / 'cameras.json').read_text())
        require(set(metadata) == set(CAMERA_NAMES), f'cameras.json must contain {CAMERA_NAMES}')
        with h5py.File(root / 'trajectory.h5', 'r') as trajectory:
            require('robot_body_pose_w' in trajectory['obs'], 'Robot body poses are required to validate moving-camera extrinsics')
            visible_motion_required = require_visible_motion(result, (root/'safety_observer_binding.json').exists(),
                trajectory['obs/q'][0], trajectory['obs/q'][n//2],
                trajectory['obs/robot_body_pose_w'][0], trajectory['obs/robot_body_pose_w'][n//2])
            expected_poses = camera_poses_from_body_states(
                trajectory['obs/robot_body_pose_w'][:],
                json.loads(trajectory.attrs['robot_body_names']), metadata)
        with h5py.File(image_root / 'images.h5', 'r') as images:
            require(set(images) == set(CAMERA_NAMES), f'images.h5 must contain {CAMERA_NAMES}')
            require(images.attrs.get('frames') == n, 'images.h5 frame attribute disagrees with actions')
            require(np.isclose(images.attrs['dt'], dt, rtol=0, atol=1e-10), 'Image/action dt differs')
            require(images.attrs.get('alignment') == RGB_ALIGNMENT, 'Missing or unsupported RGB/action alignment')
            item['cameras'] = {}
            for name in CAMERA_NAMES:
                camera = metadata[name]
                require((camera['width'], camera['height']) == (1280, 720), f'{name}: expected original 1280x720 resolution')
                intrinsics = np.asarray(camera['intrinsics'])
                require(intrinsics.shape == (3, 3) and np.isfinite(intrinsics).all(), f'{name}: invalid intrinsics')
                group = images[name]
                dataset = group['jpeg']
                require(len(dataset) == n, f'{name}: RGB/action counts differ')
                for key, width in [('position_world', 3), ('quaternion_world_ros_xyzw', 4)]:
                    poses = group[key][:]
                    require(poses.shape == (n, width) and np.isfinite(poses).all(), f'{name}: invalid {key}')
                pose_errors = camera_pose_errors(group['position_world'][:],
                                                 group['quaternion_world_ros_xyzw'][:],
                                                 expected_poses[name])
                # Fixed mount quaternions were authored to three decimal places;
                # live USD poses differ by ~0.02 degrees from normalized config.
                # Millimetre/0.1-degree agreement catches stale wrist poses while
                # allowing that known serialization/normalization discrepancy.
                require(pose_errors['position_max_error_m'] < .001 and
                        pose_errors['rotation_max_error_deg'] < .1,
                        f'{name}: camera world pose disagrees with obs[t] parent body: {pose_errors}')
                first = middle = None
                for index in range(n):
                    with Image.open(io.BytesIO(dataset[index].tobytes())) as image:
                        image.load()
                        require(image.mode == 'RGB' and image.size == (1280, 720), f'{name} frame {index}: unexpected RGB dimensions')
                        if index == 0:
                            first = np.asarray(image).copy()
                        if index == n // 2:
                            middle = np.asarray(image).copy()
                delta = float(np.abs(first.astype(float) - middle.astype(float)).mean())
                if visible_motion_required:
                    require(delta > 5, f'{name}: no visible change during robot motion')
                item['cameras'][name] = {'frames': n, 'fully_decoded_frames': n,
                                         'mean_pixel_change_first_to_middle': delta,
                                         'visible_motion_required': visible_motion_required,
                                         'stationary_negative_evidence': not visible_motion_required,
                                         'world_pose_source': group.attrs.get('world_pose_source', 'legacy Camera.data'),
                                         'pose_error_from_obs_parent': pose_errors}
        item['rgb_action_alignment'] = RGB_ALIGNMENT

    expected_previews = {f'{name}.mp4': (1280, 720) for name in CAMERA_NAMES}
    expected_previews['all_cameras.mp4'] = (1920, 360)
    if require_previews:
        for filename in expected_previews:
            require((image_root / filename).is_file(), f'Required preview missing: {filename}')
    item['video_decoded_frames'] = {}
    for video in sorted(image_root.glob('*.mp4')):
        item['video_decoded_frames'][video.name] = decode_video(
            video, n, expected_previews.get(video.name), dt)
    item['legacy_passed'] = True
    if (root / 'safety_observer_binding.json').exists():
        if not attach_safety_validation(item, root, image_root, require_rgb):
            item['elapsed_seconds'] = round(time.perf_counter() - started, 3)
            return item
    item.update(passed=True, elapsed_seconds=round(time.perf_counter() - started, 3))
    return item


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episodes', nargs='+')
    parser.add_argument('--report', default='reports/collection_validation.json')
    parser.add_argument('--require-rgb', action='store_true', help='Require all three original RGB streams')
    parser.add_argument('--require-previews', action='store_true', help='Require RGB and all four MP4 previews')
    parser.add_argument('--render-output', type=Path,
                        help='Separate offline RGB directory for exactly one source episode')
    args = parser.parse_args()
    if args.render_output is not None and len(args.episodes) != 1:
        parser.error('--render-output requires exactly one source episode')
    results = []
    for episode in args.episodes:
        started = time.perf_counter()
        try:
            item = validate_episode(episode, args.require_rgb, args.require_previews,
                                    render_root=args.render_output)
        except Exception as error:
            item = {'path': episode, 'passed': False, 'legacy_passed': False,
                    'error': f'{type(error).__name__}: {error}',
                    'elapsed_seconds': round(time.perf_counter() - started, 3)}
        results.append(item)
    report = Path(args.report)
    report.parent.mkdir(exist_ok=True, parents=True)
    report.write_text(json.dumps(results, indent=2) + '\n')
    print(json.dumps(results, indent=2), flush=True)
    return 0 if all(item['passed'] for item in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
