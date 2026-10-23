"""Render two measured episodes at original speed, without phase synchronization.

The output is an explanatory video, not an additional physical execution or a
visual-acceptance record. Both original videos and every other input are kept.
"""
import argparse
import json
import subprocess
import tempfile
from pathlib import Path

import h5py
import imageio_ffmpeg
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from kitchen_safety.dataset import close_encoders, decode_video, video_encoder
from scripts.extensions.replicator.review_episode import ROOT, read, require, sha

CAMERAS = ('external_camera', 'external_camera_2', 'wrist_camera', 'overview_camera')


def resolve_recorded_path(path):
    path = Path(path)
    if str(path).startswith('/workspace/'):
        return ROOT / path.relative_to('/workspace')
    return path if path.is_absolute() else ROOT / path


def recording(episode, camera):
    episode = Path(episode)
    require(camera in CAMERAS, 'Unknown recorded camera')
    result_path = episode / 'result.json'
    result = read(result_path)
    require(all(result[key] is True for key in ['success', 'arena_success', 'final_on_destination']),
            'Comparison requires a successful physical source task')
    camera_root = episode / 'overview' if camera == 'overview_camera' else episode
    meta_path, images_path = camera_root / 'cameras.json', camera_root / 'images.h5'
    camera_meta = read(meta_path)[camera]
    trajectory_path = episode / 'trajectory.h5'
    with h5py.File(trajectory_path) as trajectory:
        count, dt = len(trajectory['action']), float(trajectory.attrs['dt'])
        times, phases = trajectory['time'][:], trajectory['phase'].asstr()[:]
    require(count > 0 and np.isfinite(dt) and dt > 0, 'Empty recording or invalid dt')
    require(times.shape == (count,) and len(phases) == count and np.isfinite(times).all(), 'Invalid recorded timestamps/phases')
    require(np.allclose(times, np.arange(count) * dt, atol=1e-6, rtol=0),
            'Recording is not a contiguous original-rate timeline from t=0')
    with h5py.File(images_path) as images:
        require(int(images.attrs['frames']) == count and abs(float(images.attrs['dt']) - dt) < 1e-9,
                'Image and action count/rate mismatch')
        stream = images[camera]
        require(len(stream['jpeg']) == count and stream['position_world'].shape == (count, 3) and
                stream['quaternion_world_ros_xyzw'].shape == (count, 4), 'Incomplete selected-camera measurements')
        poses = np.concatenate([stream['position_world'][:], stream['quaternion_world_ros_xyzw'][:]], axis=1)
        require(np.isfinite(poses).all(), 'Nonfinite selected-camera poses')
    video = episode / (camera + '.mp4')
    size = (int(camera_meta['width']), int(camera_meta['height']))
    require(all(value > 0 for value in size), 'Invalid camera dimensions')
    decoded = decode_video(video, count, expected_size=size, expected_dt=dt)
    return dict(episode=episode, result=result, camera=camera, camera_meta=camera_meta, count=count, dt=dt,
                times=times, phases=phases, video=video, size=size, decoded=decoded,
                input_paths=[result_path, trajectory_path, meta_path, images_path, video])


def prepare(baseline, episode, camera):
    baseline, episode = Path(baseline).resolve(), Path(episode).resolve()
    require(baseline != episode, 'Baseline and augmented episode must be distinct')
    audit_path = episode / 'avoidance_audit.json'
    audit = read(audit_path)
    require(audit.get('accepted') is True and bool(audit.get('gates')) and
            all(value is True for value in audit['gates'].values()), 'Accepted avoidance audit with all gates required')
    require(resolve_recorded_path(audit['baseline_episode']).resolve() == baseline, 'Wrong baseline for this avoidance audit')
    for key, filename in [('result', 'result.json'), ('trajectory', 'trajectory.h5'), ('physics_states', 'physics_states.npz')]:
        require(sha(baseline / filename) == audit['baseline_sha256'][key], 'Baseline artifact hash mismatch: ' + filename)
    require(audit.get('same_source_layout') is True and audit.get('same_effective_controls') is True,
            'Comparison requires matched source layout and effective controls')
    left, right = recording(baseline, camera), recording(episode, camera)
    require(left['result']['variant'] == 'base' and right['result']['variant'] in ['human', 'property', 'robot'],
            'Expected a true no-hazard baseline and one augmented execution')
    require(left['camera_meta'] == right['camera_meta'], 'Selected camera calibration/mount metadata differs')
    require(abs(left['dt'] - right['dt']) < 1e-9, 'Different recorded dt; refusing rate conversion')
    require(left['result']['task_id'] == right['result']['task_id'], 'Source task IDs differ')
    return left, right, audit_path


def sample_index(output_index, count):
    """No time warping: after one recording ends, explicitly hold its last frame."""
    require(count > 0 and output_index >= 0, 'Invalid frame request')
    return min(output_index, count - 1), output_index >= count


def font(size):
    try:
        return ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', size)
    except OSError:
        return ImageFont.load_default()


def frame_reader(path, size):
    """Stream exact RGB frames and close ffmpeg pipes even on early rejection."""
    with tempfile.TemporaryFile() as errors:
        process = subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(), '-nostdin', '-v', 'error',
            '-i', str(path), '-map', '0:v:0', '-an', '-vsync', '0', '-f', 'rawvideo',
            '-pix_fmt', 'rgb24', 'pipe:1'], stdout=subprocess.PIPE, stderr=errors)
        try:
            count = size[0] * size[1] * 3
            while True:
                frame = process.stdout.read(count)
                if not frame:
                    break
                require(len(frame) == count, 'Partial RGB frame from decoder')
                yield frame
            code = process.wait(timeout=30)
            errors.seek(0)
            require(code == 0, 'Decoder failed: ' + errors.read().decode(errors='replace'))
        finally:
            process.stdout.close()
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()


def render(baseline, episode, camera, output):
    output = Path(output)
    validation_path = output.with_suffix('.validation.json')
    if output.exists() or validation_path.exists():
        raise FileExistsError('Preserve existing video/report: ' + str(output))
    require(output.suffix.lower() == '.mp4', 'Output must be an MP4 path')
    left, right, audit_path = prepare(baseline, episode, camera)
    recordings, dt = [left, right], left['dt']
    total = max(left['count'], right['count'])
    inputs = sorted(set(left['input_paths'] + right['input_paths'] +
                        [audit_path, left['episode'] / 'physics_states.npz']))
    input_hashes = {str(path): sha(path) for path in inputs}
    output.parent.mkdir(parents=True, exist_ok=True)
    # Reserve the destination before ffmpeg, whose encoder otherwise uses -y.
    # Only this invocation's newly created file can be overwritten by ffmpeg.
    with output.open('xb'):
        pass
    report = dict(schema_version=1, passed=False, output=str(output), camera=camera,
                  baseline=str(left['episode']), episode=str(right['episode']),
                  input_sha256=input_hashes, visual_review='Pending separate inspection; rendering/decoding is not visual acceptance.',
                  timing='Both recordings start at their own t=0. Same original dt; no resampling, phase matching or task-event synchronization. Shorter stream holds its final frame with an explicit label.',
                  scope='Real no-hazard baseline vs real augmented execution. Hazardous counterfactual is geometry-only, never physically replayed.')
    readers, encoder = [], None
    try:
        for record in recordings:
            reader = frame_reader(record['video'], record['size'])
            readers.append(reader)
        encoder = video_encoder(output, 1280, 536, dt)
        last_frames = [None, None]
        title, label = font(18), font(15)
        for frame in range(total):
            panel = Image.new('RGB', (1280, 536), '#152133')
            draw = ImageDraw.Draw(panel)
            draw.text((14, 9), 'BASELINE: real recording without added hazard', font=title, fill='white')
            draw.text((654, 9), 'REPLANNED: real recording with added hazard', font=title, fill='white')
            for side, record in enumerate(recordings):
                index, held = sample_index(frame, record['count'])
                if not held:
                    last_frames[side] = Image.frombytes('RGB', record['size'], next(readers[side]))
                pic = last_frames[side].copy()
                pic.thumbnail((640, 360), Image.Resampling.LANCZOS)
                panel.paste(pic, (side * 640 + (640 - pic.width) // 2, 44 + (360 - pic.height) // 2))
                draw.text((side * 640 + 14, 414),
                          f"elapsed={record['times'][index]:.2f}s | frame={index} | phase={record['phases'][index]}",
                          font=label, fill='white')
                draw.text((side * 640 + 14, 438), 'FINISHED: final frame held' if held else 'Original recording speed',
                          font=label, fill='#ffd477' if held else '#d8e3f0')
            draw.text((14, 466), f'Camera: {camera}. Same calibration. Wrist-camera viewpoints move with each robot trajectory.', font=label, fill='#d8e3f0')
            draw.text((14, 489), 'Elapsed time starts independently; task phases are NOT synchronized. No time scaling.', font=label, fill='#d8e3f0')
            draw.text((14, 512), 'Unsafe counterfactual is checked offline in geometry only; no collision was physically replayed.', font=label, fill='#d8e3f0')
            encoder.stdin.write(panel.tobytes())
        for reader in readers:
            require(next(reader, None) is None, 'Source video has unexpected extra decoded frames')
        process, encoder = encoder, None
        close_encoders({'comparison': process})
        decoded = decode_video(output, total, expected_size=(1280, 536), expected_dt=dt)
        require(all(sha(path) == digest for path, digest in input_hashes.items()), 'Source artifacts changed during rendering')
        report.update(passed=True, output_sha256=sha(output), frames=total, decoded_frames=decoded,
                      dt=dt, duration_s=total * dt, source_records=[dict(episode=str(r['episode']),
                          video=str(r['video']), frames=r['count'], decoded_frames=r['decoded'], dt=r['dt'],
                          final_recorded_time_s=float(r['times'][-1]), held_output_frames=total-r['count']) for r in recordings])
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
        if output.is_file():
            report['partial_output_sha256'] = sha(output)
    finally:
        for reader in readers:
            reader.close()
        if encoder is not None:
            try:
                close_encoders({'comparison': encoder})
            except Exception as error:
                report['encoder_shutdown_error'] = str(error)
        with validation_path.open('x') as stream:
            json.dump(report, stream, indent=2, allow_nan=False)
            stream.write('\n')
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--episode', type=Path, required=True)
    parser.add_argument('--camera', choices=CAMERAS, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    report = render(args.baseline, args.episode, args.camera, args.output)
    print(json.dumps({key: value for key, value in report.items() if key != 'input_sha256'}, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
