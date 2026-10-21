"""Incremental three-camera recording, indexed one RGB frame per action.

RGB recording and MP4 previews are separate outputs. The default still writes
the legacy four videos; ``previews=False`` records JPEGs and camera poses only.
Offline tools can import this module without loading Isaac Sim.
"""
import io
import json
import re
import subprocess
import time
from pathlib import Path

import h5py
import numpy as np
from PIL import Image, ImageDraw

CAMERA_NAMES = ('external_camera', 'external_camera_2', 'wrist_camera')
RGB_ALIGNMENT = 'RGB frame t is obs[t], before action[t].'


def numpy(value):
    return (value.torch if hasattr(value, 'torch') else value).detach().cpu().numpy().copy()


def camera_poses_from_body_states(body_poses, body_names, camera_metadata):
    """Compose recorded pre-action parent body poses with fixed camera mounts.

    Returns camera-to-world ROS poses. This is also an auditable repair path for
    historical records whose CameraCfg returned only the initialization pose.
    Camera mount quaternions in this Isaac Lab checkout use xyzw.
    """
    from scipy.spatial.transform import Rotation
    body_poses = np.asarray(body_poses)
    if body_poses.ndim != 3 or body_poses.shape[1:] != (len(body_names), 7):
        raise ValueError('Expected robot body world poses shaped (frames, body_names, 7)')
    if not np.isfinite(body_poses).all():
        raise ValueError('Nonfinite robot body pose')
    result = {}
    for name in CAMERA_NAMES:
        config = camera_metadata[name]
        parent_name = config['prim_path'].rsplit('/', 2)[-2]
        if body_names.count(parent_name) != 1:
            raise ValueError(f'{name}: camera parent {parent_name} is not a unique recorded robot body')
        parent = body_poses[:, body_names.index(parent_name)]
        rotation = Rotation.from_quat(parent[:, 3:7])
        offset = config['offset']
        position = parent[:, :3] + rotation.apply(offset['position'])
        local_rotation = Rotation.from_quat(offset['quaternion_xyzw'])
        convention = offset['convention']
        if convention == 'opengl':
            local_rotation = local_rotation * Rotation.from_quat([1, 0, 0, 0])
        elif convention == 'world':
            local_rotation = local_rotation * Rotation.from_matrix([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        elif convention != 'ros':
            raise ValueError(f'{name}: unsupported camera offset convention {convention}')
        result[name] = {'position_world': position,
                        'quaternion_world_ros_xyzw': (rotation * local_rotation).as_quat(),
                        'parent_body': parent_name}
    return result


def camera_pose_errors(recorded_position, recorded_quaternion, expected):
    from scipy.spatial.transform import Rotation
    recorded_position = np.asarray(recorded_position)
    recorded_quaternion = np.asarray(recorded_quaternion)
    position_error = np.linalg.norm(recorded_position - expected['position_world'], axis=-1)
    angle_error = (Rotation.from_quat(recorded_quaternion).inv() *
                   Rotation.from_quat(expected['quaternion_world_ros_xyzw'])).magnitude() * 180 / np.pi
    return {'position_max_error_m': float(position_error.max()),
            'rotation_max_error_deg': float(angle_error.max()),
            'recorded_position_motion_range_m': float(np.linalg.norm(np.ptp(recorded_position, axis=0))),
            'expected_position_motion_range_m': float(np.linalg.norm(np.ptp(expected['position_world'], axis=0)))}


def video_encoder(path, width, height, dt):
    """Encode exactly the supplied RGB frames at the simulation sampling rate."""
    import imageio_ffmpeg
    return subprocess.Popen([
        imageio_ffmpeg.get_ffmpeg_exe(), '-y', '-loglevel', 'error',
        '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{width}x{height}',
        '-r', str(1 / dt), '-i', '-', '-an', '-c:v', 'libx264',
        '-preset', 'fast', '-crf', '20', '-pix_fmt', 'yuv420p',
        '-movflags', '+faststart', str(path),
    ], stdin=subprocess.PIPE)


def preview_panel(image, name, phase):
    panel = image.resize((640, 360))
    draw = ImageDraw.Draw(panel)
    draw.rectangle((0, 0, 640, 26), fill='black')
    draw.text((8, 6), f'{name} | {phase}', fill='white')
    return np.asarray(panel)


def close_encoders(encoders):
    """Drain every encoder even if one failed, then surface any failure."""
    failures = []
    for name, process in encoders.items():
        try:
            process.stdin.close()
        except (BrokenPipeError, OSError) as error:
            failures.append(f'{name}: {error}')
    for name, process in encoders.items():
        code = process.wait()
        if code:
            failures.append(f'{name}: exit {code}')
    if failures:
        raise RuntimeError('Video encoding failed: ' + '; '.join(failures))


def decode_video(path, expected_frames, expected_size=None, expected_dt=None):
    """Fully decode a video; reject corruption, wrong counts, size or rate."""
    import imageio_ffmpeg
    path = Path(path)
    if not path.is_file():
        raise ValueError(f'Missing preview: {path}')
    result = subprocess.run([
        imageio_ffmpeg.get_ffmpeg_exe(), '-loglevel', 'level+info', '-nostats', '-xerror',
        '-err_detect', 'explode', '-i', str(path), '-map', '0:v:0', '-an',
        '-progress', 'pipe:1', '-vsync', '0', '-f', 'null', '-',
    ], capture_output=True, text=True)
    if result.returncode or re.search(r'\[(?:error|fatal|panic)\]', result.stderr):
        raise ValueError(f'{path}: full decode failed: {result.stderr.strip()}')
    # Use the input-stream description from the same full decode; no ffprobe
    # dependency or second decoder process is needed. FPS is rounded in headers.
    stream = next((line for line in result.stderr.splitlines()
                   if 'Stream #0:' in line and 'Video:' in line), '')
    dimensions = re.search(r', (\d+)x(\d+)(?:[ ,])', stream)
    rate = re.search(r', ([\d.]+) fps(?:[ ,]|$)', stream)
    size = tuple(int(value) for value in dimensions.groups()) if dimensions else None
    fps = float(rate.group(1)) if rate else None
    if expected_size is not None and size != tuple(expected_size):
        raise ValueError(f'{path}: dimensions {size} != {expected_size}')
    if expected_dt is not None and (fps is None or abs(fps - 1 / expected_dt) > .011):
        raise ValueError(f'{path}: FPS {fps} != {1 / expected_dt}')
    counts = re.findall(r'^frame=(\d+)', result.stdout, re.M)
    frames = int(counts[-1]) if counts else 0
    if frames != expected_frames:
        raise ValueError(f'{path}: decoded {frames} frames, expected {expected_frames}')
    return frames


class CameraWriter:
    def __init__(self, out, scene, names, dt, *, previews=True):
        self.out = Path(out)
        self.scene = scene
        self.names = tuple(names)
        self.dt = dt
        self.previews = previews
        # Camera.update(force_recompute=True) refreshes RGB but, by default,
        # CameraCfg still returns only its initialization pose. Explicitly enable
        # the native FrameView refresh for world poses of moving wrist cameras.
        for name in self.names:
            self.scene[name].cfg.update_latest_camera_pose = True
        self.encoders = {}
        self.datasets = {}
        self.file = h5py.File(self.out / 'images.h5', 'w')
        self.frames = 0
        self.last_phase = None
        self.metadata = {}
        self.closed = False
        self.timings = {'render_capture_s': 0., 'jpeg_write_s': 0.,
                        'preview_s': 0., 'close_s': 0., 'frames': 0,
                        'appearance_warmup_s': 0.}

    def encoder(self, name, width, height):
        return video_encoder(self.out / f'{name}.mp4', width, height, self.dt)

    def warm_up(self, sim, *, min_frames=128, max_frames=1024, sample_every=16,
                stable_windows=3, mean_difference_limit=.35, percentile95_limit=1.5):
        """Let fixed randomized appearance converge without advancing physics.

        The RTX helper deduplicates by physics step AND render generation. Calling
        it repeatedly alone would read the same old image; sim.render() must
        advance render_generation before each forced update. Sparse low-resolution
        samples compare several temporal windows so slow color drift cannot pass
        just because two immediately adjacent renders look similar.
        """
        from isaaclab_physx.renderers.isaac_rtx_renderer_utils import ensure_isaac_rtx_render_update
        if self.frames or self.closed:
            raise ValueError('Appearance warm-up must run before recording any action frames')
        if (min_frames < 1 or max_frames < min_frames or sample_every < 1 or stable_windows < 1 or
                max_frames < (stable_windows + 1) * sample_every or
                mean_difference_limit < 0 or percentile95_limit < 0):
            raise ValueError('Invalid appearance warm-up limits')
        started = time.perf_counter()
        before_physics = int(sim.get_physics_step_count())
        before_generation = int(sim.render_generation)
        report = {
            'passed': False, 'render_frames': 0, 'elapsed_seconds': 0.,
            'physics_advanced': False, 'physics_step_count_before': before_physics,
            'physics_step_count_after': before_physics,
            'render_generation_before': before_generation,
            'render_generation_after': before_generation,
            'settings': {'min_frames': min_frames, 'max_frames': max_frames,
                         'sample_every': sample_every, 'stable_windows': stable_windows,
                         'sample_size': [160, 90], 'mean_difference_limit': mean_difference_limit,
                         'percentile95_limit': percentile95_limit,
                         'units': 'Absolute RGB intensity difference on 0..255 scale'},
            'samples': [], 'error': None,
        }
        history = []
        consecutive = 0
        try:
            for frame in range(1, max_frames + 1):
                previous_generation = int(sim.render_generation)
                sim.render()
                if int(sim.render_generation) <= previous_generation:
                    raise RuntimeError('sim.render() did not advance render generation; RTX would reuse its previous image')
                ensure_isaac_rtx_render_update(force=True)
                report['render_frames'] = frame
                if int(sim.get_physics_step_count()) != before_physics:
                    raise RuntimeError('Physics advanced during appearance-only warm-up')
                if frame % sample_every:
                    continue
                snapshot = {}
                for name in self.names:
                    camera = self.scene[name]
                    camera.update(0., force_recompute=True)
                    rgb = numpy(camera.data.output['rgb'])[0, :, :, :3].astype(np.uint8)
                    snapshot[name] = np.asarray(Image.fromarray(rgb).resize((160, 90), Image.Resampling.BOX), dtype=np.float32)
                sample = {'render_frame': frame, 'windows': [], 'stable': False}
                for lag in range(1, min(stable_windows, len(history)) + 1):
                    metrics = {}
                    for name in self.names:
                        difference = np.abs(snapshot[name] - history[-lag][name])
                        metrics[name] = {'mean_absolute_rgb_difference': float(difference.mean()),
                                         'percentile95_absolute_rgb_difference': float(np.percentile(difference, 95))}
                    sample['windows'].append({'lag_render_frames': lag * sample_every, 'cameras': metrics})
                if len(sample['windows']) == stable_windows:
                    sample['stable'] = all(
                        value['mean_absolute_rgb_difference'] <= mean_difference_limit and
                        value['percentile95_absolute_rgb_difference'] <= percentile95_limit
                        for window in sample['windows'] for value in window['cameras'].values())
                consecutive = consecutive + 1 if sample['stable'] else 0
                sample['consecutive_stable_checks'] = consecutive
                report['samples'].append(sample)
                history.append(snapshot)
                history = history[-stable_windows:]
                if frame >= min_frames and consecutive >= stable_windows:
                    report['passed'] = True
                    break
            if not report['passed']:
                report['error'] = 'Appearance did not converge within the maximum pure-render frame budget'
        except Exception as error:
            report['error'] = f'{type(error).__name__}: {error}'
        finally:
            report['physics_step_count_after'] = int(sim.get_physics_step_count())
            report['physics_advanced'] = report['physics_step_count_after'] != before_physics
            report['render_generation_after'] = int(sim.render_generation)
            if report['physics_advanced']:
                report['passed'] = False
            report['elapsed_seconds'] = time.perf_counter() - started
            self.timings['appearance_warmup_s'] += report['elapsed_seconds']
            # Warm-up does not write JPEGs, poses, actions or preview frames.
            self.file.attrs['appearance_warmup'] = json.dumps(report)
        return report

    def write(self, phase):
        from isaaclab_physx.renderers.isaac_rtx_renderer_utils import ensure_isaac_rtx_render_update
        started = time.perf_counter()
        ensure_isaac_rtx_render_update(force=True)
        self.timings['render_capture_s'] += time.perf_counter() - started
        panels = []
        for name in self.names:
            started = time.perf_counter()
            camera = self.scene[name]
            camera.update(0.0, force_recompute=True)
            rgb = numpy(camera.data.output['rgb'])[0, :, :, :3].astype(np.uint8)
            position = numpy(camera.data.pos_w)[0]
            quaternion = numpy(camera.data.quat_w_ros)[0]
            self.timings['render_capture_s'] += time.perf_counter() - started

            started = time.perf_counter()
            height, width = rgb.shape[:2]
            im = Image.fromarray(rgb)
            if name not in self.datasets:
                group = self.file.create_group(name)
                group.attrs['world_pose_source'] = 'IsaacLab Camera.data with update_latest_camera_pose=True'
                self.datasets[name] = group.create_dataset(
                    'jpeg', shape=(0,), maxshape=(None,),
                    dtype=h5py.vlen_dtype(np.dtype('uint8')))
                group.create_dataset('position_world', shape=(0, 3), maxshape=(None, 3), dtype='f4')
                group.create_dataset('quaternion_world_ros_xyzw', shape=(0, 4), maxshape=(None, 4), dtype='f4')
                self.metadata[name] = {
                    'prim_path': camera.cfg.prim_path, 'width': width, 'height': height,
                    'intrinsics': numpy(camera.data.intrinsic_matrices)[0].tolist(),
                    'offset': {'position': list(camera.cfg.offset.pos),
                               'quaternion_xyzw': list(camera.cfg.offset.rot),
                               'convention': camera.cfg.offset.convention},
                }
            elif (width, height) != (self.metadata[name]['width'], self.metadata[name]['height']):
                raise RuntimeError(f'{name}: camera resolution changed during collection')
            for key, value in [('position_world', position),
                               ('quaternion_world_ros_xyzw', quaternion)]:
                ds = self.file[name][key]
                ds.resize(self.frames + 1, axis=0)
                ds[self.frames] = value
            stream = io.BytesIO()
            im.save(stream, format='JPEG', quality=92)
            ds = self.datasets[name]
            ds.resize(self.frames + 1, axis=0)
            ds[self.frames] = np.frombuffer(stream.getvalue(), dtype=np.uint8)
            self.timings['jpeg_write_s'] += time.perf_counter() - started

            if self.previews:
                started = time.perf_counter()
                if name not in self.encoders:
                    self.encoders[name] = self.encoder(name, width, height)
                self.encoders[name].stdin.write(rgb.tobytes())
                if phase != self.last_phase:
                    im.save(self.out / f'{phase}_{name}.jpg')
                panels.append(preview_panel(im, name, phase))
                self.timings['preview_s'] += time.perf_counter() - started
        if self.previews:
            started = time.perf_counter()
            combined = np.concatenate(panels, axis=1)
            if 'all_cameras' not in self.encoders:
                self.encoders['all_cameras'] = self.encoder('all_cameras', combined.shape[1], combined.shape[0])
            self.encoders['all_cameras'].stdin.write(combined.tobytes())
            self.timings['preview_s'] += time.perf_counter() - started
        self.frames += 1
        self.timings['frames'] = self.frames
        self.last_phase = phase

    def close(self):
        if self.closed:
            return
        started = time.perf_counter()
        error = None
        try:
            close_encoders(self.encoders)
        except Exception as exception:
            error = exception
        self.file.attrs['frames'] = self.frames
        self.file.attrs['dt'] = self.dt
        self.file.attrs['jpeg_quality'] = 92
        self.file.attrs['alignment'] = RGB_ALIGNMENT
        self.file.attrs['camera_order'] = json.dumps(self.names)
        self.file.flush()
        (self.out / 'cameras.json').write_text(json.dumps(self.metadata, indent=2))
        self.timings['close_s'] = time.perf_counter() - started
        self.file.attrs['timings'] = json.dumps(self.timings)
        self.file.close()
        self.closed = True
        if error is not None:
            raise error
