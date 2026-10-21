"""One batched camera read per action tick, with an independent file per env.

Call ``write`` after taking obs[t] and before env.step(action[t]). Finished
environments can be omitted from ``phases`` without adding padding frames.
This module does not change sensor poses, intrinsics or rendering settings.
"""
import io
import json
import os
import sys
import time
from collections import deque
from collections.abc import Mapping
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy

import numpy as np
from PIL import Image

from kitchen_safety.dataset import CAMERA_NAMES, CameraWriter, numpy


def _ensure_render_update():
    from isaaclab_physx.renderers.isaac_rtx_renderer_utils import ensure_isaac_rtx_render_update
    ensure_isaac_rtx_render_update(force=True)


def _open_jpeg_buffer():
    """An anonymous Linux file lets Pillow release the GIL while encoding.

    Arena's Python build omits os.memfd_create, although the host libc provides
    it. Both paths create the same CLOEXEC anonymous memory file. Unsupported
    platforms keep the original BytesIO encoding, without a disk temporary.
    """
    if sys.platform != 'linux':
        return io.BytesIO(), 'bytesio'
    try:
        if hasattr(os, 'memfd_create'):
            fd = os.memfd_create('arena-rgb-jpeg', flags=getattr(os, 'MFD_CLOEXEC', 1))
        else:
            import ctypes
            create = ctypes.CDLL(None, use_errno=True).memfd_create
            create.argtypes = [ctypes.c_char_p, ctypes.c_uint]
            create.restype = ctypes.c_int
            fd = create(b'arena-rgb-jpeg', 1)  # Linux MFD_CLOEXEC
            if fd < 0:
                code = ctypes.get_errno()
                raise OSError(code, os.strerror(code))
    except (AttributeError, OSError):
        return io.BytesIO(), 'bytesio'
    try:
        return os.fdopen(fd, 'w+b', buffering=0), 'memfd'
    except BaseException:
        os.close(fd)
        raise


def _jpeg_bytes(rgb, stream=None):
    if stream is None:
        stream = io.BytesIO()
    else:
        stream.seek(0)
        stream.truncate(0)
    Image.fromarray(rgb).save(stream, format='JPEG', quality=92)
    stream.seek(0)
    return stream.read()


def _box_sample_batch(rgb):
    """Match PIL's uint8 BOX 1280x720 -> 160x90, reading only the small GPU result.

    PIL rounds each separable eight-pixel average to uint8. Retaining both
    rounding steps makes optional temporal averaging change only time sampling,
    not the spatial statistic or its original intensity thresholds.
    """
    import torch
    value = rgb.torch if hasattr(rgb, 'torch') else rgb
    if isinstance(value, torch.Tensor):
        if tuple(value.shape[1:]) != (720, 1280, 3) or value.dtype != torch.uint8:
            raise ValueError('Expected uint8 RGB camera batch at 1280x720')
        count = value.shape[0]
        horizontal = ((value.reshape(count, 720, 160, 8, 3).sum(dim=3, dtype=torch.int32) + 4) // 8)
        small = (horizontal.reshape(count, 90, 8, 160, 3).sum(dim=2, dtype=torch.int32) + 4) // 8
        return small.detach().cpu().numpy().astype(np.float32)
    value = value if isinstance(value, np.ndarray) else numpy(value)
    if tuple(value.shape[1:]) != (720, 1280, 3) or value.dtype != np.uint8:
        raise ValueError('Expected uint8 RGB camera batch at 1280x720')
    count = value.shape[0]
    horizontal = (value.reshape(count, 720, 160, 8, 3).sum(axis=3, dtype=np.uint16) + 4) // 8
    small = (horizontal.reshape(count, 90, 8, 160, 3).sum(axis=2, dtype=np.uint16) + 4) // 8
    return small.astype(np.float32)


class _EnvironmentWriter(CameraWriter):
    """Reuse CameraWriter's file lifecycle without its single-env capture."""

    def __init__(self, out, scene, names, dt, env_id, origin):
        super().__init__(out, scene, names, dt, previews=False)
        self.env_id = env_id
        self.origin = origin
        self.file.attrs['environment_id'] = env_id
        self.file.attrs['coordinate_frame'] = 'environment_local'
        self.file.attrs['environment_origin_world_m'] = origin
        self.file.attrs['position_world_field_semantics'] = (
            'Legacy field name: world translation minus environment_origin_world_m; '
            'axes and ROS quaternion retain world orientation.')
        self.file.attrs['render_capture_timing_scope'] = 'Shared batch; see vectorized writer timings.'
        # The optional writer thread must never query a live scene or sensor,
        # including on its first frame when camera metadata is constructed.
        self.camera_config = {}
        for name in self.names:
            cfg = scene[name].cfg
            self.camera_config[name] = {
                'prim_path': cfg.prim_path,
                'offset': {'position': list(cfg.offset.pos), 'quaternion_xyzw': list(cfg.offset.rot),
                           'convention': cfg.offset.convention}}

    def write_frame(self, capture, phase, *, jpeg_stream=None):
        import h5py
        started = time.perf_counter()
        for name in self.names:
            data = capture[name]
            rgb = data['rgb'][self.env_id]
            if name not in self.datasets:
                group = self.file.create_group(name)
                group.attrs['world_pose_source'] = (
                    'IsaacLab Camera.data with update_latest_camera_pose=True; '
                    'translation relative to environment_origin_world_m')
                group.attrs['coordinate_frame'] = 'environment_local'
                group.attrs['environment_origin_world_m'] = self.origin
                self.datasets[name] = group.create_dataset(
                    'jpeg', shape=(0,), maxshape=(None,),
                    dtype=h5py.vlen_dtype(np.dtype('uint8')))
                group.create_dataset('position_world', shape=(0, 3), maxshape=(None, 3), dtype='f4')
                group.create_dataset('quaternion_world_ros_xyzw', shape=(0, 4), maxshape=(None, 4), dtype='f4')
                self.metadata[name] = {
                    **deepcopy(self.camera_config[name]), 'width': rgb.shape[1], 'height': rgb.shape[0],
                    'intrinsics': data['intrinsics'][self.env_id].tolist(),
                    'environment_id': self.env_id, 'coordinate_frame': 'environment_local',
                    'environment_origin_world_m': self.origin.tolist(),
                }
            for key, value in (
                ('position_world', data['position'][self.env_id] - self.origin),
                ('quaternion_world_ros_xyzw', data['quaternion'][self.env_id]),
            ):
                ds = self.file[name][key]
                ds.resize(self.frames + 1, axis=0)
                ds[self.frames] = value
            encoded = _jpeg_bytes(rgb, jpeg_stream)
            ds = self.datasets[name]
            ds.resize(self.frames + 1, axis=0)
            ds[self.frames] = np.frombuffer(encoded, dtype=np.uint8)
        self.frames += 1
        self.last_phase = phase
        self.timings['frames'] = self.frames
        self.timings['jpeg_write_s'] += time.perf_counter() - started


class VectorizedCameraWriter:
    """Record the original three 720p cameras for multiple environments.

    ``outputs`` maps simulator environment indices to separate episode folders.
    ``env_origins`` has shape (scene.num_envs, 3), in world metres. Image poses
    use the same translated local frame as the collector's recorded body poses;
    the legacy dataset name ``position_world`` is retained and annotated.
    Capture timings belong to this batch writer, not each environment sink.
    ``background_write=True`` keeps capture on the caller thread, while one
    worker encodes/writes at most two outstanding immutable batches. ``frames``
    counts committed frames; ``submitted_frames`` counts accepted action images.
    close() drains all work before flushing files or reporting any worker error.
    """

    def __init__(self, outputs, scene, names=CAMERA_NAMES, dt=1 / 15, *, env_origins=None,
                 background_write=False):
        from pathlib import Path
        if not isinstance(outputs, Mapping) or not outputs:
            raise ValueError('outputs must map environment indices to episode directories')
        if tuple(names) != CAMERA_NAMES or not np.isclose(dt, 1 / 15, rtol=0, atol=1e-10):
            raise ValueError('Vectorized recording retains the three original cameras at 15 Hz')
        self.scene, self.names, self.dt = scene, tuple(names), dt
        self.num_envs = int(scene.num_envs)
        if any(type(index) is not int or not 0 <= index < self.num_envs for index in outputs):
            raise ValueError('outputs contains an invalid simulator environment index')
        paths = {index: Path(path) for index, path in outputs.items()}
        if len({path.resolve() for path in paths.values()}) != len(paths):
            raise ValueError('Each environment must have its own output directory')
        origins = np.zeros((self.num_envs, 3)) if env_origins is None else np.asarray(env_origins, dtype=float)
        if origins.shape != (self.num_envs, 3) or not np.isfinite(origins).all():
            raise ValueError('env_origins must contain one finite xyz origin per simulator environment')
        self.origins = origins.copy()
        for name in self.names:
            cfg = scene[name].cfg
            if (cfg.width, cfg.height) != (1280, 720):
                raise ValueError(f'{name}: recording requires the original 1280x720 resolution')
        self.writers = {}
        try:
            for index, path in paths.items():
                path.mkdir(parents=True, exist_ok=True)
                self.writers[index] = _EnvironmentWriter(path, scene, self.names, dt, index, self.origins[index])
        except Exception:
            for writer in self.writers.values():
                writer.close()
            raise
        self._intrinsics = {}
        self.closed = False
        self.background_write = bool(background_write)
        self._submitted_frames = {index: 0 for index in self.writers}
        self._pending = deque()
        self._executor = None
        self._write_failure = None
        self._jpeg_stream = None
        self.timings = {'render_capture_s': 0., 'jpeg_write_s': 0., 'capture_ticks': 0,
                        'appearance_warmup_s': 0., 'close_s': 0.,
                        'background_write': self.background_write, 'submit_wait_s': 0.,
                        'close_drain_s': 0., 'max_pending_batches': 0,
                        'jpeg_output_buffer': 'bytesio'}
        if self.background_write:
            try:
                self._jpeg_stream, self.timings['jpeg_output_buffer'] = _open_jpeg_buffer()
                self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='arena-rgb-writer')
                for writer in self.writers.values():
                    writer.file.attrs['background_write'] = True
                    writer.file.attrs['maximum_pending_batches'] = 2
                    writer.file.attrs['jpeg_output_buffer'] = self.timings['jpeg_output_buffer']
            except BaseException:
                self.close()
                raise

    @property
    def frames(self):
        return {index: writer.frames for index, writer in self.writers.items()}

    @property
    def submitted_frames(self):
        return self._submitted_frames.copy() if self.background_write else self.frames

    def _raise_write_failure(self):
        if self._write_failure is not None:
            raise RuntimeError(f'Background RGB write failed: {self._write_failure!r}') from self._write_failure

    def _reserve_pending_slot(self):
        """Check completed jobs and wait before capture, bounding all live batches.

        The two slots include the job currently running; an executor's otherwise
        unbounded internal queue never receives more than this amount of work.
        """
        self._raise_write_failure()
        started = time.perf_counter()
        try:
            while self._pending and self._pending[0].done():
                self._pending.popleft().result()
            if len(self._pending) == 2:
                self._pending.popleft().result()
            self._raise_write_failure()
        finally:
            self.timings['submit_wait_s'] += time.perf_counter() - started

    def _write_capture(self, capture, phases, indices):
        """CPU-only sink: no rendering, camera data or live scene access."""
        self._raise_write_failure()
        started = time.perf_counter()
        try:
            for index, phase in phases.items():
                writer = self.writers[index]
                if writer.frames != indices[index]:
                    raise ValueError('Committed RGB frame index differs from submitted action index')
                writer.write_frame(capture, phase, jpeg_stream=self._jpeg_stream)
        except BaseException as error:
            if self.background_write and self._write_failure is None:
                self._write_failure = error
            raise
        finally:
            # Worker service time overlaps the collection loop in background
            # mode; it must not be added to caller wall time as a separate cost.
            self.timings['jpeg_write_s'] += time.perf_counter() - started

    def _capture(self, *, ensure=True, poses=True):
        if ensure:
            _ensure_render_update()
        capture = {}
        for name in self.names:
            camera = self.scene[name]
            camera.update(0., force_recompute=True)
            data = camera.data
            # Exactly one full RGB batch device-to-host read per named camera.
            rgb = numpy(data.output['rgb'])
            if rgb.shape != (self.num_envs, 720, 1280, 3) or rgb.dtype != np.uint8:
                raise ValueError(f'{name}: expected uint8 RGB batch {(self.num_envs, 720, 1280, 3)}, got {rgb.shape}/{rgb.dtype}')
            item = {'rgb': rgb}
            if poses:
                item['position'] = numpy(data.pos_w)
                item['quaternion'] = numpy(data.quat_w_ros)
                if name not in self._intrinsics:
                    self._intrinsics[name] = numpy(data.intrinsic_matrices)
                item['intrinsics'] = self._intrinsics[name]
                for key, shape in (('position', (self.num_envs, 3)), ('quaternion', (self.num_envs, 4)),
                                   ('intrinsics', (self.num_envs, 3, 3))):
                    if item[key].shape != shape or not np.isfinite(item[key]).all():
                        raise ValueError(f'{name}: invalid batched camera {key}')
            capture[name] = item
        return capture

    def write(self, phases, *, action_indices=None):
        """Write obs[t] images before its action; omit completed environments."""
        if self.closed:
            raise ValueError('Camera writer is closed')
        self._raise_write_failure()
        if not isinstance(phases, Mapping) or any(index not in self.writers for index in phases):
            raise ValueError('phases must map recorded environment indices to their current phase')
        if action_indices is not None:
            if set(action_indices) != set(phases) or any(
                    action_indices[index] != self.submitted_frames[index] for index in phases):
                raise ValueError('Pre-action image index must match each environment action index')
        if not phases:
            return {}
        if self.background_write:
            self._reserve_pending_slot()
        started = time.perf_counter()
        capture = self._capture()
        self.timings['render_capture_s'] += time.perf_counter() - started
        self.timings['capture_ticks'] += 1
        indices = {index: self.submitted_frames[index] for index in phases}
        if self.background_write:
            # numpy() already copies RGB and poses away from simulator buffers.
            # Freeze those owned arrays in place; do not add another full copy.
            for data in capture.values():
                for array in data.values():
                    array.setflags(write=False)
            self._raise_write_failure()
            self._pending.append(self._executor.submit(self._write_capture, capture, dict(phases), dict(indices)))
            for index in phases:
                self._submitted_frames[index] += 1
            self.timings['max_pending_batches'] = max(self.timings['max_pending_batches'], len(self._pending))
        else:
            self._write_capture(capture, phases, indices)
        return indices

    def _warmup_snapshot(self, *, small_readback=False):
        if not small_readback:
            capture = self._capture(ensure=False, poses=False)
            return {(index, name): np.asarray(Image.fromarray(capture[name]['rgb'][index]).resize(
                (160, 90), Image.Resampling.BOX), dtype=np.float32)
                for index in self.writers for name in self.names}
        snapshot = {}
        for name in self.names:
            camera = self.scene[name]
            camera.update(0., force_recompute=True)
            small = _box_sample_batch(camera.data.output['rgb'])
            if small.shape != (self.num_envs, 90, 160, 3):
                raise ValueError(f'{name}: unexpected environment count during appearance warm-up')
            snapshot.update({(index, name): small[index] for index in self.writers})
        return snapshot

    def warm_up(self, sim, *, min_frames=128, max_frames=1024, sample_every=16,
                stable_windows=3, mean_difference_limit=.35, percentile95_limit=1.5,
                mode='single_frame'):
        """Require stable appearance in every recorded env without physics steps.

        Optional ``temporal_mean`` compares non-overlapping means of
        ``sample_every`` pure-render frames, at the original sample cadence.
        The averaged comparisons are diagnostic: the same lag comparisons must
        also pass on individual, unaveraged window-end frames. Each window's
        raw pixel range is checked at the same limits, so a transient inside a
        window cannot disappear in its mean or before its final frame.
        ``fixed_preparation`` performs exactly 128 pure renders and retains
        the original single-frame comparisons as diagnostics. Unchanged authored
        USD already failed these comparisons in appearance_control4_authored_raw;
        pixel fluctuations do not prove that sampled material/light inputs drift.
        Its preparation result is separate from the unchanged pixel ``passed``.
        Saved RGB/action images always come from an individual pre-action render.
        """
        if self.closed or any(self.submitted_frames.values()) or any(self.frames.values()):
            raise ValueError('Appearance warm-up must precede action frames')
        if mode not in ('single_frame', 'temporal_mean', 'fixed_preparation'):
            raise ValueError('Unknown appearance warm-up mode')
        fixed_preparation = mode == 'fixed_preparation'
        if fixed_preparation:
            min_frames = max_frames = 128
        if (min_frames < 1 or max_frames < min_frames or sample_every < 1 or stable_windows < 1 or
                max_frames < (stable_windows + 1) * sample_every or
                mean_difference_limit < 0 or percentile95_limit < 0):
            raise ValueError('Invalid appearance warm-up limits')
        started = time.perf_counter()
        physics_before = int(sim.get_physics_step_count())
        report = {
            'passed': False, 'render_frames': 0, 'elapsed_seconds': 0., 'physics_advanced': False,
            'physics_step_count_before': physics_before, 'physics_step_count_after': physics_before,
            'render_generation_before': int(sim.render_generation), 'render_generation_after': int(sim.render_generation),
            'environment_ids': list(self.writers), 'samples': [], 'error': None,
            'settings': {'min_frames': min_frames, 'max_frames': max_frames, 'sample_every': sample_every,
                         'stable_windows': stable_windows, 'sample_size': [160, 90],
                         'mode': mode, 'temporal_window_frames': sample_every if mode == 'temporal_mean' else 1,
                         'unaveraged_window_end_comparisons_required': mode == 'temporal_mean',
                         'unaveraged_window_range_required': mode == 'temporal_mean',
                         'mean_difference_limit': mean_difference_limit, 'percentile95_limit': percentile95_limit,
                         'units': 'Absolute RGB intensity difference on 0..255 scale'},
        }
        if fixed_preparation:
            report.update(mode='fixed_preparation', requested_render_frames=128,
                          preparation_passed=False, preparation_error=None,
                          convergence_checked=True, convergence_required=False)
        render_loop_completed = False
        history, raw_history, consecutive, accumulated = [], [], 0, {}
        window_minimum, window_maximum = {}, {}

        def compare_windows(snapshot, previous):
            windows = []
            for lag in range(1, min(stable_windows, len(previous)) + 1):
                metrics = {}
                for (index, name), value in snapshot.items():
                    difference = np.abs(value - previous[-lag][index, name])
                    metrics.setdefault(str(index), {})[name] = {
                        'mean_absolute_rgb_difference': float(difference.mean()),
                        'percentile95_absolute_rgb_difference': float(np.percentile(difference, 95))}
                windows.append({'lag_render_frames': lag * sample_every, 'environments': metrics})
            return windows

        def windows_pass(windows):
            return len(windows) == stable_windows and all(
                value['mean_absolute_rgb_difference'] <= mean_difference_limit and
                value['percentile95_absolute_rgb_difference'] <= percentile95_limit
                for window in windows for cameras in window['environments'].values()
                for value in cameras.values())
        try:
            for frame in range(1, max_frames + 1):
                generation = int(sim.render_generation)
                sim.render()
                if int(sim.render_generation) <= generation:
                    raise RuntimeError('sim.render() did not advance render generation')
                _ensure_render_update()
                report['render_frames'] = frame
                if int(sim.get_physics_step_count()) != physics_before:
                    raise RuntimeError('Physics advanced during appearance-only warm-up')
                if mode == 'temporal_mean':
                    current = self._warmup_snapshot(small_readback=True)
                    if not accumulated:
                        accumulated = {key: value.copy() for key, value in current.items()}
                        window_minimum = {key: value.copy() for key, value in current.items()}
                        window_maximum = {key: value.copy() for key, value in current.items()}
                    else:
                        for key, value in current.items():
                            accumulated[key] += value
                            np.minimum(window_minimum[key], value, out=window_minimum[key])
                            np.maximum(window_maximum[key], value, out=window_maximum[key])
                    if frame % sample_every:
                        continue
                    raw_snapshot = current
                    snapshot = {key: value / sample_every for key, value in accumulated.items()}
                    accumulated = {}
                else:
                    if frame % sample_every:
                        continue
                    snapshot = self._warmup_snapshot()
                sample = {'render_frame': frame, 'windows': compare_windows(snapshot, history), 'stable': False}
                if mode == 'temporal_mean':
                    sample['mean_rgb_by_environment'] = {
                        str(index): {name: snapshot[index, name].mean(axis=(0, 1)).tolist() for name in self.names}
                        for index in self.writers}
                sample['stable'] = windows_pass(sample['windows'])
                if mode == 'temporal_mean':
                    sample['unaveraged_windows'] = compare_windows(raw_snapshot, raw_history)
                    sample['averaged_windows_stable'] = sample['stable']
                    sample['unaveraged_windows_stable'] = windows_pass(sample['unaveraged_windows'])
                    ranges = {}
                    for (index, name), maximum in window_maximum.items():
                        pixel_range = maximum - window_minimum[index, name]
                        ranges.setdefault(str(index), {})[name] = {
                            'mean_absolute_rgb_difference': float(pixel_range.mean()),
                            'percentile95_absolute_rgb_difference': float(np.percentile(pixel_range, 95))}
                    sample['unaveraged_window_range'] = ranges
                    sample['unaveraged_window_range_stable'] = all(
                        value['mean_absolute_rgb_difference'] <= mean_difference_limit and
                        value['percentile95_absolute_rgb_difference'] <= percentile95_limit
                        for cameras in ranges.values() for value in cameras.values())
                    sample['stable'] = (sample['stable'] and sample['unaveraged_windows_stable']
                                        and sample['unaveraged_window_range_stable'])
                    raw_history.append(raw_snapshot)
                    raw_history = raw_history[-stable_windows:]
                consecutive = consecutive + 1 if sample['stable'] else 0
                sample['consecutive_stable_checks'] = consecutive
                report['samples'].append(sample)
                history.append(snapshot)
                history = history[-stable_windows:]
                if frame >= min_frames and consecutive >= stable_windows:
                    report['passed'] = True
                    break
            render_loop_completed = True
            if not report['passed']:
                report['error'] = 'Appearance did not converge within the maximum pure-render frame budget'
        except Exception as error:
            report['error'] = f'{type(error).__name__}: {error}'
            if fixed_preparation:
                report['preparation_error'] = report['error']
        finally:
            report['physics_step_count_after'] = int(sim.get_physics_step_count())
            report['physics_advanced'] = report['physics_step_count_after'] != physics_before
            report['render_generation_after'] = int(sim.render_generation)
            report['elapsed_seconds'] = time.perf_counter() - started
            if report['physics_advanced']:
                report['passed'] = False
            if fixed_preparation:
                report['preparation_passed'] = (
                    render_loop_completed and report['render_frames'] == 128 and
                    not report['physics_advanced'] and
                    report['render_generation_after'] - report['render_generation_before'] >= 128)
            self.timings['appearance_warmup_s'] += report['elapsed_seconds']
            for writer in self.writers.values():
                writer.file.attrs['appearance_warmup'] = json.dumps(report)
        return report

    def close(self):
        if self.closed:
            return
        started = time.perf_counter()
        errors = []
        drain_started = time.perf_counter()
        while self._pending:
            try:
                self._pending.popleft().result()
            except BaseException as error:
                errors.append(error)
        if self._executor is not None:
            self._executor.shutdown(wait=True)
        self.timings['close_drain_s'] += time.perf_counter() - drain_started
        if self._write_failure is not None:
            errors.insert(0, self._write_failure)
        if self.background_write and not errors and self.frames != self._submitted_frames:
            errors.append(RuntimeError('Committed RGB frame counts differ from submitted action counts'))
        if self._jpeg_stream is not None:
            try:
                self._jpeg_stream.close()
            except Exception as error:
                errors.append(error)
        for writer in self.writers.values():
            try:
                writer.close()
            except Exception as error:
                errors.append(error)
        self.closed = True
        self.timings['close_s'] += time.perf_counter() - started
        if errors:
            raise errors[0]
