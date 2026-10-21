"""Single-episode interface to the existing bounded background RGB writer.

Only encoding and H5 writes run in the worker. The normal collector captures
every original pre-action image and camera pose on its simulator thread.
"""
import time

from kitchen_safety.dataset import CameraWriter
from kitchen_safety.vectorized_dataset import VectorizedCameraWriter


class BackgroundCameraWriter:
    def __init__(self, out, scene, names, dt):
        if int(scene.num_envs) != 1:
            raise ValueError('Single-episode background writer requires exactly one environment')
        self.scene, self.names, self.dt = scene, tuple(names), dt
        # Keep the single collector's world frame. A zero translation makes
        # the batch sink's annotated local coordinates exactly world values.
        self._batch = VectorizedCameraWriter(
            {0: out}, scene, self.names, dt, background_write=True)
        self.timings = self._batch.timings
        self.timings.update(preview_s=0., frames=0, main_thread_write_s=0.)

    @property
    def frames(self):
        return self._batch.submitted_frames[0]

    @property
    def closed(self):
        return self._batch.closed

    @property
    def file(self):
        return self._batch.writers[0].file

    def warm_up(self, sim, **kwargs):
        # Retain the original single-episode appearance convergence predicate
        # and report format; the batch-only experimental modes are not used.
        return CameraWriter.warm_up(self, sim, **kwargs)

    def write(self, phase):
        started = time.perf_counter()
        try:
            self._batch.write({0: phase}, action_indices={0: self.frames})
            self.timings['frames'] = self.frames
        finally:
            self.timings['main_thread_write_s'] += time.perf_counter() - started

    def close(self):
        try:
            self._batch.close()
        finally:
            self.timings['frames'] = self._batch.frames[0]


def camera_caller_seconds(writer):
    """Caller write time, excluding overlapping worker service and warm-up."""
    if writer is None:
        return 0.
    if writer.timings.get('background_write'):
        return writer.timings['main_thread_write_s']
    return sum(writer.timings[key] for key in ('render_capture_s', 'jpeg_write_s', 'preview_s'))
