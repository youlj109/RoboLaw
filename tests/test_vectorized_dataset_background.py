"""Bounded CPU writer tests; no simulator, GPU, or performance benchmark."""
import io
import json
import os
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

import h5py
import numpy as np
from PIL import Image

from kitchen_safety.dataset import CAMERA_NAMES
from kitchen_safety.vectorized_dataset import (VectorizedCameraWriter, _EnvironmentWriter,
                                               _jpeg_bytes, _open_jpeg_buffer)
from test_vectorized_dataset import Scene, Sim


class BackgroundWriterTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.origins = np.array([[0., 0., 0.], [8., -8., 0.]])
        self.scene = Scene(self.origins)
        self.ensure = patch('kitchen_safety.vectorized_dataset._ensure_render_update').start()
        self.addCleanup(patch.stopall)

    def writer(self, label='async', background=True):
        outputs = {i: Path(self.tmp.name) / label / str(i) for i in range(2)}
        writer = VectorizedCameraWriter(outputs, self.scene, env_origins=self.origins,
                                        background_write=background)
        self.addCleanup(writer.close)
        return writer, outputs

    def test_memfd_reuse_matches_original_jpeg_bytes_and_fallback(self):
        rng = np.random.default_rng(17)
        images = [rng.integers(0, 256, (33, 47, 3), dtype=np.uint8),
                  np.zeros((33, 47, 3), np.uint8), np.full((19, 27, 3), 133, np.uint8)]
        stream, backend = _open_jpeg_buffer()
        try:
            if backend == 'memfd':
                self.assertFalse(os.get_inheritable(stream.fileno()))
            for rgb in images:
                original = io.BytesIO()
                Image.fromarray(rgb).save(original, format='JPEG', quality=92)
                self.assertEqual(_jpeg_bytes(rgb, stream), original.getvalue())
        finally:
            stream.close()
        with patch('kitchen_safety.vectorized_dataset.sys.platform', 'unsupported-test-platform'):
            fallback, backend = _open_jpeg_buffer()
        with fallback:
            self.assertEqual(backend, 'bytesio')
            self.assertEqual(_jpeg_bytes(images[0], fallback), _jpeg_bytes(images[0]))

    def test_background_matches_sync_jpeg_poses_metadata_and_per_env_counts(self):
        sync, sync_paths = self.writer('sync', background=False)
        background, async_paths = self.writer()
        for writer in (sync, background):
            writer.write({0: 'grasp', 1: 'transfer'}, action_indices={0: 0, 1: 0})
            writer.write({1: 'release'}, action_indices={1: 1})
            writer.close()
        self.assertEqual(background.frames, {0: 1, 1: 2})
        self.assertEqual(background.submitted_frames, background.frames)
        self.assertEqual(background.timings['capture_ticks'], sync.timings['capture_ticks'])
        self.assertLessEqual(background.timings['max_pending_batches'], 2)
        for index in range(2):
            self.assertEqual(json.loads((sync_paths[index] / 'cameras.json').read_text()),
                             json.loads((async_paths[index] / 'cameras.json').read_text()))
            with h5py.File(sync_paths[index] / 'images.h5') as original, h5py.File(async_paths[index] / 'images.h5') as actual:
                self.assertEqual(actual.attrs['frames'], original.attrs['frames'])
                self.assertTrue(actual.attrs['background_write'])
                for name in CAMERA_NAMES:
                    for before, after in zip(original[name]['jpeg'], actual[name]['jpeg']):
                        np.testing.assert_array_equal(before, after)
                    for key in ('position_world', 'quaternion_world_ros_xyzw'):
                        np.testing.assert_array_equal(original[name][key], actual[name][key])

    def test_two_total_pending_batches_immutable_capture_and_no_worker_scene_access(self):
        writer, outputs = self.writer()
        entered, release, third_waiting, third_done = (threading.Event() for _ in range(4))
        self.addCleanup(release.set)
        original_write = writer._write_capture
        original_reserve = writer._reserve_pending_slot
        original_getitem = Scene.__getitem__
        failures = []
        def guarded_scene(scene, key):
            if threading.current_thread().name.startswith('arena-rgb-writer'):
                raise AssertionError('Worker accessed the scene')
            return original_getitem(scene, key)
        def blocked_write(capture, phases, indices):
            for values in capture.values():
                self.assertTrue(all(not array.flags.writeable for array in values.values()))
            entered.set()
            self.assertTrue(release.wait(3.), 'Test did not release the writer')
            return original_write(capture, phases, indices)
        def reserve():
            if len(writer._pending) == 2:
                third_waiting.set()
            return original_reserve()
        def third_submit():
            try:
                writer.write({1: 'settle'}, action_indices={1: 2})
            except BaseException as error:
                failures.append(error)
            finally:
                third_done.set()
        with patch.object(Scene, '__getitem__', guarded_scene), \
                patch.object(writer, '_write_capture', side_effect=blocked_write), \
                patch.object(writer, '_reserve_pending_slot', side_effect=reserve):
            indices = writer.write({0: 'first', 1: 'first'}, action_indices={0: 0, 1: 0})
            self.assertTrue(entered.wait(3.))
            indices[0] = 999  # Returned indices must not alias a worker job.
            for camera in self.scene.values():
                camera.data.output['rgb'].value[:] += 10
                camera.data.pos_w.value[:, 2] += .4
                camera.cfg.offset.pos = (9., 9., 9.)  # Metadata was frozen before the worker.
            writer.write({1: 'second'}, action_indices={1: 1})
            self.assertEqual(writer.frames, {0: 0, 1: 0})
            self.assertEqual(writer.submitted_frames, {0: 1, 1: 2})
            with self.assertRaisesRegex(ValueError, 'action index'):
                writer.write({1: 'bad-index'}, action_indices={1: 1})
            with self.assertRaisesRegex(ValueError, 'precede action frames'):
                writer.warm_up(Sim())
            producer = threading.Thread(target=third_submit)
            producer.start()
            self.assertTrue(third_waiting.wait(3.))
            self.assertFalse(third_done.wait(.02))
            self.assertEqual(self.ensure.call_count, 2, 'Queue-full wait must happen before rendering')
            release.set()
            producer.join(3.)
            self.assertFalse(producer.is_alive())
            self.assertEqual(failures, [])
            writer.close()
        self.assertEqual(writer.frames, {0: 1, 1: 3})
        self.assertEqual(writer.timings['max_pending_batches'], 2)
        self.assertTrue(all(not thread.is_alive() for thread in writer._executor._threads))
        for env_id, path in outputs.items():
            metadata = json.loads((path / 'cameras.json').read_text())
            with h5py.File(path / 'images.h5') as handle:
                for camera_index, name in enumerate(CAMERA_NAMES):
                    self.assertEqual(metadata[name]['offset']['position'], [.1, .2, .3])
                    for frame_index, encoded in enumerate(handle[name]['jpeg']):
                        with Image.open(io.BytesIO(encoded.tobytes())) as image:
                            image.load()
                            expected = 20 + env_id * 70 + camera_index * 5 + (10 if frame_index else 0)
                            self.assertLess(abs(np.asarray(image).mean() - expected), 1.)
                    np.testing.assert_allclose(handle[name]['position_world'][0], [camera_index+.1, .2, .3], atol=1e-6)

    def test_worker_failure_propagates_next_submit_and_close_after_join(self):
        writer, _ = self.writer()
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        original = _EnvironmentWriter.write_frame
        def fail_second_env(sink, capture, phase, **kwargs):
            if sink.env_id == 0:
                entered.set()
                self.assertTrue(release.wait(3.))
                return original(sink, capture, phase, **kwargs)
            raise OSError('deliberate H5 write failure')
        with patch.object(_EnvironmentWriter, 'write_frame', fail_second_env):
            writer.write({0: 'first', 1: 'first'}, action_indices={0: 0, 1: 0})
            self.assertTrue(entered.wait(3.))
            writer.write({0: 'second', 1: 'second'}, action_indices={0: 1, 1: 1})
            release.set()
            with self.assertRaisesRegex(RuntimeError, 'deliberate H5 write failure'):
                writer._pending[-1].result(timeout=3.)
            captures = self.ensure.call_count
            with self.assertRaisesRegex(RuntimeError, 'deliberate H5 write failure'):
                writer.write({0: 'third'}, action_indices={0: 2})
            self.assertEqual(self.ensure.call_count, captures)
            with self.assertRaisesRegex(OSError, 'deliberate H5 write failure'):
                writer.close()
        self.assertTrue(writer.closed)
        self.assertEqual(writer.frames, {0: 1, 1: 0})
        self.assertEqual(writer.submitted_frames, {0: 2, 1: 2})
        self.assertEqual(len(writer._pending), 0)
        self.assertTrue(all(not thread.is_alive() for thread in writer._executor._threads))
        self.assertTrue(all(not sink.file.id.valid for sink in writer.writers.values()))
        self.assertTrue(writer._jpeg_stream.closed)
        writer.close()  # Idempotent after reporting the original failure.


if __name__ == '__main__':
    unittest.main()
