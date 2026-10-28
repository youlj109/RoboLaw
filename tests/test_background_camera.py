"""Single collector adapter equivalence against the original camera writer."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import h5py
import numpy as np

from kitchen_safety.background_camera import BackgroundCameraWriter, camera_caller_seconds
from kitchen_safety.dataset import CAMERA_NAMES, CameraWriter
from test_vectorized_dataset import Scene, Sim


class SingleBackgroundCameraTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.scene = Scene(np.array([[3., -2., .5]]))
        fake = SimpleNamespace(ensure_isaac_rtx_render_update=lambda **kwargs: None)
        self.module_patch = patch.dict('sys.modules', {
            'isaaclab_physx.renderers.isaac_rtx_renderer_utils': fake})
        self.module_patch.start()
        self.addCleanup(self.module_patch.stop)

    def test_original_and_background_have_identical_jpegs_world_poses_and_mounts(self):
        outputs = [Path(self.temp.name) / name for name in ('legacy', 'background')]
        for output in outputs:
            output.mkdir()
        legacy = CameraWriter(outputs[0], self.scene, CAMERA_NAMES, 1 / 15, previews=False)
        background = BackgroundCameraWriter(outputs[1], self.scene, CAMERA_NAMES, 1 / 15)
        for i in range(3):
            for camera in self.scene.values():
                camera.data.output['rgb'].value[:] += i * 11
                camera.data.pos_w.value[:, 2] += i * .1
            legacy.write('transfer')
            background.write('transfer')
            self.assertEqual(background.frames, i + 1)
        legacy.close()
        background.close()
        self.assertTrue(background.closed)
        self.assertEqual(background.timings['frames'], 3)
        self.assertEqual(camera_caller_seconds(background), background.timings['main_thread_write_s'])
        self.assertGreater(camera_caller_seconds(background), 0.)
        expected_meta = json.loads((outputs[0] / 'cameras.json').read_text())
        actual_meta = json.loads((outputs[1] / 'cameras.json').read_text())
        with h5py.File(outputs[0] / 'images.h5') as expected, h5py.File(outputs[1] / 'images.h5') as actual:
            for key in ('frames', 'dt', 'alignment', 'jpeg_quality', 'camera_order'):
                self.assertEqual(expected.attrs[key], actual.attrs[key])
            for name in CAMERA_NAMES:
                for index in range(3):
                    np.testing.assert_array_equal(expected[name]['jpeg'][index], actual[name]['jpeg'][index])
                for field in ('position_world', 'quaternion_world_ros_xyzw'):
                    np.testing.assert_array_equal(expected[name][field][:], actual[name][field][:])
                self.assertEqual(expected_meta[name], {key: actual_meta[name][key] for key in expected_meta[name]})
                self.assertEqual(actual_meta[name]['environment_origin_world_m'], [0., 0., 0.])
        background.close()

    def test_original_warmup_predicate_and_report_format_are_retained(self):
        output = Path(self.temp.name) / 'warmup'
        writer = BackgroundCameraWriter(output, self.scene, CAMERA_NAMES, 1 / 15)
        self.addCleanup(writer.close)
        report = writer.warm_up(Sim(), min_frames=4, max_frames=4, sample_every=1, stable_windows=1)
        self.assertTrue(report['passed'])
        self.assertFalse(report['physics_advanced'])
        self.assertEqual(report['render_frames'], 4)
        self.assertEqual(writer.frames, 0)
        self.assertEqual(set(report['samples'][-1]['windows'][0]['cameras']), set(CAMERA_NAMES))
        writer.write('settle')
        with self.assertRaisesRegex(ValueError, 'before recording'):
            writer.warm_up(Sim())

    def test_worker_error_is_raised_by_close_and_file_is_closed(self):
        writer = BackgroundCameraWriter(Path(self.temp.name) / 'failure', self.scene, CAMERA_NAMES, 1 / 15)
        with patch('kitchen_safety.vectorized_dataset._jpeg_bytes', side_effect=OSError('write failed')):
            writer.write('settle')
            with self.assertRaisesRegex(OSError, 'write failed'):
                writer.close()
        self.assertTrue(writer.closed)
        self.assertFalse(writer.file.id.valid)
        self.assertEqual(writer.timings['frames'], 0)


if __name__ == '__main__':
    unittest.main()
