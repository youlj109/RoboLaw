"""CPU-only checks of batched camera reads and independent episode storage."""
import io
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import h5py
import numpy as np
from PIL import Image

from kitchen_safety.dataset import CAMERA_NAMES, RGB_ALIGNMENT
from kitchen_safety.vectorized_dataset import VectorizedCameraWriter, _box_sample_batch


class CountedTensor:
    def __init__(self, value):
        self.value = value
        self.cpu_reads = 0

    def detach(self):
        return self

    def cpu(self):
        self.cpu_reads += 1
        return self

    def numpy(self):
        return self.value


class Camera:
    def __init__(self, index, origins):
        n = len(origins)
        rgb = np.empty((n, 720, 1280, 3), dtype=np.uint8)
        for env_id in range(n):
            rgb[env_id].fill(20 + env_id * 70 + index * 5)
        self.cfg = SimpleNamespace(
            width=1280, height=720, update_latest_camera_pose=False,
            prim_path='{ENV_REGEX_NS}/Robot/panda_link0/' + CAMERA_NAMES[index],
            offset=SimpleNamespace(pos=(.1, .2, .3), rot=(0., 0., 0., 1.), convention='ros'))
        self.data = SimpleNamespace(
            output={'rgb': CountedTensor(rgb)},
            pos_w=CountedTensor(np.asarray(origins) + [index + .1, .2, .3]),
            quat_w_ros=CountedTensor(np.tile([0., 0., 0., 1.], (n, 1))),
            intrinsic_matrices=CountedTensor(np.tile([[1000., 0., 640.], [0., 1000., 360.], [0., 0., 1.]], (n, 1, 1))))
        self.updates = 0

    def update(self, dt, force_recompute):
        assert dt == 0 and force_recompute
        self.updates += 1


class Scene(dict):
    def __init__(self, origins):
        super().__init__((name, Camera(index, origins)) for index, name in enumerate(CAMERA_NAMES))
        self.num_envs = len(origins)


class Sim:
    def __init__(self, callback=None):
        self.render_generation = 0
        self.steps = 50
        self.callback = callback

    def render(self):
        self.render_generation += 1
        if self.callback:
            self.callback(self)

    def get_physics_step_count(self):
        return self.steps


class VectorizedDatasetTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.origins = np.asarray([[0., 0., 0.], [8., -8., 0.]])
        self.scene = Scene(self.origins)
        self.outputs = {index: Path(self.tmp.name) / f'env_{index}' for index in range(2)}
        self.ensure = patch('kitchen_safety.vectorized_dataset._ensure_render_update').start()
        self.addCleanup(patch.stopall)
        self.addCleanup(self.tmp.cleanup)

    def writer(self, **kwargs):
        writer = VectorizedCameraWriter(self.outputs, self.scene, env_origins=self.origins, **kwargs)
        self.addCleanup(writer.close)
        return writer

    def test_batch_read_once_env_isolation_and_independent_action_indices(self):
        writer = self.writer()
        self.assertEqual(writer.write({0: 'grasp', 1: 'transfer'}, action_indices={0: 0, 1: 0}), {0: 0, 1: 0})
        for camera in self.scene.values():
            camera.data.output['rgb'].value[1] += 10
            camera.data.pos_w.value[1, 2] += .4
        self.assertEqual(writer.write({1: 'release'}, action_indices={1: 1}), {1: 1})
        self.assertEqual(writer.write({}), {})
        self.assertEqual(writer.frames, {0: 1, 1: 2})
        self.assertEqual(self.ensure.call_count, 2)
        for camera in self.scene.values():
            self.assertEqual(camera.updates, 2)
            self.assertEqual(camera.data.output['rgb'].cpu_reads, 2)
            self.assertEqual(camera.data.pos_w.cpu_reads, 2)
            self.assertEqual(camera.data.intrinsic_matrices.cpu_reads, 1)
            self.assertTrue(camera.cfg.update_latest_camera_pose)
        writer.close()
        for env_id, out in self.outputs.items():
            metadata = json.loads((out / 'cameras.json').read_text())
            self.assertEqual(set(metadata), set(CAMERA_NAMES))
            with h5py.File(out / 'images.h5', 'r') as handle:
                self.assertEqual(set(handle), set(CAMERA_NAMES))
                self.assertEqual(handle.attrs['frames'], env_id + 1)
                self.assertEqual(handle.attrs['alignment'], RGB_ALIGNMENT)
                self.assertAlmostEqual(handle.attrs['dt'], 1 / 15)
                self.assertEqual(handle.attrs['coordinate_frame'], 'environment_local')
                np.testing.assert_array_equal(handle.attrs['environment_origin_world_m'], self.origins[env_id])
                for cam_index, name in enumerate(CAMERA_NAMES):
                    camera_meta = metadata[name]
                    self.assertEqual(camera_meta['coordinate_frame'], 'environment_local')
                    self.assertEqual(camera_meta['environment_origin_world_m'], self.origins[env_id].tolist())
                    self.assertEqual(camera_meta['offset']['position'], [.1, .2, .3])
                    self.assertEqual((camera_meta['width'], camera_meta['height']), (1280, 720))
                    group = handle[name]
                    for frame_index, encoded in enumerate(group['jpeg']):
                        with Image.open(io.BytesIO(encoded.tobytes())) as image:
                            image.load()
                            self.assertEqual(image.size, (1280, 720))
                            expected = 20 + 70 * env_id + 5 * cam_index + 10 * frame_index
                            self.assertLess(abs(np.asarray(image).astype(float).mean() - expected), 1.)
                    np.testing.assert_allclose(group['position_world'][0], [cam_index + .1, .2, .3], atol=1e-6)
                    np.testing.assert_array_equal(group['quaternion_world_ros_xyzw'][0], [0., 0., 0., 1.])
                    if env_id == 1:
                        np.testing.assert_allclose(group['position_world'][1], [cam_index + .1, .2, .7], atol=1e-6)

    def test_shape_error_writes_no_partial_frames(self):
        writer = self.writer()
        self.scene[CAMERA_NAMES[-1]].data.output['rgb'].value = np.zeros((1, 720, 1280, 3), np.uint8)
        with self.assertRaisesRegex(ValueError, 'expected uint8 RGB batch'):
            writer.write({0: 'test', 1: 'test'})
        self.assertEqual(writer.frames, {0: 0, 1: 0})
        self.assertTrue(all(not sink.datasets for sink in writer.writers.values()))

    def test_alignment_error_precedes_render_or_write(self):
        writer = self.writer()
        with self.assertRaisesRegex(ValueError, 'action index'):
            writer.write({0: 'test'}, action_indices={0: 1})
        self.ensure.assert_not_called()
        self.assertEqual(writer.frames, {0: 0, 1: 0})
        writer.close()
        with self.assertRaisesRegex(ValueError, 'closed'):
            writer.write({0: 'test'})

    def test_warmup_checks_all_envs_and_does_not_record_or_advance_physics(self):
        writer = self.writer()
        sim = Sim()
        report = writer.warm_up(sim, min_frames=4, max_frames=6, sample_every=1, stable_windows=1)
        self.assertTrue(report['passed'])
        self.assertFalse(report['physics_advanced'])
        self.assertEqual(report['render_frames'], 4)
        self.assertEqual(writer.frames, {0: 0, 1: 0})
        self.assertEqual(self.ensure.call_count, 4)
        metrics = report['samples'][-1]['windows'][0]['environments']
        self.assertEqual(set(metrics), {'0', '1'})
        self.assertTrue(all(set(cameras) == set(CAMERA_NAMES) for cameras in metrics.values()))
        for sink in writer.writers.values():
            self.assertTrue(json.loads(sink.file.attrs['appearance_warmup'])['passed'])

    def test_only_second_environment_drift_rejects_batch_warmup(self):
        writer = self.writer()
        def change_other_env(sim):
            self.scene[CAMERA_NAMES[-1]].data.output['rgb'].value[1] = 40 + sim.render_generation * 10
        report = writer.warm_up(Sim(change_other_env), min_frames=4, max_frames=6,
                                sample_every=1, stable_windows=1)
        self.assertFalse(report['passed'])
        self.assertFalse(report['physics_advanced'])
        self.assertEqual(writer.frames, {0: 0, 1: 0})

    def test_warmup_physics_advance_is_reported(self):
        writer = self.writer()
        def advance(sim):
            sim.steps += 1
        report = writer.warm_up(Sim(advance), min_frames=4, max_frames=6, sample_every=1, stable_windows=1)
        self.assertFalse(report['passed'])
        self.assertTrue(report['physics_advanced'])

    def test_fixed_preparation_keeps_pixel_result_without_requiring_convergence(self):
        for noisy in (False, True):
            with self.subTest(noisy=noisy):
                writer = self.writer()
                sim = Sim()
                def snapshot(**kwargs):
                    pixels = np.full((4, 4, 3), 128., dtype=np.float32)
                    if noisy:
                        pixels += np.random.default_rng(sim.render_generation).normal(0., 4., pixels.shape)
                    return {(index, name): pixels.copy() for index in writer.writers for name in CAMERA_NAMES}
                with patch.object(writer, '_warmup_snapshot', side_effect=snapshot):
                    report = writer.warm_up(sim, mode='fixed_preparation')
                self.assertEqual(report['passed'], not noisy)
                self.assertTrue(report['preparation_passed'])
                self.assertFalse(report['convergence_required'])
                self.assertTrue(report['convergence_checked'])
                self.assertFalse(report['physics_advanced'])
                self.assertEqual(report['render_frames'], 128)
                self.assertEqual(report['render_generation_after'] - report['render_generation_before'], 128)
                self.assertEqual(len(report['samples']), 8)
                self.assertEqual(report['settings']['mean_difference_limit'], .35)
                self.assertEqual(report['settings']['percentile95_limit'], 1.5)
                self.assertEqual(writer.frames, {0: 0, 1: 0})
                for sink in writer.writers.values():
                    self.assertEqual(json.loads(sink.file.attrs['appearance_warmup']), report)
                writer.close()

    def test_fixed_preparation_rejects_physics_advance_and_stale_render_generation(self):
        for error_kind in ('physics', 'generation', 'renderer'):
            with self.subTest(error_kind=error_kind):
                writer = self.writer()
                def change(sim):
                    if error_kind == 'physics':
                        sim.steps += 1
                    if error_kind == 'generation':
                        sim.render_generation -= 1
                self.ensure.side_effect = RuntimeError('renderer failed') if error_kind == 'renderer' else None
                report = writer.warm_up(Sim(change), mode='fixed_preparation')
                self.assertFalse(report['preparation_passed'])
                self.assertFalse(report['passed'])
                self.assertTrue(report['preparation_error'])
                self.assertEqual(report['physics_advanced'], error_kind == 'physics')
                self.assertEqual(writer.frames, {0: 0, 1: 0})
                writer.close()
        self.ensure.side_effect = None

    def test_box_sampling_matches_existing_pil_rounding(self):
        import torch
        rgb = np.random.default_rng(42).integers(0, 256, (2, 720, 1280, 3), dtype=np.uint8)
        expected = np.stack([np.asarray(Image.fromarray(frame).resize((160, 90), Image.Resampling.BOX))
                             for frame in rgb])
        # Exercise both CPU implementations without creating a CUDA context.
        np.testing.assert_array_equal(_box_sample_batch(rgb), expected)
        np.testing.assert_array_equal(_box_sample_batch(torch.from_numpy(rgb)), expected)

    def test_temporal_mean_cannot_certify_noise_failing_unaveraged_frames(self):
        writer = self.writer()
        sim = Sim()
        def noisy_snapshot(**kwargs):
            rng = np.random.default_rng(sim.render_generation)
            sample = (128 + np.rint(rng.normal(0., 1., (90, 160, 3)))).astype(np.float32)
            return {(index, name): sample.copy() for index in writer.writers for name in CAMERA_NAMES}
        with patch.object(writer, '_warmup_snapshot', side_effect=noisy_snapshot):
            single = writer.warm_up(sim, max_frames=160)
            self.assertFalse(single['passed'])
            self.assertEqual(single['settings']['mode'], 'single_frame')
            sim.render_generation = 0
            averaged = writer.warm_up(sim, max_frames=160, mode='temporal_mean')
        self.assertFalse(averaged['passed'])
        self.assertTrue(averaged['samples'][-1]['averaged_windows_stable'])
        self.assertFalse(averaged['samples'][-1]['unaveraged_windows_stable'])
        self.assertEqual(averaged['settings']['mean_difference_limit'], .35)
        self.assertEqual(averaged['settings']['percentile95_limit'], 1.5)
        self.assertEqual(averaged['settings']['temporal_window_frames'], 16)
        self.assertFalse(averaged['physics_advanced'])
        self.assertEqual(writer.frames, {0: 0, 1: 0})
        # Warm-up's synthetic values must not enter the recorded RGB path.
        writer.write({0: 'test'}, action_indices={0: 0})
        encoded = writer.writers[0].file[CAMERA_NAMES[0]]['jpeg'][0]
        with Image.open(io.BytesIO(encoded.tobytes())) as image:
            self.assertLess(abs(np.asarray(image).astype(float).mean() - 20.), 1.)

    def test_temporal_mean_rejects_persistent_drift(self):
        writer = self.writer()
        sim = Sim()
        def drifting_snapshot(**kwargs):
            sample = np.full((90, 160, 3), 128 + sim.render_generation * .05, dtype=np.float32)
            return {(index, name): sample.copy() for index in writer.writers for name in CAMERA_NAMES}
        with patch.object(writer, '_warmup_snapshot', side_effect=drifting_snapshot):
            report = writer.warm_up(sim, max_frames=192, mode='temporal_mean')
        self.assertFalse(report['passed'])
        self.assertEqual(report['render_frames'], 192)
        self.assertEqual(writer.frames, {0: 0, 1: 0})

    def test_temporal_mean_material_change_requires_new_stable_history(self):
        writer = self.writer()
        sim = Sim()
        def material_change(**kwargs):
            sample = np.full((90, 160, 3), 128 if sim.render_generation <= 100 else 148, dtype=np.float32)
            return {(index, name): sample.copy() for index in writer.writers for name in CAMERA_NAMES}
        with patch.object(writer, '_warmup_snapshot', side_effect=material_change):
            short = writer.warm_up(sim, max_frames=160, mode='temporal_mean')
            self.assertFalse(short['passed'])
            sim.render_generation = 0
            enough = writer.warm_up(sim, max_frames=256, mode='temporal_mean')
        self.assertTrue(enough['passed'])
        self.assertEqual(enough['render_frames'], 208)
        changed_sample = next(s for s in enough['samples'] if s['render_frame'] == 112)
        self.assertFalse(changed_sample['stable'])
        self.assertEqual(changed_sample['consecutive_stable_checks'], 0)

    def test_temporal_mean_never_accepts_physics_advance(self):
        writer = self.writer()
        def advance(sim):
            if sim.render_generation == 3:
                sim.steps += 1
        sim = Sim(advance)
        constant = {(index, name): np.zeros((90, 160, 3), np.float32)
                    for index in writer.writers for name in CAMERA_NAMES}
        with patch.object(writer, '_warmup_snapshot', return_value=constant):
            report = writer.warm_up(sim, mode='temporal_mean')
        self.assertFalse(report['passed'])
        self.assertTrue(report['physics_advanced'])
        self.assertEqual(report['render_frames'], 3)
        self.assertEqual(writer.frames, {0: 0, 1: 0})

    def test_long_temporal_windows_reject_continuous_drift(self):
        writer = self.writer()
        for window in (64, 128):
            with self.subTest(window=window):
                sim = Sim()
                def drifting_snapshot(**kwargs):
                    sample = np.full((4, 4, 3), 128 + sim.render_generation * .02, dtype=np.float32)
                    return {(index, name): sample.copy() for index in writer.writers for name in CAMERA_NAMES}
                with patch.object(writer, '_warmup_snapshot', side_effect=drifting_snapshot):
                    report = writer.warm_up(sim, max_frames=6 * window, sample_every=window, mode='temporal_mean')
                self.assertFalse(report['passed'])
                self.assertFalse(report['physics_advanced'])
                self.assertEqual(report['settings']['temporal_window_frames'], window)
                self.assertEqual(writer.frames, {0: 0, 1: 0})

    def test_long_temporal_windows_rewarm_after_midwindow_material_change(self):
        writer = self.writer()
        for window in (64, 128):
            with self.subTest(window=window):
                sim = Sim()
                jump_frame = 3 * window + window // 2
                def material_change(**kwargs):
                    sample = np.full((4, 4, 3), 128 if sim.render_generation < jump_frame else 148, dtype=np.float32)
                    return {(index, name): sample.copy() for index in writer.writers for name in CAMERA_NAMES}
                with patch.object(writer, '_warmup_snapshot', side_effect=material_change):
                    short = writer.warm_up(sim, max_frames=9 * window, sample_every=window, mode='temporal_mean')
                    self.assertFalse(short['passed'])
                    sim.render_generation = 0
                    enough = writer.warm_up(sim, max_frames=11 * window, sample_every=window, mode='temporal_mean')
                self.assertTrue(enough['passed'])
                self.assertEqual(enough['render_frames'], 10 * window)
                self.assertFalse(enough['physics_advanced'])

    def test_long_temporal_windows_still_detect_physics_advance(self):
        writer = self.writer()
        for window in (64, 128):
            with self.subTest(window=window):
                def advance(sim):
                    if sim.render_generation == window + 1:
                        sim.steps += 1
                sim = Sim(advance)
                constant = {(index, name): np.zeros((4, 4, 3), np.float32)
                            for index in writer.writers for name in CAMERA_NAMES}
                with patch.object(writer, '_warmup_snapshot', return_value=constant):
                    report = writer.warm_up(sim, max_frames=6 * window, sample_every=window, mode='temporal_mean')
                self.assertFalse(report['passed'])
                self.assertTrue(report['physics_advanced'])
                self.assertEqual(report['render_frames'], window + 1)
                self.assertEqual(writer.frames, {0: 0, 1: 0})

    def test_window_end_material_jump_cannot_be_diluted_into_success(self):
        writer = self.writer()
        for window in (16, 64, 128):
            with self.subTest(window=window):
                sim = Sim()
                budget = max(128, 6 * window)
                def material_change(**kwargs):
                    sample = np.full((4, 4, 3), 148 if sim.render_generation == budget else 128,
                                     dtype=np.float32)
                    return {(index, name): sample.copy() for index in writer.writers for name in CAMERA_NAMES}
                with patch.object(writer, '_warmup_snapshot', side_effect=material_change):
                    report = writer.warm_up(sim, min_frames=budget, max_frames=budget,
                                            sample_every=window, mode='temporal_mean')
                self.assertFalse(report['passed'])
                final = report['samples'][-1]
                self.assertFalse(final['unaveraged_windows_stable'])
                self.assertEqual(final['consecutive_stable_checks'], 0)
                if window > 16:
                    self.assertTrue(final['averaged_windows_stable'])
                self.assertEqual(writer.frames, {0: 0, 1: 0})

    def test_midwindow_flash_cannot_disappear_before_window_end(self):
        writer = self.writer()
        for window in (64, 128):
            with self.subTest(window=window):
                sim = Sim()
                budget = 6 * window
                def brief_flash(**kwargs):
                    sample = np.full((4, 4, 3),
                                     148 if sim.render_generation == budget - window // 2 else 128,
                                     dtype=np.float32)
                    return {(index, name): sample.copy() for index in writer.writers for name in CAMERA_NAMES}
                with patch.object(writer, '_warmup_snapshot', side_effect=brief_flash):
                    report = writer.warm_up(sim, min_frames=budget, max_frames=budget,
                                            sample_every=window, mode='temporal_mean')
                final = report['samples'][-1]
                self.assertTrue(final['averaged_windows_stable'])
                self.assertTrue(final['unaveraged_windows_stable'])
                self.assertFalse(final['unaveraged_window_range_stable'])
                self.assertFalse(report['passed'])
                self.assertEqual(final['consecutive_stable_checks'], 0)

    def test_configuration_errors_do_not_create_output(self):
        with self.assertRaisesRegex(ValueError, 'own output directory'):
            VectorizedCameraWriter({0: self.outputs[0], 1: self.outputs[0]}, self.scene)
        with self.assertRaisesRegex(ValueError, 'origin'):
            VectorizedCameraWriter(self.outputs, self.scene, env_origins=np.zeros((1, 3)))
        with self.assertRaisesRegex(ValueError, '15 Hz'):
            VectorizedCameraWriter(self.outputs, self.scene, dt=1 / 30)
        self.assertTrue(all(not path.exists() for path in self.outputs.values()))


if __name__ == '__main__':
    unittest.main()
