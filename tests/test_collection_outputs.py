"""CPU-only checks for recording, offline previews, and RGB/action validation."""
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import h5py
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from kitchen_safety.dataset import (CAMERA_NAMES, RGB_ALIGNMENT, CameraWriter,
                                   camera_poses_from_body_states, decode_video)


def load_script(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'scripts' / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


exporter = load_script('export_collection_previews')
validator = load_script('validate_collection')
repair = load_script('repair_collection_camera_poses')


def episode(root, size=(1280, 720), count=5):
    dt = 1 / 15
    metadata = {}
    body_names = ['panda_link0', 'base_link']
    bodies = np.zeros((count + 1, 2, 7))
    bodies[:, :, 6] = 1
    bodies[:, 1, 0] = np.arange(count + 1) * .01
    for name in CAMERA_NAMES:
        parent = 'base_link' if name == 'wrist_camera' else 'panda_link0'
        metadata[name] = {'width': size[0], 'height': size[1],
                          'intrinsics': np.eye(3).tolist(), 'prim_path': '/World/' + parent + '/' + name,
                          'offset': {'position': [.01, .02, .03], 'quaternion_xyzw': [0, 0, 0, 1], 'convention': 'ros'}}
    poses = camera_poses_from_body_states(bodies[:-1], body_names, metadata)
    with h5py.File(root / 'trajectory.h5', 'w') as trajectory:
        trajectory.attrs['dt'] = dt
        trajectory.attrs['robot_body_names'] = json.dumps(body_names)
        trajectory.create_dataset('action', data=np.zeros((count, 8), dtype=np.float32))
        trajectory.create_dataset('time', data=np.arange(count) * dt)
        trajectory.create_dataset('phase', data=np.asarray(
            ['approach'] + ['transfer'] * (count - 2) + ['settle'], dtype=h5py.string_dtype()))
        trajectory.create_dataset('obs/q', data=np.arange(count)[:, None])
        trajectory.create_dataset('next_obs/q', data=np.arange(1, count + 1)[:, None])
        trajectory.create_dataset('obs/robot_body_pose_w', data=bodies[:-1])
        trajectory.create_dataset('next_obs/robot_body_pose_w', data=bodies[1:])
    with h5py.File(root / 'images.h5', 'w') as images:
        images.attrs.update(frames=count, dt=dt, alignment=RGB_ALIGNMENT, jpeg_quality=92,
                            camera_order=json.dumps(CAMERA_NAMES))
        for camera_index, name in enumerate(CAMERA_NAMES):
            group = images.create_group(name)
            jpeg = group.create_dataset('jpeg', shape=(count,), dtype=h5py.vlen_dtype(np.dtype('uint8')))
            group.create_dataset('position_world', data=poses[name]['position_world'])
            group.create_dataset('quaternion_world_ros_xyzw', data=poses[name]['quaternion_world_ros_xyzw'])
            for index in range(count):
                stream = io.BytesIO()
                Image.new('RGB', size, (index * 40, camera_index * 60, 20)).save(stream, format='JPEG', quality=92)
                jpeg[index] = np.frombuffer(stream.getvalue(), dtype=np.uint8)
    (root / 'result.json').write_text(json.dumps({'success': True, 'steps': count}))
    (root / 'cameras.json').write_text(json.dumps(metadata))


class ArrayTensor:
    def __init__(self, array):
        self.array = np.asarray(array)

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.array


def fake_camera(name):
    return types.SimpleNamespace(
        cfg=types.SimpleNamespace(prim_path='/World/' + name,
                                  offset=types.SimpleNamespace(pos=(0, 0, 0), rot=(0, 0, 0, 1), convention='ros')),
        data=types.SimpleNamespace(output={'rgb': ArrayTensor(np.zeros((1, 12, 16, 3), dtype=np.uint8))},
                                   pos_w=ArrayTensor([[0., 0., 0.]]),
                                   quat_w_ros=ArrayTensor([[0., 0., 0., 1.]]),
                                   intrinsic_matrices=ArrayTensor(np.eye(3)[None])),
        update=lambda *args, **kwargs: None)


class CollectionOutputsTest(unittest.TestCase):
    def test_split_directory_previews_keep_source_phase_and_write_beside_rendered_rgb(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source, rendered = root/'source', root/'rendered'
            source.mkdir()
            rendered.mkdir()
            episode(source, count=3)
            for name in ('images.h5', 'cameras.json'):
                (source/name).rename(rendered/name)
            report = dict(source_episode=str(source.resolve()), completed=True, complete_trajectory=True,
                          source_obs_indices=list(range(3)), collection_purpose='production',
                          appearance_warmup={'passed': True, 'physics_advanced': False})
            (rendered/'render_report.json').write_text(json.dumps(report))
            output = exporter.export_episode(source, render_root=rendered)
            self.assertTrue(output['passed'])
            self.assertEqual(len(list(rendered.glob('*.mp4'))), 4)
            self.assertEqual(list(source.glob('*.mp4')), [])
            self.assertFalse((rendered/'trajectory.h5').exists())
            checked = validator.validate_episode(source, require_previews=True, render_root=rendered)
            self.assertTrue(checked['passed'])
            self.assertEqual(checked['render_collection_purpose'], 'production')
            self.assertFalse(checked['production_quota_eligible'])
            report['source_episode'] = str(root/'wrong')
            (rendered/'render_report.json').write_text(json.dumps(report))
            with self.assertRaisesRegex(ValueError, 'different source'):
                exporter.export_episode(source, render_root=rendered)

    def test_separate_offline_images_keep_source_success_and_require_complete_obs_order(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source, rendered = root / 'source', root / 'rendered'
            source.mkdir()
            rendered.mkdir()
            episode(source)
            for name in ('images.h5', 'cameras.json'):
                (source / name).rename(rendered / name)
            original = {'success': False, 'steps': 5}
            (source / 'result.json').write_text(json.dumps(original))
            (source / 'episode_spec.json').write_text('{}')
            report = dict(source_episode=str(source.resolve()), completed=True, complete_trajectory=True,
                          source_obs_indices=list(range(5)),
                          appearance_warmup=dict(passed=True, physics_advanced=False))
            def save():
                (rendered / 'render_report.json').write_text(json.dumps(report))
            save()
            checked = validator.validate_episode(source, render_root=rendered)
            self.assertTrue(checked['passed'])
            self.assertFalse(checked['success'])
            self.assertEqual(json.loads((source / 'result.json').read_text()), original)
            for patch_report, error in (({'source_obs_indices': [1, 0, 2, 3, 4]}, 'action order'),
                                        ({'complete_trajectory': False}, 'entire measured'),
                                        ({'source_episode': str(root)}, 'different source'),
                                        ({'appearance_warmup': {}}, 'appearance_warmup')):
                prior = report.copy()
                report.update(patch_report)
                save()
                with self.assertRaisesRegex(ValueError, error):
                    validator.validate_episode(source, render_root=rendered)
                report = prior

    def test_appearance_warmup_waits_for_multiframe_stability_without_recording(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            scene = {name: fake_camera(name) for name in CAMERA_NAMES}
            sim = types.SimpleNamespace(render_generation=0, physics_steps=8)
            sim.get_physics_step_count = lambda: sim.physics_steps
            def render():
                sim.render_generation += 1
            sim.render = render
            pumped_generations = []
            def pump(**kwargs):
                # Models renderer dedup: calling ensure repeatedly without render
                # generation would keep returning the same apparently stable RGB.
                if pumped_generations and pumped_generations[-1] == sim.render_generation:
                    return
                pumped_generations.append(sim.render_generation)
                scene['wrist_camera'].data.output['rgb'].array.fill(max(100, 180 - sim.render_generation))
            renderer = types.SimpleNamespace(ensure_isaac_rtx_render_update=pump)
            with patch.dict(sys.modules, {'isaaclab_physx.renderers.isaac_rtx_renderer_utils': renderer}):
                writer = CameraWriter(root, scene, CAMERA_NAMES, 1 / 15, previews=False)
                with patch.object(writer, 'encoder', side_effect=AssertionError('Warm-up started an encoder')):
                    report = writer.warm_up(sim)
                self.assertTrue(report['passed'])
                self.assertEqual(report['render_frames'], 160)
                self.assertEqual(len(pumped_generations), report['render_frames'])
                self.assertEqual(sim.physics_steps, 8)
                self.assertFalse(report['physics_advanced'])
                self.assertEqual(writer.frames, 0)
                self.assertEqual(list(writer.file), [])
                writer.close()
            self.assertEqual(list(root.glob('*.mp4')), [])
            self.assertEqual(list(root.glob('*.jpg')), [])
            with h5py.File(root / 'images.h5') as images:
                self.assertEqual(images.attrs['frames'], 0)
                self.assertTrue(json.loads(images.attrs['appearance_warmup'])['passed'])

    def test_appearance_warmup_rejects_slow_drift_and_physics_advancement(self):
        for advance_physics in (False, True):
            with self.subTest(advance_physics=advance_physics), tempfile.TemporaryDirectory() as temporary:
                scene = {name: fake_camera(name) for name in CAMERA_NAMES}
                sim = types.SimpleNamespace(render_generation=0, physics_steps=8)
                sim.get_physics_step_count = lambda: sim.physics_steps
                def render():
                    sim.render_generation += 1
                    if advance_physics:
                        sim.physics_steps += 1
                sim.render = render
                def pump(**kwargs):
                    scene['wrist_camera'].data.output['rgb'].array.fill(255 - sim.render_generation // 2)
                renderer = types.SimpleNamespace(ensure_isaac_rtx_render_update=pump)
                with patch.dict(sys.modules, {'isaaclab_physx.renderers.isaac_rtx_renderer_utils': renderer}):
                    writer = CameraWriter(temporary, scene, CAMERA_NAMES, 1 / 15, previews=False)
                    report = writer.warm_up(sim, max_frames=128)
                    writer.close()
                self.assertFalse(report['passed'])
                self.assertEqual(report['physics_advanced'], advance_physics)
                self.assertEqual(report['render_frames'], 1 if advance_physics else 128)
                self.assertIn('Physics advanced' if advance_physics else 'did not converge', report['error'])

    def test_random_rgb_requires_successful_appearance_warmup(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            episode(root)
            result = json.loads((root / 'result.json').read_text())
            result['episode_spec'] = {'version': 1}
            (root / 'result.json').write_text(json.dumps(result))
            with self.assertRaisesRegex(ValueError, 'appearance_warmup.passed'):
                validator.validate_episode(root, require_rgb=True)
            result['appearance_warmup'] = {'passed': True, 'physics_advanced': False,
                                           'render_frames': 160, 'elapsed_seconds': .2}
            (root / 'result.json').write_text(json.dumps(result))
            self.assertTrue(validator.validate_episode(root, require_rgb=True)['passed'])
            result['appearance_warmup']['physics_advanced'] = True
            (root / 'result.json').write_text(json.dumps(result))
            with self.assertRaisesRegex(ValueError, 'without advancing physics'):
                validator.validate_episode(root, require_rgb=True)

    def test_fixed_appearance_benchmark_discloses_scope_and_still_checks_rgb(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            episode(root)
            result = json.loads((root / 'result.json').read_text())
            result.update(episode_spec={'version': 1}, appearance_mode='authored_fixed_benchmark',
                          production_quota_eligible=False,
                          appearance_warmup={'passed': None, 'convergence_checked': False,
                                             'physics_advanced': False, 'render_frames': 128})
            (root / 'result.json').write_text(json.dumps(result))
            report = validator.validate_episode(root, require_rgb=True)
            self.assertTrue(report['passed'])
            self.assertFalse(report['production_quota_eligible'])
            self.assertIn('was not measured', report['appearance_validation_scope'])
            result['production_quota_eligible'] = True
            (root / 'result.json').write_text(json.dumps(result))
            with self.assertRaisesRegex(ValueError, 'cannot contribute'):
                validator.validate_episode(root, require_rgb=True)
            result['production_quota_eligible'] = False
            (root / 'result.json').write_text(json.dumps(result))
            with h5py.File(root / 'images.h5', 'r+') as images:
                images['wrist_camera/jpeg'][2] = np.frombuffer(b'corrupted JPEG', dtype=np.uint8)
            with self.assertRaises(Exception):
                validator.validate_episode(root, require_rgb=True)

    def test_random_fixed_preparation_preserves_diagnostic_and_data_validation(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            episode(root)
            result = json.loads((root / 'result.json').read_text())
            warmup = dict(mode='fixed_preparation', passed=False, preparation_passed=True,
                          convergence_checked=True, convergence_required=False,
                          physics_advanced=False, physics_step_count_before=64, physics_step_count_after=64,
                          requested_render_frames=128, render_frames=128,
                          render_generation_before=8, render_generation_after=136)
            result.update(episode_spec={'version': 1}, appearance_mode='randomized_episode',
                          production_quota_eligible=False, appearance_warmup=warmup)
            (root / 'result.json').write_text(json.dumps(result))
            checked = validator.validate_episode(root, require_rgb=True)
            self.assertTrue(checked['passed'])
            self.assertFalse(checked['appearance_warmup']['passed'])
            self.assertTrue(checked['appearance_warmup']['preparation_passed'])
            self.assertFalse(checked['production_quota_eligible'])
            for key, value in [('preparation_passed', False), ('convergence_required', True),
                               ('physics_advanced', True), ('physics_step_count_after', 65),
                               ('render_frames', 127), ('render_generation_after', 135)]:
                with self.subTest(key=key):
                    result['appearance_warmup'] = {**warmup, key: value}
                    (root / 'result.json').write_text(json.dumps(result))
                    with self.assertRaises(ValueError):
                        validator.validate_episode(root, require_rgb=True)
            result['appearance_warmup'] = warmup
            (root / 'result.json').write_text(json.dumps(result))
            with h5py.File(root / 'images.h5', 'r+') as images:
                images['wrist_camera/jpeg'][2] = np.frombuffer(b'corrupted JPEG', dtype=np.uint8)
            with self.assertRaises(Exception):
                validator.validate_episode(root, require_rgb=True)

    def test_writer_enables_native_dynamic_world_pose_refresh(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            scene = {name: fake_camera(name) for name in CAMERA_NAMES}
            camera = scene['wrist_camera']
            camera.cfg.update_latest_camera_pose = False
            def native_like_update(*args, **kwargs):
                if camera.cfg.update_latest_camera_pose:
                    camera.data.pos_w.array += np.array([[.01, 0., 0.]])
            camera.update = native_like_update
            renderer = types.SimpleNamespace(ensure_isaac_rtx_render_update=lambda **kwargs: None)
            with patch.dict(sys.modules, {'isaaclab_physx.renderers.isaac_rtx_renderer_utils': renderer}):
                writer = CameraWriter(root, scene, CAMERA_NAMES, 1 / 15, previews=False)
                writer.write('approach')
                writer.write('transfer')
                writer.close()
            with h5py.File(root / 'images.h5') as images:
                positions = images['wrist_camera/position_world'][:]
                np.testing.assert_allclose(positions[:, 0], [.01, .02])
                self.assertIn('update_latest_camera_pose=True', images['wrist_camera'].attrs['world_pose_source'])

    def test_stale_wrist_pose_repair_preserves_rgb_originals_and_is_idempotent(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            episode(root)
            with h5py.File(root / 'images.h5', 'r+') as images:
                camera = images['wrist_camera']
                camera['position_world'][:] = camera['position_world'][0]
                old_positions = camera['position_world'][:]
                old_jpegs = [value.tobytes() for value in camera['jpeg']]
            with self.assertRaisesRegex(ValueError, 'camera world pose disagrees'):
                validator.validate_episode(root, require_rgb=True)
            self.assertFalse(repair.repair_episode(root)['passed'])
            self.assertTrue(repair.repair_episode(root, apply=True)['passed'])
            self.assertTrue(validator.validate_episode(root, require_rgb=True)['passed'])
            second = repair.repair_episode(root, apply=True)
            self.assertTrue(second['passed'])
            self.assertFalse(second['cameras']['wrist_camera']['repaired'])
            with h5py.File(root / 'images.h5') as images:
                camera = images['wrist_camera']
                np.testing.assert_array_equal(camera[repair.BACKUP]['position_world'][:], old_positions)
                self.assertEqual([value.tobytes() for value in camera['jpeg']], old_jpegs)
    def test_rgb_only_writer_does_not_start_encoders(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            scene = {name: fake_camera(name) for name in CAMERA_NAMES}
            renderer = types.SimpleNamespace(ensure_isaac_rtx_render_update=lambda **kwargs: None)
            with patch.dict(sys.modules, {'isaaclab_physx.renderers.isaac_rtx_renderer_utils': renderer}):
                writer = CameraWriter(root, scene, CAMERA_NAMES, 1 / 15, previews=False)
                with patch.object(writer, 'encoder', side_effect=AssertionError('H5-only recording started ffmpeg')):
                    writer.write('approach')
                    writer.write('transfer')
                writer.close()
                writer.close()
            self.assertEqual(list(root.glob('*.mp4')), [])
            self.assertEqual(list(root.glob('*.jpg')), [])
            self.assertEqual(writer.timings['frames'], 2)
            self.assertEqual(writer.timings['preview_s'], 0)
            with h5py.File(root / 'images.h5') as images:
                self.assertEqual(set(images), set(CAMERA_NAMES))
                self.assertEqual(images.attrs['alignment'], RGB_ALIGNMENT)
                self.assertEqual(images.attrs['jpeg_quality'], 92)
                for name in CAMERA_NAMES:
                    self.assertEqual(len(images[name]['jpeg']), 2)
                    self.assertEqual(images[name]['position_world'].shape, (2, 3))
                    self.assertEqual(images[name]['quaternion_world_ros_xyzw'].shape, (2, 4))

    def test_legacy_writer_still_produces_four_previews_and_phase_snapshots(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            scene = {name: fake_camera(name) for name in CAMERA_NAMES}
            renderer = types.SimpleNamespace(ensure_isaac_rtx_render_update=lambda **kwargs: None)
            with patch.dict(sys.modules, {'isaaclab_physx.renderers.isaac_rtx_renderer_utils': renderer}):
                writer = CameraWriter(root, scene, CAMERA_NAMES, 1 / 15)
                writer.write('approach')
                writer.write('transfer')
                writer.close()
            self.assertEqual(len(list(root.glob('*.mp4'))), 4)
            self.assertEqual(len(list(root.glob('*.jpg'))), 6)
            for name in CAMERA_NAMES:
                self.assertEqual(decode_video(root / f'{name}.mp4', 2, (16, 12), 1 / 15), 2)
            self.assertEqual(decode_video(root / 'all_cameras.mp4', 2, (1920, 360), 1 / 15), 2)
            self.assertGreater(writer.timings['preview_s'], 0)

    def test_validator_requires_rgb_without_requiring_previews(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            episode(root)
            result = validator.validate_episode(root, require_rgb=True)
            self.assertTrue(result['passed'])
            self.assertEqual(result['video_decoded_frames'], {})
            self.assertTrue(all(value['fully_decoded_frames'] == 5 for value in result['cameras'].values()))
            with self.assertRaisesRegex(ValueError, 'Required preview missing'):
                validator.validate_episode(root, require_previews=True)
            (root / 'images.h5').unlink()
            with self.assertRaisesRegex(ValueError, 'Required images.h5'):
                validator.validate_episode(root, require_rgb=True)
            self.assertTrue(validator.validate_episode(root)['passed'])

    def test_validator_rejects_missing_camera_and_midstream_corruption(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            episode(root)
            with h5py.File(root / 'images.h5', 'r+') as images:
                del images[CAMERA_NAMES[2]]
            with self.assertRaisesRegex(ValueError, 'must contain'):
                validator.validate_episode(root, require_rgb=True)
            episode(root)
            with h5py.File(root / 'images.h5', 'r+') as images:
                # This index is not one of the old validator's first/middle/last probes.
                images[CAMERA_NAMES[0]]['jpeg'][1] = np.frombuffer(b'not a JPEG', dtype=np.uint8)
            with self.assertRaises(OSError):
                validator.validate_episode(root, require_rgb=True)

    def test_validator_rejects_state_or_alignment_mismatch(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            episode(root)
            with h5py.File(root / 'trajectory.h5', 'r+') as trajectory:
                trajectory['next_obs/q'][1] = [20]
            with self.assertRaises(AssertionError):
                validator.validate_episode(root, require_rgb=True)
            episode(root)
            with h5py.File(root / 'images.h5', 'r+') as images:
                images.attrs['alignment'] = 'post-action'
            with self.assertRaisesRegex(ValueError, 'alignment'):
                validator.validate_episode(root, require_rgb=True)

    def test_offline_previews_decode_resume_and_repair(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            episode(root, size=(32, 24))
            result = exporter.export_episode(root)
            self.assertTrue(result['passed'])
            self.assertFalse(result['skipped_existing'])
            self.assertEqual(len(result['video_decoded_frames']), 4)
            before = {path.name: path.stat().st_mtime_ns for path in root.glob('*.mp4')}
            self.assertTrue(exporter.export_episode(root)['skipped_existing'])
            self.assertEqual(before, {path.name: path.stat().st_mtime_ns for path in root.glob('*.mp4')})
            path = root / 'wrist_camera.mp4'
            with self.assertRaisesRegex(ValueError, 'expected 6'):
                decode_video(path, 6)
            path.write_bytes(b'interrupted video')
            repaired = exporter.export_episode(root)
            self.assertTrue(repaired['passed'])
            self.assertFalse(repaired['skipped_existing'])
            self.assertEqual(repaired['video_decoded_frames']['wrist_camera.mp4'], 5)
            self.assertEqual(list(root.glob('.collection_previews_*')), [])


if __name__ == '__main__':
    unittest.main()
