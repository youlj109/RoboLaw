"""CPU source validation; no SimulationApp imports or GPU work."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import h5py
import numpy as np

from scripts.render_recorded_episode import (
    CAMERAS, KITCHEN_SHAPES, apply_camera_configuration, fresh_render,
    load_source, pose_errors, prepare_episode_visuals, refresh_native_render_products,
    select_indices, translated_state,
)


def make_source(path, *, dynamic=True):
    count = 3
    bodies = ['panda_link0'] + [f'body_{i}' for i in range(17)]
    joints = [f'joint_{i}' for i in range(13)]
    (path / 'result.json').write_text(json.dumps({'success': True}))
    (path / 'episode_spec.json').write_text(json.dumps({'layout': {'obstacle': {'static': not dynamic}}}))
    metadata = {'coordinate_frame': 'environment_local', 'quaternion_order': 'xyzw',
                'kitchen': {'coordinate_frame': 'environment_local'},
                'robot': {'joint_names': joints, 'body_names': bodies, 'root_body_name': 'panda_link0'},
                'hazard': {'dynamic': dynamic, 'name': 'glass'}}
    (path / 'render_state_metadata.json').write_text(json.dumps(metadata))
    cameras = {name: {'width': 1280, 'height': 720, 'prim_path': '{ENV_REGEX_NS}/Robot/panda_link0/' + name,
                      'offset': {'position': [0., 0., 0.], 'quaternion_xyzw': [0., 0., 0., 1.], 'convention': 'ros'},
                      'spawn': {'focal_length': 25., 'clipping_range': [.1, 1000.]}}
               for name in CAMERAS}
    (path / 'camera_config.json').write_text(json.dumps(cameras))
    with h5py.File(path / 'trajectory.h5', 'w') as handle:
        handle.attrs.update(dt=1 / 15, coordinate_frame='environment_local',
                            environment_origin_world_m=[30., 0., 0.],
                            joint_names=json.dumps(joints), robot_body_names=json.dumps(bodies))
        handle.create_dataset('phase', data=np.array(['settle', 'lift', 'release'], dtype=h5py.string_dtype()))
        handle.create_dataset('time', data=np.arange(count) / 15)
        handle.create_dataset('action', data=np.full((count, 8), 999.))
        shapes = {**KITCHEN_SHAPES, 'q': (13,), 'dq': (13,), 'robot_body_pose_w': (18, 7),
                  'robot_body_velocity': (18, 6), 'mustard': (13,), 'bowl': (13,)}
        if dynamic:
            shapes['hazard_state'] = (13,)
        for group in ('obs', 'next_obs'):
            for key, shape in shapes.items():
                value = np.zeros((count, *shape), dtype=np.float32)
                if key in ('robot_body_pose_w', 'mustard', 'bowl', 'hazard_state', 'kitchen_root_state', 'kitchen_link_pose'):
                    value[..., 6] = 1
                elif key in ('q', 'kitchen_joint_pos'):
                    value[:] = np.arange(count)[:, None] + (100 if group == 'next_obs' else 0)
                handle.create_dataset(group + '/' + key, data=value)


class RecordedEpisodeRendererTest(unittest.TestCase):
    def test_full_no_rgb_source_uses_only_measured_obs_and_needs_no_appearance_report(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            make_source(path)
            data = load_source(path)
            self.assertFalse(data['partial'])
            self.assertEqual(data['indices'], [0, 1, 2])
            self.assertEqual([frame['state']['q'][0] for frame in data['frames']], [0, 1, 2])
            self.assertEqual([frame['state']['kitchen_joint_pos'][0] for frame in data['frames']], [0, 1, 2])
            self.assertIsNone(data['appearance'])
            self.assertNotIn('recorded_cameras', data['frames'][0])

    def test_legacy_missing_kitchen_or_dynamic_hazard_is_rejected(self):
        for missing in ('render_state_metadata.json', 'obs/kitchen_link_pose', 'next_obs/kitchen_joint_vel', 'obs/hazard_state'):
            with self.subTest(missing=missing), tempfile.TemporaryDirectory() as tmp:
                path = Path(tmp)
                make_source(path)
                if missing.endswith('.json'):
                    (path / missing).unlink()
                else:
                    with h5py.File(path / 'trajectory.h5', 'a') as handle:
                        del handle[missing]
                with self.assertRaisesRegex(ValueError, 'record-render-state'):
                    load_source(path)

    def test_static_hazard_has_no_invented_state_and_partial_is_explicit(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            make_source(path, dynamic=False)
            data = load_source(path, [0, 2])
            self.assertTrue(data['partial'])
            self.assertFalse(data['dynamic_hazard'])
            self.assertNotIn('hazard_state', data['frames'][0]['state'])
        for indices in ([0, 0], [2, 1], [], [-1], [3]):
            with self.assertRaises(ValueError):
                select_indices(3, indices)

    def test_wrong_order_nonfinite_and_zero_quaternion_are_rejected(self):
        for case in ('order', 'nonfinite', 'quaternion'):
            with self.subTest(case=case), tempfile.TemporaryDirectory() as tmp:
                path = Path(tmp)
                make_source(path)
                if case == 'order':
                    meta = json.loads((path / 'render_state_metadata.json').read_text())
                    meta['robot']['joint_names'].reverse()
                    (path / 'render_state_metadata.json').write_text(json.dumps(meta))
                else:
                    with h5py.File(path / 'trajectory.h5', 'a') as handle:
                        if case == 'nonfinite':
                            handle['next_obs/kitchen_link_velocity'][1, 2, 3] = np.nan
                        else:
                            handle['obs/kitchen_link_pose'][1, 2, 3:7] = 0
                with self.assertRaises(ValueError):
                    load_source(path)

    def test_optional_camera_pose_alignment_loads_without_decoding_rgb(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            make_source(path)
            with h5py.File(path / 'images.h5', 'w') as handle:
                for name in CAMERAS:
                    group = handle.create_group(name)
                    group.attrs['coordinate_frame'] = 'environment_local'
                    group.create_dataset('position_world', data=np.arange(9).reshape(3, 3))
                    group.create_dataset('quaternion_world_ros_xyzw', data=np.tile([0., 0., 0., 1.], (3, 1)))
            data = load_source(path, [1])
            np.testing.assert_array_equal(data['frames'][0]['recorded_cameras'][CAMERAS[0]]['pose'][:3], [3, 4, 5])

    def test_origin_mapping_preserves_velocity_and_quaternion_sign(self):
        value = np.array([1., 2., 3., 0., 0., 0., 1., 7., 8., 9., 10., 11., 12.])
        actual = translated_state(value, 'environment_local', [30., 0., 0.], [3., 0., 0.])
        np.testing.assert_array_equal(actual[:3], [4, 2, 3])
        np.testing.assert_array_equal(actual[3:], value[3:])
        flipped = actual[:7].copy()
        flipped[3:] *= -1
        self.assertEqual(pose_errors(actual[:7], flipped)['rotation_max_error_rad'], 0.)

    def test_fresh_render_requires_generation_and_never_calls_physics(self):
        class Sim:
            render_generation = 0
            physics = 0
            def render(self):
                self.render_generation += 1
            def step(self):
                raise AssertionError('No physics step')
            def get_physics_step_count(self):
                return self.physics
        sim = Sim()
        fresh_render(sim, 0)
        self.assertEqual(sim.render_generation, 1)
        sim.physics = 1
        with self.assertRaisesRegex(RuntimeError, 'Physics advanced'):
            fresh_render(sim, 0)

    def test_authored_camera_configuration_preserves_optics_and_mount(self):
        cfg = SimpleNamespace(spawn=SimpleNamespace(focal_length=10., clipping_range=(1., 10.)), offset=SimpleNamespace())
        recorded = {'prim_path': 'robot/link/camera', 'width': 1280, 'height': 720,
                    'offset': {'position': [.1, .2, .3], 'quaternion_xyzw': [0., 0., 0., 1.], 'convention': 'ros'},
                    'spawn': {'focal_length': 25., 'clipping_range': [.01, 1000.]}}
        apply_camera_configuration(cfg, recorded)
        self.assertEqual(cfg.spawn.clipping_range, (.01, 1000.))
        self.assertEqual(cfg.offset.pos, (.1, .2, .3))
        self.assertTrue(cfg.update_latest_camera_pose)

    def test_fresh_render_products_retain_camera_pose_and_output_allocations(self):
        calls = []
        class Renderer:
            def cleanup(self, old):
                calls.append(('cleanup', old.render_product.path))
                old.spec = None
            def create_render_data(self, spec):
                calls.append(('create', spec))
                return SimpleNamespace(spec=spec, render_product=SimpleNamespace(path=spec + '_new'))
            def set_outputs(self, data, outputs):
                data.outputs = outputs
        scene = {}
        for name in CAMERAS:
            scene[name] = SimpleNamespace(_view=object(), _data=SimpleNamespace(output={'rgb': object()}),
                                          _renderer=Renderer(), _render_data=SimpleNamespace(
                                              spec=name, render_product=SimpleNamespace(path=name + '_old')))
        original = {name: (camera, camera._view, camera._data, camera._data.output)
                    for name, camera in scene.items()}
        events = refresh_native_render_products(scene)
        self.assertEqual(len(events), 3)
        for name, event in zip(CAMERAS, events):
            camera = scene[name]
            self.assertEqual((camera, camera._view, camera._data, camera._data.output), original[name])
            self.assertIs(camera._render_data.outputs, camera._data.output)
            self.assertEqual(camera._render_data.spec, name)
            self.assertNotEqual(event['old_render_product_path'], event['new_render_product_path'])
            self.assertTrue(event['passed'])
        self.assertEqual([call[0] for call in calls], ['cleanup', 'create'] * 3)

    def test_failed_product_refresh_cannot_keep_stale_pixels(self):
        old = SimpleNamespace(spec='old', render_product=SimpleNamespace(path='oldRP'))
        def fail(spec):
            raise RuntimeError('allocation failed')
        renderer = SimpleNamespace(cleanup=lambda data: None, create_render_data=fail)
        camera = SimpleNamespace(_render_data=old, _renderer=renderer,
                                 _data=SimpleNamespace(output={'rgb': object()}))
        events = []
        with self.assertRaisesRegex(RuntimeError, 'allocation failed'):
            refresh_native_render_products({'camera': camera}, ['camera'], events=events)
        self.assertIsNone(camera._render_data)
        self.assertFalse(events[0]['passed'])
        self.assertIn('allocation failed', events[0]['error'])

    def test_reused_product_lifecycle_finishes_before_new_appearance_edits(self):
        # Model a renderer consuming pending stage edits during product teardown.
        # Reauthoring the new appearance must happen after that lifecycle ends.
        stage = SimpleNamespace(pending_appearance=None)
        events = []
        class Renderer:
            def cleanup(self, old):
                events.append('retire')
                self.assert_no_new_edits = stage.pending_appearance is None
                if not self.assert_no_new_edits:
                    raise AssertionError('New appearance edits reached a retiring product')
                old.spec = None
            def create_render_data(self, spec):
                events.append('create')
                return SimpleNamespace(spec=spec, render_product=SimpleNamespace(path=spec + '_new'))
            def set_outputs(self, data, outputs):
                events.append('outputs')
        scene = {name: SimpleNamespace(_renderer=Renderer(),
                                      _data=SimpleNamespace(output={'rgb': object()}),
                                      _render_data=SimpleNamespace(spec=name, render_product=SimpleNamespace(path=name)))
                 for name in CAMERAS}
        spec = {'appearance': 'second'}
        def apply(stage, source):
            events.append('appearance')
            stage.pending_appearance = source['appearance']
            return {'actual': source['appearance']}
        cache = SimpleNamespace(apply_appearance=apply)
        report = {'scene_reuse': {'scene_reused': True}}
        prepare_episode_visuals(SimpleNamespace(refresh_render_products=True), scene, stage, spec, report,
                                scene_cache=cache, apply_appearance_fn=apply)
        self.assertEqual(events, ['retire', 'create', 'outputs'] * 3 + ['appearance'])
        self.assertEqual(report['appearance'], {'actual': 'second'})
        self.assertTrue(report['render_product_refresh']['performed'])

    def test_default_and_new_scene_apply_original_appearance_without_refresh(self):
        for options, report in [(SimpleNamespace(), {}),
                                (SimpleNamespace(refresh_render_products=True),
                                 {'scene_reuse': {'scene_reused': False}})]:
            calls = []
            def apply(stage, spec):
                calls.append((stage, spec))
                return {'actual': 'unchanged path'}
            with patch('scripts.render_recorded_episode.refresh_native_render_products',
                       side_effect=AssertionError('New/default scene must not refresh')):
                owner, before = prepare_episode_visuals(options, {}, 'stage', 'spec', report,
                                                       apply_appearance_fn=apply)
            self.assertEqual(calls, [('stage', 'spec')])
            self.assertIsNone(owner)
            self.assertIsNone(before)
            self.assertEqual(report['appearance'], {'actual': 'unchanged path'})


if __name__ == '__main__':
    unittest.main()
