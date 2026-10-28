from types import SimpleNamespace
import unittest

from kitchen_safety.grouped_camera import DEFAULT_NAMES, GroupedReadout, _compatible_cameras


class FakeMatrix:
    def __init__(self, focal):
        self.values = [[[focal, 0., 640.], [0., focal, 360.], [0., 0., 1.]]]
        self.torch = self

    def detach(self):
        return self

    def cpu(self):
        return self

    def tolist(self):
        return self.values


class FakeCamera:
    def __init__(self, name, renderer, focal):
        self.cfg = SimpleNamespace(width=1280, height=720, data_types=['rgb'], background_color=None,
                                   update_period=0, isp_cfg=None, focal_length=focal)
        self.device = 'cuda:0'
        self.num_instances = 1
        self._is_initialized = True
        self._renderer = renderer
        self._ALL_ENV_MASK = True
        self._view = SimpleNamespace(count=1, prim_paths=['/World/envs/env_0/robot/' + name])
        self._render_data = SimpleNamespace(spec=SimpleNamespace(camera_prim_paths=self._view.prim_paths), name=name)
        self._data = SimpleNamespace(_output={'rgb': [name + '_old']}, intrinsics=focal,
                                     intrinsic_matrices=FakeMatrix(focal))
        self.pose_calls = self.mask_reads = self.original_update_calls = 0

    def _update_poses(self, *, env_mask, frame_op):
        assert env_mask is True and frame_op == 1
        self.pose_calls += 1

    def _env_mask_has_any(self, mask):
        self.mask_reads += 1
        return mask

    def _update_buffers_impl(self, mask):
        self.original_update_calls += 1


class FakeScene(dict):
    num_envs = 1


def setup_owner(*, mixed_intrinsics=False):
    cleaned, reads = [], []
    renderer = SimpleNamespace(cleanup=lambda data: cleaned.append(data))
    cameras = [FakeCamera(name, renderer, 666.6667 if mixed_intrinsics and index == 2 else 500)
               for index, name in enumerate(DEFAULT_NAMES)]
    scene = FakeScene(zip(DEFAULT_NAMES, cameras))
    sim = SimpleNamespace(render_generation=1, get_physics_step_count=lambda: 0)
    outputs = [{'rgb': [index]} for index in range(3)]
    group = SimpleNamespace(name='group', spec=SimpleNamespace(view_count=3),
                            render_product=SimpleNamespace(path='/RP/group'))

    def read(render, data, camera_data, step):
        assert render is renderer and data is owner.render_data and step == 0
        reads.append(sim.render_generation)
        for index, output in enumerate(outputs):
            output['rgb'][0] = 10 * sim.render_generation + index

    sim.render_context = SimpleNamespace(render_into_camera=read)
    owner = GroupedReadout(cameras, DEFAULT_NAMES, renderer, group, SimpleNamespace(output={'rgb': object()}), sim, outputs,
                           {'camera_names': list(DEFAULT_NAMES), 'production_quota_eligible': False})
    return owner, scene, cameras, sim, outputs, cleaned, reads


class GroupedCameraTests(unittest.TestCase):
    def test_three_original_camera_apis_share_one_native_read_per_generation(self):
        owner, scene, cameras, sim, outputs, cleaned, reads = setup_owner()
        validated, paths = _compatible_cameras(scene, DEFAULT_NAMES)
        self.assertEqual(validated, tuple(cameras))
        self.assertEqual(len(set(paths)), 3)
        original_data = [camera._data for camera in cameras]
        original_views = [camera._view for camera in cameras]
        owner.install()
        self.assertEqual([data.name for data in cleaned], list(DEFAULT_NAMES))
        for camera in cameras:
            camera._update_buffers_impl(True)
        self.assertEqual(reads, [1])
        self.assertEqual([camera.pose_calls for camera in cameras], [1, 1, 1])
        self.assertEqual(sum(camera.mask_reads for camera in cameras), 1)
        self.assertEqual([camera._data._output['rgb'][0] for camera in cameras], [10, 11, 12])
        self.assertEqual([camera._data.intrinsics for camera in cameras], [500, 500, 500])
        for i, camera in enumerate(cameras):
            self.assertIs(camera._data, original_data[i])
            self.assertIs(camera._view, original_views[i])
            self.assertIs(camera._data._output, outputs[i])
            self.assertIsNone(camera._render_data)
        sim.render_generation = 2
        # Caller order does not change the permanent name-to-output mapping.
        for camera in reversed(cameras):
            camera._update_buffers_impl(True)
        self.assertEqual(reads, [1, 2])
        self.assertEqual([camera._data._output['rgb'][0] for camera in cameras], [20, 21, 22])
        self.assertEqual(owner.report()['grouped_read_count'], 2)
        self.assertEqual(owner.report()['requests'], 6)
        owner.close()

    def test_empty_mask_does_not_consume_the_generation_and_failure_can_retry(self):
        owner, _, cameras, sim, _, _, reads = setup_owner()
        owner.install()
        cameras[0]._update_buffers_impl(False)
        self.assertEqual(reads, [])
        original = sim.render_context.render_into_camera

        def broken(*args):
            raise RuntimeError('annotator failure')

        sim.render_context.render_into_camera = broken
        with self.assertRaisesRegex(RuntimeError, 'annotator failure'):
            cameras[0]._update_buffers_impl(True)
        self.assertEqual(owner.grouped_read_count, 0)
        sim.render_context.render_into_camera = original
        cameras[1]._update_buffers_impl(True)
        self.assertEqual(reads, [1])
        self.assertEqual(owner.report()['empty_mask_requests'], 1)
        owner.close()

    def test_close_destroys_group_once_and_detaches_hooks_before_environment_close(self):
        owner, _, cameras, _, _, cleaned, _ = setup_owner()
        old_methods = [camera._update_buffers_impl for camera in cameras]
        old_outputs = [camera._data._output for camera in cameras]
        owner.install()
        owner.close()
        owner.close()
        self.assertEqual([data.name for data in cleaned], list(DEFAULT_NAMES) + ['group'])
        for index, camera in enumerate(cameras):
            self.assertEqual(camera._update_buffers_impl, old_methods[index])
            self.assertIs(camera._data._output, old_outputs[index])
            self.assertIsNone(camera._render_data)
            self.assertFalse(hasattr(camera, '_grouped_readout_owner'))
        self.assertTrue(owner.report()['closed'])
        with self.assertRaisesRegex(RuntimeError, 'closed'):
            owner.update(True, cameras[0])

    def test_incompatible_shared_settings_are_rejected_without_touching_products(self):
        for field, value in [('width', 640), ('height', 480), ('data_types', ['rgb', 'depth']),
                             ('background_color', (1, 0, 0)), ('update_period', .1), ('isp_cfg', object())]:
            with self.subTest(field=field):
                _, scene, cameras, _, _, cleaned, _ = setup_owner()
                setattr(cameras[2].cfg, field, value)
                with self.assertRaises(ValueError):
                    _compatible_cameras(scene, DEFAULT_NAMES)
                self.assertEqual(cleaned, [])
                self.assertTrue(all(camera._render_data is not None for camera in cameras))

    def test_measured_mixed_intrinsics_failure_is_rejected_before_render_product_mutation(self):
        owner, scene, cameras, _, _, cleaned, _ = setup_owner(mixed_intrinsics=True)
        original_products = [camera._render_data for camera in cameras]
        original_outputs = [camera._data._output for camera in cameras]
        with self.assertRaisesRegex(ValueError, 'does not support mixed intrinsics'):
            _compatible_cameras(scene, DEFAULT_NAMES)
        self.assertEqual(cleaned, [])
        self.assertFalse(owner._installed)
        for index, camera in enumerate(cameras):
            self.assertIs(camera._render_data, original_products[index])
            self.assertIs(camera._data._output, original_outputs[index])
            self.assertFalse(hasattr(camera, '_grouped_readout_owner'))

    def test_refresh_reuses_spec_buffers_and_views_but_reads_same_generation_again(self):
        owner, _, cameras, _, outputs, cleaned, reads = setup_owner()
        owner.install()
        cameras[0]._update_buffers_impl(True)
        original_product, original_spec = owner.render_data, owner.render_data.spec
        originals = [(camera._data, camera._view, camera._data.intrinsics) for camera in cameras]
        created, bound_outputs = [], []

        def cleanup(data):
            cleaned.append(data)
            if data is not None:
                data.spec = None  # Actual native cleanup invalidates this field.

        def create(spec):
            self.assertIs(spec, original_spec)
            fresh = SimpleNamespace(name='fresh', spec=spec,
                                    render_product=SimpleNamespace(path='/RP/fresh'))
            created.append(fresh)
            return fresh

        owner.renderer.cleanup = cleanup
        owner.renderer.create_render_data = create
        owner.renderer.set_outputs = lambda data, buffers: bound_outputs.append((data, buffers))
        result = owner.refresh_render_product()
        self.assertTrue(result['passed'])
        self.assertIsNone(original_product.spec)
        self.assertEqual(owner.report()['render_product_history'], ['/RP/group', '/RP/fresh'])
        self.assertEqual(bound_outputs, [(created[0], owner.camera_data.output)])
        for index, camera in enumerate(cameras):
            self.assertIs(camera._data, originals[index][0])
            self.assertIs(camera._view, originals[index][1])
            self.assertEqual(camera._data.intrinsics, originals[index][2])
            self.assertIs(camera._data._output, outputs[index])
        # A new RP must be read even if physical time/render_generation has not
        # advanced since its predecessor's last read.
        cameras[2]._update_buffers_impl(True)
        self.assertEqual(reads, [1, 1])
        owner.close()
        self.assertEqual([data.name for data in cleaned], list(DEFAULT_NAMES) + ['group', 'fresh'])

    def test_failed_refresh_cannot_reuse_stale_pixels_and_closes_new_product(self):
        owner, _, cameras, _, _, cleaned, _ = setup_owner()
        owner.install()
        cameras[0]._update_buffers_impl(True)
        fresh = SimpleNamespace(name='fresh', spec=owner._render_spec,
                                render_product=SimpleNamespace(path='/RP/fresh'))
        owner.renderer.create_render_data = lambda spec: fresh

        def fail_bind(data, outputs):
            raise RuntimeError('buffer binding failed')

        owner.renderer.set_outputs = fail_bind
        with self.assertRaisesRegex(RuntimeError, 'buffer binding failed'):
            owner.refresh_render_product()
        self.assertIsNone(owner.render_data)
        self.assertIsNone(owner._last_key)
        self.assertFalse(owner.report()['refresh_events'][0]['passed'])
        self.assertEqual([data.name for data in cleaned], list(DEFAULT_NAMES) + ['group', 'fresh'])
        with self.assertRaisesRegex(RuntimeError, 'unavailable'):
            cameras[1]._update_buffers_impl(True)
        owner.close()


if __name__ == '__main__':
    unittest.main()
