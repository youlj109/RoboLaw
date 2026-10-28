"""CPU checks for raw diagnostic traces; no Isaac Sim or renderer is started."""
import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import h5py
import numpy as np


_spec = importlib.util.spec_from_file_location(
    'lighting_probe', Path(__file__).resolve().parents[1] / 'scripts/probe_vectorized_lighting.py')
probe = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(probe)


class AaOverrideTests(unittest.TestCase):
    def settings(self):
        values = {key: 0 for key in probe.RENDERER_SETTING_PATHS}
        values.update({'/rtx/rendermode': 'RealTimePathTracing', '/rtx/post/aa/op': 3,
                       '/rtx/directLighting/sampledLighting/samplesPerPixel': 1})
        return values, SimpleNamespace(get=values.get, set=Mock(side_effect=values.__setitem__))

    def test_default_does_not_write_any_renderer_settings(self):
        values, settings = self.settings()
        original = values.copy()
        result = probe.apply_aa_override(settings, None)
        settings.set.assert_not_called()
        self.assertEqual(values, original)
        self.assertEqual(result['before'], result['after'])
        self.assertFalse(result['set_attempted'])

    def test_override_changes_only_aa_and_later_readback_detects_overwrite(self):
        values, settings = self.settings()
        result = probe.apply_aa_override(settings, 4)
        settings.set.assert_called_once_with('/rtx/post/aa/op', 4)
        self.assertEqual(result['requested'], 4)
        self.assertEqual(result['before']['/rtx/post/aa/op'], 3)
        self.assertEqual(result['after']['/rtx/post/aa/op'], 4)
        self.assertTrue(result['requested_value_observed_after_set'])
        for key in probe.RENDERER_SETTING_PATHS:
            if key != '/rtx/post/aa/op':
                self.assertEqual(result['before'][key], result['after'][key])
        values['/rtx/post/aa/op'] = 3
        self.assertEqual(probe.renderer_settings_readback(settings)['/rtx/post/aa/op'], 3)
        self.assertEqual(result['after']['/rtx/post/aa/op'], 4)


class StandaloneLightTests(unittest.TestCase):
    def make_scene(self):
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux

        stage = Usd.Stage.CreateInMemory()
        globals_by_name = {}
        for name, cls in [('Light', UsdLux.DomeLight), ('DirectionalLight', UsdLux.DistantLight)]:
            light = cls.Define(stage, f'/World/{name}')
            light.CreateIntensityAttr(1000.)
            light.CreateColorAttr(Gf.Vec3f(.9, .8, .7))
            light.CreateEnableColorTemperatureAttr(True)
            light.CreateColorTemperatureAttr(5100.)
            UsdGeom.Xformable(light.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(20, 30, 40))
            attr = light.GetPrim().CreateAttribute('diagnostic:sampled', Sdf.ValueTypeNames.Double, custom=True)
            attr.Set(3.)
            attr.Set(4., 1.)
            attr.Set(5., 2.)
            attr.SetCustomData({'purpose': 'preserve composed attribute metadata'})
            light.GetPrim().CreateAttribute('diagnostic:connected', Sdf.ValueTypeNames.Float).SetConnections(
                [Sdf.Path('/World/driver.outputs:value')])
            globals_by_name[name] = light
        stage.DefinePrim('/World/envs/env_0', 'Xform')
        UsdGeom.Cube.Define(stage, '/World/envs/env_0/mustard/visual')
        UsdGeom.Cube.Define(stage, '/World/envs/env_0/robot/visual')
        second = stage.DefinePrim('/World/envs/env_1', 'Xform')
        second.GetInherits().AddInherit('/World/envs/env_0')
        appearance = {'lighting_mode': 'linked_experimental', 'lights': [],
                      'environments': [{'requested_lights': {f'/World/{name}': {} for name in globals_by_name}}]}
        root_before = stage.GetRootLayer().ExportToString()
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            for env in range(2):
                for name in globals_by_name:
                    path = f'/World/envs/env_{env}/EpisodeLights/{name}'
                    prim = stage.DefinePrim(path)
                    prim.GetReferences().AddInternalReference(f'/World/{name}')
                    UsdLux.LightAPI(prim).CreateIntensityAttr(700. + 100 * env)
                    for key in ('lightLink', 'shadowLink'):
                        collection = Usd.CollectionAPI.Apply(prim, key)
                        collection.CreateIncludeRootAttr(False)
                        collection.CreateExpansionRuleAttr(Usd.Tokens.expandPrims)
                        collection.CreateIncludesRel().SetTargets([Sdf.Path(f'/World/envs/env_{env}')])
                    appearance['lights'].append({'env_index': env, 'path': path})
            for light in globals_by_name.values():
                light.CreateIntensityAttr(0.)
        return stage, appearance, root_before, globals_by_name

    def test_composed_values_and_links_survive_reference_removal_and_template_deactivation(self):
        from pxr import Gf, Usd, UsdLux

        stage, appearance, root_before, globals_by_name = self.make_scene()
        result = probe.make_linked_lights_standalone(stage, appearance)
        json.dumps(result)  # Metadata is independently inspectable without pxr objects.
        self.assertEqual(stage.GetRootLayer().ExportToString(), root_before)
        self.assertTrue(all(not row['active'] for row in result['global_lights_after']))
        self.assertEqual(len(result['local_lights_after']), 4)
        for before, after in zip(result['local_lights_before'], result['local_lights_after']):
            self.assertTrue(before['has_authored_references'])
            self.assertTrue(before['composed_reference_items'])
            self.assertEqual(after['composed_reference_items'], [])
            self.assertTrue(after['active'])
            self.assertEqual(before['type'], after['type'])
            self.assertEqual(before['attributes'], after['attributes'])
            self.assertEqual(before['relationships'], after['relationships'])
            prim = stage.GetPrimAtPath(after['path'])
            self.assertTrue(prim.HasAPI(UsdLux.LightAPI))
            self.assertEqual(prim.GetAttribute('diagnostic:sampled').Get(2.), 5.)
        # A later change to the inactive template cannot leak into flattened lights.
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            for light in globals_by_name.values():
                light.CreateColorAttr(Gf.Vec3f(1, 0, 0))
        for row in appearance['lights']:
            color = stage.GetPrimAtPath(row['path']).GetAttribute('inputs:color').Get()
            np.testing.assert_allclose(color, [.9, .8, .7])

    def test_membership_expansion_does_not_inherit_env_0_targets_into_env_1(self):
        stage, appearance, _, _ = self.make_scene()
        probe.make_linked_lights_standalone(stage, appearance)
        report = probe.light_collection_membership(stage, ['/World/envs/env_0', '/World/envs/env_1'])
        self.assertIn('/World/envs/env_0', report['environment_roots'][1]['direct_inherits'])
        self.assertEqual(len(report['lights']), 4)
        for row in report['lights']:
            own_root = row['path'].split('/EpisodeLights')[0]
            for collection in row['collections'].values():
                for root, membership in collection.items():
                    self.assertEqual(membership['root_included'], root == own_root)
                    self.assertTrue(membership['geometry'])
                    self.assertEqual(set(membership['geometry'].values()), {root == own_root})

    def test_dome_restore_and_distant_interventions_change_only_one_local_light(self):
        stage, appearance, _, _ = self.make_scene()
        probe.make_linked_lights_standalone(stage, appearance)
        baselines = probe.linked_light_parameters(stage, appearance)
        dome = probe.set_light_type_intervention(stage, baselines, 'DomeLight')
        self.assertEqual(len(dome['lights']), 1)
        self.assertEqual(dome['lights'][0]['path'], '/World/envs/env_0/EpisodeLights/Light')
        self.assertEqual(dome['lights'][0]['after']['intensity'], 1400.)
        self.assertEqual(dome['lights'][0]['after']['temperature_k'], 2000.)
        restored = probe.set_light_type_intervention(stage, baselines)
        self.assertEqual(len(restored['lights']), 1)
        self.assertEqual(probe.linked_light_parameters(stage, appearance), baselines)
        self.assertEqual(probe.set_light_type_intervention(stage, baselines)['lights'], [])
        distant = probe.set_light_type_intervention(stage, baselines, 'DistantLight')
        self.assertEqual(len(distant['lights']), 1)
        self.assertEqual(distant['lights'][0]['path'], '/World/envs/env_0/EpisodeLights/DirectionalLight')
        current = probe.linked_light_parameters(stage, appearance)
        for original, actual in zip(baselines, current):
            if actual['path'] != distant['lights'][0]['path']:
                self.assertEqual(original, actual)


class StaticTraceTests(unittest.TestCase):
    def make_sources(self, *, physics_at=None, fresh_render=True):
        sim = SimpleNamespace(render_generation=0, physics_steps=64)

        def render():
            if fresh_render:
                sim.render_generation += 1
            if sim.render_generation == physics_at:
                sim.physics_steps += 1

        sim.render = render
        sim.get_physics_step_count = lambda: sim.physics_steps
        writer = SimpleNamespace(writers={0: object(), 1: object()},
                                 names=('external_camera', 'external_camera_2', 'wrist_camera'))

        def snapshot(**kwargs):
            self.assertTrue(kwargs['small_readback'])
            return {(index, name): np.full((90, 160, 3),
                    128 + index * 10 + (20 if sim.render_generation == 3 else 0), dtype=np.float32)
                    for index in writer.writers for name in writer.names}

        writer._warmup_snapshot = snapshot
        return writer, sim

    def test_trace_preserves_transient_and_individual_environment_frames(self):
        writer, sim = self.make_sources()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'trace.h5'
            with patch('kitchen_safety.vectorized_dataset._ensure_render_update') as ensure:
                result = probe.record_static_rgb_trace(writer, sim, path, 4)
            self.assertEqual(ensure.call_count, 4)
            self.assertEqual(result['frames'], 4)
            self.assertFalse(result['physics_advanced'])
            self.assertFalse(result['convergence_checked'])
            self.assertIsNone(result['error'])
            with h5py.File(path, 'r') as handle:
                self.assertEqual(handle.attrs['frames_written'], 4)
                self.assertEqual(handle.attrs['temporal_filter'], 'none')
                np.testing.assert_array_equal(handle['render_generation'][:], [1, 2, 3, 4])
                for name in writer.names:
                    self.assertEqual(handle[name].shape, (4, 2, 90, 160, 3))
                    np.testing.assert_array_equal(handle[name][:, 0, 0, 0, 0], [128, 128, 148, 128])
                    np.testing.assert_array_equal(handle[name][:, 1, 0, 0, 0], [138, 138, 158, 138])

    def test_trace_records_partial_count_when_physics_advances(self):
        writer, sim = self.make_sources(physics_at=3)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'trace.h5'
            with patch('kitchen_safety.vectorized_dataset._ensure_render_update'):
                result = probe.record_static_rgb_trace(writer, sim, path, 4)
            self.assertTrue(result['physics_advanced'])
            self.assertIn('Physics advanced', result['error'])
            self.assertEqual(result['frames'], 2)
            with h5py.File(path, 'r') as handle:
                self.assertEqual(handle.attrs['frames_written'], 2)

    def test_trace_rejects_stale_render_generation(self):
        writer, sim = self.make_sources(fresh_render=False)
        with tempfile.TemporaryDirectory() as directory:
            with patch('kitchen_safety.vectorized_dataset._ensure_render_update') as ensure:
                result = probe.record_static_rgb_trace(writer, sim, Path(directory) / 'trace.h5', 4)
            self.assertEqual(result['frames'], 0)
            self.assertIn('did not advance', result['error'])
            ensure.assert_not_called()


if __name__ == '__main__':
    unittest.main()
