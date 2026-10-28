"""CPU checks for provenance/explicit options; these never grant quota."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from kitchen_safety.render_worker import RenderWorker
from scripts.collect_kitchen_vectorized import collection_inputs, resolved_collector_options
from scripts.render_recorded_episode import render_provenance
from scripts.run_nonhuman_augmentation import build_stages


class ProductionAdapterTests(unittest.TestCase):
    def test_fixed_regression_keeps_single_backend_while_random_stages_receive_native_options(self):
        args = SimpleNamespace(ids=['property_tall_glass'], seed=42, max_attempts=100,
            randomization_config='configs/augmentation_nonhuman10.json', pilot_per_id=2,
            episodes_per_id=20, preview_per_id=3, execution_backend='vectorized', num_envs=4,
            validation_workers=2, pipeline=True, device='cpu', physics_threads=0, torch_threads=1)
        stages = build_stages(args, Path('/tmp/augmentation'))
        for flag in ('--execution-backend', '--num-envs', '--validation-workers', '--pipeline'):
            self.assertNotIn(flag, stages[0]['command'])
            self.assertTrue(all(flag in stage['command'] for stage in stages[1:]))
        for stage in stages:
            command = stage['command']
            self.assertEqual(command[command.index('--device')+1], 'cpu')
            self.assertEqual(command[command.index('--physics-threads')+1], '0')
            self.assertEqual(command[command.index('--torch-threads')+1], '1')

    def test_explicit_grasp_and_false_zero_override_presets_without_losing_hard_options(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            overrides = {'grasp_tilt_deg': 20., 'grasp_yaw_deg': 0.,
                         'grasp_offset_z_m': 0., 'supported_release': False}
            (root / 'overrides.json').write_text(json.dumps(overrides))
            jobs = [{'env_index': 0, 'attempt_path': 'episodes/wine/attempt_007', 'attempt': 7},
                    {'env_index': 1, 'attempt_path': 'episodes/wine/attempt_008', 'attempt': 8}]
            (root / 'jobs.json').write_text(json.dumps(jobs))
            explicit, mapping = collection_inputs([{}, {}], root/'overrides.json', root/'jobs.json')
            self.assertEqual(mapping, jobs)
            default = resolved_collector_options('property_wine_glass', {})
            resolved = resolved_collector_options('property_wine_glass', explicit)
            for key, value in overrides.items():
                self.assertEqual(resolved[key], value)
            for key in set(default)-set(overrides):
                self.assertEqual(resolved[key], default[key])
            self.assertEqual(collection_inputs([{}]), ({}, [{'env_index': 0}]))
            (root / 'jobs.json').write_text(json.dumps(jobs[::-1]))
            with self.assertRaisesRegex(ValueError, 'env_index'):
                collection_inputs([{}, {}], jobs_path=root/'jobs.json')
            (root / 'overrides.json').write_text('{"unknown_option": 1}')
            with self.assertRaisesRegex(ValueError, 'Unknown collector'):
                collection_inputs([{}], overrides_path=root/'overrides.json')

    def test_production_render_provenance_retains_actual_source_and_stays_unaccepted(self):
        source = {'collection_purpose': 'production', 'collection_source': 'batch-A',
                  'collection_job': {'env_index': 1, 'attempt': 8}, 'success': True}
        args = SimpleNamespace(collection_purpose='production', collection_source='batch-A')
        result = render_provenance(args, source)
        self.assertEqual(result['collection_purpose'], 'production')
        self.assertEqual(result['source_collection_job'], source['collection_job'])
        self.assertFalse(result['production_quota_eligible'])
        self.assertEqual(result['acceptance_status'], 'pending_batch_verification')
        self.assertNotIn('success', result)
        self.assertEqual(render_provenance(SimpleNamespace(), {})['acceptance_status'], 'diagnostic')

    def test_worker_forwards_production_origin_without_altering_renderer_or_acceptance(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            worker = RenderWorker(cwd=root, directory=root/'worker', collection_purpose='production',
                                  collection_source='episodes/run-A', refresh_render_products=True)
            child = Mock(pid=42, wait=Mock(return_value=0))
            child.stdin = None
            with patch('kitchen_safety.render_worker.subprocess.Popen', return_value=child) as popen:
                worker._start()
                worker.close()
            command = popen.call_args.args[0]
            self.assertEqual(command[command.index('--collection-purpose')+1], 'production')
            self.assertEqual(command[command.index('--collection-source')+1], 'episodes/run-A')
            self.assertIn('--refresh-render-products', command)
            self.assertNotIn('--grouped-camera', command)
            self.assertTrue(worker.report()['process_reaped'])


if __name__ == '__main__':
    unittest.main()
