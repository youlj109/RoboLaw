import json
from pathlib import Path
import tempfile
import unittest

from scripts.summarize_vectorized_sweep import summarize


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))


class SweepSummaryTests(unittest.TestCase):
    def test_actual_render_settings_keep_different_quality_in_separate_groups(self):
        with tempfile.TemporaryDirectory() as tmp:
            roots = [Path(tmp) / str(i) for i in range(2)]
            for root, spp in zip(roots, [1, 4]):
                write(root / 'benchmark.json', dict(num_envs=4, completed=True,
                    record_rgb=True, source_episode='source', requested_steps=180,
                    executed_control_steps=180, render_spp_requested=None,
                    renderer_settings={'/rtx/directLighting/sampledLighting/samplesPerPixel': spp},
                    timings_seconds={'replay_loop_s': 40.}))
            result = summarize(roots)
            self.assertEqual([row['render_spp'] for row in result['rows']], [1, 4])
            self.assertEqual(len(result['rgb_replay_comparisons']), 2)

    def test_rgb_loop_rate_excludes_no_rgb_and_does_not_claim_short_success(self):
        with tempfile.TemporaryDirectory() as tmp:
            roots = [Path(tmp) / name for name in ['rgb4', 'no_rgb8', 'rgb8']]
            for root, n, rgb, wall in zip(roots, [4, 8, 8], [True, False, True], [20., 2., 45.]):
                write(root / 'benchmark.json', dict(num_envs=n, completed=True,
                    record_rgb=rgb, source_episode='same_source', requested_steps=30,
                    executed_control_steps=30, full_source_replay=False,
                    diagnostic_pick_place_success_count=n,
                    timings_seconds={'replay_loop_s': wall, 'physics_step_call_s': 8.,
                                     'total_before_close_s': wall+4.},
                    camera_timings_seconds={'render_capture_s': 9. if rgb else 0.,
                        'jpeg_write_s': 2. if rgb else 0., 'capture_ticks': 30 if rgb else 0}))
            result = summarize(roots)
            first = result['rows'][0]
            self.assertEqual(first['action_samples_per_loop_second'], 6.)
            self.assertEqual(first['physics_plus_rgb_write_wall_s'], 19.)
            self.assertIsNone(first['full_replay_diagnostic_successes'])
            self.assertIsNone(first['collector_successes'])
            self.assertEqual(result['rgb_replay_comparisons'][0]['best_measured_num_envs'], 4)
            self.assertEqual(len(result['rgb_replay_comparisons'][0]['ranked_directories']), 2)

    def test_collection_requires_actual_rgb_validation_and_pool_wall_is_preserved(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write(root / 'batch.json', dict(num_envs=2, status='completed',
                timings_seconds={'physics_execution_s': 10., 'camera_s': 6.,
                                 'deferred_validation_s': 3., 'total_before_close_s': 30.},
                camera_timings_seconds={'render_capture_s': 4., 'jpeg_write_s': 2., 'capture_ticks': 100}))
            for i, count in enumerate([100, 50]):
                write(root / f'env_{i:03d}/result.json', dict(success=True, steps=count))
                (root / f'env_{i:03d}/images.h5').touch()
            write(root / 'validation.json', [
                {'path': '/workspace/run/env_000', 'passed': True,
                 'cameras': {'a': {}, 'b': {}, 'c': {}}, 'elapsed_seconds': 2.},
                {'path': '/workspace/run/env_001', 'passed': False, 'elapsed_seconds': 1.}])
            row = summarize([root])['rows'][0]
            self.assertEqual(row['collector_successes'], 2)
            self.assertEqual(row['validated_rgb_collector_successes'], 1)
            self.assertEqual(row['validated_rgb_collector_successes_per_hour_before_close'], 120.)
            self.assertEqual(row['physics_plus_rgb_write_wall_s'], 16.)
            self.assertEqual(row['active_slot_fraction'], .75)
            self.assertEqual(row['timings_seconds']['deferred_validation_s'], 3.)
            self.assertIsNone(row['loop_wall_s'])
            self.assertEqual(row['validation_elapsed_seconds_sum'], 3.)


if __name__ == '__main__':
    unittest.main()
