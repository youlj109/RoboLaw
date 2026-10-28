"""CPU checks for the RGB replay benchmark; no simulation or renderer startup."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import h5py
import numpy as np


PATH = Path(__file__).resolve().parents[1] / 'scripts/benchmark_collection_vectorized.py'
SPEC = importlib.util.spec_from_file_location('vectorized_rgb_benchmark', PATH)
benchmark = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(benchmark)


class VectorizedRgbBenchmarkTest(unittest.TestCase):
    def test_warmup_advances_render_generation_not_physics(self):
        class Sim:
            render_generation = 3
            physics = 64

            def get_physics_step_count(self):
                return self.physics

            def render(self):
                self.render_generation += 1

        sim = Sim()
        generations = []

        def update(*, force):
            self.assertTrue(force)
            generations.append(sim.render_generation)

        report = benchmark.fixed_render_warmup(sim, update)
        self.assertEqual(generations, list(range(4, 132)))
        self.assertEqual(report['render_frames'], 128)
        self.assertEqual(report['physics_step_count_after'], 64)
        self.assertFalse(report['appearance_convergence_tested'])
        self.assertFalse(report['randomization_applied'])

        def illegal_update(*, force):
            sim.physics += 1

        with self.assertRaisesRegex(RuntimeError, 'Physics advanced'):
            benchmark.fixed_render_warmup(sim, illegal_update)

    def test_each_env_trajectory_keeps_pre_action_chain_and_local_frame(self):
        count = 3
        origin = np.array([10., -20., 0.])
        arrays = {'q': np.zeros((count + 1, 2, 13)), 'dq': np.zeros((count + 1, 2, 13)),
                  'mustard': np.zeros((count + 1, 2, 13)), 'bowl': np.zeros((count + 1, 2, 13)),
                  'hazard': np.zeros((count + 1, 2, 13)),
                  'robot_body_pose_w': np.zeros((count + 1, 2, 2, 7)),
                  'robot_contact_forces': np.zeros((count + 1, 2, 2, 3))}
        for name in ('mustard', 'bowl', 'hazard'):
            arrays[name][:, 1, :3] = origin + [1, 2, 3]
            arrays[name][:, :, 6] = 1
        arrays['robot_body_pose_w'][:, :, :, 6] = 1
        arrays['robot_body_pose_w'][:, 1, :, :3] = origin + [4, 5, 6]
        arrays['q'][:, 1, 0] = np.arange(count + 1)
        actions = np.zeros((count, 8), np.float32)
        actions[:, 0] = [5, 6, 7]
        untouched_world = arrays['mustard'].copy()
        with tempfile.TemporaryDirectory() as tmp:
            benchmark.write_environment_trajectory(tmp, arrays, 1, origin, actions,
                ['pregrasp', 'grasp', 'lift'], 1/15, ['j'] * 13, ['base_link', 'panda_link0'], ['base_link'])
            with h5py.File(Path(tmp) / 'trajectory.h5') as f:
                np.testing.assert_array_equal(f['obs/q'][:, 0], [0, 1, 2])
                np.testing.assert_array_equal(f['next_obs/q'][:, 0], [1, 2, 3])
                np.testing.assert_array_equal(f['action'][:, 0], [5, 6, 7])
                for name in f['obs']:
                    np.testing.assert_array_equal(f['obs'][name][1:], f['next_obs'][name][:-1])
                np.testing.assert_allclose(f['obs/mustard'][0, :3], [1, 2, 3])
                np.testing.assert_allclose(f['obs/tcp'][0], [4.131, 5, 6])
                self.assertEqual(f['obs/hazard_pose'].shape, (count, 7))
                self.assertEqual(f['obs/robot_body_pose_w'].shape, (count, 2, 7))
                self.assertEqual(json.loads(f.attrs['robot_body_names']), ['base_link', 'panda_link0'])
        np.testing.assert_array_equal(arrays['mustard'], untouched_world)


if __name__ == '__main__':
    unittest.main()
