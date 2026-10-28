"""CPU checks for measured-state provenance and physics-free render cadence."""
import json
from pathlib import Path
import tempfile
import unittest

import h5py
import numpy as np

from scripts.probe_recorded_state_render import (
    load_source, pose_error, pure_render, select_indices, translated_pose,
)


class RecordedStateRenderProbeTest(unittest.TestCase):
    def test_selection_preserves_actual_preaction_indices(self):
        phases = ['settle', 'close', 'lift', 'lift', 'above_bowl', 'above_bowl', 'above_bowl', 'release']
        self.assertEqual(select_indices(phases), [0, 2, 5])
        self.assertEqual(select_indices(phases, [6, 1, 0]), [6, 1, 0])
        with self.assertRaises(ValueError):
            select_indices(phases, [0, 0, 5])

    def test_local_origin_is_not_added_twice_and_quaternion_sign_is_equivalent(self):
        state = np.array([1., 2., 3., 0., 0., 0., 1., 7., 8., 9., 10., 11., 12.])
        restored = translated_pose(state, coordinate_frame='environment_local',
                                   source_origin=[45., -30., 0.], replay_origin=[2., 3., 0.])
        np.testing.assert_array_equal(restored[:3], [3., 5., 3.])
        np.testing.assert_array_equal(restored[3:], state[3:])
        world = state.copy()
        world[:3] += [45., -30., 0.]
        np.testing.assert_array_equal(translated_pose(world, coordinate_frame='world',
            source_origin=[45., -30., 0.], replay_origin=[2., 3., 0.]), restored)
        opposite = restored[:7].copy()
        opposite[3:] *= -1
        self.assertEqual(pose_error(opposite, restored[:7])['rotation_error_rad'], 0.)

    def test_render_pumps_each_generation_without_calling_step(self):
        class Sim:
            render_generation = 4
            steps = 10

            def get_physics_step_count(self):
                return self.steps

            def step(self):
                raise AssertionError('No physics step is allowed')

            def render(self):
                self.render_generation += 1

        sim, seen = Sim(), []
        def pump(*, force):
            self.assertTrue(force)
            seen.append(sim.render_generation)
        report = pure_render(sim, pump, 3)
        self.assertEqual(seen, [5, 6, 7])
        self.assertFalse(report['physics_advanced'])
        self.assertEqual(report['physics_step_count_after'], 10)
        def bad_pump(*, force):
            sim.steps += 1
        with self.assertRaisesRegex(RuntimeError, 'Physics advanced'):
            pure_render(sim, bad_pump, 1)

    def test_loader_reads_obs_not_next_obs_or_planned_actions(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            (path / 'result.json').write_text(json.dumps({'success': True}))
            (path / 'episode_spec.json').write_text('{}')
            (path / 'cameras.json').write_text(json.dumps({'camera': {}}))
            with h5py.File(path / 'trajectory.h5', 'w') as handle:
                handle.attrs.update(dt=1 / 15, joint_names='["joint"]', robot_body_names='["body"]',
                                    coordinate_frame='environment_local', environment_origin_world_m=[10., 0., 0.])
                handle.create_dataset('phase', data=np.asarray(['settle', 'close', 'lift', 'above_bowl', 'above_bowl', 'release'], dtype=h5py.string_dtype()))
                handle.create_dataset('time', data=np.arange(6) / 15)
                for key, width in [('q', 1), ('dq', 1), ('mustard', 13), ('bowl', 13)]:
                    values = np.repeat(np.arange(6)[:, None], width, axis=1)
                    handle.create_dataset('obs/' + key, data=values)
                    handle.create_dataset('next_obs/' + key, data=values + 100)
                handle.create_dataset('obs/robot_body_pose_w', data=np.zeros((6, 1, 7)))
                handle.create_dataset('action', data=np.full((6, 8), 999.))
            with h5py.File(path / 'images.h5', 'w') as handle:
                group = handle.create_group('camera')
                group.create_dataset('position_world', data=np.zeros((6, 3)))
                group.create_dataset('quaternion_world_ros_xyzw', data=np.tile([0., 0., 0., 1.], (6, 1)))
                group.create_dataset('jpeg', data=np.zeros((6, 3), dtype=np.uint8))
            source = load_source(path)
            self.assertEqual([row['source_obs_index'] for row in source['frames']], [0, 2, 4])
            self.assertEqual([row['state']['q'][0] for row in source['frames']], [0, 2, 4])
            self.assertEqual(source['coordinate_frame'], 'environment_local')


if __name__ == '__main__':
    unittest.main()
