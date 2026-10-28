"""CPU-only checks for optional complete render-state recording."""
import argparse
import ast
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np


PATH = Path(__file__).resolve().parents[1] / 'scripts/collect_kitchen_vectorized.py'
SPEC = importlib.util.spec_from_file_location('vectorized_render_state_collector', PATH)
collector = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(collector)


class RenderStateRecordingTest(unittest.TestCase):
    def test_config_serialization_uses_authored_optics_without_sensor_access(self):
        camera = SimpleNamespace(
            prim_path='{ENV_REGEX_NS}/Robot/base_link/wrist_camera', width=1280, height=720,
            update_period=0., data_types=['rgb'],
            offset=SimpleNamespace(pos=(.01, -.03, -.07), rot=(0., 0., 0., 1.), convention='opengl'),
            spawn=SimpleNamespace(focal_length=2.8, focus_distance=28., horizontal_aperture=5.376,
                                  vertical_aperture=3.024, horizontal_aperture_offset=0.,
                                  vertical_aperture_offset=0., clipping_range=(.01, 1000.),
                                  projection_type='pinhole', f_stop=0.))
        rig = SimpleNamespace(wrist_camera=camera, camera_names=lambda: ['wrist_camera'])
        saved = json.loads(json.dumps(collector.render_camera_config(rig)))['wrist_camera']
        self.assertEqual(saved['spawn']['focal_length'], 2.8)
        self.assertEqual(saved['spawn']['horizontal_aperture'], 5.376)
        self.assertEqual(saved['spawn']['vertical_aperture'], 3.024)
        self.assertEqual(saved['spawn']['clipping_range'], [.01, 1000.])
        self.assertEqual(saved['offset']['position'], [.01, -.03, -.07])
        self.assertEqual(saved['offset']['convention'], 'opengl')
        self.assertNotIn('intrinsics', saved)  # No fabricated measured sensor K.
        self.assertEqual(camera.offset.pos, (.01, -.03, -.07))

    def test_capture_keeps_local_kitchen_and_velocity_axes_and_copies_hazard(self):
        origins = np.array([[0., 0., 0.], [30., -30., 2.]])
        roots = np.zeros((2, 21, 13)); roots[:, :, :3] = [1., 2., 3.]
        kitchen_arrays = {
            'kitchen_root_state': roots, 'kitchen_joint_pos': np.zeros((2, 48)),
            'kitchen_joint_vel': np.ones((2, 48)), 'kitchen_link_pose': np.zeros((2, 87, 7)),
            'kitchen_link_velocity': np.ones((2, 87, 6)),
        }
        calls = []
        kitchen = SimpleNamespace(capture=lambda: calls.append(True) or kitchen_arrays)
        robot = SimpleNamespace(body_lin_vel_w=np.ones((2, 18, 3)),
                                body_ang_vel_w=np.full((2, 18, 3), 2.))
        hazard = np.tile([1., 2., 3., 0., 0., 0., 1., 4., 5., 6., 7., 8., 9.], (2, 1))
        hazard[:, :3] += origins
        original = hazard.copy()
        saved = collector.capture_render_observation(
            kitchen, robot, SimpleNamespace(root_state_w=hazard), origins,
            lambda value: np.asarray(value).copy())
        self.assertEqual(len(calls), 1)
        np.testing.assert_array_equal(saved['kitchen_root_state'], roots)
        np.testing.assert_array_equal(saved['hazard_state'][:, :3], [[1., 2., 3.]] * 2)
        np.testing.assert_array_equal(saved['hazard_state'][:, 3:], original[:, 3:])
        np.testing.assert_array_equal(hazard, original)
        self.assertEqual(saved['robot_body_velocity'].shape, (2, 18, 6))
        np.testing.assert_array_equal(saved['robot_body_velocity'][1, 0], [1., 1., 1., 2., 2., 2.])
        robot.body_lin_vel_w[:] = 9
        self.assertEqual(saved['robot_body_velocity'][0, 0, 0], 1.)

    def test_static_hazard_has_no_fabricated_dynamic_state(self):
        saved = collector.capture_render_observation(
            SimpleNamespace(capture=lambda: {}),
            SimpleNamespace(body_lin_vel_w=np.zeros((1, 18, 3)),
                            body_ang_vel_w=np.zeros((1, 18, 3))),
            None, np.zeros((1, 3)), lambda value: np.asarray(value).copy())
        self.assertEqual(set(saved), {'robot_body_velocity'})

    def test_actual_cli_keeps_render_state_opt_in_and_independent_of_rgb(self):
        # Extract the real parser additions and camera-enable assignment without
        # importing Arena or starting its SimulationAppContext.
        tree = ast.parse(PATH.read_text())
        main = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == 'main')
        additions = [node for node in main.body if isinstance(node, ast.Expr)
                     and isinstance(node.value, ast.Call) and node.value.args
                     and isinstance(node.value.args[0], ast.Constant)
                     and node.value.args[0].value in ('--record-rgb', '--record-render-state')]
        enable = next(node for node in main.body if isinstance(node, ast.Assign)
                      and any(isinstance(target, ast.Attribute) and target.attr == 'enable_cameras'
                              for target in node.targets))
        for argv, state_enabled, cameras_enabled in [([], False, False),
                (['--record-render-state'], True, False),
                (['--record-render-state', '--record-rgb'], True, True)]:
            with self.subTest(argv=argv):
                namespace = {'parser': argparse.ArgumentParser()}
                exec(compile(ast.Module(body=additions, type_ignores=[]), str(PATH), 'exec'), namespace)
                args = namespace['parser'].parse_args(argv)
                namespace['args'] = args
                exec(compile(ast.Module(body=[enable], type_ignores=[]), str(PATH), 'exec'), namespace)
                self.assertEqual(args.record_render_state, state_enabled)
                self.assertEqual(args.enable_cameras, cameras_enabled)


if __name__ == '__main__':
    unittest.main()
