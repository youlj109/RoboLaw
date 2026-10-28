"""Exercise the environment task factory with CPU doubles for Arena assets.

The real factory body and configuration class execute; only imports of Kit
assets are replaced. This does not simulate physics or certify a trajectory.
"""
import ast
import copy
from dataclasses import dataclass, replace
import math
from pathlib import Path
from types import SimpleNamespace
from typing import Optional
import unittest
from unittest.mock import Mock


ROOT = Path(__file__).resolve().parents[1]


class CollectionEpisodeLengthTests(unittest.TestCase):
    def factory(self):
        tree = ast.parse((ROOT / 'kitchen_safety/environment.py').read_text())
        cfg_node = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                        and n.name == 'KitchenSafetyCfg')
        factory_node = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                            and n.name == 'KitchenSafetyEnvironment')
        build = copy.deepcopy(next(n for n in factory_node.body
                                   if isinstance(n, ast.FunctionDef) and n.name == 'build'))
        build.body = [n for n in build.body if not isinstance(n, (ast.Import, ast.ImportFrom))]

        @dataclass
        class BaseCfg:
            enable_cameras: bool = False

        @dataclass
        class Prim:
            robot_usd_path: str = ''
            stand_usd_path: str = ''

        class Pose:
            def __init__(self, *args):
                self.args = args
            @classmethod
            def identity(cls):
                return cls()

        droid = SimpleNamespace(_DROID_ROBOT_PRIM=Prim(), _DROID_STAND_PRIM=Prim(),
            DroidAbsoluteJointPositionEmbodiment=lambda **kwargs: SimpleNamespace(
                event_config=SimpleNamespace(randomize_franka_joint_state=SimpleNamespace(params={}))))
        task = Mock(side_effect=lambda *args, **kwargs: SimpleNamespace(**kwargs))
        layout = dict(task='pick and place', mustard_position=[1, 0, 1], bowl_position=[2, 0, 1],
                      robot_yaw_deg=0, robot_position=[0, 0, 0], stand_height=1,
                      camera_eye=[0, 0, 2], camera_target=[1, 0, 1])
        namespace = dict(dataclass=dataclass, ArenaEnvironmentCfg=BaseCfg, Optional=Optional,
            math=math, copy=copy, ROOT=ROOT, replace=replace, Pose=Pose, droid=droid,
            Background=lambda **kwargs: SimpleNamespace(**kwargs),
            Object=lambda **kwargs: SimpleNamespace(**kwargs),
            ObjectType=SimpleNamespace(BASE='base', RIGID='rigid'),
            PickAndPlaceTask=task, Scene=lambda **kwargs: SimpleNamespace(**kwargs),
            IsaacLabArenaEnvironment=lambda **kwargs: SimpleNamespace(**kwargs),
            ViewerCfg=lambda **kwargs: SimpleNamespace(**kwargs),
            resolve_layout=lambda *args: copy.deepcopy(layout))
        exec(compile(ast.Module(body=[cfg_node, build], type_ignores=[]),
                     str(ROOT / 'kitchen_safety/environment.py'), 'exec'), namespace)
        owner = SimpleNamespace(name='kitchen_safety', asset_registry=SimpleNamespace(
            get_asset_by_name=lambda name: lambda: name))
        return namespace['KitchenSafetyCfg'], lambda cfg: namespace['build'](owner, cfg), task

    def test_default_and_explicit_timeout_reach_existing_task_unchanged(self):
        cfg_type, build, task = self.factory()
        default = build(cfg_type(variant='base'))
        extended = build(cfg_type(variant='base', episode_length_s=180.))
        self.assertEqual(default.task.episode_length_s, 120.)
        self.assertEqual(extended.task.episode_length_s, 180.)
        self.assertEqual(task.call_count, 2)
        # Initialization takes 8 env.step calls; the action-rate budget remains
        # 2400 at 15 Hz. Explicit 180 s allows all of these original steps.
        dt = 1/15
        self.assertEqual(math.ceil(default.task.episode_length_s / dt)-8, 1792)
        self.assertGreater(math.ceil(extended.task.episode_length_s / dt), 2400+8)

    def test_invalid_timeout_never_constructs_the_task(self):
        cfg_type, build, task = self.factory()
        for value in (0., -1., float('nan'), float('inf')):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'finite and positive'):
                build(cfg_type(variant='base', episode_length_s=value))
        task.assert_not_called()


if __name__ == '__main__':
    unittest.main()
