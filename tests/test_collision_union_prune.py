"""CPU FCL checks for conservative whole-robot AABB candidate rejection."""
import unittest
from unittest.mock import patch

import fcl
import numpy as np

from kitchen_safety.collision_planner import CollisionWorld


def box(name, low, high):
    low, high = np.asarray(low, dtype=float), np.asarray(high, dtype=float)
    geometry = fcl.CollisionObject(fcl.Box(*(high - low)), fcl.Transform((low + high) / 2))
    return name, geometry, low, high


def world_with_boxes(objects, shapes, *, margin=.125, rules=None):
    world = CollisionWorld.__new__(CollisionWorld)
    world.fcl = fcl
    world.objects = objects
    world.low = np.array([x[2] for x in objects]).reshape(-1, 3)
    world.high = np.array([x[3] for x in objects]).reshape(-1, 3)
    world.clearances = dict(rules or {})
    world.execution_clearances = dict(rules or {})
    world.default_clearance = margin
    world.attachment = None
    world.last_collision = world.last_clearance = None
    world._pair_selection_key = None
    world._pair_selection_cache = {}
    world.shapes = lambda *args, **kwargs: shapes
    return world


def pairs(world, slack, ignore=(), obstacle_filter=None):
    return world._pairs(np.zeros(7), ignore, 'execution', None, None, obstacle_filter, max_slack=slack)


class UnionAabbPruneTests(unittest.TestCase):
    def assert_same_candidates(self, world, slack=0., ignore=(), obstacle_filter=None):
        # The inf branch is intentionally untouched. Filtering its sorted
        # output supplies an independent oracle for the finite fast path.
        unrestricted = pairs(world, float('inf'), ignore, obstacle_filter)
        expected = [entry for entry in unrestricted if entry[0] <= slack]
        self.assertEqual(pairs(world, slack, ignore, obstacle_filter), expected)
        return expected

    def test_far_columns_removed_without_changing_candidate_order_or_ties(self):
        shapes = [box(label, [-.5] * 3, [.5] * 3) for label in ('arm', 'finger')]
        objects = [box('/World/near_a', [.25] * 3, [.75] * 3),
                   box('/World/far', [10.] * 3, [11.] * 3),
                   box('/World/near_b', [.25] * 3, [.75] * 3)]
        world = world_with_boxes(objects, shapes)
        expected = self.assert_same_candidates(world)
        self.assertEqual([(p[1], p[2]) for p in expected], [
            ('arm', '/World/near_a'), ('arm', '/World/near_b'),
            ('finger', '/World/near_a'), ('finger', '/World/near_b')])
        original_norm = np.linalg.norm
        with patch('kitchen_safety.collision_planner.np.linalg.norm', wraps=original_norm) as norm:
            self.assertEqual(pairs(world, 0.), expected)
        # Actual broad-phase work shrinks from 2x3 to 2x2, not just its result.
        self.assertEqual([call.args[0].shape for call in norm.call_args_list], [(3, 3), (2, 2, 3)])
        self.assertFalse(world.valid(np.zeros(7), mode='execution'))
        self.assertEqual(world.last_collision, ('arm', '/World/near_a'))

    def test_exact_boundary_and_adjacent_floats_retain_original_acceptance(self):
        shape = box('arm', [-.5] * 3, [.5] * 3)
        for low, valid, candidates in [(np.nextafter(.625, 0.), None, 1),
                                       (.625, True, 1),
                                       (np.nextafter(.625, np.inf), True, 0),
                                       (.625 - 1e-6, False, 1)]:
            with self.subTest(low=low):
                world = world_with_boxes([box('/World/wall', [low, -.5, -.5], [1., .5, .5])], [shape])
                self.assertEqual(len(self.assert_same_candidates(world)), candidates)
                actual = world.valid(np.zeros(7), mode='execution')
                # FCL box-center/size arithmetic can round a one-ulp bound
                # difference. Preserve its actual distance decision, too.
                self.assertEqual(actual, world.clearance(np.zeros(7), mode='execution')['valid'])
                if valid is not None:
                    self.assertEqual(actual, valid)

    def test_large_per_obstacle_margin_and_filter_ignore_are_preserved(self):
        world = world_with_boxes([
            box('/World/glass/body', [5., -.5, -.5], [6., .5, .5]),
            box('/World/ignored/body', [0.] * 3, [1.] * 3),
            box('/World/other/body', [20.] * 3, [21.] * 3)],
            [box('arm', [-.5] * 3, [.5] * 3)], rules={'glass': 5.})
        expected = self.assert_same_candidates(world, ignore=('ignored',), obstacle_filter=('glass',))
        self.assertEqual(len(expected), 1)
        self.assertEqual(expected[0][5], 5.)
        self.assertFalse(world.valid(np.zeros(7), ('ignored',), mode='execution', obstacle_filter=('glass',)))
        self.assert_same_candidates(world, ignore=('glass', 'ignored'))

    def test_union_includes_bottle_outside_arm_and_live_world_pose(self):
        shapes = [box('arm', [-.1] * 3, [.1] * 3),
                  box('bottle', [4.9, -.1, -.1], [5.1, .1, .1])]
        far = box('/World/glass/body', [9.9, -.1, -.1], [10.1, .1, .1])
        world = world_with_boxes([far], shapes)
        self.assertEqual(self.assert_same_candidates(world), [])
        self.assertTrue(world.valid(np.zeros(7), mode='execution'))
        cached = world._pair_selection_cache
        # A dynamic hazard changes bounds/FCL transforms, not cached names.
        _, obj, _, _ = far
        obj.setTranslation(np.array([5., 0., 0.]))
        world.low[0] -= [5., 0., 0.]
        world.high[0] -= [5., 0., 0.]
        world.objects[0] = (far[0], obj, world.low[0], world.high[0])
        expected = self.assert_same_candidates(world)
        self.assertIs(world._pair_selection_cache, cached)
        self.assertEqual([p[1] for p in expected], ['bottle'])
        self.assertFalse(world.valid(np.zeros(7), mode='execution'))
        self.assertEqual(world.last_collision, ('bottle', '/World/glass/body'))

    def test_counter_bottle_support_allowance_unchanged_at_exact_limit(self):
        support_limit = .5 - .006
        for bottom, expected in [(support_limit, True),
                                 (np.nextafter(support_limit, -np.inf), False)]:
            with self.subTest(bottom=bottom):
                world = world_with_boxes([box('/World/counter_top/body', [-1., -1., 0.], [1., 1., .5])],
                                         [box('bottle', [-.1, -.1, bottom], [.1, .1, .7])])
                self.assert_same_candidates(world)
                self.assertEqual(world.valid(np.zeros(7), mode='execution'), expected)

    def test_finite_slacks_random_boxes_and_one_shot_shape_iterators(self):
        rng = np.random.default_rng(207)
        shapes = []
        for i in range(9):
            low = rng.uniform(-2, 2, 3)
            shapes.append(box('bottle' if i == 8 else f'arm{i}', low, low + rng.uniform(.01, .7, 3)))
        objects = []
        for i in range(80):
            low = rng.uniform(-10, 10, 3)
            objects.append(box(f'/World/obstacle{i}', low, low + rng.uniform(.01, .7, 3)))
        world = world_with_boxes(objects, shapes)
        for slack in (-.125, 0., .25, 2., 20.):
            for iterator in (False, True):
                with self.subTest(slack=slack, iterator=iterator):
                    world.shapes = lambda *args, **kwargs: iter(shapes) if iterator else shapes
                    self.assert_same_candidates(world, slack)
        # inf must still visit all columns without any union pass.
        original_norm = np.linalg.norm
        with patch('kitchen_safety.collision_planner.np.linalg.norm', wraps=original_norm) as norm:
            pairs(world, float('inf'))
        self.assertEqual([call.args[0].shape for call in norm.call_args_list], [(9, 80, 3)])


if __name__ == '__main__':
    unittest.main()
