"""Source support contact must not erase arm, side, or other-table checks."""
import unittest
import numpy as np
from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld

class SourceSupportContactTest(unittest.TestCase):
    def candidates(self, part, surface, bottom):
        world = CollisionWorld.__new__(CollisionWorld)
        low = np.array([-1., -1., .85]); high = np.array([1., 1., .895])
        world.objects = [(surface, object(), low, high)]
        world.low = np.array([low]); world.high = np.array([high])
        world.clearances = {}; world.execution_clearances = {}; world.default_clearance = .002
        world._pair_selection_key = None
        world.shapes = lambda *a, **kw: [(part, object(), np.array([-.03, -.03, bottom]), np.array([.03, .03, bottom + .16]))]
        return world._pairs(np.zeros(7), (), 'planning', 0., None, None)

    def test_only_bottle_top_support_contact_is_permitted(self):
        island = '/World/envs/env_0/kitchen/CapSurface_03'
        self.assertEqual(self.candidates('bottle', island, .894), [])
        self.assertEqual(len(self.candidates('panda_hand', island, .894)), 1)
        self.assertEqual(len(self.candidates('bottle', island, .88)), 1)
        self.assertEqual(len(self.candidates('bottle', island + '_other', .894)), 1)
        self.assertEqual(len(self.candidates('bottle', '/World/envs/env_0/kitchen/counter_other', .894)), 1)

if __name__ == '__main__': unittest.main()
