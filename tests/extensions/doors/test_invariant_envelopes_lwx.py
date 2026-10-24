"""Verify discovery constraints against the original USD-derived kinematic chain."""
import unittest

import fcl
import numpy as np

from kitchen_safety.extensions.doors.collision import DoorCollisionWorld
from kitchen_safety.extensions.doors.invariant_envelopes_lwx import invariant_envelopes
from kitchen_safety.planning import DroidKinematics


class InvariantEnvelopeTests(unittest.TestCase):
    def test_source_distal_proxies_are_rigid_relative_to_tcp(self):
        kin = DroidKinematics([1.2, -.7, .9], 61.)
        # Exercise source capsule construction without scene or gripper hulls.
        # This test checks proxy kinematics, not physical gripper geometry.
        world = DoorCollisionWorld.__new__(DoorCollisionWorld)
        world.kin, world.fcl = kin, fcl
        world.robot_hulls, world.gripper_models = [], []
        world.gripper_root = list(kin.links(kin.limits.mean(axis=1)))[-1]
        world._gripper_links = lambda angles: {}
        world.attachment = None
        rng = np.random.default_rng(20261024)
        relative, proximal = [], []
        for q in rng.uniform(kin.limits[:, 0] + .03, kin.limits[:, 1] - .03, (24, 7)):
            tcp_inverse = np.linalg.inv(kin.fk(q))
            shapes = world.shapes(q, {})
            self.assertEqual([s[0] for s in invariant_envelopes(shapes)],
                             ['arm_distal_envelope', 'gripper', 'finger', 'finger'])
            frame_list = []
            for _, obj, _, _ in invariant_envelopes(shapes):
                frame = np.eye(4)
                frame[:3, :3], frame[:3, 3] = obj.getRotation(), obj.getTranslation()
                frame_list.append(tcp_inverse @ frame)
            # Capsule axial rotations do not affect geometry. Compare centers
            # and capsule axial direction (up to sign) instead of transverse axes.
            relative.append([(frame[:3, 3], np.outer(frame[:3, 2], frame[:3, 2]))
                             for frame in frame_list])
            proximal.append((tcp_inverse @ np.r_[shapes[3][1].getTranslation(), 1.])[:3])
        for values in relative[1:]:
            for (center, axis), (ref_center, ref_axis) in zip(values, relative[0]):
                np.testing.assert_allclose(center, ref_center, atol=1e-12)
                np.testing.assert_allclose(axis, ref_axis, atol=1e-12)
        self.assertGreater(float(np.ptp(np.asarray(proximal), axis=0).max()), .05)

    def test_changed_topology_fails_closed(self):
        with self.assertRaises(RuntimeError):
            invariant_envelopes([])
        dummy = [('arm_mount_envelope', None, None, None)] + [('arm', None, None, None)] * 5
        with self.assertRaises(RuntimeError):
            invariant_envelopes(dummy + [('finger', None, None, None)])


if __name__ == '__main__':
    unittest.main()
