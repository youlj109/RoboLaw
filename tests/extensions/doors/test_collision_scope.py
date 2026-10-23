import unittest
from unittest.mock import patch
from kitchen_safety.collision_planner import CollisionWorld
from kitchen_safety.extensions.doors.collision import DoorCollisionWorld


class MountingScopeTests(unittest.TestCase):
    def test_only_nonphysical_mounting_proxy_exempts_its_own_stand(self):
        world = DoorCollisionWorld.__new__(DoorCollisionWorld)
        stand = '/World/envs/env_0/Robot/panda_link0/stand_instanceable/collider'
        pairs = [(0, 'arm_mount_envelope', stand),
                 (0, 'robot_hull', stand),
                 (0, 'gripper/left_inner_finger', stand),
                 (0, 'arm', stand),
                 (0, 'arm_mount_envelope', '/World/envs/env_0/person/body'),
                 (0, 'arm_mount_envelope', '/World/envs/env_0/kitchen/counter')]
        with patch.object(CollisionWorld, '_pairs', return_value=pairs):
            self.assertEqual(world._pairs(None), pairs[1:])

    def test_handle_contact_permission_does_not_disable_stand_or_foreign_geometry(self):
        world = DoorCollisionWorld.__new__(DoorCollisionWorld)
        world.allow_handle_contact = True
        handle = '/World/envs/env_0/kitchen/door/Collisions/fridge_door_handle_main'
        pairs = [(0, 'gripper/left_inner_finger', handle),
                 (0, 'robot_hull', handle),
                 (0, 'gripper/left_inner_finger', '/Robot/stand_instanceable/collider'),
                 (0, 'gripper/left_inner_finger', '/World/person/hand')]
        with patch.object(CollisionWorld, '_pairs', return_value=pairs):
            self.assertEqual(world._pairs(None), pairs[1:])


if __name__ == '__main__':
    unittest.main()
