"""Task-specific handle contact exemption with full source door collision coverage."""
from kitchen_safety.extensions.doors.collision import DoorCollisionWorld
from kitchen_safety.collision_planner import CollisionWorld
class TaskCollisionWorld(DoorCollisionWorld):
 def __init__(self,*args,**kwargs):
  self.handle_contact_colliders=set()
  super().__init__(*args,**kwargs)
 def _pairs(self,*args,**kwargs):
  pairs=CollisionWorld._pairs(self,*args,**kwargs)
  pairs=[p for p in pairs if not(p[1]=='arm_mount_envelope' and '/stand_instanceable/' in p[2])]
  if not getattr(self,'allow_handle_contact',False):return pairs
  return [p for p in pairs if not(p[2] in self.handle_contact_colliders and p[1] in ('finger','gripper/left_inner_finger','gripper/right_inner_finger'))]
