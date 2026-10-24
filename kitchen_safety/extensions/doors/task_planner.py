"""Task-local cuRobo adapter with exact, fully qualified handle collider matching."""
from kitchen_safety.curobo_backend import CuroboPlanner
class TaskPlanner(CuroboPlanner):
 def __init__(self,*args,hazard_extra_padding_m=0.,**kwargs):
  self.hazard_extra_padding_m=float(hazard_extra_padding_m)
  if not 0.<=self.hazard_extra_padding_m<=.03:raise ValueError('Hazard planning padding must be in [0,.03]m')
  super().__init__(*args,**kwargs)
  if self.hazard_extra_padding_m:
   self.collision_geometry['door_local_hazard_extra_padding_m']=self.hazard_extra_padding_m
   self.collision_geometry['door_local_padding_scope']='Additional conservative cuRobo glass/barrier boxes only; source FCL and execution thresholds unchanged.'
 def _planning_box(self,lo,hi,padding):
  # The shared adapter passes nonzero padding only for glass/barrier boxes.
  # This also runs during dynamic-box updates, preserving the same policy.
  extra=self.hazard_extra_padding_m if padding>0. else 0.
  return super()._planning_box(lo,hi,padding+extra)
 @staticmethod
 def _ignored(name,ignore_names):
  return any(name==key or name.startswith(key.rstrip('/')+'/') for key in ignore_names)
