"""Task-local cuRobo adapter with exact, fully qualified handle collider matching."""
from kitchen_safety.curobo_backend import CuroboPlanner
class TaskPlanner(CuroboPlanner):
 def __init__(self,*args,hazard_extra_padding_m=0.,pregrasp_shortcut_policy="none",pregrasp_goal_q=None,**kwargs):
  self.pregrasp_goal_q=pregrasp_goal_q
  if pregrasp_goal_q is not None:
   import numpy as np
   if np.asarray(pregrasp_goal_q).shape!=(7,) or not np.isfinite(pregrasp_goal_q).all():raise ValueError("Expected seven finite pregrasp goal joints")
  self.hazard_extra_padding_m=float(hazard_extra_padding_m)
  if not 0.<=self.hazard_extra_padding_m<=.03:raise ValueError('Hazard planning padding must be in [0,.03]m')
  if pregrasp_shortcut_policy not in ("none","bounded_chords_v1"):raise ValueError("Unknown pregrasp shortcut policy")
  self.pregrasp_shortcut_policy=pregrasp_shortcut_policy
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

 def plan_pose(self,q_arm7,T_W_TCP,ignore_names=(),grip_binary=0,phase=None):
  if phase=='door_pregrasp' and self.pregrasp_goal_q is not None:
   import numpy as np
   from scipy.spatial.transform import Rotation
   goal=np.asarray(self.pregrasp_goal_q);actual=self.kin.fk(goal);requested=np.asarray(T_W_TCP)
   position_error=float(np.linalg.norm(actual[:3,3]-requested[:3,3]));rotation_error=float(Rotation.from_matrix(actual[:3,:3]@requested[:3,:3].T).magnitude())
   if position_error>.001 or rotation_error>.01:raise ValueError('Fixed redundant joint goal differs from requested pregrasp TCP')
   trajectory=super().plan_joint_goal(q_arm7,goal,ignore_names,grip_binary,phase)
   trajectory['metadata']['door_fixed_redundancy_goal']={'q':goal.tolist(),'position_error_m':position_error,'rotation_error_rad':rotation_error,'scope':'Same fixed TCP and redundant joint endpoint in obstacle on/off; cuRobo plans entire route, no route waypoints.'}
  else:
   trajectory=super().plan_pose(q_arm7,T_W_TCP,ignore_names,grip_binary,phase)
  if self.pregrasp_shortcut_policy=="none" or phase!="door_pregrasp":return trajectory
  import numpy as np
  from kitchen_safety.extensions.doors.pregrasp_shortcuts_lwx import POLICY,metrics,shortcut
  original=np.asarray(trajectory['positions'])
  validate=lambda path:self.validate_path(path,ignore_names=ignore_names,grip_binary=grip_binary,phase=phase,resolution=POLICY['resolution_rad'])
  positions,trials=shortcut(original,self.kin,validate,budget=POLICY['max_proposals'])
  if np.abs(np.diff(positions,axis=0)).max()>np.abs(np.diff(original,axis=0)).max()+1e-9:raise RuntimeError('Shortcut increased peak joint step')
  final=validate(positions)
  if not (final['valid'] and final['validation_complete'] and final['fcl_samples_unchecked']==0):raise RuntimeError('Shortcut full path geometry gate rejected: '+__import__('json').dumps({k:final.get(k) for k in ['early_rejection','fcl_failures','curobo_failures','samples','fcl_samples_checked']}))
  trajectory['positions']=positions
  trajectory['metadata']['pregrasp_shortcut']={'policy':POLICY.copy(),'before':metrics(original,self.kin),'after':metrics(positions,self.kin),'trials':trials,'final_validation':final,'same_endpoints':True,'same_action_times':True}
  return trajectory
