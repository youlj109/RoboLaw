"""Task-local cuRobo adapter with exact, fully qualified handle collider matching."""
from kitchen_safety.curobo_backend import CuroboPlanner
class TaskPlanner(CuroboPlanner):
 @staticmethod
 def _ignored(name,ignore_names):
  return any(name==key or name.startswith(key.rstrip('/')+'/') for key in ignore_names)
