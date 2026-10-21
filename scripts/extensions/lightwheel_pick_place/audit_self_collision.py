"""Audit every saved physics-step arm and measured gripper for self/limits.

Empty cuRobo world isolates self-collision/limits. External complete kitchen
collision is independently covered by full_world_audit's actual lime geometry.
"""
import argparse,json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.curobo_backend import CuroboPlanner
p=argparse.ArgumentParser();p.add_argument('episodes',nargs='+');a=p.parse_args();overall=True
for directory in a.episodes:
 ep=Path(directory);r=json.loads((ep/'result.json').read_text());rows=json.loads((ep/'full_world_audit.json').read_text())['measurements'];b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);w=SimpleNamespace(objects=[],meshes={},attachment=None,gripper_joint_position=0.)
 planner=CuroboPlanner(w,kin,dt=r['dt']);violations=[]
 for i,row in enumerate(rows):
  w.gripper_joint_position=float(row['gripper_q']['finger_joint']);planner._sync_gripper_and_attachment(1)
  ok=planner.motion_gen.ik_solver.check_valid(planner.args.to_device(np.array(row['q_arm'])[None]))
  if not bool(ok.all()):violations.append({'physics_index':i,'action':row['action'],'phase':row['phase'],'q_arm':row['q_arm'],'finger_joint':w.gripper_joint_position})
 report={'passed':not violations,'samples':len(rows),'violations':violations,'scope':'Every recorded physics update: cuRobo robot self-collision and joint limits using measured finger_joint for locked mimic joints. Complete kitchen and actual source-lime geometry checked separately by physics-rate FCL.'}
 (ep/'self_collision_audit.json').write_text(json.dumps(report,indent=2));print(directory,json.dumps(report),flush=True);overall &= report['passed']
raise SystemExit(0 if overall else 2)
