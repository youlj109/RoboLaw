import numpy as np
from pxr import Usd
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.collision_planner import CollisionWorld
kin=DroidKinematics([2.2,-.93,.8],90)
world=CollisionWorld(Usd.Stage.Open('/workspace/scenes/human.usda'),kin)
q=np.deg2rad([0,-36,0,-144,0,108,0])
def pose(xyz,tilt,yaw):
 t=np.eye(4);t[:3,3]=xyz
 t[:3,:3]=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@Rotation.from_euler('x',tilt,degrees=True).as_matrix()@np.array([[0,-1,0],[0,0,1],[-1,0,0]])
 return t
grasp=pose([2,-.4,1.035],35,0);obj=np.eye(4);obj[:3,3]=[2,-.4,.999]
world.attachment=np.linalg.inv(grasp)@obj
q=kin.ik(pose([2.49,-.56,1.18],35,0),q)
for xyz in [[2.49,-.31,1.15],[2.49,-.31,1.085]]:
 for tilt,yaw in [(35,0),(35,-30),(35,-45),(45,-30),(45,0),(55,-45),(55,0),(35,30),(35,45)]:
  try:
   g=kin.ik(pose(xyz,tilt,yaw),q,attempts=4)
   valid=world.valid(g,('Mustard','Bowl','Stand'))
   print(xyz,tilt,yaw,'valid',valid,'collision',world.last_collision,'direct',world.edge(q,g,('Mustard','Bowl','Stand')) if valid else None,flush=True)
  except Exception as e:print(str(e),flush=True)
print('LEFT ROUTE',flush=True)
q=kin.ik(pose([2,-.56,1.18],35,0),np.deg2rad([0,-36,0,-144,0,108,0]))
for xyz in [[2.30,-.56,1.14],[2.30,-.31,1.13],[2.49,-.31,1.085]]:
 found=False
 for tilt,yaw in [(35,0),(35,-30),(35,-45),(45,-30),(45,0),(55,-45),(55,0),(35,30),(35,45)]:
  try:
   g=kin.ik(pose(xyz,tilt,yaw),q,attempts=4)
   if world.valid(g,('Mustard','Bowl','Stand')) and world.edge(q,g,('Mustard','Bowl','Stand')):
    print('LEFT_OK',xyz,tilt,yaw,flush=True);q=g;found=True;break
  except Exception:pass
 if not found:print('LEFT_FAIL',xyz,world.last_collision,flush=True);break
for yaw in [30,45]:
 try:
  g=kin.ik(pose([2.49,-.31,1.085],35,yaw),q,attempts=4)
  path=world.plan(q,g,('Mustard','Bowl','Stand'),seed=42,max_iterations=3000)
  print('RRT_LEFT_OK',yaw,len(path),flush=True)
  np.savez('/workspace/reports/human_left_path.npz',path=path)
  break
 except Exception as ex:print('RRT_LEFT_FAIL',yaw,str(ex),flush=True)
