import json, sys
from pathlib import Path
import h5py, numpy as np
from pxr import Usd
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
from kitchen_safety.extensions.lightwheel_pick_place.planner import LimePlanner
from kitchen_safety.planning import DroidKinematics, transform

ep = Path('datasets/skill_trials/lightwheel_pick_place/base_lgd_b03_20261024T122330Z')
r = json.loads((ep/'result.json').read_text())
base = np.asarray(r['robot_base_pose'])
kin = DroidKinematics(base[:3], Rotation.from_quat(base[3:7]).as_euler('xyz', degrees=True)[2])
stage = Usd.Stage.Open(str(ep/'scene.usda'))
pick='banana_1'; dest='wooden_bowl'
rigid=('banana_1','banana_2','wooden_bowl','plate','apple','orange')
world=CollisionWorld(stage,kin,pick_object=pick,destination_object=dest,
  source_surface_suffix='/island_island_group/top_geometry_right',
  movable_objects=tuple(n for n in rigid if n!=pick)+('glass',))
with h5py.File(ep/'trajectory.h5') as f:
  q0=np.asarray(f['obs/q'][0,:7])
  poses={n:np.asarray(f['obs/'+n][0,:7]) for n in rigid if 'obs/'+n in f}
for n,p in poses.items():
  if n!=pick:
    try: world.set_obstacle_pose(n,p)
    except Exception as e: print('warn',n,e)
world.bind_pick(pick)
pick_pose=poses[pick]
center=(transform(pick_pose[:3],pick_pose[3:7])@np.r_[world.object_center,1])[:3]
center=center+np.array([0,0,0.01])
planner=LimePlanner(world,kin,dt=float(r.get('dt') or 1/30),seed=43,max_joint_speed=.4)
R0=np.array([[0,-1,0],[0,0,1],[-1,0,0]],float)
yaw=-90; tilt=0; st=0.18; remain=0.02
R=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@Rotation.from_euler('x',tilt,degrees=True).as_matrix()@R0
approach=R[:,2]; approach=approach/np.linalg.norm(approach)
hover=center-st*approach; contact=center-remain*approach
print('center',center,'hover',hover,'contact',contact, flush=True)
T_h=np.eye(4); T_h[:3,:3]=R; T_h[:3,3]=hover
T_c=np.eye(4); T_c[:3,:3]=R; T_c[:3,3]=contact
out={'yaw':yaw,'tilt':tilt,'standoff':st,'remain':remain}
try:
  pre=planner.plan_pose(q0,T_h,(pick,),0.,phase='transfer')
  q_pre=pre['positions'][-1]
  out['pregrasp_ok']=True
  print('PRE_OK', flush=True)
except Exception as e:
  out['pregrasp_ok']=False; out['pre_err']=str(e)[:200]; print('PRE_FAIL',e); print(json.dumps(out)); sys.exit(0)
# MotionGen transfer (known fail)
try:
  planner.plan_pose(q_pre,T_c,(pick,),0.,phase='transfer')
  out['transfer_approach_ok']=True
except Exception as e:
  out['transfer_approach_ok']=False; out['transfer_err']=str(e)[:120]
  print('TRANSFER_AP_FAIL', out['transfer_err'], flush=True)
# cartesian_line
try:
  ap=planner.plan_cartesian_line(q_pre,T_c,(pick,),0.,phase='lift')
  out['cart_approach_ok']=True
  out['cart_samples']=len(ap['positions'])
  print('CART_AP_OK samples', out['cart_samples'], flush=True)
except Exception as e:
  out['cart_approach_ok']=False; out['cart_err']=str(e)[:200]
  print('CART_AP_FAIL', out['cart_err'], flush=True)
# also try remain=0.08 with cartesian
for rem in (0.05,0.08,0.10):
  Tc=np.eye(4); Tc[:3,:3]=R; Tc[:3,3]=center-rem*approach
  key=f'cart_remain_{rem}'
  try:
    planner.plan_cartesian_line(q_pre,Tc,(pick,),0.,phase='lift')
    out[key]=True; print(key,'OK', flush=True)
  except Exception as e:
    out[key]=False; out[key+'_err']=str(e)[:120]; print(key,'FAIL',str(e)[:100], flush=True)
Path('/tmp/banana_cart_probe.json').write_text(json.dumps(out,indent=2))
print('COMPLETE', json.dumps(out), flush=True)
