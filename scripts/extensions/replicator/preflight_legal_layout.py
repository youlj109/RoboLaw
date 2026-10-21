"""No-Kit CPU IK/FCL probe of a legal reset-layout candidate.

Uses saved successful baseline keyframes as world TCP/object targets, moving only
bowl-related targets by the proposed reset bowl displacement. It is not an
executed trajectory, cuRobo plan, grasp guarantee, or continuous safety proof.
"""
import argparse,copy,json
from pathlib import Path
import h5py,numpy as np
from scipy.spatial.transform import Rotation
from pxr import Gf,Usd,UsdGeom
from audit_source_relation import check_layout
from kitchen_safety.planning import DroidKinematics,transform
from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld
ROOT=Path(__file__).resolve().parents[3]
p=argparse.ArgumentParser();p.add_argument('--robot-y',type=float,default=0.);p.add_argument('--bowl-y',type=float,default=-.475);p.add_argument('--report',required=True);a=p.parse_args()
old=json.loads((ROOT/'datasets/skill_trials/replicator/base/diagnostic_05/result.json').read_text());layout=copy.deepcopy(old['layout']);layout['robot_position'][1]=a.robot_y;layout['bowl_position'][1]=a.bowl_y;dy=a.bowl_y-old['layout']['bowl_position'][1]
relations=check_layout(layout);stage=Usd.Stage.CreateInMemory();prefix='/World/envs/env_0'
def ref(name,path,pose):
 r=UsdGeom.Xform.Define(stage,prefix+'/'+name);r.GetPrim().GetReferences().AddReference(str(ROOT/path));r.ClearXformOpOrder();r.AddTranslateOp().Set(Gf.Vec3d(*map(float,pose[:3])));q=pose[3:];r.AddOrientOp().Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,q[:3]))))
ref('kitchen',layout['kitchen_usd'],[0,0,0,0,0,0,1]);ref('bowl','assets/robolab/objects/ycb/bowl.usd',[*layout['bowl_position'],*Rotation.from_euler('z',layout['bowl_yaw_deg'],degrees=True).as_quat()])
kin=DroidKinematics(np.array(layout['robot_position'])+[0,0,layout['stand_height']],layout['robot_yaw_deg']);w=CollisionWorld(stage,kin,default_clearance=layout['planning']['default_clearance_m']);rows=[]
with h5py.File(ROOT/'datasets/skill_trials/replicator/base/diagnostic_05/trajectory.h5') as f:
 names=json.loads(f.attrs['joint_names']);ids=[names.index(f'panda_joint{i}') for i in range(1,8)];phases=f['phase'].asstr()[:]
 for phase in ('settle','pregrasp','approach','close','lift','above_bowl','lower','release','retreat'):
  ix=np.flatnonzero(phases==phase)[0 if phase=='settle' else -1];qold=f['next_obs/q'][ix,ids];bottle=f['next_obs/mustard'][ix,:7];bowl=f['next_obs/bowl'][ix,:7];bowl[1]+=dy;w.set_obstacle_pose('bowl',bowl);w.sync_background(old['background_rigid_paths'],f['next_obs/background_rigid_poses'][ix]);grip={n:float(f['next_obs/q'][ix,names.index(n)]) for n in w.GRIPPER_JOINT_NAMES};target=transform(f['next_obs/tcp'][ix],f['next_obs/tcp_quat_xyzw'][ix]);ignore=('mustard_bottle',)+(('bowl',) if phase in ('lower','release','retreat','settle') else ())
  if phase in ('above_bowl','lower','release','retreat'):target[1,3]+=dy;bottle[1]+=dy
  try:
   q=qold if phase=='settle' else kin.ik(target,qold,attempts=8)
   # Original home configuration is retained; its TCP naturally moves with the base.
   error=None if phase=='settle' else float(np.linalg.norm(kin.fk(q)[:3,3]-target[:3,3]))
   carried=phase in ('lift','above_bowl','lower');w.attachment=np.linalg.inv(kin.fk(q))@transform(bottle[:3],bottle[3:]) if carried else None
   valid=w.valid(q,ignore,mode='planning',gripper_q=grip,object_pose=bottle if carried else None)
   rows.append(dict(phase=phase,index=int(ix),ik=True,fcl=bool(valid),position_error_m=error,q=q.tolist(),first_collision=w.last_collision,clearance_failure=w.last_clearance))
  except Exception as ex:rows.append(dict(phase=phase,index=int(ix),ik=False,error=str(ex)))
  print(json.dumps(rows[-1]),flush=True)
report=dict(passed=bool(relations['passed'] and all(r.get('ik') and r.get('fcl') for r in rows)),scope=__doc__,candidate_layout=layout,source_relations=relations,collision_shapes=len(w.objects),checks=rows);Path(a.report).write_text(json.dumps(report,indent=2)+'\n');raise SystemExit(0 if report['passed'] else 1)
