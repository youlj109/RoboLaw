"""Read-only CPU reconstruction of rejected property04 retreat geometry."""
import sys,json
from pathlib import Path
import h5py,numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,Gf
sys.path.insert(0,'scripts/extensions/replicator')
from audit import hazard_stage
from kitchen_safety.planning import DroidKinematics,transform
from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld
root=Path(__file__).resolve().parents[3];ep=root/'datasets/skill_trials/replicator/property/attempt_04';r=json.loads((ep/'result.json').read_text());l=r['layout'];k=DroidKinematics(np.array(l['robot_position'])+[0,0,l['stand_height']],l['robot_yaw_deg']);stage,_=hazard_stage(l,'property',r['obstacle']);cfg=l['planning'];fail=r['plan_candidate_failures'][-1];q=np.array(fail['start_q_arm']);start=k.fk(q);target=np.array(fail['target_pose_world']);target[:3,:3]=start[:3,:3]
with h5py.File(ep/'trajectory.h5') as f:
 names=json.loads(f.attrs['joint_names']);grip={n:float(f['next_obs/q'][-1,names.index(n)]) for n in CollisionWorld.GRIPPER_JOINT_NAMES};bowl=f['next_obs/bowl'][-1,:7];hazard=f['next_obs/hazard_pose'][-1]
x=UsdGeom.Xform.Define(stage,'/World/bowl');x.GetPrim().GetReferences().AddReference(str(root/'assets/robolab/objects/ycb/bowl.usd'));x.ClearXformOpOrder();x.AddTransformOp().Set(Gf.Matrix4d(transform(bowl[:3],bowl[3:]).T.tolist()));w=CollisionWorld(stage,k,clearances=cfg['clearances_m'],execution_clearances=cfg['execution_clearances_m']);w.set_obstacle_pose('glass',hazard)
path=[q];poses=[start]
for t in np.linspace(0,1,max(2,int(np.ceil(np.linalg.norm(target[:3,3]-start[:3,3])/.002))+1))[1:]:
 p=start.copy();p[:3,3]=start[:3,3]+t*(target[:3,3]-start[:3,3]);path.append(k.ik(p,path[-1],attempts=1));poses.append(k.fk(path[-1]))
report={'source':str(ep.relative_to(root)),'actual_gripper':grip,'actual_bowl_pose':bowl.tolist(),'actual_hazard_pose':hazard.tolist(),'start_tcp':start.tolist(),'goal':target.tolist(),'samples':len(path),'minimum_joint_limit_margin_rad':float(min((np.array(path)-k.limits[:,0]).min(),(k.limits[:,1]-np.array(path)).min())),'released_arm_vs_bowl':w.clearance(q,gripper_q=grip,obstacle_filter=['bowl']),'released_arm_vs_vase':w.clearance(q,gripper_q=grip,obstacle_filter=['glass']),'candidates':[]}
for dx,dy in [(0,0),(0,-.02),(0,-.04),(0,-.06),(.02,0),(.04,0),(-.02,0)]:
 h=hazard.copy();h[:2]+=[dx,dy];w.set_obstacle_pose('glass',h);rows=[w.clearance(x,mode='planning',gripper_q=grip,obstacle_filter=['glass']) for x in path];report['candidates'].append({'delta_xy':[dx,dy],'vase_pose':h.tolist(),'all_local_path_fcl_valid':all(x['valid'] for x in rows),'minimum':min(rows,key=lambda x:x['slack_m'])})
out=root/'reports/skill_trials/replicator/replicator_retreat_review.json';out.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
