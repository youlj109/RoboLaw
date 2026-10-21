"""CPU feasibility of redundant arm postures for the unchanged lime TCP grasp."""
import argparse,json
from pathlib import Path
import h5py,numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,Gf
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld

p=argparse.ArgumentParser();p.add_argument('--position',nargs=3,type=float,required=True);p.add_argument('--yaw',type=float,required=True);p.add_argument('--output',required=True);p.add_argument('--baseline',default='datasets/skill_trials/lightwheel_pick_place/base_attempt12');a=p.parse_args()
root=Path(__file__).resolve().parents[3];ep=Path(a.baseline);r=json.loads((ep/'result.json').read_text());b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:]).as_euler('xyz',degrees=True)[2]);s=Usd.Stage.Open(str(ep/'scene.usda'));s.SetEditTarget(s.GetSessionLayer());x=UsdGeom.Xform.Define(s,'/World/envs/env_0/person');x.GetPrim().GetReferences().AddReference(str(root/'assets/people/mpfb_corridor/collidable.usda'));x.AddTranslateOp().Set(Gf.Vec3d(*a.position));x.AddRotateZOp().Set(a.yaw);w=CollisionWorld(s,kin)
for key,state in r['initial_object_states'].items():w.set_obstacle_pose(key,state[:7])
with h5py.File(ep/'trajectory.h5') as f:
    phases=f['phase'].asstr()[:];q=f['next_obs/q'][:];pre_q=q[np.flatnonzero(phases=='pregrasp')[-1],:7];closed=q[np.flatnonzero(phases=='close')[-1]]
target=np.array(next(v['target'] for v in r['plans'] if v['phase']=='pregrasp' and v.get('executed')))
rng=np.random.default_rng(42);seeds=[pre_q]+[pre_q+rng.normal(0,scale,7) for scale in (.2,.4,.7,1.) for _ in range(16)];rows=[]
for i,seed in enumerate(seeds):
    try:
        solution=kin.ik(target,seed,attempts=1);row={'candidate':i,'q_pregrasp':solution.tolist()};w.allowed_phase='pregrasp'
        if not w.valid(solution,('lime',),mode='planning',gripper_q=dict.fromkeys(w.GRIPPER_JOINT_NAMES,0.)):
            row['failed_phase']='pregrasp';row['failure']=w.last_clearance;rows.append(row);continue
        path=[solution];ok=True
        for phase,delta in [('approach',-.125),('lift',.16)]:
            initial=kin.fk(path[-1]);goal=initial.copy();goal[2,3]+=delta;w.allowed_phase=phase
            finger=dict.fromkeys(w.GRIPPER_JOINT_NAMES,0.) if phase=='approach' else dict(zip(w.GRIPPER_JOINT_NAMES,closed[7:]))
            for fraction in np.linspace(0,1,int(abs(delta)/.002)+1)[1:]:
                pose=initial.copy();pose[2,3]+=delta*fraction;solution=kin.ik(pose,path[-1],attempts=1)
                if not w.valid(solution,('lime',),mode='planning',gripper_q=finger):
                    ok=False;row['failed_phase']=phase;row['failure']=w.last_clearance;break
                path.append(solution)
            if not ok:break
        row['passed_arm_external_geometry']=ok
        if ok:row['q_local_path']=[v.tolist() for v in path]
        rows.append(row);print('IK',i,ok,row.get('failed_phase'),row.get('failure'),flush=True)
        if ok:break
    except RuntimeError as exc:rows.append({'candidate':i,'error':str(exc)})
report={'position':a.position,'yaw_deg':a.yaw,'same_original_tcp_target':target.tolist(),'scope':'CPU arm and articulated gripper vs full source world and person at original planning margins; not a self-collision or physical grasp certificate','candidates':rows,'has_feasible_candidate':any(v.get('passed_arm_external_geometry') for v in rows)}
Path(a.output).write_text(json.dumps(report,indent=2));print('COMPLETE',report['has_feasible_candidate'],len(rows),flush=True)
