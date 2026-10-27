"""Explain one saved goal against property planning proxies; no planning or edits."""
import argparse
import json
from pathlib import Path
import numpy as np
from pxr import Usd, UsdGeom, Gf
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.controls.collision import ControlsCollisionWorld
from kitchen_safety.curobo_backend import CuroboPlanner


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--result',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--include-kitchen',action='store_true')
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    root=Path(__file__).resolve().parents[3]
    report=json.loads(a.result.read_text());config=json.loads((root/report['config']).read_text())
    spec=config['variants']['property'];base=config['robot_pose_xyzw']
    kin=DroidKinematics(base[:3],Rotation.from_quat(base[3:]).as_euler('xyz',degrees=True)[2])
    stage=Usd.Stage.CreateInMemory();stage.GetRootLayer().subLayerPaths.append(str((root/report['stage']).resolve()))
    prim=UsdGeom.Xform.Define(stage,'/World/envs/env_0/glass')
    prim.GetPrim().GetReferences().AddReference(str(root/spec['asset']))
    prim.AddTranslateOp().Set(Gf.Vec3d(*spec['position']));prim.AddRotateZOp().Set(spec['yaw_deg'])
    world=ControlsCollisionWorld(stage,kin,env_root='/World/envs/env_0')
    world.gripper_joint_position=report['gripper_joint_position']
    planner=CuroboPlanner(world,kin,seed=42)
    planner._prepare_path_query((),0,'pregrasp')
    model=planner.motion_gen.kinematics
    state=model.get_state(planner.args.to_device(np.asarray(report['goal_q'])[None]))
    spheres=state.link_spheres_tensor.detach().cpu().numpy()[0]
    mapping=model.kinematics_config.link_sphere_idx_map.detach().cpu().numpy().reshape(-1)
    link_names={int(v):k for k,v in model.kinematics_config.link_name_to_idx_map.items()}
    overlaps=[]
    kitchen_contacts=[]
    for name,_,lo,hi in world.objects:
        if '/glass/' not in name:continue
        margin=world._margin(name,'planning');lo=lo-margin;hi=hi+margin
        for index,sphere in enumerate(spheres):
            if sphere[3]<=0:continue
            center=kin.base[:3,:3]@sphere[:3]+kin.base[:3,3]
            outside=np.maximum(np.maximum(lo-center,center-hi),0.)
            distance=float(np.linalg.norm(outside))
            if not np.any(outside):distance=-float(np.min(np.minimum(center-lo,hi-center)))
            gap=distance-float(sphere[3])
            if gap<0:overlaps.append(dict(link=link_names.get(int(mapping[index]),'unknown'),
                sphere_index=index,obstacle=name,proxy_signed_gap_m=gap))
    overlaps.sort(key=lambda row:row['proxy_signed_gap_m'])
    if a.include_kitchen:
        for name,obj,lo,hi in world.objects:
            if '/kitchen/' not in name:continue
            for index,sphere in enumerate(spheres):
                if sphere[3]<=0:continue
                center=kin.base[:3,:3]@sphere[:3]+kin.base[:3,3]
                if np.any(center+sphere[3]<lo) or np.any(center-sphere[3]>hi):continue
                sphere_obj=world.fcl.CollisionObject(world.fcl.Sphere(float(sphere[3])),world.fcl.Transform(center))
                collision=world.fcl.collide(sphere_obj,obj,world.fcl.CollisionRequest(),world.fcl.CollisionResult())
                if collision:kitchen_contacts.append(dict(link=link_names.get(int(mapping[index]),'unknown'),sphere_index=index,obstacle=name))
    component_costs={}
    try:
        from curobo.types.robot import JointState
        rollout=planner.motion_gen.graph_planner.safety_rollout_fn
        js=JointState.from_position(planner.args.to_device(np.asarray(report['goal_q'])[None,None]),
                                   joint_names=['panda_joint'+str(i) for i in range(1,8)])
        aug=rollout.compute_kinematics(js)
        component_costs['bounds']=float(rollout.bound_constraint.forward(aug.state_seq).detach().sum().cpu())
        component_costs['world']=float(rollout.primitive_collision_constraint.forward(aug.robot_spheres,env_query_idx=None).detach().sum().cpu())
        component_costs['self']=float(rollout.robot_self_collision_constraint.forward(aug.robot_spheres).detach().sum().cpu())
    except Exception as exc:component_costs['unavailable']=str(exc)
    result=dict(scope='One saved endpoint only. cuRobo robot-sphere versus padded property AABB explanation; '
                'not actual mesh contact, path validation or physics.',motiongen_calls=0,ik_calls=0,
                source_result=str(a.result),layout_modified=False,overlaps=overlaps[:24],
                kitchen_source_shape_hints=kitchen_contacts[:24],constraint_components=component_costs)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x') as stream:json.dump(result,stream,indent=2)
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
