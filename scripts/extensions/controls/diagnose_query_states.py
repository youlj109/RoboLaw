"""Annotate saved query endpoints using cuRobo's own constraints, without planning."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom,Gf
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.controls.collision import ControlsCollisionWorld
from kitchen_safety.curobo_backend import CuroboPlanner


def write_report(path,report):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x') as stream:json.dump(report,stream,indent=2);stream.write('\n')
    print(json.dumps(report,indent=2))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--result',type=Path,required=True)
    p.add_argument('--variant',choices=['human','property','robot'],required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--cpu-source',action='store_true',help='Diagnose source shapes and the existing FCL model, with no CUDA/IK/path query')
    args=p.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    source=json.loads(args.result.read_text());config_path=Path(source['config'])
    config=json.loads(config_path.read_text())
    if hashlib.sha256(config_path.read_bytes()).hexdigest()!=source['config_sha256']:
        raise ValueError('Saved failed query configuration changed')
    base=config['robot_pose_xyzw']
    kin=DroidKinematics(base[:3],Rotation.from_quat(base[3:]).as_euler('xyz',degrees=True)[2])
    stage=Usd.Stage.CreateInMemory();stage.GetRootLayer().subLayerPaths.append(str(Path(source['stage']).resolve()))
    key={'human':'person','property':'glass','robot':'barrier'}[args.variant]
    spec=config['variants'][args.variant]
    prim=UsdGeom.Xform.Define(stage,'/World/envs/env_0/'+key)
    prim.GetPrim().GetReferences().AddReference(str(Path(spec['asset']).resolve()))
    prim.AddTranslateOp().Set(Gf.Vec3d(*spec['position']));prim.AddRotateZOp().Set(spec['yaw_deg'])
    world=ControlsCollisionWorld(stage,kin,env_root='/World/envs/env_0',
        fixture_cylinder_segments=config.get('planning_fixture_cylinder_segments',1))
    world.gripper_joint_position=source['gripper_joint_position']
    if args.cpu_source:
        from kitchen_safety.geometry_audit import static_collision_shapes,minimum_distance
        from scripts.extensions.controls.render_first_violation import label_robot_hulls
        baseline=Path(source['baseline'])
        names=json.loads((baseline/'result.json').read_text())['robot_joint_names']
        records=json.loads((baseline/'records.json').read_text())
        state=next(row['before'] for row in records if row['phase']==source['phase'])
        gripper=dict(zip(names,state['q']))
        exact=static_collision_shapes(stage,str(prim.GetPath()))
        report=dict(scope='CPU source-shape and conservative FCL endpoint diagnosis only; not a path or task acceptance.',
            result=str(args.result),result_sha256=hashlib.sha256(args.result.read_bytes()).hexdigest(),
            config_sha256=source['config_sha256'],ik_calls=0,motiongen_calls=0,layout_modified=False,states={})
        for name,field in (('start','start_q'),('goal','goal_q')):
            if field not in source:continue
            q=np.asarray(source[field])
            shapes=[s for s in world.shapes(q,gripper) if s[0]=='robot_hull' or s[0].startswith('gripper/')]
            labeled=label_robot_hulls(shapes,[body for body,_,_ in world.robot_hulls])
            report['states'][name]=dict(source_hazard=minimum_distance(labeled,exact,world.clearances[key]),
                conservative_hazard=world.clearance(q,mode='planning',gripper_q=gripper,obstacle_filter=(key,)))
        write_report(args.output,report)
        return
    planner=CuroboPlanner(world,kin,seed=42)
    planner._prepare_path_query((),0 if source['phase']=='pregrasp' else 1,source['phase'])
    rollout=planner.motion_gen.graph_planner.safety_rollout_fn
    from curobo.types.robot import JointState
    report=dict(scope='cuRobo endpoint-constraint diagnosis only, not source-geometry clearance, '
        'independent avoidance acceptance, physical execution or stability.',
        result=str(args.result),result_sha256=hashlib.sha256(args.result.read_bytes()).hexdigest(),
        config_sha256=source['config_sha256'],ik_calls=0,motiongen_calls=0,layout_modified=False,states={})
    for name,key in (('start','start_q'),('goal','goal_q')):
        q=planner.args.to_device(np.asarray(source[key])[None,None])
        metrics=rollout.rollout_constraint(q,use_batch_env=False)
        state=rollout.compute_kinematics(JointState.from_position(q,joint_names=planner.motion_gen.joint_names))
        def value(tensor):return float(tensor.detach().sum().cpu())
        report['states'][name]=dict(feasible=bool(metrics.feasible.all().item()),
            total_constraint=value(metrics.constraint),bounds=value(rollout.bound_constraint.forward(state.state_seq)),
            world=value(rollout.primitive_collision_constraint.forward(state.robot_spheres,env_query_idx=None)),
            self=value(rollout.robot_self_collision_constraint.forward(state.robot_spheres)))
        spheres=state.robot_spheres.detach().cpu().numpy().reshape(-1,4)
        model=planner.motion_gen.kinematics
        mapping=model.kinematics_config.link_sphere_idx_map.detach().cpu().numpy().reshape(-1)
        names={int(v):k for k,v in model.kinematics_config.link_name_to_idx_map.items()}
        hints=[]
        for obstacle,_,lo,hi in world.objects:
            hazard=any(world._matches(obstacle,key) for key in ('person','glass','barrier'))
            padding=world._margin(obstacle,'planning') if hazard else 0.
            for index,sphere in enumerate(spheres):
                if sphere[3]<=0:continue
                center=kin.base[:3,:3]@sphere[:3]+kin.base[:3,3]
                outside=np.maximum(np.maximum(lo-padding-center,center-hi-padding),0.)
                distance=float(np.linalg.norm(outside))
                if not np.any(outside):distance=-float(np.min(np.minimum(center-lo+padding,hi+padding-center)))
                gap=distance-float(sphere[3])
                if gap<0:hints.append(dict(robot_link=names.get(int(mapping[index]),'unknown'),
                    obstacle=obstacle,proxy_signed_gap_m=gap,
                    only_aabb_hint_not_source_mesh_collision=obstacle in world.meshes))
        report['states'][name]['proxy_overlap_hints']=sorted(hints,key=lambda r:r['proxy_signed_gap_m'])[:12]
    write_report(args.output,report)


if __name__=='__main__':main()
