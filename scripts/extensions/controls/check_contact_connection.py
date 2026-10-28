"""One nominal CPU IK/FCL preflight, never a physical or benchmark acceptance."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from pxr import Gf, Usd, UsdGeom
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.controls.collision import ControlsCollisionWorld
from kitchen_safety.extensions.controls.temperature_execution import (
    face_friction_connection, face_friction_pose, temperature_pregrasp_pose)
from kitchen_safety.extensions.controls.task_contract import task_contract


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--reference',type=Path,required=True)
    parser.add_argument('--variant',choices=['base','human'],required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    read=lambda p:json.loads(Path(p).read_text())
    config=read(args.config);result=read(args.reference/'result.json')
    contract=task_contract('temperature')
    if (config['task_id']!=contract['task_id'] or result['task_id']!=config['task_id']
            or result['source_sha256']!=config['source_sha256']
            or result['initial_robot_pose']!=config['robot_pose_xyzw']):
        raise ValueError('Reference must share the original task, source and robot pose')
    state=next(row['before'] for row in read(args.reference/'records.json') if row['phase']=='approach')
    start=np.asarray(state['tcp']);expected=temperature_pregrasp_pose(config)
    if (np.linalg.norm(start[:3,3]-expected[:3,3])>.002 or
            (Rotation.from_matrix(start[:3,:3]).inv()*Rotation.from_matrix(expected[:3,:3])).magnitude()>.02):
        raise ValueError('Measured reference does not start at this configured pregrasp')
    stage_path=Path(config['inspection_stage'])
    stage=Usd.Stage.CreateInMemory();stage.GetRootLayer().subLayerPaths=[str(stage_path.resolve())]
    if args.variant=='human':
        spec=config['variants']['human'];root=UsdGeom.Xform.Define(stage,'/World/envs/env_0/person')
        root.GetPrim().GetReferences().AddReference(str((ROOT/spec['asset']).resolve()))
        root.AddTranslateOp().Set(Gf.Vec3d(*spec['position']));root.AddRotateZOp().Set(spec['yaw_deg'])
    pose=config['robot_pose_xyzw'];kin=DroidKinematics(pose[:3],Rotation.from_quat(pose[3:]).as_euler('xyz',degrees=True)[2])
    world=ControlsCollisionWorld(stage,kin,env_root='/World/envs/env_0',
        contact_body_fragment='/'+contract['contact_body'].split('/')[-1]+'/')
    world.allow_button_contact=True
    names=result['robot_joint_names'];ids=[names.index(f'panda_joint{i}') for i in range(1,8)]
    q=np.asarray(state['q'])[ids];gripper=dict(zip(names,state['q']))
    try:
        endpoint=kin.ik(face_friction_pose(config,0.,0.),q.copy(),attempts=4)
        goal_diagnostic=dict(clearance=world.clearance(endpoint,mode='execution',gripper_q=gripper),
            scope='Independent endpoint IK branch diagnosis only; the actual nominal connection is evaluated separately')
    except Exception as error:goal_diagnostic=dict(error=str(error))
    samples=[];failure=None;requested=None
    try:
        for requested in face_friction_connection(start,config):
            q=kin.ik(requested,q,attempts=4)
            gap=world.clearance(q,mode='execution',gripper_q=gripper)
            samples.append(dict(index=len(samples),clearance=gap))
            if not gap['valid']:raise RuntimeError('Nominal conservative-model clearance violation')
    except Exception as error:failure=f'{type(error).__name__}: {error}'
    inputs=[args.config,args.reference/'records.json',stage_path]
    if args.variant=='human':inputs.append(ROOT/config['variants']['human']['asset'])
    report=dict(feasible=failure is None and bool(samples),accepted=False,variant=args.variant,
        config=str(args.config),reference=str(args.reference),failure=failure,
        requested_pose=None if failure is None else np.asarray(requested).tolist(),
        ik_queries=len(samples)+(failure is not None and failure.startswith('RuntimeError: IK failed')),
        max_ik_attempts_per_query=4,motiongen_queries=0,physical_steps=0,
        endpoint_ik_queries=1,nominal_contact_goal=goal_diagnostic,
        scope='One declared candidate, nominal IK/FCL connection only. Frozen measured closed fingers; '
            'not force/contact simulation, physical tracking, stability or benchmark acceptance. '
            'A failed recorded reference may supply its verified precontact state, never success evidence.',
        inputs_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},samples=samples)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ('feasible','accepted','failure','ik_queries','motiongen_queries')}))
    if not report['feasible']:raise SystemExit(2)


if __name__=='__main__':main()
