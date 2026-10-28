"""Annotate recorded successful task states against a frozen human, never replay."""
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
from kitchen_safety.extensions.controls.collision import ControlsCollisionWorld
from kitchen_safety.geometry_audit import static_collision_shapes, minimum_distance
from kitchen_safety.planning import DroidKinematics
from scripts.extensions.controls.render_first_violation import label_robot_hulls


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--episodes',type=Path,nargs='+',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    read=lambda p:json.loads(Path(p).read_text())
    sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
    config=read(args.config);spec=config['variants']['human'];pose=config['robot_pose_xyzw']
    kin=DroidKinematics(pose[:3],Rotation.from_quat(pose[3:]).as_euler('xyz',degrees=True)[2])
    stage=Usd.Stage.CreateInMemory();person=UsdGeom.Xform.Define(stage,'/World/envs/env_0/person')
    person.GetPrim().GetReferences().AddReference(str(ROOT/spec['asset']))
    person.AddTranslateOp().Set(Gf.Vec3d(*spec['position']));person.AddRotateZOp().Set(spec['yaw_deg'])
    world=ControlsCollisionWorld(stage,kin,env_root='/World/envs/env_0')
    exact=static_collision_shapes(stage,str(person.GetPath()));episodes=[]
    for episode in args.episodes:
        result=read(episode/'result.json');records=read(episode/'records.json')
        if (not result.get('accepted') or result['task_id']!=config['task_id']
                or result['source_sha256']!=config['source_sha256'] or result['initial_robot_pose']!=pose):
            raise ValueError('Expected a successful same-source recorded task')
        names=result['robot_joint_names'];ids=[names.index(f'panda_joint{i}') for i in range(1,8)]
        phases={}
        for index,row in enumerate(records):
            phase=phases.setdefault(row['phase'],dict(samples=0,conservative_violations=0,source_violations=0))
            for when in ('before','after'):
                state=row[when];q=np.asarray(state['q'])[ids];gripper=dict(zip(names,state['q']))
                coarse=world.clearance(q,mode='execution',gripper_q=gripper)
                raw=world.shapes(q,gripper)
                labeled=label_robot_hulls(raw,[body for body,_,_ in world.robot_hulls])
                source=minimum_distance([s for s in labeled if s[0] not in ('arm','gripper')],exact,.02)
                phase['samples']+=1
                for key,measurement in (('conservative',coarse),('source',source)):
                    event=dict(index=index,state=when,time_s=(index+(when=='after'))*result['dt'],**measurement)
                    if not measurement['valid']:
                        phase[key+'_violations']+=1
                        phase.setdefault(key+'_first_violation',event)
                    if key+'_minimum' not in phase or measurement['distance_m']<phase[key+'_minimum']['distance_m']:
                        phase[key+'_minimum']=event
        entry=dict(episode=str(episode),records_sha256=sha(episode/'records.json'),phases=phases)
        episodes.append(entry)
        print(json.dumps(dict(episode=str(episode),phases={k:dict(conservative=v['conservative_violations'],
            source=v['source_violations'],minimum_m=v['conservative_minimum']['distance_m']) for k,v in phases.items()})))
    report=dict(accepted=False,config=str(args.config),config_sha256=sha(args.config),
        human_asset_sha256=sha(ROOT/spec['asset']),episodes=episodes,
        scope='Offline counterfactual annotation of successful recorded states only. Controllers may differ '
            'from the frozen visual config; not a matched pair, planning query, physical collision or benchmark acceptance. '
            'Original conservative20mm model and separate source-hull checks are both retained.')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
