"""Identify exact source body and humanoid bone region in a rejected endpoint."""
import argparse
import json
from pathlib import Path
import fcl
import numpy as np
from pxr import Gf,Usd,UsdGeom
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.controls.collision import ControlsCollisionWorld
from kitchen_safety.geometry_audit import static_collision_shapes

ROOT=Path(__file__).resolve().parents[3]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--config',type=Path,required=True);p.add_argument('--baseline',type=Path,required=True)
p.add_argument('--phase',default='approach')
p.add_argument('--output',type=Path)
a=p.parse_args();config=json.loads(a.config.read_text());spec=config['variants']['human']
if a.output is not None and a.output.exists():raise FileExistsError(a.output)
result=json.loads((a.baseline/'result.json').read_text());rows=json.loads((a.baseline/'records.json').read_text())
state=next(r['before'] for r in rows if r['phase']==a.phase)
pose=result['initial_robot_pose'];kin=DroidKinematics(pose[:3],Rotation.from_quat(pose[3:]).as_euler('xyz',degrees=True)[2])
names=result['robot_joint_names'];q=np.asarray(state['q'])[[names.index(f'panda_joint{i}') for i in range(1,8)]]
stage=Usd.Stage.CreateInMemory();prim=UsdGeom.Xform.Define(stage,'/World/envs/env_0/person')
prim.GetPrim().GetReferences().AddReference(str(ROOT/spec['asset']))
prim.AddTranslateOp().Set(Gf.Vec3d(*spec['position']));prim.AddRotateZOp().Set(spec['yaw_deg'])
world=ControlsCollisionWorld(stage,kin,env_root='/World/envs/env_0')
exact=static_collision_shapes(stage,str(prim.GetPath()));links=kin.links(q)
skeleton=json.loads(((ROOT/spec['asset']).parent/'pose.json').read_text())['after']
rotation=Rotation.from_euler('z',spec['yaw_deg'],degrees=True)
diagnostics=[]
for body,model,verts in world.robot_hulls:
    transform=links[body]
    obj=fcl.CollisionObject(model,fcl.Transform(transform[:3,:3],transform[:3,3]))
    for obstacle in exact:
        collision=fcl.CollisionResult()
        if not fcl.collide(obj,obstacle[1],fcl.CollisionRequest(num_max_contacts=1,enable_contact=True),collision):continue
        point=np.asarray(collision.contacts[0].pos)
        local=rotation.inv().apply(point-spec['position'])
        nearby=[]
        for bone,(start,end) in skeleton.items():
            start,end=np.asarray(start),np.asarray(end);axis=end-start
            t=np.clip(np.dot(local-start,axis)/max(1e-12,np.dot(axis,axis)),0,1)
            nearby.append((float(np.linalg.norm(local-start-t*axis)),bone))
        diagnostics.append({'robot_source_link':body,'human_collider':obstacle[0],
            'contact_world_m':point.tolist(),'contact_asset_m':local.tolist(),'nearest_bones':sorted(nearby)[:5]})
print(json.dumps(diagnostics,indent=2))
if a.output is not None:
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps({'config':str(a.config),'baseline':str(a.baseline),
        'phase':a.phase,'scope':'Offline source-geometry inspection of a recorded baseline state; no physical replay.',
        'collisions':diagnostics},indent=2)+'\n')
