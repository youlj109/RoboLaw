"""Rebuild the assigned source scene and recorded source stand on CPU."""
from pathlib import Path
import numpy as np
from kitchen_safety.extensions.doors.task_model import ROOT,resolve,assignment

def reconstruct(metadata,config=None,spec=None):
 from pxr import Usd,UsdGeom,Gf
 from scipy.spatial.transform import Rotation
 from kitchen_safety.planning import DroidKinematics,transform
 from kitchen_safety.extensions.doors.task_collision import TaskCollisionWorld
 task=assignment(metadata['task']);registry=task['background']['registry_name']
 cache={'lightwheel_kitchen_u_shaped_with_island_farmhouse2':'robocasa-robocasakitchen-7-10','lightwheel_kitchen_g_shaped_large_scandinavian':'robocasa-robocasakitchen-9-8'}
 kitchen_path=ROOT/config['_kitchen_asset'] if config and config.get('_kitchen_asset') else (ROOT/'assets/kitchen/scene.usd' if registry=='lightwheel_kitchen_one_wall_coastal' else Path.home()/'.cache/lightwheel_sdk/floorplan'/cache[registry]/'scene.usd')
 stage=Usd.Stage.CreateInMemory();kitchen=UsdGeom.Xform.Define(stage,'/World/envs/env_0/kitchen');kitchen.GetPrim().GetReferences().AddReference(str(kitchen_path))
 robot_path=Path(metadata['robot_usd']);robot_stage=Usd.Stage.Open(str(robot_path));base=next(p for p in robot_stage.Traverse() if p.GetName()=='panda_link0');original=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(base)).T
 target=transform(metadata['robot_base'],Rotation.from_euler('z',metadata['robot_yaw_deg'],degrees=True).as_quat());delta=target@np.linalg.inv(original)
 robot=UsdGeom.Xform.Define(stage,'/World/envs/env_0/Robot');robot.GetPrim().GetReferences().AddReference(str(robot_path));robot.AddTransformOp().Set(Gf.Matrix4d(*delta.T.reshape(-1)))
 if spec:
  for obj in [spec]+([spec['support']] if spec.get('support') else []):
   p=UsdGeom.Xform.Define(stage,'/World/envs/env_0/'+obj['name']);p.GetPrim().GetReferences().AddReference(str(ROOT/obj['asset']));p.AddTranslateOp().Set(Gf.Vec3d(*obj['position']));p.AddRotateZOp().Set(obj.get('yaw_deg',0.))
 kin=DroidKinematics(metadata['robot_base'],metadata['robot_yaw_deg']);world=TaskCollisionWorld(stage,kin,env_root='/World/envs/env_0');geometry=resolve(stage,metadata['task'],world);world.handle_contact_colliders=set(geometry['handle_colliders'])
 if not any('/stand_instanceable/' in n for n,*_ in world.objects):raise RuntimeError('Source stand missing')
 return stage,kin,world,geometry
