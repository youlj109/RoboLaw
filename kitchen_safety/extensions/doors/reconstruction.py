"""CPU reconstruction from recorded source layout, including the real stand."""
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
DOOR = '/World/envs/env_0/kitchen/fridge_main_group/Refrigerator032_fridge_door'


def reconstruct(metadata, spec=None, hazard_pose=None):
    from pxr import Usd, UsdGeom, Gf
    from scipy.spatial.transform import Rotation
    from kitchen_safety.planning import DroidKinematics, transform
    from kitchen_safety.extensions.doors.collision import DoorCollisionWorld
    stage = Usd.Stage.CreateInMemory()
    kitchen = UsdGeom.Xform.Define(stage, '/World/envs/env_0/kitchen')
    kitchen.GetPrim().GetReferences().AddReference(str(ROOT / 'assets/kitchen/scene.usd'))
    cached = Path.home() / '.cache/isaaclab_arena/usd/robot_on_stand/doors_droid_on_stand_0.8_1.08x0.91032.usd'
    robot_stage = Usd.Stage.Open(str(cached))
    if robot_stage is None:
        raise FileNotFoundError('Run the original graph once to compose its source robot/stand asset')
    base = next(p for p in robot_stage.Traverse() if p.GetName() == 'panda_link0')
    original = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(base)).T
    target = transform(metadata['robot_base'], Rotation.from_euler(
        'z', metadata['robot_yaw_deg'], degrees=True).as_quat())
    root_transform = target @ np.linalg.inv(original)
    robot = UsdGeom.Xform.Define(stage, '/World/envs/env_0/Robot')
    robot.GetPrim().GetReferences().AddReference(str(cached))
    robot.AddTransformOp().Set(Gf.Matrix4d(*root_transform.T.reshape(-1)))
    if spec is not None:
        for obj in [spec] + ([spec['support']] if spec.get('support') else []):
            prim = UsdGeom.Xform.Define(stage, '/World/envs/env_0/' + obj['name'])
            prim.GetPrim().GetReferences().AddReference(str(ROOT / obj['asset']))
            prim.AddTranslateOp().Set(Gf.Vec3d(*obj['position']))
            prim.AddRotateZOp().Set(obj.get('yaw_deg', 0.))
    kin = DroidKinematics(metadata['robot_base'], metadata['robot_yaw_deg'])
    world = DoorCollisionWorld(stage, kin, env_root='/World/envs/env_0')
    stand = [n for n, *_ in world.objects if '/stand_instanceable/' in n]
    if not stand:
        raise RuntimeError('Reconstruction omitted the source robot stand')
    if spec is not None and hazard_pose is not None:
        world.set_obstacle_pose(spec['name'], hazard_pose)
    return stage, kin, world
