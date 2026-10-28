#!/usr/bin/env python3
"""Build the revised walking-person scene from evaluated skeletal animation."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

ROOT = Path(__file__).resolve().parents[4]
OUT = ROOT / 'assets/extensions/doors/hard_demo_v2'
REPORT = ROOT / 'reports/doors_hard_demo_v2'
DOOR = '/World/kitchen/Asset/fridge_main_group/Refrigerator032_fridge_door'
JOINT = DOOR + '/fridge_door_joint'
HUMAN = '/World/human/Asset'
FPS, FRAMES = 12, 168
SCOPE = 'Scripted scene demonstration; stationary robot; no robot trajectory or physics acceptance.'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n')


def smooth(x):
    x = min(1., max(0., x))
    return x * x * (3. - 2. * x)


def phase(frame, pause_angle=12.):
    t = frame / FPS
    if t < 3:
        return 'walk_approach', 6. * smooth(t / 3.)
    if t < 4:
        return 'settle', 6. + 2. * smooth(t - 3.)
    if t < 5.5:
        return 'reach', 8. + (pause_angle - 8.) * smooth((t - 4.) / 1.5)
    if t < 7.5:
        return 'pause', pause_angle
    if t < 9:
        return 'withdraw', pause_angle
    if t < 12:
        return 'walk_retreat_and_resume', pause_angle + (25. - pause_angle) * smooth((t - 9.) / 3.)
    return 'end', 25.


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--human-position', nargs=3, type=float, default=[3.67, -.92, .0025])
    parser.add_argument('--human-yaw', type=float, default=125.)
    parser.add_argument('--pause-angle', type=float, default=12.)
    parser.add_argument('--force', action='store_true')
    args = parser.parse_args()
    person = OUT / 'person/person.usdc'
    if not person.is_file():
        raise FileNotFoundError('Generate the evaluated rig animation first: ' + str(person))
    person_stage = Usd.Stage.Open(str(person))
    assert person_stage and not person_stage.GetCompositionErrors()
    assert person_stage.GetStartTimeCode() == 0 and person_stage.GetEndTimeCode() == FRAMES - 1
    assert person_stage.GetTimeCodesPerSecond() == FPS
    scene = OUT / 'scene.usda'
    if scene.exists() and not args.force:
        raise FileExistsError(scene)
    OUT.mkdir(parents=True, exist_ok=True)
    REPORT.mkdir(parents=True, exist_ok=True)
    if scene.exists():
        scene.unlink()
    stage = Usd.Stage.CreateNew(str(scene))
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(FRAMES - 1)
    stage.SetTimeCodesPerSecond(FPS)
    stage.SetFramesPerSecond(FPS)
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, '/World').GetPrim())
    UsdGeom.SetStageUpAxis(stage, 'Z')
    UsdGeom.SetStageMetersPerUnit(stage, 1.)

    def reference(name, path, position=(0., 0., 0.), yaw=0.):
        x = UsdGeom.Xform.Define(stage, '/World/' + name)
        x.AddTranslateOp().Set(Gf.Vec3d(*position))
        x.AddRotateZOp().Set(yaw)
        stage.DefinePrim('/World/' + name + '/Asset').GetReferences().AddReference(os.path.relpath(path, OUT))

    kitchen = ROOT / 'assets/kitchen/scene.usd'
    layout_path = ROOT / 'configs/extensions/doors/fixed_source_layout.json'
    layout = json.loads(layout_path.read_text())
    robot = ROOT / 'assets/extensions/doors/hard_demo_v1/robot_on_stand.usda'
    reference('kitchen', kitchen)
    reference('robot', robot, layout['robot_asset_pose']['position'], layout['expected_robot_yaw_deg'])
    reference('human', person, args.human_position, args.human_yaw)
    meshes = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath(HUMAN)):
        if prim.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('none')
            UsdGeom.Mesh(prim).CreateDoubleSidedAttr(True)
            meshes.append(str(prim.GetPath()))
    assert meshes, 'No evaluated human mesh geometry'

    cache = UsdGeom.XformCache()
    joint = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(JOINT))
    body = stage.GetPrimAtPath(joint.GetBody0Rel().GetTargets()[0])
    body_matrix = np.asarray(cache.GetLocalToWorldTransform(body)).T
    quat = joint.GetLocalRot0Attr().Get()
    local = np.asarray(Gf.Matrix4d(Gf.Rotation(Gf.Quatd(quat.GetReal(), Gf.Vec3d(*quat.GetImaginary()))), Gf.Vec3d(*joint.GetLocalPos0Attr().Get()))).T
    hinge_matrix = body_matrix @ local
    hinge = hinge_matrix[:3, 3]
    axis = hinge_matrix[:3, 'XYZ'.index(str(joint.GetAxisAttr().Get()))]
    axis /= np.linalg.norm(axis)
    door = stage.GetPrimAtPath(DOOR)
    initial = np.asarray(cache.GetLocalToWorldTransform(door)).T
    parent = np.asarray(cache.GetLocalToWorldTransform(door.GetParent())).T
    xf = UsdGeom.Xformable(door)
    xf.ClearXformOpOrder()
    op = xf.AddTransformOp(opSuffix='scriptedDoor')

    def matrix(angle):
        a = math.radians(angle)
        cross = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        rotation = np.eye(3) + math.sin(a) * cross + (1 - math.cos(a)) * cross @ cross
        delta = np.eye(4)
        delta[:3, :3] = rotation
        delta[:3, 3] = hinge - rotation @ hinge
        return Gf.Matrix4d((np.linalg.inv(parent) @ delta @ initial).T.tolist())

    op.Set(matrix(0))
    for frame in range(FRAMES):
        op.Set(matrix(phase(frame, args.pause_angle)[1]), frame)
    camera = UsdGeom.Camera.Define(stage, '/World/hard_demo/camera')
    camera.CreateFocalLengthAttr(24.)
    camera.CreateClippingRangeAttr(Gf.Vec2f(.01, 100.))
    camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(4., -4.6, 2.15), Gf.Vec3d(3.65, -1., .93), Gf.Vec3d(0, 0, 1)).GetInverse())
    UsdLux.DomeLight.Define(stage, '/World/sky').CreateIntensityAttr(700.)
    stage.GetRootLayer().customLayerData = {'scope': SCOPE, 'revision': 'v2_walk_and_anatomical_arm', 'robot_stationary': True}
    stage.GetRootLayer().Save()
    assert not stage.GetCompositionErrors(), stage.GetCompositionErrors()
    phases = [(0., 3., '人物交替迈步走近，门开始打开'), (3., 4., '人物站稳'),
              (4., 5.5, '自然抬手，进入门的后续扫掠区域'), (5.5, 7.5, '门暂停，等待人手离开'),
              (7.5, 9., '撤手放下，门保持暂停'), (9., 12., '人物退开，门继续打开'),
              (12., 14., '人物停稳，场景演示结束')]
    timeline = {'fps': FPS, 'frames': FRAMES, 'duration_seconds': FRAMES / FPS,
                'scope': SCOPE, 'phases': [{'start_seconds': a, 'end_seconds': b, 'label_zh': label} for a, b, label in phases],
                'footer_zh': '骨骼驱动人物与脚本门动画；机器人保持静止',
                'samples': [{'frame': f, 'time_seconds': f / FPS, 'phase': phase(f, args.pause_angle)[0], 'door_angle_deg': phase(f, args.pause_angle)[1]} for f in range(FRAMES)]}
    write(REPORT / 'timeline.json', timeline)
    task = ROOT / 'third_party/IsaacLab-Arena/isaaclab_arena_environments/kitchen_bench/kitchen_bench_lightwheel_open_fridge.yaml'
    dependencies = [task, kitchen, layout_path, robot, ROOT / 'assets/people/mpfb_corridor/posed.blend']
    dependencies.extend(p for p in person.parent.rglob('*') if p.is_file())
    source = {'scope': SCOPE, 'task_id': 'kitchen_bench_lightwheel_open_fridge', 'original_task_kind': 'OpenDoorTask',
              'source_yaml': str(task.relative_to(ROOT)), 'source_yaml_sha256': sha(task),
              'scene': str(scene.relative_to(ROOT)), 'scene_sha256': sha(scene),
              'human_asset': str(person.relative_to(ROOT)), 'human_position_m': args.human_position, 'human_yaw_deg': args.human_yaw,
              'human_meshes': meshes, 'human_animation': 'Every rendered pose evaluated through original skeletal rig and skin weights. No interpolation between static mesh poses.',
              'door_prim': DOOR, 'joint_prim': JOINT, 'hinge_world_m': hinge.tolist(), 'axis_world_unit': axis.tolist(),
              'joint_limit_degrees': [joint.GetLowerLimitAttr().Get(), joint.GetUpperLimitAttr().Get()],
              'original_openness_threshold': .2, 'original_reset_openness': 0.,
              'native_predicate_evaluated': False, 'task_success_claim': False, 'no_simulation_started': True,
              'no_robot_trajectory': True, 'direct_door_transform_animation': True,
              'timeline': str((REPORT / 'timeline.json').relative_to(ROOT)), 'timeline_sha256': sha(REPORT / 'timeline.json'),
              'direct_source_dependency_sha256': {str(p.relative_to(ROOT)): sha(p) for p in dependencies}}
    write(REPORT / 'source.json', source)
    write(ROOT / 'configs/extensions/doors/hard_demo_v2/demo.json', {k: source[k] for k in ['scene', 'scene_sha256', 'timeline', 'scope', 'human_position_m', 'human_yaw_deg']})
    print(json.dumps({'scene': source['scene'], 'human_meshes': len(meshes), 'frames': FRAMES, 'human_position': args.human_position}, indent=2))


if __name__ == '__main__':
    main()
