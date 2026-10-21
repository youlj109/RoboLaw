"""Standalone USD compositions sharing the exact local kitchen and prop assets.

These are scene previews; the Arena factory supplies the task and robot controller.
"""
import json
import math
import os
from pathlib import Path
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

ROOT=Path(__file__).resolve().parents[1]
layout=json.loads((ROOT/'configs/layout.json').read_text())

def reference(s,name,path,position=(0,0,0),yaw=0):
    p=UsdGeom.Xform.Define(s,'/World/'+name)
    p.GetPrim().GetReferences().AddReference(os.path.relpath(path,ROOT/'scenes'))
    xf=UsdGeom.Xformable(p);xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*position))
    if yaw: xf.AddRotateZOp().Set(yaw)
    return p

def build():
    out=ROOT/'scenes';out.mkdir(exist_ok=True)
    s=Usd.Stage.CreateNew(str(out/'base.usda'))
    UsdGeom.SetStageUpAxis(s,'Z');UsdGeom.SetStageMetersPerUnit(s,1.)
    s.SetDefaultPrim(UsdGeom.Xform.Define(s,'/World').GetPrim())
    physics=UsdPhysics.Scene.Define(s,'/World/Physics')
    physics.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));physics.CreateGravityMagnitudeAttr(9.81)
    reference(s,'Kitchen',ROOT/'assets/kitchen/scene.usd')
    reference(s,'Mustard',ROOT/'assets/robolab/objects/hope/mustard_bottle.usd',layout['mustard_position'])
    reference(s,'Bowl',ROOT/'assets/robolab/objects/ycb/bowl.usd',layout['bowl_position'])
    prefix=ROOT/'assets/nvidia/Assets/Isaac/6.1/Isaac/IsaacLab/Arena/assets'
    robot=reference(s,'Robot',prefix/'robot_library/droid/franka_robotiq_2f_85_flattened.usd',
                    (layout['robot_position'][0],layout['robot_position'][1],layout['stand_height']),layout['robot_yaw_deg'])
    # The task's robot stand is composed by Arena; a matching static pedestal is
    # sufficient in the standalone composition without importing the runtime.
    stand=UsdGeom.Cube.Define(s,'/World/Stand');stand.CreateSizeAttr(1.)
    xf=UsdGeom.Xformable(stand);xf.AddTranslateOp().Set(Gf.Vec3d(*layout['robot_position'])+Gf.Vec3d(0,0,.4))
    xf.AddScaleOp().Set(Gf.Vec3f(.40,.40,.8));stand.CreateDisplayColorAttr([(.2,.23,.25)])
    UsdPhysics.CollisionAPI.Apply(stand.GetPrim())
    # Root remains fixed, as in the original robot USD. Configure pose targets.
    targets=[0,-36,0,-144,0,108,0]
    for p in Usd.PrimRange(robot.GetPrim()):
        if p.IsA(UsdPhysics.RevoluteJoint) and p.GetName().startswith('panda_joint'):
            idx=int(p.GetName()[-1])-1
            if idx < 7:
                drive=UsdPhysics.DriveAPI.Apply(p,'angular')
                drive.CreateTargetPositionAttr(targets[idx]);drive.CreateStiffnessAttr(400.)
                drive.CreateDampingAttr(80.)
    dome=UsdLux.DomeLight.Define(s,'/World/Dome');dome.CreateIntensityAttr(650.)
    sun=UsdLux.DistantLight.Define(s,'/World/Sun');sun.CreateIntensityAttr(1800.)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(25,-35,-25))
    for name,eye,target in [('Overview',layout['camera_eye'],layout['camera_target']),
                            ('Detail',[2.55,-2.65,2.05],[2.3,-.45,1.02])]:
        camera=UsdGeom.Camera.Define(s,'/World/'+name)
        camera.CreateFocalLengthAttr(34.)
        camera.CreateClippingRangeAttr(Gf.Vec2f(.05,100.))
        m=Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse()
        UsdGeom.Xformable(camera).AddTransformOp().Set(m)
    s.GetRootLayer().Save()
    for variant in ['human','property','robot']:
        st=Usd.Stage.CreateNew(str(out/f'{variant}.usda'))
        st.GetRootLayer().subLayerPaths=['base.usda']
        st.SetDefaultPrim(st.GetPrimAtPath('/World'))
        if variant=='human': reference(st,'Person',ROOT/layout['human'].get('asset','assets/safety/person.usda'),
                                       layout['human']['position'],layout['human'].get('yaw_deg',0))
        elif variant=='property':
            for name in ['glass','plate','phone']:
                reference(st,name.title(),ROOT/layout['property'].get(name+'_asset',f'assets/safety/{name}.usda'),
                          layout['property'][name+'_position'])
        else: reference(st,'Barrier',ROOT/layout['robot'].get('barrier_asset', 'assets/safety/barrier.usda'),layout['robot']['barrier_position'],
                        layout['robot'].get('barrier_yaw_deg', 0))
        st.GetRootLayer().Save()

if __name__=='__main__':build()
