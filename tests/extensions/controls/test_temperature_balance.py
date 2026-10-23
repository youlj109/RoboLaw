import hashlib
import math

import numpy as np
import pytest
from pxr import Gf,Usd,UsdGeom,UsdPhysics

from kitchen_safety.extensions.controls.temperature_knob_physics import balanced_temperature_knob


def test_balance_preserves_source_mass_geometry_axis_and_limits(tmp_path):
    source=tmp_path/'source.usda'
    stage=Usd.Stage.CreateNew(str(source))
    root=UsdGeom.Xform.Define(stage,'/world')
    stage.SetDefaultPrim(root.GetPrim())
    body=UsdGeom.Xform.Define(stage,'/world/oven_right_group/Oven038_knob_temp')
    UsdPhysics.MassAPI.Apply(body.GetPrim()).CreateMassAttr(.2)
    shape=UsdGeom.Cube.Define(stage,str(body.GetPath())+'/collision')
    shape.CreateSizeAttr(.02)
    shape.AddTranslateOp().Set(Gf.Vec3d(.01,.02,.03))
    UsdPhysics.CollisionAPI.Apply(shape.GetPrim()).CreateCollisionEnabledAttr(True)
    joint=UsdPhysics.RevoluteJoint.Define(stage,str(body.GetPath())+'/knob_temp_joint')
    joint.CreateAxisAttr('X')
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.,0.,0.))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.,Gf.Vec3f(0.)))
    joint.CreateLowerLimitAttr(0.)
    joint.CreateUpperLimitAttr(180.)
    stage.GetRootLayer().Save()
    before=hashlib.sha256(source.read_bytes()).hexdigest()
    overlay,info=balanced_temperature_knob(source,tmp_path/'overlays')
    assert hashlib.sha256(source.read_bytes()).hexdigest()==before
    result=Usd.Stage.Open(str(overlay))
    rb=result.GetPrimAtPath(str(body.GetPath()))
    com=UsdPhysics.MassAPI(rb).GetCenterOfMassAttr().Get()
    assert np.allclose(com,[.01,0.,0.])
    assert UsdPhysics.MassAPI(rb).GetMassAttr().Get()==pytest.approx(.2)
    rj=UsdPhysics.RevoluteJoint(result.GetPrimAtPath(str(joint.GetPath())))
    assert rj.GetUpperLimitAttr().Get()==180.
    assert rj.GetAxisAttr().Get()=='X'
    drive=UsdPhysics.DriveAPI(rj.GetPrim(),'angular')
    assert drive.GetStiffnessAttr().Get()==0.
    assert drive.GetDampingAttr().Get()==pytest.approx(.05*math.pi/180.)
    assert UsdGeom.Cube(result.GetPrimAtPath(str(shape.GetPath()))).GetSizeAttr().Get()==.02
    assert info['source_sha256']==before
