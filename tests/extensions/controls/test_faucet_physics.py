import hashlib
import math

import pytest
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from kitchen_safety.extensions.controls.faucet_physics import balanced_faucet


def source(tmp_path):
    path = tmp_path/'source.usda'
    stage = Usd.Stage.CreateNew(str(path))
    root = UsdGeom.Xform.Define(stage, '/world')
    stage.SetDefaultPrim(root.GetPrim())
    body = UsdGeom.Xform.Define(stage, '/world/sink_left_group/Sink009_handle').GetPrim()
    UsdPhysics.MassAPI.Apply(body).CreateMassAttr(.05)
    UsdGeom.Cylinder.Define(stage, str(body.GetPath())+'/geometry').CreateRadiusAttr(.02)
    joint = UsdPhysics.Joint.Define(stage, str(body.GetPath())+'/handle_joint')
    joint.CreateLocalPos1Attr(Gf.Vec3f(.02, .18, -.007))
    for axis in ('rotX', 'rotY'):
        limits = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        limits.CreateLowAttr(0.)
        limits.CreateHighAttr(45.)
        UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis).CreateMaxForceAttr(0.)
    stage.GetRootLayer().Save()
    return path


def test_only_passive_balance_and_damping_are_authored(tmp_path):
    path = source(tmp_path)
    before = hashlib.sha256(path.read_bytes()).hexdigest()
    overlay, provenance = balanced_faucet(path, tmp_path/'overlays')
    assert hashlib.sha256(path.read_bytes()).hexdigest() == before
    stage = Usd.Stage.Open(str(overlay))
    body = stage.GetPrimAtPath('/world/sink_left_group/Sink009_handle')
    joint = UsdPhysics.Joint(stage.GetPrimAtPath(str(body.GetPath())+'/handle_joint'))
    assert UsdPhysics.MassAPI(body).GetMassAttr().Get() == pytest.approx(.05)
    assert UsdPhysics.MassAPI(body).GetCenterOfMassAttr().Get() == joint.GetLocalPos1Attr().Get()
    assert UsdGeom.Cylinder(stage.GetPrimAtPath(str(body.GetPath())+'/geometry')).GetRadiusAttr().Get() == .02
    for axis in ('rotX', 'rotY'):
        limits = UsdPhysics.LimitAPI(joint.GetPrim(), axis)
        assert (limits.GetLowAttr().Get(), limits.GetHighAttr().Get()) == (0., 45.)
        drive = UsdPhysics.DriveAPI(joint.GetPrim(), axis)
        assert drive.GetStiffnessAttr().Get() == 0.
        assert drive.GetDampingAttr().Get() == pytest.approx(.05*math.pi/180.)
    assert provenance['source_sha256'] == before


def test_changed_cached_overlay_is_rejected(tmp_path):
    path = source(tmp_path)
    overlay, _ = balanced_faucet(path, tmp_path/'overlays')
    overlay.write_text(overlay.read_text()+'\n')
    with pytest.raises(ValueError, match='Changed cached'):
        balanced_faucet(path, tmp_path/'overlays')
