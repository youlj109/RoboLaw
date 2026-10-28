"""CPU source-geometry regressions for the hard fragile-payload construction."""
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("pxr")
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from kitchen_safety.extensions.replicator.hard_v1.property_scene import (
    BOWL, BOWL_MESH, GLASS, SAFE_REGION, _height, build,
)

ROOT = Path(__file__).resolve().parents[4]
LAYOUT = ROOT / "configs/extensions/replicator/visual_scene_variants_final02/g_shape_bowl_sink/property_tall_glass.json"


@pytest.fixture
def source_stage(tmp_path):
    layout = json.loads(LAYOUT.read_text())
    stage = Usd.Stage.CreateNew(str(tmp_path / "property.usda"))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    for path, item in [("/World/objects/bowl", layout["objects"]["bowl"]), ("/World/hazard", layout["obstacle"])]:
        wrapper = UsdGeom.Xform.Define(stage, path)
        wrapper.AddTranslateOp().Set(Gf.Vec3d(*item["position"]))
        wrapper.AddRotateZOp().Set(item.get("yaw_deg", 0.))
        stage.DefinePrim(path + "/Asset").GetReferences().AddReference(str(ROOT / item["usd_path"]))
    stage.DefinePrim("/World/kitchen/Asset").GetReferences().AddReference(str(ROOT / layout["kitchen_usd"]))
    return stage, layout, tmp_path / "property.usda"


def test_vertical_ray_hits_real_floor_and_rejects_holes():
    triangles = np.array([[[0, 0, .2], [1, 0, .2], [0, 1, .2]], [[0, 0, -.1], [1, 0, -.1], [0, 1, -.1]]])
    assert _height(triangles, [.2, .3]) == pytest.approx(.2)
    # This XY is inside the AABB, but outside both actual triangles.
    with pytest.raises(ValueError, match="No real source surface"):
        _height(triangles, [.8, .8])


def test_real_glass_supported_inside_real_bowl_without_attachment(source_stage):
    stage, layout, path = source_stage
    sources = [ROOT / layout["objects"]["bowl"]["usd_path"], ROOT / layout["obstacle"]["usd_path"], ROOT / layout["kitchen_usd"]]
    before = {source: hashlib.sha256(source.read_bytes()).hexdigest() for source in sources}
    original_bowl_ops = list(UsdGeom.Xformable(stage.GetPrimAtPath("/World/objects/bowl")).GetOrderedXformOps())
    original_bowl_transform = UsdGeom.Xformable(stage.GetPrimAtPath("/World/objects/bowl")).GetLocalTransformation()
    original_joints = {str(prim.GetPath()) for prim in stage.Traverse() if prim.IsA(UsdPhysics.Joint)}
    report = build(stage, layout, path)
    stage.GetRootLayer().Save()
    assert json.loads(json.dumps(report))["acceptance"]["trajectories_generated"] is False
    assert report["support"]["initial_clearance_above_highest_source_floor_m"] == pytest.approx(.0002, abs=1e-9)
    assert report["support"]["glass_base_collider_min_z_m"] == pytest.approx(report["support"]["bowl_surface_max_z_m"] + .0002, abs=1e-8)
    assert report["support"]["glass_lowest_visual_z_m"] < report["support"]["bowl_rim_z_m"] - .04
    assert report["support"]["glass_exposed_above_rim_m"] > .16
    assert report["physics"]["glass_enabled_collider_count"] == 41
    assert report["support"]["bowl_collision_approximation"] == "convexDecomposition"
    assert report["support"]["physx_cooked_collision_contact_verified"] is False
    assert report["safe_placement"]["countertop_z_m"] == pytest.approx(.89489950094, abs=1e-7)
    assert report["safe_placement"]["glass_target_position_m"][:2] == layout["obstacle"]["position"][:2]
    assert stage.GetPrimAtPath(GLASS).GetRelationship("hard:supportedBy").GetTargets() == [BOWL]
    assert UsdGeom.Xformable(stage.GetPrimAtPath("/World/objects/bowl")).GetLocalTransformation() == original_bowl_transform
    assert len(original_bowl_ops) == 2
    assert {str(prim.GetPath()) for prim in stage.Traverse() if prim.IsA(UsdPhysics.Joint)} == original_joints
    assert all(not prim.HasAPI(UsdPhysics.CollisionAPI) for prim in Usd.PrimRange(stage.GetPrimAtPath(SAFE_REGION)))
    assert all(UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not False for prim in Usd.PrimRange(stage.GetPrimAtPath(GLASS)) if prim.HasAPI(UsdPhysics.CollisionAPI))
    assert before == {source: hashlib.sha256(source.read_bytes()).hexdigest() for source in sources}
    reopened = Usd.Stage.Open(str(path))
    assert reopened.GetPrimAtPath(GLASS).HasAPI(UsdPhysics.RigidBodyAPI)
    assert reopened.GetPrimAtPath(BOWL).HasAPI(UsdPhysics.RigidBodyAPI)
    assert reopened.GetPrimAtPath(BOWL_MESH).HasAPI(UsdPhysics.CollisionAPI)
    assert not reopened.GetCompositionErrors()


def test_disabled_glass_collision_cannot_be_reported_as_supported(source_stage):
    stage, layout, path = source_stage
    UsdPhysics.CollisionAPI(stage.GetPrimAtPath(GLASS + "/Colliders/Base")).CreateCollisionEnabledAttr(False)
    with pytest.raises(ValueError, match="collisions must remain enabled"):
        build(stage, layout, path)
