from pathlib import Path

import pytest
from pxr import Usd, UsdGeom, UsdPhysics

ROOT=Path(__file__).resolve().parents[3]


@pytest.mark.parametrize('asset,root,count,dynamic',[
    ('oven_glass_floor_lamp.usda','Lamp',4,True),
    ('oven_glass_floor_lamp_low.usda','Lamp',4,True),
    ('oven_corridor_guard.usda','Guard',3,False),
    ('oven_corridor_guard_low.usda','Guard',3,False),
    ('oven_corridor_guard_wide.usda','Guard',3,False),
    ('oven_corridor_guard_padded.usda','Guard',3,False),
    ('oven_corridor_guard_padded_v2.usda','Guard',3,False),
    ('oven_corridor_guard_endpad.usda','Guard',4,False),
])
def test_visible_supported_source_colliders(asset,root,count,dynamic):
    stage=Usd.Stage.Open(str(ROOT/'assets/extensions/controls'/asset))
    colliders=[p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
    assert len(colliders)==count
    assert stage.GetDefaultPrim().GetName()==root
    assert stage.GetDefaultPrim().HasAPI(UsdPhysics.RigidBodyAPI)==dynamic
    assert not any(p.IsA(UsdPhysics.Joint) for p in stage.Traverse())
    for prim in colliders:
        assert UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
        assert UsdGeom.Imageable(prim).ComputeVisibility()!='invisible'
    base=stage.GetPrimAtPath('/'+root+'/Base')
    box=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','proxy']).ComputeWorldBound(base).ComputeAlignedRange()
    assert box.GetMin()[2]==pytest.approx(0.,abs=1e-7)
    if dynamic:
        assert UsdPhysics.MassAPI(stage.GetDefaultPrim()).GetMassAttr().Get()==6
