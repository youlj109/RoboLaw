import hashlib
import json
from pathlib import Path
import pytest
from pxr import Usd,UsdGeom,UsdPhysics

ROOT=Path(__file__).resolve().parents[3]
@pytest.mark.parametrize('version,height', [('v1', .174), ('v2', .164)])
def test_low_fixture_binds_source_and_native_collision_shapes(version, height):
    asset=ROOT/f'assets/extensions/controls/low_projecting_crossbar_{version}.usda'
    manifest=json.loads(asset.with_suffix('.manifest.json').read_text())
    assert hashlib.sha256(asset.read_bytes()).hexdigest()==manifest['output_sha256']
    assert hashlib.sha256((ROOT/manifest['source']).read_bytes()).hexdigest()==manifest['source_sha256']
    source=Usd.Stage.Open(str(ROOT/manifest['source']));low=Usd.Stage.Open(str(asset))
    colliders=lambda s:{str(p.GetPath()) for p in s.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI) and
        UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()}
    assert len(colliders(low))==5 and colliders(low)==colliders(source)
    tube=UsdGeom.Cylinder(low.GetPrimAtPath('/Asset/ProjectingTube'))
    assert tube.GetRadiusAttr().Get()==pytest.approx(.018)
    assert tube.GetHeightAttr().Get()==pytest.approx(.4)
    assert tube.GetAxisAttr().Get()=='Y'
    assert UsdGeom.Xformable(tube).GetLocalTransformation().ExtractTranslation()[2]==pytest.approx(height)
    assert UsdGeom.GetStageMetersPerUnit(low)==1.
