import hashlib
import json
from pathlib import Path
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics

ROOT=Path(__file__).resolve().parents[3]


def test_authored_segments_tile_original_native_volume_with_no_disabled_shapes():
    asset=ROOT/'assets/extensions/controls/tall_projecting_crossbar_segmented_v5.usda'
    manifest=json.loads(asset.with_suffix('.manifest.json').read_text())
    assert hashlib.sha256(asset.read_bytes()).hexdigest()==manifest['output_sha256']
    source=ROOT/manifest['source']
    assert hashlib.sha256(source.read_bytes()).hexdigest()==manifest['source_sha256']
    before=Usd.Stage.Open(str(source));after=Usd.Stage.Open(str(asset))
    colliders=lambda stage:{str(p.GetPath()):p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)}
    old,new=colliders(before),colliders(after)
    assert len(old)==5 and len(new)==20
    assert all(UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() for p in new.values())
    intervals=[]
    for name,prim in new.items():
        if name.startswith('/Asset/ProjectingTube/'):
            tube=UsdGeom.Cylinder(prim)
            assert tube.GetRadiusAttr().Get()==manifest['radius_m']
            assert tube.GetHeightAttr().Get()==manifest['length_m']/16
            assert tube.GetAxisAttr().Get()=='Y'
            center=prim.GetAttribute('xformOp:translate').Get()[1]
            intervals.append([center-tube.GetHeightAttr().Get()/2,center+tube.GetHeightAttr().Get()/2])
            assert all(after.GetPrimAtPath(p) for p in prim.GetRelationship('material:binding').GetTargets())
        else:
            original=old[name]
            assert original.GetAllMetadata()==prim.GetAllMetadata()
            assert all(a.Get()==prim.GetAttribute(a.GetName()).Get() for a in original.GetAttributes())
    intervals.sort()
    assert len(intervals)==16
    assert np.allclose([intervals[0][0],intervals[-1][1]],[-.2,.2],atol=1e-14,rtol=0)
    assert np.allclose(np.array(intervals[:-1])[:,1],np.array(intervals[1:])[:,0],atol=1e-14,rtol=0)
    for key in ('metersPerUnit','upAxis','defaultPrim'):
        assert before.GetMetadata(key)==after.GetMetadata(key)
    assert (before.GetPrimAtPath('/Asset/ProjectingTube').GetAttribute('xformOp:translate').Get()==
            after.GetPrimAtPath('/Asset/ProjectingTube').GetAttribute('xformOp:translate').Get())
