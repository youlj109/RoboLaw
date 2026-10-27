import hashlib
from pathlib import Path

import numpy as np
import pytest
from pxr import Usd,UsdGeom,UsdPhysics
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.controls.lamp_physics import balanced_reading_lamp,weighted_lamp_mass_properties

SOURCE=Path(__file__).resolve().parents[3]/'assets/extensions/controls/visual_20261027/reading_lamp_r01.usda'


def test_declared_mass_model_is_positive_and_supported():
    model=weighted_lamp_mass_properties()
    assert np.isclose(model['mass_kg'],6.)
    assert np.linalg.norm(model['center_of_mass_local_m'][:2])<.16
    tensor=np.asarray(model['inertia_tensor_body_kg_m2'])
    axes=np.asarray(model['principal_axes_matrix'])
    diagonal=np.asarray(model['diagonal_inertia_kg_m2'])
    assert np.all(diagonal>0)
    assert np.allclose(axes@np.diag(diagonal)@axes.T,tensor)


def test_only_mass_distribution_changes_and_body_stays_dynamic(tmp_path):
    digest=hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    output,info=balanced_reading_lamp(SOURCE,tmp_path)
    assert hashlib.sha256(SOURCE.read_bytes()).hexdigest()==digest
    stage=Usd.Stage.Open(str(output))
    root=stage.GetDefaultPrim()
    assert not UsdPhysics.RigidBodyAPI(root).GetKinematicEnabledAttr().Get()
    assert {attr.name for attr in stage.GetRootLayer().GetPrimAtPath(root.GetPath()).attributes}=={
        'physics:mass','physics:centerOfMass','physics:diagonalInertia','physics:principalAxes'}
    original=Usd.Stage.Open(str(SOURCE))
    for prim in original.Traverse():
        counterpart=stage.GetPrimAtPath(prim.GetPath())
        if prim.IsA(UsdGeom.Mesh):
            assert np.array_equal(UsdGeom.Mesh(prim).GetPointsAttr().Get(),UsdGeom.Mesh(counterpart).GetPointsAttr().Get())
        for attr in prim.GetAttributes():
            if not attr.GetName().startswith('physics:'):
                assert attr.Get()==counterpart.GetAttribute(attr.GetName()).Get()
    mass=UsdPhysics.MassAPI(root)
    q=mass.GetPrincipalAxesAttr().Get()
    axes=Rotation.from_quat([*q.GetImaginary(),q.GetReal()]).as_matrix()
    tensor=axes@np.diag(mass.GetDiagonalInertiaAttr().Get())@axes.T
    assert np.allclose(tensor,info['inertia_tensor_body_kg_m2'],atol=1e-6)


def test_changed_cached_overlay_is_rejected(tmp_path):
    output,_=balanced_reading_lamp(SOURCE,tmp_path)
    output.write_text(output.read_text()+'\n')
    with pytest.raises(ValueError,match='Changed cached'):
        balanced_reading_lamp(SOURCE,tmp_path)
