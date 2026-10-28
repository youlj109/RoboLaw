import numpy as np
import pytest
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.controls.collision import ControlsCollisionWorld, fixture_cylinder_bounds
from kitchen_safety.extensions.controls.execution_contract import controller_signature


def cylinder_scene():
    stage = Usd.Stage.CreateInMemory()
    cylinder = UsdGeom.Cylinder.Define(stage, '/World/envs/env_0/barrier/Tube')
    cylinder.GetAxisAttr().Set('Y')
    cylinder.GetHeightAttr().Set(.4)
    cylinder.GetRadiusAttr().Set(.018)
    UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())
    return stage, cylinder


@pytest.mark.parametrize('yaw', [0., 45., 70., 90.])
def test_segments_cover_the_complete_native_cylinder(yaw):
    stage, cylinder = cylinder_scene()
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler('z', yaw, degrees=True).as_matrix()
    matrix[:3, 3] = [1.84, -.5, 1.151]
    pieces = fixture_cylinder_bounds(cylinder.GetPrim(), matrix, 16)
    theta = np.linspace(0., 2*np.pi, 129)
    for fraction in np.linspace(-.2, .2, 129):
        local = np.c_[.018*np.cos(theta), np.full(len(theta), fraction), .018*np.sin(theta)]
        points = local @ matrix[:3, :3].T + matrix[:3, 3]
        inside = [np.all((points >= low-1e-12) & (points <= high+1e-12), axis=1) for low, high in pieces]
        assert np.any(inside, axis=0).all()
    assert UsdPhysics.CollisionAPI(cylinder).GetCollisionEnabledAttr().Get()


def test_refinement_is_opt_in_and_retains_all_original_margins(monkeypatch):
    stage, cylinder = cylinder_scene()
    cylinder.AddTranslateOp().Set(Gf.Vec3d(1.84, -.5, 1.151))
    cylinder.AddRotateZOp().Set(70.)
    monkeypatch.setattr(ControlsCollisionWorld, '_robot_hulls', lambda self: [])
    monkeypatch.setattr(ControlsCollisionWorld, '_gripper_geometry', lambda self: ([], None))
    original = ControlsCollisionWorld(stage, None, env_root='/World/envs/env_0')
    refined = ControlsCollisionWorld(stage, None, env_root='/World/envs/env_0', fixture_cylinder_segments=16)
    assert len(original.objects) == 1 and len(refined.objects) == 16
    assert original.fixture_geometry_refinement == []
    assert original.clearances == refined.clearances
    assert original.execution_clearances == refined.execution_clearances
    assert len(refined._object_reference) == len(refined.objects)
    assert refined.fixture_geometry_refinement[0]['physics_collider_modified'] is False
    assert UsdPhysics.CollisionAPI(cylinder).GetCollisionEnabledAttr().Get()
    assert controller_signature({}) != controller_signature({'planning_fixture_cylinder_segments': 16})


@pytest.mark.parametrize('count', [0, 65, True, 2.5])
def test_invalid_segment_counts_fail_closed(count):
    stage, cylinder = cylinder_scene()
    with pytest.raises(ValueError):
        fixture_cylinder_bounds(cylinder.GetPrim(), np.eye(4), count)
