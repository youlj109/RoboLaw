import numpy as np
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld


def test_rebinding_uses_original_local_hull_after_obstacle_moves():
    w=CollisionWorld.__new__(CollisionWorld);w.env_root=None;w._root_reference_transforms={'/World/envs/env_0/banana_1':np.eye(4)}
    name='/World/envs/env_0/banana_1/mesh';vertices=np.array([[0.,0,0],[1,0,0],[0,1,0],[0,0,1]])
    w.meshes={name:(vertices+10,np.array([[0,1,2]]))};w.objects=[(name,None,None,None)];w._object_reference=[(None,None,None,None,(vertices.copy(),np.array([[0,1,2]])))];w._convex=lambda v:(None,v)
    w.bind_pick('banana_1')
    np.testing.assert_array_equal(w.lime_vertices,vertices)
    assert w.attachment is None


def test_only_actual_plate_and_its_children_are_legal_source():
    w=CollisionWorld.__new__(CollisionWorld);w.source_surface_root='/World/envs/env_0/plate';w.source_surface_suffix='/plate'
    assert w._is_source_surface('/World/envs/env_0/plate/plate_large/mesh')
    assert not w._is_source_surface('/World/envs/env_0/plate_other/mesh')
    assert not w._is_source_surface('/World/envs/env_1/plate/mesh')
