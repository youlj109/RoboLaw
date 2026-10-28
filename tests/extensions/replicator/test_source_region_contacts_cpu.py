from types import SimpleNamespace

import numpy as np

from scripts.extensions.replicator.banana_region_contacts_cpu import SourceRegionContacts, triangle_model


def fixtures(right_x=0.):
    geometry = SimpleNamespace(vertices=np.array([[-1., -1., 0.], [1., -1., 0.], [0., 1., 0.]]),
        triangles=np.array([[0, 1, 2]]), provenance={"source_sha256": "test_triangle"})
    def finger(x):
        triangle = np.array([[[x-.1, 0., -1.], [x+.1, 0., -1.], [x, 0., 1.]]])
        return [("mesh", triangle_model(triangle))]
    parts = {"left": finger(0.), "right": finger(right_x)}
    world = SimpleNamespace(payload_pose=np.eye(4), gripper_root="base",
        GRIPPER_JOINT_NAMES=(), gripper_joint_positions=(),
        _gripper_links=lambda _: {"left": np.eye(4), "right": np.eye(4)})
    kin = SimpleNamespace(links=lambda _: {"base": np.eye(4)})
    return geometry, parts, world, kin


def test_known_square_partition_area_and_central_surface_contact():
    geometry, parts, world, kin = fixtures()
    model = SourceRegionContacts(geometry, [[-.2, -.2, -.1], [.2, .2, .1]], parts)
    assert model.provenance["original_area_m2"] == 2.
    assert abs(model.provenance["partition_area_m2"]-2.) < 1e-12
    result = model.at(world, kin, np.empty(0))
    assert result["bilateral_all_central"]
    assert all(row["inside"] > 0 and row["outside"] == 0 for row in result["fingers"].values())


def test_outside_surface_contact_is_not_accepted_by_inside_contact():
    geometry, parts, world, kin = fixtures(right_x=.35)
    model = SourceRegionContacts(geometry, [[-.2, -.2, -.1], [.2, .2, .1]], parts)
    result = model.at(world, kin, np.empty(0))
    assert not result["bilateral_all_central"]
    assert result["fingers"]["left"]["central_only"]
    assert result["fingers"]["right"]["inside"] == 0
    assert result["fingers"]["right"]["outside"] > 0


def test_partial_finger_contact_crossing_region_boundary_rejected():
    geometry, parts, world, kin = fixtures(right_x=.19)
    model = SourceRegionContacts(geometry, [[-.2, -.2, -.1], [.2, .2, .1]], parts)
    result = model.at(world, kin, np.empty(0))
    assert result["fingers"]["right"]["inside"] > 0
    assert result["fingers"]["right"]["outside"] > 0
    assert not result["bilateral_all_central"]
