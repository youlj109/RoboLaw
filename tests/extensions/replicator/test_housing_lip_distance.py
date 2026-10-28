import numpy as np
from scripts.extensions.replicator.refine_l_island_housing_grasp import closest_segments


def test_parallel_source_edge_distance_is_not_equipment_box_distance():
    r=closest_segments(np.array([.2,.03]),np.array([.4,.03]),np.array([0.,0.]),np.array([1.,0.]))
    assert np.isclose(r['horizontal_distance_m'],.03)
    assert .2<=r['guide_fraction']<=.4


def test_real_edge_crossing_and_endpoint_clamping():
    r=closest_segments(np.array([.5,-1.]),np.array([.5,1.]),np.array([0.,0.]),np.array([1.,0.]))
    assert r['horizontal_distance_m']==0. and r['guide_fraction']==.5
    r=closest_segments(np.array([2.,0.]),np.array([3.,0.]),np.array([0.,0.]),np.array([1.,0.]))
    assert r['horizontal_distance_m']==1. and r['guide_fraction']==1.
