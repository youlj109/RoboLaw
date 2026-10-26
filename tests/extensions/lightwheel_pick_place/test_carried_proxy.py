import itertools
import numpy as np
import pytest
from scipy.spatial import ConvexHull
from kitchen_safety.extensions.lightwheel_pick_place.carried_proxy import distributed_interior_spheres
def shaft():
    return np.array(list(itertools.product([-.16,.16],[-.006,.006],[-.004,.004])))
def test_thin_shaft_gets_centers_at_both_ends_and_within_budget():
    centers,radii,report=distributed_interior_spheres(shaft())
    assert len(centers)<=100 and centers[:,0].min()<-.13 and centers[:,0].max()>.13
    assert report['maximum_source_vertex_distance_outside_proxy_m']<.012
    assert not report['full_surface_coverage']
def test_every_sphere_remains_inside_source_hull():
    vertices=shaft();centers,radii,report=distributed_interior_spheres(vertices);planes=ConvexHull(vertices).equations
    assert np.all(centers@planes[:,:3].T+planes[:,3]+radii[:,None]<=1e-7)
    assert report['all_spheres_inside_original_source_convex_hull']
def test_deterministic_geometry():
    a=distributed_interior_spheres(shaft());b=distributed_interior_spheres(shaft())
    assert np.array_equal(a[0],b[0]) and np.array_equal(a[1],b[1])
@pytest.mark.parametrize('bad',[np.zeros((3,3)),np.array([[0,0,0],[1,0,0],[0,1,0],[0,0,float('nan')]])])
def test_invalid_source_vertices_rejected(bad):
    with pytest.raises(ValueError):distributed_interior_spheres(bad)
