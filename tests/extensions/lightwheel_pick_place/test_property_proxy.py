import numpy as np
from scipy.spatial import ConvexHull
from kitchen_safety.extensions.lightwheel_pick_place.property_proxy import padded_hull

def test_circumscribed_margin_covers_all_source_ball_points():
    rng=np.random.default_rng(42)
    source=rng.normal(size=(24,3))*.05
    vertices,faces,report=padded_hull(source,.06)
    equations=ConvexHull(vertices).equations
    directions=rng.normal(size=(2000,3));directions/=np.linalg.norm(directions,axis=1)[:,None]
    for p in source:
        points=p+.06*directions
        assert np.max(points@equations[:,:3].T+equations[:,3])<1e-10
    assert report['planning_margin_m']==.06
    for face in faces:
        a,b,c=vertices[face]
        assert np.dot(np.cross(b-a,c-a),a-vertices.mean(0))>=-1e-12
