"""Analytic safe/unsafe controls for the same source-shape narrow-phase audit."""
import fcl
import numpy as np
import pytest
from kitchen_safety.geometry_audit import minimum_distance
from scripts.extensions.controls.screen_benchmark_necessity import summarize
from scripts.extensions.controls.render_first_violation import source_surface_witness


def sphere(name, x):
    center=np.array([x,0.,0.])
    return (name,fcl.CollisionObject(fcl.Sphere(.01),fcl.Transform(center)),
            center-.01,center+.01)


@pytest.mark.parametrize('separation,gap,valid,intersecting',[
    (.06,.04,True,False),(.03,.01,False,False),(.015,0.,False,True)])
def test_distance_margin_and_collision_are_distinct(separation,gap,valid,intersecting):
    witness=minimum_distance([sphere('/panda/panda_link7',0.)],
        [sphere('/person/hand',separation)],.02)
    assert witness['distance_m']==pytest.approx(gap,abs=1e-8)
    assert witness['valid'] is valid
    assert witness['intersecting'] is intersecting
    assert witness['robot_part']=='/panda/panda_link7'
    assert witness['obstacle']=='/person/hand'


def test_analytic_sequence_reports_first_unsafe_not_first_collision():
    rows=[]
    for index,separation in enumerate([.06,.03,.015]):
        witness=minimum_distance([sphere('/panda/panda_link7',0.)],
            [sphere('/person/hand',separation)],.02)
        rows.append(dict(time_s=index/15,**witness))
    result=summarize(rows)
    assert result['first_violation']['time_s']==pytest.approx(1/15)
    assert not result['first_violation']['intersecting']
    assert result['minimum']['intersecting']


def test_surface_witness_is_bound_to_reported_distance():
    left,right=sphere('link',0.),sphere('hand',.03)
    witness=source_surface_witness(left,right,.01)
    assert np.linalg.norm(np.diff(witness['points_world_m'],axis=0))==pytest.approx(.01)
    with pytest.raises(ValueError,match='disagree'):
        source_surface_witness(left,right,.02)
