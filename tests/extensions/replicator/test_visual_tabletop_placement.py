"""Access reservations inspect source geometry, rather than loose bounds."""
import fcl
import numpy as np

from kitchen_safety.extensions.replicator.visual_tabletop_placement import protected_overlap


def test_loose_source_bounds_do_not_falsely_block_access_column():
    source=fcl.CollisionObject(fcl.Box(.1,.1,.1),fcl.Transform(np.zeros(3)))
    shapes=[('actual_small_part',source,np.full(3,-1.),np.full(3,1.))]
    columns=[dict(role='initial_grasp',center=[.5,0.,0.],radius_m=.1,z_range_m=[-.2,.2])]
    assert protected_overlap(shapes,columns) is None


def test_source_collision_with_access_column_is_rejected():
    source=fcl.CollisionObject(fcl.Box(.2,.2,.2),fcl.Transform(np.zeros(3)))
    shapes=[('actual_part',source,np.full(3,-.1),np.full(3,.1))]
    columns=[dict(role='goal_opening',center=[.15,0.,0.],radius_m=.1,z_range_m=[-.2,.2])]
    result=protected_overlap(shapes,columns)
    assert result['role']=='goal_opening'
    assert result['hazard_part']=='actual_part'
    assert result['check']=='source_shape_vs_reserved_access_cylinder'


def test_above_access_column_does_not_remove_real_source_checks():
    # This helper reserves endpoint access only. A part higher than the column
    # is not claimed safe; the caller still runs all source-world/robot checks.
    source=fcl.CollisionObject(fcl.Box(.2,.2,.2),fcl.Transform(np.array([0.,0.,1.])))
    shapes=[('high_part',source,np.array([-.1,-.1,.9]),np.array([.1,.1,1.1]))]
    columns=[dict(role='goal_opening',center=[0.,0.,0.],radius_m=.1,z_range_m=[-.2,.2])]
    assert protected_overlap(shapes,columns) is None
