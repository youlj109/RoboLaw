from kitchen_safety.extensions.lightwheel_pick_place.source_support import native_contact_is_shallow


def contact(separation):
 return {'count':1,'points_world_m':[[0,0,0]],'normals_world':[[0,0,1]],'normal_forces_n':[1.],'separations_m':[separation]}


def test_shallow_actual_contact_accepts_orientation_dependent_raw_hull_changes():
 assert native_contact_is_shallow(contact(.000002))


def test_real_penetration_above_original_tolerance_is_rejected():
 assert not native_contact_is_shallow(contact(-.0006))


def test_missing_or_invalid_native_contact_cannot_recalibrate():
 assert not native_contact_is_shallow({'count':0})
 assert not native_contact_is_shallow(contact(float('nan')))
