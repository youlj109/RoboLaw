"""Conservative per-sample PT distance certificates; never exact distances."""
import math

VERSION = 'arena-pt-distance-bound-1'
NUMERICAL_MARGIN_M = 1e-4


def warning_outer_m(obj):
    """Same static boundary as PT v4; consumption rechecks the evaluator's value."""
    p = obj['distance']
    names = ('robot_approach_bound_mps', 'object_approach_bound_mps', 'reaction_s',
             'stop_s', 'brake_travel_m', 'contact_clearance_m', 'robot_uncertainty_m',
             'object_uncertainty_m', 'warning_band_m')
    if any(not finite_nonnegative(p.get(k)) for k in names):
        raise ValueError('Invalid PT distance parameters')
    return ((p['robot_approach_bound_mps'] + p['object_approach_bound_mps'])*p['reaction_s']
            + p['object_approach_bound_mps']*p['stop_s'] + p['brake_travel_m']
            + p['contact_clearance_m'] + p['robot_uncertainty_m']
            + p['object_uncertainty_m'] + p['warning_band_m'])


def finite_nonnegative(value):
    return (not isinstance(value, bool) and isinstance(value, (float, int))
            and math.isfinite(value) and value >= 0)


def apply_distance_bound(rule, raw):
    """Resolve only missing exact distance, leaving all other rule failures intact."""
    e = rule['evidence']
    geometry = raw.get('distance_geometry_evidence') or {}
    bound = geometry.get('distance_bound') or {}
    lower, threshold = bound.get('lower_bound_m'), e.get('warning_outer_m')
    if (rule.get('missing') != ['pt.protected_pairs.distance_m']
            or raw.get('distance_m') is not None
            or geometry.get('distance_status') != 'lower_bound'
            or geometry.get('coverage_complete') is not True
            or raw.get('distance_coverage_complete') is not True
            or bound.get('version') != VERSION
            or bound.get('method') != 'complete_actor_asset_union_aabb'
            or bound.get('numerical_margin_m') != NUMERICAL_MARGIN_M
            or not finite_nonnegative(lower) or not finite_nonnegative(threshold)
            or lower <= threshold
            or (e.get('actor') == 'held_object' and raw.get('held_contact_active') is not True)):
        return False
    rule.update(level='L0', complete=True, missing=[],
                mapping_origin=VERSION,
                note='Conservative distance lower bound exceeds the current PT warning boundary')
    e.update(distance_lower_bound_m=lower, distance_status='lower_bound')
    return True
