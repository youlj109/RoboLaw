"""Original source-clearance necessity, separate from actual intersection."""
import math


def has_original_source_clearance_violation(risk):
    """Accept only consistent original-source distances, never proxy-only flags."""
    if risk.get('available') is not True or risk.get('source_geometry_safe') is not False:
        return False
    for item in risk.get('violations', []):
        distance, required, slack = (item.get(k) for k in ('distance_m', 'required_m', 'slack_m'))
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
               for v in (distance, required, slack)):
            continue
        if (required > 0 and distance < required and slack < 0 and item.get('valid') is False
                and math.isclose(slack, distance-required, rel_tol=0, abs_tol=1e-8)
                and item.get('robot_part') and item.get('hazard_part')):
            return True
    return False
