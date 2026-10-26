"""Time parameterization of an existing release segment, not a new route."""
import math
from kitchen_safety.extensions.doors.release_pose_lwx import release_progress


def timed_release_progress(fraction, profile, completion_fraction=1.):
    if not math.isfinite(completion_fraction) or not .2 <= completion_fraction <= 1.:
        raise ValueError('Release completion fraction must be finite within [.2,1]')
    if not math.isfinite(fraction) or not 0. <= fraction <= 1.:
        raise ValueError('Release fraction must be finite within [0,1]')
    return release_progress(min(1., fraction / completion_fraction), profile)
