"""Optional same-policy wrist alignment during a straight handle approach."""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from kitchen_safety.extensions.doors.task_model import grasp_rotation

def pregrasp_rotation(approach, policy):
    return grasp_rotation(approach, policy.get('pregrasp_roll_deg', policy['grasp_roll_deg']),
                          policy.get('pregrasp_pitch_deg', policy.get('grasp_pitch_deg', 0.)))

def approach_rotation(approach, policy, fraction):
    if not np.isfinite(fraction) or not 0. <= fraction <= 1.:
        raise ValueError('Approach fraction must be finite and in [0,1]')
    end = policy.get('approach_alignment_end_fraction', 1.)
    if not np.isfinite(end) or not .05 <= end <= 1.:
        raise ValueError('Alignment end fraction must be finite and in [.05,1]')
    final = grasp_rotation(approach, policy['grasp_roll_deg'], policy.get('grasp_pitch_deg', 0.))
    if 'pregrasp_roll_deg' not in policy and 'pregrasp_pitch_deg' not in policy:
        return final
    fraction = min(1., fraction / end)
    initial = pregrasp_rotation(approach, policy)
    if fraction == 0.: return initial
    if fraction == 1. or np.array_equal(initial, final): return final
    return Slerp([0.,1.], Rotation.from_matrix(np.stack([initial, final])))([fraction]).as_matrix()[0]
