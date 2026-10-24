"""Optional final release-target hold with the original measured-joint bound."""
import numpy as np

POLICY = dict(version='bounded_position_hold_v2', max_joint_increment_rad=.025,
              scope='After bounded_position retreat, retain the final command target during the five existing hold actions. Original command and physical safety gates remain mandatory.')

def bounded_hold_command(observed, target):
    observed = np.asarray(observed, dtype=float)
    target = np.asarray(target, dtype=float)
    if observed.shape != (7,) or target.shape != (7,) or not np.isfinite(observed).all() or not np.isfinite(target).all():
        raise ValueError('Expected finite seven-joint observation and target')
    delta = target - observed
    return observed + delta / max(1., float(np.abs(delta).max()) / .025)
