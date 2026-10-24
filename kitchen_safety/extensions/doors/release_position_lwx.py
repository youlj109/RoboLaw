"""Optional bounded Cartesian-position retreat; no simulation state writes."""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

POLICY=dict(version='bounded_position_v1',max_joint_increment_rad=.025,max_rotation_departure_deg=10.,
            position_weight=1.,rotation_weight_m_per_rad=.005,joint_weight_m_per_rad=.0005,
            max_solver_evaluations=80,scope='Release only; original cuRobo, physical safety and contact gates remain mandatory')

def bounded_position_command(kin, seed, target, reference_rotation):
    """A proposal only: prioritize position, penalize rotation and joint travel."""
    seed = np.asarray(seed, dtype=float)
    low = np.maximum(kin.limits[:, 0] + 1e-8, seed - .025)
    high = np.minimum(kin.limits[:, 1] - 1e-8, seed + .025)
    if np.any(low >= high):
        raise ValueError('No bounded feasible joint interval')
    def residual(q):
        pose = kin.fk(q)
        angle = Rotation.from_matrix(pose[:3, :3] @ reference_rotation.T).as_rotvec()
        return np.r_[pose[:3, 3] - target, .005*angle, .0005*(q-seed)]
    result = least_squares(residual, np.clip(seed, low+1e-10, high-1e-10),
                           bounds=(low, high), max_nfev=80, ftol=1e-10, xtol=1e-10, gtol=1e-10)
    if not result.success:
        raise RuntimeError('Bounded release solver did not converge')
    q=result.x
    angle=float(np.rad2deg(Rotation.from_matrix(kin.fk(q)[:3,:3] @ reference_rotation.T).magnitude()))
    if not np.all(np.isfinite(q)) or np.abs(q-seed).max()>.025+1e-10 or angle>10.:
        raise RuntimeError('Bounded release command exceeded original increment or rotation guard')
    return q

