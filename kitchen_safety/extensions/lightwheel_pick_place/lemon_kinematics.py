"""Preserve B02 baseline IK tolerance independently of later banana tuning.

The original B02 successful episode used the 0.001 weighted pose residual.
Keep shared and banana kinematics untouched; no execution safety margin changes.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from kitchen_safety.planning import DroidKinematics

class LemonKinematics(DroidKinematics):
    def ik(self, target, seed, attempts=12):
        rng = np.random.default_rng(17)
        def residual(q):
            t = self.fk(q)
            return np.r_[t[:3,3]-target[:3,3], .25*Rotation.from_matrix(target[:3,:3]@t[:3,:3].T).as_rotvec()]
        best = None
        for i in range(attempts):
            x = np.clip(seed if i == 0 else rng.uniform(self.limits[:,0],self.limits[:,1]),self.limits[:,0]+1e-5,self.limits[:,1]-1e-5)
            r = least_squares(lambda v: np.r_[residual(v), .001*(v-seed)],x,bounds=(self.limits[:,0]+1e-6,self.limits[:,1]-1e-6),max_nfev=160,gtol=1e-8)
            err = np.linalg.norm(residual(r.x))
            if best is None or err < best[0]: best=(err,r.x)
            if err < .001: return r.x
        raise RuntimeError(f'IK failed: pose residual {best[0]:.5f}, target {target[:3,3]}')
