"""USD-derived kinematics and deterministic IK for the DROID arm."""
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[1]
ROBOT_USD = ROOT/'assets/nvidia/Assets/Isaac/6.1/Isaac/IsaacLab/Arena/assets/robot_library/droid/franka_robotiq_2f_85_flattened.usd'

def transform(pos, quat):
    t = np.eye(4)
    t[:3, 3] = pos
    t[:3, :3] = Rotation.from_quat(quat).as_matrix()
    return t

class DroidKinematics:
    def __init__(self, base_position, base_yaw):
        from pxr import Usd, UsdPhysics
        stage = Usd.Stage.Open(str(ROBOT_USD))
        self.base = transform(base_position, Rotation.from_euler('z',base_yaw,degrees=True).as_quat())
        joints = {}
        for p in stage.Traverse():
            if p.IsA(UsdPhysics.Joint):
                joints[p.GetName()] = p
        self.chain = []
        self.limits = []
        def local(j, side):
            q = getattr(j, 'GetLocalRot'+side+'Attr')().Get()
            return transform(getattr(j, 'GetLocalPos'+side+'Attr')().Get(), [*q.GetImaginary(), q.GetReal()])
        for name in [f'panda_joint{i}' for i in range(1,9)]+['panda_hand_joint']:
            p = joints[name]
            j = UsdPhysics.Joint(p)
            self.chain.append((local(j,'0'), np.linalg.inv(local(j,'1')), str(j.GetBody1Rel().GetTargets()[0])))
            if p.IsA(UsdPhysics.RevoluteJoint):
                r = UsdPhysics.RevoluteJoint(p)
                self.limits.append(np.deg2rad([r.GetLowerLimitAttr().Get(),r.GetUpperLimitAttr().Get()]))
        self.limits = np.array(self.limits)

    def _joint_rotations(self, q):
        # Use the same SciPy x-axis rotation construction in one batch. Keep
        # the original multiplication order and fixed-joint identities.
        matrices = np.broadcast_to(np.eye(4), (len(self.chain), 4, 4)).copy()
        moving = min(7, len(self.chain))
        if moving:
            matrices[:moving, :3, :3] = Rotation.from_euler(
                'x', np.asarray(q[:moving]).reshape(-1, 1)).as_matrix()
        return matrices

    def links(self, q):
        t = self.base.copy()
        result = {'/panda/panda_link0': t.copy()}
        for (a,b,name),r in zip(self.chain, self._joint_rotations(q)):
            t = t @ a @ r @ b
            result[name] = t.copy()
        return result

    def fk(self, q):
        # IK needs only the TCP; avoid building/copying every link dictionary
        # for each residual and finite-difference evaluation.
        t = self.base.copy()
        for (a,b,_),r in zip(self.chain, self._joint_rotations(q)):
            t = t @ a @ r @ b
        t[:3,3] += t[:3,:3] @ np.array([.131,0,0])
        return t

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
            if err < .004: return r.x
        raise RuntimeError(f'IK failed: pose residual {best[0]:.5f}, target {target[:3,3]}')
