"""GPU FK/IK smoke test for the installed Arena cuRobo revision (no Kit app)."""
from pathlib import Path
import json
import time

import numpy as np
from scipy.spatial.transform import Rotation
import torch
import yaml

from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import RobotConfig
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig

from kitchen_safety.planning import DroidKinematics


def main():
    root = Path(__file__).resolve().parents[1]
    asset = root / "assets/curobo/droid"
    cfg = yaml.safe_load((asset / "franka_robotiq_2f_85_zero_curobo.yml").read_text())["robot_cfg"]
    cfg["kinematics"]["urdf_path"] = str(asset / "urdf/franka_robotiq_2f_85_zero.urdf")
    args = TensorDeviceType()
    robot_cfg = RobotConfig.from_dict(cfg, args)
    robot = CudaRobotModel(robot_cfg.kinematics)
    qs = np.array([
        np.deg2rad([0, -36, 0, -144, 0, 108, 0]),
        [.3, -.5, .2, -2.1, .2, 1.9, .3],
        [-.4, -.2, -.4, -1.7, .5, 2., -.1],
    ])
    state = robot.get_state(args.to_device(qs))
    calibration = json.loads((asset / "tcp_calibration.json").read_text())
    T_ee_tcp = np.array(calibration["T_urdfEE_USDtcp"])
    reference = DroidKinematics([0, 0, 0], 0)
    position_errors, orientation_errors = [], []
    for i, q in enumerate(qs):
        T = np.eye(4)
        T[:3, 3] = state.ee_position[i].cpu().numpy()
        T[:3, :3] = Rotation.from_quat(state.ee_quaternion[i].cpu().numpy()[[1, 2, 3, 0]]).as_matrix()
        T = T @ T_ee_tcp
        expected = reference.fk(q)
        position_errors.append(float(np.linalg.norm(T[:3, 3] - expected[:3, 3])))
        orientation_errors.append(float(Rotation.from_matrix(T[:3, :3] @ expected[:3, :3].T).magnitude()))
    assert max(position_errors) < 1e-5, position_errors
    assert max(orientation_errors) < 1e-5, orientation_errors

    ik_cfg = IKSolverConfig.load_from_robot_config(
        robot_cfg, None, tensor_args=args, num_seeds=32,
        self_collision_check=True, self_collision_opt=True,
        position_threshold=.002, rotation_threshold=.02, use_cuda_graph=False,
    )
    ik = IKSolver(ik_cfg)
    goal = Pose(state.ee_position, state.ee_quaternion)
    started = time.perf_counter()
    result = ik.solve_batch(goal)
    torch.cuda.synchronize()
    report = {
        "gpu": torch.cuda.get_device_name(),
        "torch": torch.__version__, "cuda": torch.version.cuda,
        "fk_max_position_error_m": max(position_errors),
        "fk_max_orientation_error_rad": max(orientation_errors),
        "ik_success": result.success.cpu().tolist(),
        "ik_position_error_m": result.position_error.cpu().tolist(),
        "ik_rotation_error_rad": result.rotation_error.cpu().tolist(),
        "ik_elapsed_s_including_first_compilation": time.perf_counter() - started,
        "collision_spheres": int(state.link_spheres_tensor.shape[-2]),
        "self_collision_check": True,
    }
    (root / "reports/curobo_smoke.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    assert bool(result.success.all()), "IK did not solve every known feasible FK pose"


if __name__ == "__main__":
    main()
