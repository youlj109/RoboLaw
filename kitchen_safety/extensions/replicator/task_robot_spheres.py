"""Read original cuRobo gripper spheres in the calibrated USD TCP frame (CPU)."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parents[3]


def open_gripper_spheres():
    asset = ROOT/"assets/curobo/droid"
    paths = dict(config=asset/"franka_robotiq_2f_85_zero_curobo.yml",
                 urdf=asset/"urdf/franka_robotiq_2f_85_zero.urdf", calibration=asset/"tcp_calibration.json")
    cfg = yaml.safe_load(paths["config"].read_text())["robot_cfg"]["kinematics"]
    calibration = np.asarray(json.loads(paths["calibration"].read_text())["T_urdfEE_USDtcp"])
    frames = {"base_link": np.linalg.inv(calibration)}
    joints = ET.parse(paths["urdf"]).getroot().findall("joint")
    pending = joints.copy()
    while pending:
        count = len(pending)
        for joint in pending.copy():
            parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
            if parent not in frames or child in frames:
                continue
            # Original open six joints are exactly zero; their axis rotation
            # is identity. Other arm joints are outside this fixed subtree.
            transform = np.eye(4)
            origin = joint.find("origin")
            if origin is not None:
                transform[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
                transform[:3, :3] = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")).as_matrix()
            frames[child] = frames[parent]@transform
            pending.remove(joint)
        if len(pending) == count:
            break
    spheres, labels = [], []
    for link in cfg["collision_link_names"]:
        if link not in frames:
            continue
        for index, item in enumerate(cfg["collision_spheres"][link]):
            transform = frames[link]
            spheres.append(np.r_[transform[:3, :3]@item["center"]+transform[:3, 3], float(item["radius"])+.005])
            labels.append(dict(link=link, local_sphere=index))
    if not {"base_link", "left_inner_finger", "right_inner_finger"} <= {row["link"] for row in labels}:
        raise ValueError("Original gripper sphere geometry is incomplete")
    return np.asarray(spheres), dict(labels=labels, collision_sphere_buffer_m=.005,
        source_sha256={name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in paths.items()},
        coordinate_frame="USD TCP; original open gripper URDF subtree and T_urdfEE_USDtcp calibration",
        buffer_source="Unchanged public CuroboPlanner.__init__ collision_sphere_buffer=.005")


def open_gripper_minimum_z(rotation):
    spheres, provenance = open_gripper_spheres()
    lowest = spheres[:, :3]@np.asarray(rotation)[2, :]-spheres[:, 3]
    index = int(np.argmin(lowest))
    return float(lowest[index]), dict(**provenance, lowest=provenance["labels"][index],
        lowest_relative_tcp_z_m=float(lowest[index]), sphere_tcp_xyzr=spheres[index].tolist())
