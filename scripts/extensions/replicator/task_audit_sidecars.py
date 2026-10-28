#!/usr/bin/env python3
"""Independent audits for the task-aware collector schema; never replay actions.

source/fixed/cameras are CPU-only. self-collision explicitly uses CUDA under the
existing shared lock, without Kit, stepping or modifying recorded joint values.
All reports preserve their precise scope; none turns an episode accepted=true.
"""
import argparse
import ast
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[3]
CAMERA_SOURCE = ROOT/"third_party/IsaacLab-Arena/isaaclab_arena/embodiments/droid/droid.py"
_GPU_LOCK = None


def sha(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8*1024*1024), b""):
            value.update(block)
    return value.hexdigest()


def load_episode(path):
    path = Path(path)
    metadata = json.loads((path/"physics_metadata.json").read_text())
    with np.load(path/"physics_states.npz", allow_pickle=False) as raw:
        data = {name: raw[name] for name in raw.files}
    with np.load(path/"trajectory.npz", allow_pickle=False) as raw:
        trajectory = {name: raw[name] for name in raw.files}
    steps = data["physics_step"]
    if steps.ndim != 1 or len(steps) < 2 or np.any(np.diff(steps) != 1) or steps[0] != metadata["reset_physics_step"]:
        raise ValueError("Missing reset/continuous physics-step evidence")
    for name, value in data.items():
        if len(value) != len(steps) or (value.dtype.kind not in "US" and not np.isfinite(value).all()):
            raise ValueError("Missing/nonfinite/misaligned actual state: " + name)
    if data["q"].shape != (len(steps), len(metadata["joint_names"])):
        raise ValueError("Actual joint/name dimensions differ")
    dt = metadata["physics_dt_s"]
    decimation = round(metadata["action_dt_s"]/dt)
    if decimation < 1 or abs(decimation*dt-metadata["action_dt_s"]) > 1e-9:
        raise ValueError("Invalid original physics/action period")
    count = len(trajectory["actions"])
    expected = steps[0]+np.arange(count)*decimation
    if (len(steps) != count*decimation+1 or not np.array_equal(trajectory["before_physics_step"], expected)
            or not np.array_equal(trajectory["after_physics_step"], expected+decimation)):
        raise ValueError("Physical steps do not cover every saved action, including startup")
    return metadata, data, trajectory


def source_audit(metadata, data):
    from scripts.extensions.replicator.audit_task_source_relation import check_layout
    layout = metadata["source_layout"]
    relations = check_layout(layout)
    base = data["robot_body_pose_w"][:, metadata["robot_body_names"].index("panda_link0")]
    expected_position = np.asarray(layout["robot_position"])+[0., 0., layout["stand_height"]]
    expected_rotation = Rotation.from_euler("z", layout["robot_yaw_deg"], degrees=True)
    translation = np.linalg.norm(base[:, :3]-expected_position, axis=-1)
    angular = (expected_rotation.inv()*Rotation.from_quat(base[:, 3:])).magnitude()
    objects = {}
    for name, item in layout["objects"].items():
        actual = data["task_object_states"][0, metadata["task_object_names"].index(name), :7]
        distance = float(np.linalg.norm(actual[:3]-item["position"]))
        angle = float((Rotation.from_euler("z", item["yaw_deg"], degrees=True).inv()*Rotation.from_quat(actual[3:])).magnitude())
        source = ROOT/item["usd_path"]
        objects[name] = dict(reset_position_error_m=distance, reset_rotation_error_rad=angle,
            source_sha256=sha(source), recorded_source_sha256=metadata["source_hashes"][name],
            passed=distance <= .001 and angle <= 1e-4 and sha(source) == metadata["source_hashes"][name])
    checks = dict(original_authored_relations=relations["initial_source_relations_passed"],
        actual_fixed_base_all_samples=bool(translation.max() <= 1e-5 and angular.max() <= 1e-5),
        recorded_reset_objects_match_source=all(row["passed"] for row in objects.values()),
        recorded_static_anchor_transforms_constant=not data["static_anchor_transforms"].size or
            float(np.abs(data["static_anchor_transforms"]-data["static_anchor_transforms"][0]).max()) <= 1e-6,
        recorded_source_references_constant=not data["task_reference_transforms"].size or
            float(np.abs(data["task_reference_transforms"]-data["task_reference_transforms"][0]).max()) <= 1e-6)
    return dict(passed=all(checks.values()), checks=checks, original_relations=relations,
        maximum_base_position_error_m=float(translation.max()), maximum_base_rotation_error_rad=float(angular.max()),
        original_task_objects=objects, physics_samples=len(base),
        thresholds=dict(base_translation_m=1e-5, base_rotation_rad=1e-5, reset_object_translation_m=.001,
                        reset_object_rotation_rad=1e-4, static_matrix=1e-6),
        scope="Original authored relation validators + all measured fixed-base poses + observed reset task object poses and source hashes. Native static-reference-to-source composition is separately checked by the neutral probe.")


def source_camera_config(path=CAMERA_SOURCE):
    """Read pinned literal camera definitions without importing Kit/PhysxSchema."""
    tree = ast.parse(Path(path).read_text())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "DroidCameraCfg")
    def literals(call):
        return {row.arg: literals(row.value) if isinstance(row.value, ast.Call) else ast.literal_eval(row.value)
                for row in call.keywords}
    return {node.target.id: literals(node.value) for node in cls.body if isinstance(node, ast.AnnAssign)}


def cameras_audit(episode, metadata, data, trajectory):
    import h5py
    from kitchen_safety.dataset import CAMERA_NAMES, RGB_ALIGNMENT, camera_poses_from_body_states, camera_pose_errors
    episode = Path(episode)
    recorded = json.loads((episode/"cameras.json").read_text())
    source = source_camera_config()
    if set(recorded) != set(CAMERA_NAMES) or set(source) != set(CAMERA_NAMES):
        raise ValueError("Original camera inventory differs from pinned DROID source")
    indices = np.searchsorted(data["physics_step"], trajectory["before_physics_step"])
    if not np.array_equal(data["physics_step"][indices], trajectory["before_physics_step"]):
        raise ValueError("No exact recorded pre-action parent-body state for a camera frame")
    expected = camera_poses_from_body_states(data["robot_body_pose_w"][indices], metadata["robot_body_names"], recorded)
    count = len(indices)
    camera_reports = {}
    with h5py.File(episode/"images.h5", "r") as images:
        if int(images.attrs["frames"]) != count or images.attrs["alignment"] != RGB_ALIGNMENT:
            raise ValueError("Camera file frame/alignment convention mismatch")
        if abs(float(images.attrs["dt"])-metadata["action_dt_s"]) > 1e-9:
            raise ValueError("Camera action period changed")
        for name in CAMERA_NAMES:
            config, original = recorded[name], source[name]
            spawn = original["spawn"]
            intrinsics = np.array([[original["width"]*spawn["focal_length"]/spawn["horizontal_aperture"], 0., original["width"]/2],
                [0., original["height"]*spawn["focal_length"]/spawn["vertical_aperture"], original["height"]/2], [0., 0., 1.]])
            mount_same = (config["prim_path"].split("/Robot/")[-1] == original["prim_path"].split("/Robot/")[-1]
                and np.allclose(config["offset"]["position"], original["offset"]["pos"], rtol=0., atol=1e-8)
                and np.allclose(config["offset"]["quaternion_xyzw"], original["offset"]["rot"], rtol=0., atol=1e-8)
                and config["offset"]["convention"] == original["offset"]["convention"])
            intrinsic_error = float(np.max(np.abs(np.asarray(config["intrinsics"])-intrinsics)))
            group = images[name]
            if (group["position_world"].shape != (count, 3) or group["quaternion_world_ros_xyzw"].shape != (count, 4)
                    or len(group["jpeg"]) != count):
                raise ValueError("Camera pose/image count mismatch: " + name)
            error = camera_pose_errors(group["position_world"][:], group["quaternion_world_ros_xyzw"][:], expected[name])
            camera_reports[name] = dict(**error, original_mount_preserved=bool(mount_same), intrinsic_max_error_pixels=intrinsic_error,
                expected_intrinsics=intrinsics.tolist(), passed=bool(mount_same and intrinsic_error < .001
                    and config["width"] == original["width"] and config["height"] == original["height"]
                    and error["position_max_error_m"] < .0001 and error["rotation_max_error_deg"] < .02))
    aligned = np.array_equal(trajectory["camera_physics_steps"], np.repeat(trajectory["before_physics_step"][:, None], 2, axis=1))
    return dict(passed=bool(aligned and all(row["passed"] for row in camera_reports.values())), frames=count,
        exact_physics_action_alignment=bool(aligned), cameras=camera_reports, camera_source_sha256=sha(CAMERA_SOURCE),
        scope="All original camera mounts/intrinsics against pinned source literals and every saved ROS world pose against measured pre-action parent-body poses. Does not certify image semantics, JPEG/video decode or overview visibility.")


def world_from_layout(layout, base):
    from pxr import Gf, Usd, UsdGeom
    from kitchen_safety.planning import DroidKinematics
    from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld
    stage = Usd.Stage.CreateInMemory()
    root = "/World/envs/env_0"
    UsdGeom.Xform.Define(stage, root)
    UsdGeom.Xform.Define(stage, root+"/kitchen").GetPrim().GetReferences().AddReference(str(ROOT/layout["kitchen_usd"]))
    for name, item in layout["objects"].items():
        xf = UsdGeom.Xform.Define(stage, root+"/"+name)
        xf.GetPrim().GetReferences().AddReference(str(ROOT/item["usd_path"]))
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*item["position"]))
        xf.AddRotateZOp().Set(item["yaw_deg"])
    kin = DroidKinematics(base[:3], layout["robot_yaw_deg"])
    return CollisionWorld(stage, kin, env_root=root)


def fixed_audit(metadata, data):
    import fcl
    from scripts.extensions.replicator.audit_fixed_base import fixed_colliders, ASSEMBLY
    layout = metadata["source_layout"]
    if "obstacle" in layout:
        raise ValueError("This source-world fixed-base audit does not yet compose added hazard assets")
    bases = data["robot_body_pose_w"][:, metadata["robot_body_names"].index("panda_link0")]
    source = source_audit(metadata, data)
    world = world_from_layout(layout, bases[0])
    fixed = fixed_colliders(bases[0])
    floor = "/World/envs/env_0/kitchen/KitchenRoom/Floor"
    if len([item for item in world.objects if item[0] == floor]) != 1:
        raise ValueError("Expected exact original floor support mesh")
    # Fixed base is numerically stationary; rebuilding from a changed observed
    # pose is exact, never clamped. Spatial broad phase alone excludes distant pairs.
    previous, failures, tests, allowed_floor = None, [], 0, 0
    for index, base in enumerate(bases):
        if previous is None or not np.array_equal(previous, base):
            fixed = fixed_colliders(base)
            previous = base.copy()
        world.sync_background(metadata["background_paths"], data["background_rigid_poses"][index])
        for name in metadata["task_object_names"]:
            world.set_obstacle_pose(name, data["task_object_states"][index, metadata["task_object_names"].index(name), :7])
        for shape, info in fixed:
            low, high = np.asarray(info["bounds"])
            bound = np.linalg.norm(np.maximum(np.maximum(world.low-high, low-world.high), 0.), axis=1)
            candidates = np.flatnonzero(bound <= .002)
            for candidate in candidates:
                name, obstacle, obstacle_low, obstacle_high = world.objects[candidate]
                if (name == floor and "stand" in info["path"].lower() and low[2] >= obstacle_high[2]-1e-6):
                    allowed_floor += 1
                    continue
                tests += 1
                raw = float(fcl.distance(shape, obstacle, fcl.DistanceRequest(), fcl.DistanceResult()))
                intersecting = bool(raw < 0 or (raw == 0 and fcl.collide(shape, obstacle,
                    fcl.CollisionRequest(), fcl.CollisionResult())))
                if intersecting or raw < .002:
                    failures.append(dict(physics_index=index, physics_step=int(data["physics_step"][index]),
                        fixed=info["path"], obstacle=name, distance_m=max(0., raw), intersecting=intersecting, required_m=.002))
        if failures:
            break
    complete = not failures
    return dict(passed=bool(complete and source["passed"]), physics_samples=len(bases),
        checked_samples=index+1, complete=complete, precise_near_pairs_checked=tests,
        allowed_exact_stand_floor_support_pairs=allowed_floor, failures=failures,
        source_and_base=source, fixed_geometry=[info for _, info in fixed],
        assembly_sha256=sha(ASSEMBLY), kitchen_source_sha256=sha(ROOT/layout["kitchen_usd"]),
        scope="Source stand/link0 solids against all original kitchen collider meshes and recorded task-object poses at every physics sample; 2 mm non-contact margin unchanged. Only exact stand/floor support is allowed (1 micrometre numerical depth). Source convexHull/primitives preserved; no PhysX cooked-shape claim. Visual stand mesh needs separate image/source review.")


def self_collision_audit(metadata, data):
    """GPU only: all actual six finger angles, no mimic substitution or clamp."""
    import xml.etree.ElementTree as ET
    from kitchen_safety.planning import DroidKinematics
    from kitchen_safety.curobo_backend import CuroboPlanner
    from curobo.types.state import JointState
    layout = metadata["source_layout"]
    names = metadata["joint_names"]
    kin = DroidKinematics(np.asarray(layout["robot_position"])+[0., 0., layout["stand_height"]], layout["robot_yaw_deg"])
    world = SimpleNamespace(objects=[], meshes={}, attachment=None)
    planner = CuroboPlanner(world, kin, dt=metadata["action_dt_s"])
    solver = planner.motion_gen.ik_solver
    rollout = solver.rollout_fn
    if not rollout.robot_self_collision_constraint.enabled:
        raise ValueError("Self-collision constraint is disabled")
    arm_names = solver.joint_names
    arm_ids = [names.index(name) for name in arm_names]
    finger_names = list(planner.robot_dict["kinematics"]["lock_joints"])
    expected = {"finger_joint", "right_outer_knuckle_joint", "left_inner_finger_joint", "right_inner_finger_joint",
                "left_inner_finger_knuckle_joint", "right_inner_finger_knuckle_joint"}
    if set(finger_names) != expected or set(names) != set(arm_names) | expected:
        raise ValueError("Recorded/locked joint inventory is unsupported; do not omit a joint")
    finger_ids = [names.index(name) for name in finger_names]
    urdf = Path(planner.robot_dict["kinematics"]["urdf_path"])
    source = ET.parse(urdf)
    limits = {}
    for node in source.getroot().findall("joint"):
        limit = node.find("limit")
        if node.get("name") in names and limit is not None:
            limits[node.get("name")] = [float(limit.get("lower")), float(limit.get("upper"))]
    if set(limits) != set(names):
        raise ValueError("Source URDF does not provide every recorded actual joint limit")
    bounds = np.array([limits[name] for name in names]).T
    violations, exact_exceptions, maximum_excess, maximum_cost = [], [], 0., 0.
    previous = None
    costs = []
    for index, q in enumerate(data["q"]):
        fingers = q[finger_ids]
        if previous is None or not np.array_equal(previous, fingers):
            # Every physical joint's measured angle is supplied independently.
            planner.motion_gen.update_locked_joints(dict(zip(finger_names, map(float, fingers))), deepcopy(planner.robot_dict))
            previous = fingers.copy()
        actual = planner.args.to_device(q[arm_ids][None])
        state = rollout._get_augmented_state(JointState(position=actual))
        cost = float(rollout.robot_self_collision_constraint.forward(state.robot_spheres).max().item())
        if not np.isfinite(cost):
            raise RuntimeError(f"Nonfinite cuRobo self-collision cost at physics sample {index}")
        excess = float(np.maximum(bounds[0]-q, q-bounds[1]).clip(min=0).max())
        maximum_excess, maximum_cost = max(maximum_excess, excess), max(maximum_cost, cost)
        costs.append(cost)
        entry = dict(index=index, physics_step=int(data["physics_step"][index]), phase=str(data["phase"][index]),
                     self_collision_constraint=cost, joint_limit_excess_rad=excess, actual_q=q.tolist())
        if excess > 0:
            exact_exceptions.append(entry)
        if cost != 0. or excess > 1e-5:
            violations.append(entry)
    return dict(passed=not violations, physics_samples=len(data["q"]), checked_samples=len(costs),
        violations=violations, exact_bound_exceptions=exact_exceptions, maximum_self_collision_constraint=maximum_cost,
        maximum_joint_limit_excess_rad=maximum_excess, joint_names=names, joint_limits_rad=limits,
        locked_measured_joint_names=finger_names, recorded_positions_modified=False, mimic_angles_substituted=False,
        urdf_sha256=sha(urdf), sphere_model_sha256=sha(ROOT/"assets/curobo/droid/franka_robotiq_2f_85_zero_curobo.yml"),
        scope="Actual unclamped arm q and all six independent measured finger angles at every post-reset physics sample; enabled cuRobo self-collision cost must be exactly zero. URDF joint bounds allow only 1e-5 rad numeric excess. No simulation, payload/stand geometry or external-world collision claim.")


def main():
    global _GPU_LOCK
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--kind", choices=["source", "fixed", "cameras", "self-collision"], required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    target = args.output or args.episode/("task_"+args.kind.replace("-", "_")+"_audit.json")
    if target.exists():
        parser.error("Refuse to overwrite an existing audit")
    metadata, data, trajectory = load_episode(args.episode)
    if args.kind == "source":
        report = source_audit(metadata, data)
    elif args.kind == "fixed":
        report = fixed_audit(metadata, data)
    elif args.kind == "cameras":
        report = cameras_audit(args.episode, metadata, data, trajectory)
    else:
        import fcntl
        lock_path = Path("/workspace/reports/skill_trials/gpu.lock")
        if ROOT != Path("/workspace") or not lock_path.exists():
            raise RuntimeError("Self-collision GPU audit requires verified mount and canonical existing lock")
        # Lock must live through context destruction at process exit.
        lock = lock_path.open("a+")
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        _GPU_LOCK = lock
        report = self_collision_audit(metadata, data)
    report["input_sha256"] = {name: sha(args.episode/name) for name in ("physics_states.npz", "physics_metadata.json", "trajectory.npz")}
    report["kind"], report["accepted"] = args.kind, False
    target.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(json.dumps(dict(kind=args.kind, passed=report["passed"], output=str(target), accepted=False)))
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
