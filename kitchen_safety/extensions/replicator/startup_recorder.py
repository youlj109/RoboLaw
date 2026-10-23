"""Observe reset-to-planner startup without stepping, resetting or writing state.

The old physics_states.npz starts after the collision world exists. This separate
sidecar covers its missing prefix and leaves its historical schema unchanged.
"""
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def array(value):
    value = value.torch if hasattr(value, "torch") else value
    return (value.detach().cpu().numpy() if hasattr(value, "detach") else
            np.asarray(value.numpy() if hasattr(value, "numpy") else value)).copy()


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def pose_change(poses, start=0):
    values = np.asarray(poses)[start:]
    if not len(values):
        raise ValueError("Missing samples in declared stability window")
    angle = np.rad2deg((Rotation.from_quat(values[:, 3:7]) *
                       Rotation.from_quat(values[0, 3:7]).inv()).magnitude())
    return dict(max_translation_m=float(np.linalg.norm(values[:, :3] - values[0, :3], axis=1).max()),
                max_rotation_deg=float(angle.max()))


def audit_startup(data, metadata):
    """Pure CPU gates. Never infer missing substeps from a later snapshot."""
    steps = np.asarray(data["physics_step"], dtype=np.int64)
    dt = float(metadata["physics_dt_s"])
    if len(steps) < 2 or not np.isfinite(dt) or dt <= 0:
        raise ValueError("Need initial state and real physics samples with a finite dt")
    names, paths = metadata["joint_names"], metadata["background_paths"]
    n = len(steps)
    shapes = dict(q=(n, len(names)), dq=(n, len(names)), mustard=(n, 13), bowl=(n, 13),
                  hazard_state=(n, 13), has_hazard_state=(n,), hazard_pose=(n, 7), has_hazard_pose=(n,),
                  background_rigid_poses=(n, len(paths), 7), background_rigid_velocities=(n, len(paths), 6),
                  robot_body_pose_w=(n, len(metadata["robot_body_names"]), 7),
                  static_anchor_transforms=(n, len(metadata["static_anchor_paths"]), 4, 4))
    for key, shape in shapes.items():
        if np.asarray(data[key]).shape != shape or not np.isfinite(data[key]).all():
            raise ValueError("Missing/nonfinite/misaligned startup field: " + key)
    elapsed = (steps - steps[0]) * dt
    task_start = np.flatnonzero(elapsed >= 2. - 1e-8)
    free_start = np.flatnonzero(elapsed >= 5. - 1e-8)
    if not len(task_start) or not len(free_start):
        raise ValueError("Startup did not cover the original 2s task / 5s free-background settling windows")
    expected = metadata.get("world_ready_physics_step")
    gaps = [dict(previous=int(steps[i]), current=int(steps[i + 1]))
            for i in np.flatnonzero(np.diff(steps) != 1)]
    coverage = dict(reset_physics_step=int(metadata["reset_physics_step"]),
                    world_ready_physics_step=expected, first_recorded_physics_step=int(steps[0]),
                    last_recorded_physics_step=int(steps[-1]), samples=n,
                    expected_samples=None if expected is None else expected - metadata["reset_physics_step"] + 1,
                    gaps=gaps, duration_s=float(elapsed[-1]),
                    initial_state_recorded=str(data["startup_stage"][0]) == "reset_initial",
                    callback_errors=metadata.get("callback_errors", []))
    topology = metadata["background_topology"]
    linked = set(topology["joint_connected_native_paths"])
    background = []
    for i, path in enumerate(paths):
        start = 0 if path in linked else int(free_start[0])
        motion = pose_change(data["background_rigid_poses"][:, i], start)
        background.append(dict(path=path, joint_connected=path in linked,
                               reference_time_s=float(elapsed[start]), **motion,
                               passed=motion["max_translation_m"] <= .002 and motion["max_rotation_deg"] <= 1.))
    task = {name: pose_change(data[name], int(task_start[0])) for name in ("mustard", "bowl")}
    robot_change = float(np.max(np.abs(data["q"] - data["q"][0])))
    hazard = dict(present=metadata["dynamic_hazard"], passed=True)
    if metadata["dynamic_hazard"]:
        if not data["has_hazard_state"].all():
            raise ValueError("Dynamic hazard lacks an actual pose at a startup sample")
        poses = data["hazard_state"][:, :7]
        motion = pose_change(poses)
        vertical = float(np.abs(poses[:, 2] - poses[0, 2]).max())
        upright = float(np.rad2deg(np.arccos(np.clip(Rotation.from_quat(poses[:, 3:7]).as_matrix()[:, 2, 2], -1, 1))).max())
        hazard.update(**motion, max_vertical_displacement_m=vertical, max_upright_deviation_deg=upright,
                      passed=motion["max_translation_m"] < .002 and motion["max_rotation_deg"] <= 2.
                      and vertical < .002 and upright <= 2.)
    elif metadata.get("hazard_kind") == "static":
        if not data["has_hazard_pose"].all():
            raise ValueError("Static hazard lacks its composed USD root pose")
        motion = pose_change(data["hazard_pose"])
        hazard.update(present=True, pose_source="composed static USD root; no simulated rigid state", **motion,
                      passed=motion["max_translation_m"] <= 1e-6 and motion["max_rotation_deg"] <= .001)
    static = data["static_anchor_transforms"]
    static_change = float(np.abs(static - static[0]).max()) if static.size else 0.
    checks = dict(world_ready_reached=expected is not None,
                  reset_state_and_every_physics_step=bool(expected is not None and steps[0] == metadata["reset_physics_step"]
                    and steps[-1] == expected and n == coverage["expected_samples"] and not gaps
                    and coverage["initial_state_recorded"] and not coverage["callback_errors"]),
                  topology_complete=bool(linked) and not topology["unobserved_joint_connected_bodies"],
                  background_stable=all(row["passed"] for row in background),
                  task_objects_settled=all(row["max_translation_m"] <= .001 and row["max_rotation_deg"] <= 1.
                                           for row in task.values()),
                  robot_neutral_hold=robot_change <= .001,
                  dynamic_hazard_stable_from_reset=bool(hazard["passed"]),
                  static_anchors_unchanged=static_change <= 1e-6 and not topology["animated_static_anchors"])
    return dict(schema_version=1, passed=all(checks.values()), checks=checks, coverage=coverage,
                thresholds=dict(robot_joint_rad=.001, task_translation_m=.001, task_rotation_deg=1.,
                                task_settling_seconds=2., free_background_settling_seconds=5.,
                                background_translation_m=.002, background_rotation_deg=1.,
                                hazard_translation_m=.002, hazard_vertical_m=.002,
                                hazard_rotation_deg=2., hazard_upright_deg=2., static_anchor_matrix=1e-6),
                hazard=hazard, task_objects=task, robot_max_joint_change_rad=robot_change,
                background=background, static_anchor_max_matrix_difference=static_change,
                background_physics=metadata.get("background_physics"),
                scope="Actual post-reset initial state and every observed physics substep through collision-world readiness. "
                "The existing 8 warmup actions and 5s recorded hold are unchanged. Free background bodies get only the "
                "original 5s settling window, joint-connected bodies and dynamic hazard are checked from reset. "
                "Task stability follows the existing 2s neutral-probe window. No claim for unrecorded reset-internal physics "
                "or camera images before the existing camera writer starts; full-task safety remains separately required.")


def validate_prefix(data, metadata, world, arm_ids):
    """Apply the unchanged execution checker to the newly recorded prefix."""
    from kitchen_safety.extensions.replicator.deferred_safety import DeferredSafetyBuffer
    buffer = DeferredSafetyBuffer()
    buffer.background_paths = metadata["background_paths"]
    for i, q in enumerate(data["q"]):
        buffer.background_poses.append(data["background_rigid_poses"][i])
        buffer.append(q=q, mustard=data["mustard"][i, :7], phase="settle", action_index=i,
                      attachment=None, hazard_pose=data["hazard_state"][i, :7]
                      if metadata["dynamic_hazard"] else None)
    report = buffer.validate(world, arm_ids, metadata["joint_names"])
    if report["failure"]:
        failure = report["failure"]
        i = failure["state_index"]
        failure.update(prefix_sample_index=i, action_index=int(data["action_index"][i]),
                       startup_stage=str(data["startup_stage"][i]), physics_step=int(data["physics_step"][i]))
    report["scope"] = ("All startup samples including reset state. Unchanged DeferredSafetyBuffer execution rules: "
                       "robot/gripper against the original environment/hazards, actual free bottle against hazards; "
                       "original task-object/support exclusions retained. Background and dynamic hazard use each measured pose. "
                       "Buffer indices are prefix sample indices, not video frames; failure.action_index is remapped to real "
                       "recorded actions (-1 for the pre-camera warmup). No simulator state is written.")
    return report


def background_topology(stage, view):
    """Resolve passive-joint dynamic endpoints, including proven root-view aliases."""
    from pxr import Usd, UsdGeom, UsdPhysics
    observed = list(view.prim_paths)
    poses, masses = array(view.get_transforms()), array(view.get_masses()).reshape(-1)
    connected, endpoints, anchors, animated = set(), [], set(), []
    cache = UsdGeom.XformCache()
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.Joint):
            continue
        joint = UsdPhysics.Joint(prim)
        for role, relation in (("body0", joint.GetBody0Rel()), ("body1", joint.GetBody1Rel())):
            for target in relation.GetTargets():
                if not str(target).startswith("/World/envs/env_0/kitchen/"):
                    continue
                body = stage.GetPrimAtPath(target)
                if not body:
                    raise ValueError("Missing background joint endpoint: " + str(target))
                while body and not body.IsPseudoRoot() and not body.HasAPI(UsdPhysics.RigidBodyAPI):
                    body = body.GetParent()
                if body and not body.IsPseudoRoot():
                    path = str(body.GetPath())
                    connected.add(path)
                    endpoints.append(dict(joint=str(prim.GetPath()), role=role, rigid_body=path))
                else:
                    anchors.add(str(target))
                    ancestor = stage.GetPrimAtPath(target)
                    while ancestor and not ancestor.IsPseudoRoot():
                        xform = UsdGeom.Xformable(ancestor)
                        if xform and xform.TransformMightBeTimeVarying():
                            animated.append(str(ancestor.GetPath()))
                        ancestor = ancestor.GetParent()
    aliases = {}
    for missing in sorted(connected - set(observed)):
        body = stage.GetPrimAtPath(missing)
        parent = body.GetParent()
        while parent and not parent.IsPseudoRoot() and not parent.HasAPI(UsdPhysics.ArticulationRootAPI):
            parent = parent.GetParent()
        if not parent or parent.IsPseudoRoot() or str(parent.GetPath()) not in observed:
            continue
        root = str(parent.GetPath())
        edges = [x for x in endpoints if x["joint"].startswith(root + "/")]
        roots = {x["rigid_body"] for x in edges if x["role"] == "body0"} - {x["rigid_body"] for x in edges if x["role"] == "body1"}
        matrix = np.asarray(cache.GetLocalToWorldTransform(body)).T
        rotation = matrix[:3, :3] / np.linalg.norm(matrix[:3, :3], axis=0)
        i = observed.index(root)
        mass = UsdPhysics.MassAPI(body).GetMassAttr().Get()
        translation = float(np.linalg.norm(matrix[:3, 3] - poses[i, :3]))
        angle = float(np.rad2deg((Rotation.from_matrix(rotation) * Rotation.from_quat(poses[i, 3:7]).inv()).magnitude()))
        if roots == {missing} and mass is not None and abs(masses[i] - mass) <= 1e-4 * max(1., mass) and translation <= .002 and angle <= 1.:
            aliases[missing] = dict(native_path=root, unique_joint_graph_root=sorted(roots),
                                   source_mass_kg=float(mass), native_mass_kg=float(masses[i]),
                                   translation_error_m=translation, rotation_error_deg=angle)
    native = {aliases.get(path, {}).get("native_path", path) for path in connected}
    return dict(joint_connected_native_paths=sorted(native), authored_joint_connected_paths=sorted(connected),
                articulation_root_aliases=aliases, joint_endpoints=endpoints,
                unobserved_joint_connected_bodies=sorted(native - set(observed)),
                static_anchor_paths=sorted(anchors), animated_static_anchors=sorted(set(animated)))


class StartupRecorder:
    def __init__(self, env, layout, output, phase_and_action, background_physics=None):
        from pxr import UsdGeom
        self.env, self.scene, self.sim = env, env.scene, env.sim
        self.robot, self.output = self.scene["robot"], Path(output)
        self.phase_and_action = phase_and_action
        self.dynamic = bool(layout.get("obstacle") and not layout["obstacle"].get("static", True))
        self.hazard_root = ("/World/envs/env_0/" + {"human": "person", "property": "glass", "robot": "barrier"}[
            layout["obstacle"]["category"]]) if layout.get("obstacle") else None
        self.view = self.scene["robot_contacts"]._physics_sim_view.create_rigid_body_view("/World/envs/env_0/kitchen/**")
        topology = background_topology(self.sim.stage, self.view)
        self.stage_label, self.warmup_index = "reset_initial", -1
        self.active, self.rows, self.report = True, [], None
        root = Path(__file__).resolve().parents[3]
        self.code_paths = {"collector": root / "scripts/extensions/replicator/collect.py",
                           "startup_recorder": Path(__file__).resolve(),
                           "deferred_safety": Path(__file__).with_name("deferred_safety.py")}
        self.code_snapshots = {key: path.read_text() for key, path in self.code_paths.items()}
        self.code_hashes = {key: hashlib.sha256(text.encode()).hexdigest() for key, text in self.code_snapshots.items()}
        self.usd_xform_cache = UsdGeom.XformCache
        self.metadata = dict(schema_version=1, reset_physics_step=int(self.sim.get_physics_step_count()),
                             physics_dt_s=float(env.physics_dt), action_dt_s=float(env.step_dt),
                             world_ready_physics_step=None, dynamic_hazard=self.dynamic,
                             hazard_kind="dynamic" if self.dynamic else "static" if self.hazard_root else "none",
                             joint_names=list(self.robot.joint_names), robot_body_names=list(self.robot.body_names),
                             background_paths=list(self.view.prim_paths), background_topology=topology,
                             static_anchor_paths=topology["static_anchor_paths"], background_physics=background_physics,
                             callback_errors=[], pose_convention="world xyz + xyzw")
        self.original_update = self.scene.update
        self.capture()

        def update(dt, *args, **kwargs):
            value = self.original_update(dt, *args, **kwargs)
            if self.active:
                count = int(self.sim.get_physics_step_count())
                if count != self.rows[-1]["physics_step"]:
                    try:
                        self.capture()
                    except Exception as error:
                        self.metadata["callback_errors"].append(dict(physics_step=count, error=repr(error)))
                        raise
            return value

        self.update_wrapper = update
        self.scene.update = update

    def set_stage(self, stage, warmup_index=-1):
        self.stage_label, self.warmup_index = stage, int(warmup_index)

    def capture(self):
        phase, action_index = self.phase_and_action()
        cache = self.usd_xform_cache()
        anchors = [np.asarray(cache.GetLocalToWorldTransform(self.sim.stage.GetPrimAtPath(path))).T
                   for path in self.metadata["static_anchor_paths"]]
        hazard_state = array(self.scene["glass"].data.root_state_w)[0] if self.dynamic else np.zeros(13)
        hazard_pose = hazard_state[:7].copy()
        if self.hazard_root and not self.dynamic:
            matrix = np.asarray(cache.GetLocalToWorldTransform(self.sim.stage.GetPrimAtPath(self.hazard_root))).T
            rotation = matrix[:3, :3] / np.linalg.norm(matrix[:3, :3], axis=0)
            hazard_pose = np.r_[matrix[:3, 3], Rotation.from_matrix(rotation).as_quat()]
        row = dict(physics_step=int(self.sim.get_physics_step_count()), startup_stage=self.stage_label,
                   warmup_action_index=self.warmup_index, phase=phase,
                   action_index=-1 if self.stage_label in ("reset_initial", "warmup") else int(action_index),
                   q=array(self.robot.data.joint_pos)[0], dq=array(self.robot.data.joint_vel)[0],
                   mustard=array(self.scene["mustard_bottle"].data.root_state_w)[0],
                   bowl=array(self.scene["bowl"].data.root_state_w)[0],
                   hazard_state=hazard_state, has_hazard_state=self.dynamic,
                   hazard_pose=hazard_pose, has_hazard_pose=self.hazard_root is not None,
                   robot_body_pose_w=np.concatenate([array(self.robot.data.body_pos_w)[0], array(self.robot.data.body_quat_w)[0]], axis=-1),
                   background_rigid_poses=array(self.view.get_transforms()),
                   background_rigid_velocities=array(self.view.get_velocities()),
                   static_anchor_transforms=np.asarray(anchors).reshape(-1, 4, 4))
        self.rows.append(row)

    def code_integrity(self):
        final = {key: sha(path) for key, path in self.code_paths.items()}
        return dict(passed=final == self.code_hashes, initial_sha256=self.code_hashes,
                    checked_sha256=final, checked_physics_step=int(self.sim.get_physics_step_count()),
                    scope="Source captured before warmup and compared at this check; no physical steps added")

    def finish(self, *, world_ready=False, world=None, arm_ids=None):
        if self.report is not None:
            return self.report
        self.active = False
        if world_ready:
            self.metadata["world_ready_physics_step"] = int(self.sim.get_physics_step_count())
        self.output.mkdir(parents=True, exist_ok=True)
        data = {key: np.asarray([row[key] for row in self.rows]) for key in self.rows[0]}
        path = self.output / "startup_physics_states.npz"
        if path.exists() or (self.output / "startup_audit.json").exists():
            raise FileExistsError("Startup sidecar already exists; never overwrite evidence")
        np.savez_compressed(path, **data,
                            joint_names=np.asarray(self.metadata["joint_names"]),
                            robot_body_names=np.asarray(self.metadata["robot_body_names"]),
                            background_paths=np.asarray(self.metadata["background_paths"]),
                            static_anchor_paths=np.asarray(self.metadata["static_anchor_paths"]))
        try:
            report = audit_startup(data, self.metadata)
            if world is None or arm_ids is None:
                raise ValueError("Collision world/arm mapping absent; prefix geometry is unverified")
            report["prefix_safety"] = validate_prefix(data, self.metadata, world, arm_ids)
            report["checks"]["prefix_execution_safety"] = bool(report["prefix_safety"]["passed"]
                and report["prefix_safety"]["checks"] == len(self.rows))
            report["checks"]["code_unchanged_during_startup"] = self.code_integrity()["passed"]
            report["passed"] = all(report["checks"].values())
        except Exception as error:
            report = dict(schema_version=1, passed=False, checks={"complete_valid_startup_evidence": False}, error=repr(error))
        report.update(metadata=self.metadata, input_sha256={"startup_physics_states.npz": sha(path),
                      **self.code_hashes}, code_snapshots=self.code_snapshots)
        (self.output / "startup_audit.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        self.report = report
        return report
