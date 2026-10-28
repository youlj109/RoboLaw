"""CPU reconstruction of a fixed, measured bowl-transport planning query.

No simulator, torch or CUDA initialization. This is a planning comparison, not
an episode validator or a physical grasp/avoidance guarantee. Only the hazard
presence may differ within a seed pair; no path splicing or waypoint selection.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[3]
TASK = "kitchen_bench_replicator_g_shape_bowl_sink"
SEEDS = (42, 43, 44)


def require(value, message):
    if not value:
        raise ValueError(message)


def sha(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                    allow_nan=False).encode()).hexdigest()


def plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(type(value).__name__)


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False, default=plain) + "\n")


def inventory(directory):
    directory = Path(directory)
    return {str(p.relative_to(directory)): sha(p) for p in sorted(directory.rglob("*")) if p.is_file()}


def validate_seeds(seeds):
    require(tuple(seeds) == SEEDS, "Predeclared ordered seeds must remain 42, 43, 44")


def require_hazard_reference(obstacle, hazard_episode):
    require(type(obstacle.get("static")) is bool, "Explicit hazard static/dynamic identity required")
    require(obstacle["static"] or hazard_episode is not None,
            "Dynamic property requires a successful measured hazard episode; no spawn fallback")


def cuda_zero(report):
    return report.get("valid") is True and set(report.get("components", {})) == {
        "joint_bound", "world_collision", "self_collision"} and all(
        row.get("nonzero_samples") == [] for row in report["components"].values())


def transport_index(result, data):
    plans = [(i, p) for i, p in enumerate(result["plans"], 1) if p["phase"] == "transport"]
    require(len(plans) == 1, "Exactly one recorded transport query required")
    number, plan = plans[0]
    require(plan["status"] == "executed", "Recorded transport did not execute successfully")
    hits = np.flatnonzero(data["physics_step"] == plan["start_physics_step"])
    require(len(hits) == 1, "Transport start must be a unique retained physical sample")
    index = int(hits[0])
    lifts = np.flatnonzero(data["phase"] == "lift")
    require(len(lifts) and index == int(lifts[-1]) and str(data["phase"][index]) == "lift",
            "Transport must start at the final actual lift sample, not the first transport sample")
    require(index + 1 < len(data["phase"]) and str(data["phase"][index + 1]) == "transport",
            "Transport boundary must immediately follow the retained lift state")
    require(result["observed_transport_binding"]["physics_step"] == int(data["physics_step"][index]),
            "Recorded attachment was not rebound at the actual transport start")
    return index, number, plan


def successful_record(result, metadata, expected_variant):
    require(result["task_id"] == metadata["task_id"] == TASK, "Original bowl/sink task identity required")
    require(result["variant"] == expected_variant, "Unexpected baseline/hazard variant")
    require(result["status"] == "sequence_complete" and all(result.get(k) is True for k in
        ("physical_trial_passed", "physical_sequence_complete", "original_task_success",
         "final_original_destination_predicate")), "Complete successful original-task episode required")
    require(result.get("physics_state_setters_used") is False and result.get("physical_weld_used") is False,
            "Physical setters or welding are not valid paired evidence")
    require(metadata["task_object_names"] == ["bowl"], "Bowl payload schema required")
    require(abs(metadata["action_dt_s"] - 1 / 15) < 1e-12 and
            abs(metadata["physics_dt_s"] - 1 / 120) < 1e-12, "Original 15 Hz / 120 Hz sampling required")
    require(result["runtime_threads"] == dict(requested_physics_threads=2, requested_torch_threads=2,
        live_physics_threads=2, live_torch_threads=2, requested_physics_device="cpu",
        actual_environment_device="cpu"), "Matched original CPU physics / 2+2 thread conditions required")


def controls(result, metadata, profile, variant):
    padding = .015 if variant == "human" else 0.
    require(result["controller_profile"] == profile, "Matched source-bound controller profile required")
    plans = [p for p in result["plans"] if p["phase"] == "transport"]
    require(len(plans) == 1, "Exactly one original transport padding record required")
    plan = plans[0]
    recorded_padding = [plan["metadata"]["transport_object_padding_m"],
                        plan["validation"]["transport_object_padding_m"], plan["payload_geometry"]["padding_m"]]
    if "transport_object_padding_m" in result:
        recorded_padding.append(result["transport_object_padding_m"])
    require(all(value == padding for value in recorded_padding), "Baseline/variant transport padding mismatch")
    if variant == "human":
        require(result.get("control_profile") == "human", "Human requires its .015 m matched control baseline")
    return dict(controller_profile=profile, padding_m=padding, action_dt_s=metadata["action_dt_s"],
        physics_dt_s=metadata["physics_dt_s"], max_joint_speed_rad_s=.4,
        runtime_threads=result["runtime_threads"], target_support_policy_sha256=result["target_support_policy_sha256"],
        background_physics=metadata["background_physics"], source_hashes=metadata["source_hashes"])


def densify(path):
    path = np.asarray(path, float)
    require(path.ndim == 2 and path.shape[1] == 7 and len(path) >= 2 and np.isfinite(path).all(),
            "At least two finite original seven-arm waypoints required")
    return np.concatenate([path[:1]] + [np.linspace(a, b, 9)[1:] for a, b in zip(path[:-1], path[1:])])


def path_metrics(path, kin, q, target):
    path = np.asarray(path)
    tcp = np.asarray([kin.fk(row) for row in path])
    start_error = float(np.max(np.abs(path[0] - q)))
    translation = float(np.linalg.norm(tcp[-1, :3, 3] - target[:3, 3]))
    rotation = float(Rotation.from_matrix(tcp[-1, :3, :3] @ target[:3, :3].T).magnitude())
    return dict(tcp_path_length_m=float(np.linalg.norm(np.diff(tcp[:, :3, 3], axis=0), axis=1).sum()),
        joint_path_length_rad=float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()),
        duration_s=(len(path) - 1) / 15, waypoints=len(path), start_joint_error_rad=start_error,
        goal_translation_error_m=translation, goal_rotation_error_rad=rotation,
        fixed_query_passed=start_error <= 1e-4 and translation <= .002 and rotation <= .02)


class BowlPairedQuery:
    def __init__(self, baseline, layout_path, hazard_episode=None, controller_profile=None, provenance_output_dir=None):
        from .bowl_controller_profile import read_profile
        from .bowl_safety_policy import load_layout
        from .task_collection_evidence import audit_provenance
        from scripts.extensions.replicator.task_audit_sidecars import load_episode

        self.baseline, self.layout_path = Path(baseline), Path(layout_path)
        self.provenance_output_dir = Path(provenance_output_dir) if provenance_output_dir is not None else None
        candidate = json.loads(self.layout_path.read_text())
        self.variant = candidate["obstacle"]["category"]
        self.layout, self.policy = load_layout(self.layout_path, self.variant)
        require_hazard_reference(self.layout["obstacle"], hazard_episode)
        self.source_layout = {k: v for k, v in self.layout.items() if k != "obstacle"}
        self.profile = read_profile(controller_profile)
        self.result = json.loads((self.baseline / "result.json").read_text())
        self.meta, self.data, self.trajectory = load_episode(self.baseline)
        successful_record(self.result, self.meta, "base")
        require(self.result["layout"] == self.meta["source_layout"] == self.source_layout,
                "Baseline original source layout differs")
        self.controls = controls(self.result, self.meta, self.profile, self.variant)
        self.index, self.plan_number, self.plan = transport_index(self.result, self.data)
        self.provenance = audit_provenance(self.baseline / "task_collection_provenance.json", self.meta,
                                          self.data["physics_step"][-1])
        self.baseline_inputs = inventory(self.baseline)
        require(self.meta["source_hashes"]["bowl"] == self.profile["source_bowl_sha256"], "Source bowl hash differs")
        self.contacts = json.loads((self.baseline / "grasp_contacts.json").read_text())
        self.contacts = [row for row in self.contacts if row["physics_step"] <= self.start_step]
        item = self.layout["obstacle"]
        self.spawn_pose = np.r_[item["position"], Rotation.from_euler("z", item.get("yaw_deg", 0), degrees=True).as_quat()]
        self.hazard_pose = self.spawn_pose.copy()
        self.hazard_inputs = None
        self.hazard_evidence = dict(static=item["static"], pose_source="Authored immutable static obstacle",
                                   actual_dynamic_pose_verified=False)
        if hazard_episode is not None:
            self._load_hazard_episode(Path(hazard_episode), load_episode, audit_provenance)

    @property
    def start_step(self):
        return int(self.data["physics_step"][self.index])

    def _load_hazard_episode(self, path, loader, provenance_auditor):
        from .bowl_safety_recorder import audit_hazard
        meta, data, _ = loader(path)
        result = json.loads((path / "result.json").read_text())
        successful_record(result, meta, self.variant)
        require(result["layout"] == meta["source_layout"] == self.layout, "Measured hazard layout differs")
        require(controls(result, meta, self.profile, self.variant) == self.controls, "Matched episode controls differ")
        require(result["safety_policy"] == self.policy, "Measured hazard policy/asset binding differs")
        index, _, _ = transport_index(result, data)
        saved = json.loads((path / "hazard_audit.json").read_text())
        raw = json.loads((path / "hazard_contacts.json").read_text())
        require(raw["metadata"]["policy"] == self.policy, "Native hazard identity/policy differs")
        independent = audit_hazard(data, raw["metadata"], raw["rows"])
        require(saved["passed"] is True and independent["passed"] is True and
                saved == independent, "Complete independent hazard stability/contact audit required")
        # The production safety collector captures the unchanged canonical
        # layout separately from its additive full layout. Keep the old generic
        # refusal and verify their exact archived roles independently.
        from .bowl_safety_provenance_review import audit
        require(self.provenance_output_dir is not None, "Hazard provenance needs a separate derived-review output directory")
        audited = audit(path / "task_collection_provenance.json", meta, result, data["physics_step"][-1],
            self.provenance_output_dir / "hazard_episode_source_role_projection.json")
        self.hazard_pose = data["hazard_pose_w"][index].copy()
        require(self.hazard_pose.shape == (7,) and np.isfinite(self.hazard_pose).all(), "Missing measured hazard pose")
        self.hazard_inputs = inventory(path)
        self.hazard_evidence = dict(episode=str(path), input_sha256=self.hazard_inputs,
            transport_start_physics_step=int(data["physics_step"][index]), array_row=index,
            static=self.layout["obstacle"]["static"], pose_source="Actual hazard pose at its successful transport planning boundary",
            pose_w=self.hazard_pose.tolist(), reset_pose_w=data["hazard_pose_w"][0].tolist(),
            authored_spawn_pose_w=self.spawn_pose.tolist(), independent_hazard_audit=independent,
            provenance=audited, actual_dynamic_pose_verified=not self.layout["obstacle"]["static"],
            scope="Pose frozen for the planning comparison only; dynamic future motion is not simulated")

    def build(self, hazard_on):
        from .bowl_safety_cpu import build_world
        from .bowl_transport_binding import lift_motion_evidence, observe_transport_binding, apply_transport_binding, transport_target
        from .task_motion import pose_matrix

        world, kin, _ = build_world(self.layout if hazard_on else self.source_layout, self.policy)
        i = self.index
        world.sync_background(self.meta["background_paths"], self.data["background_rigid_poses"][i])
        synchronized, noncollision = [], []
        for path, actual in zip(self.meta["background_paths"], self.data["background_rigid_poses"][i]):
            if any(world._matches(row[0], path) for row in world.objects):
                require(path in world._background_poses and np.array_equal(world._background_poses[path], actual),
                        "An actual collision-bearing background body was not synchronized: " + path)
                synchronized.append(path)
            else:
                noncollision.append(path)
        world.paired_background_sync = dict(synchronized_collision_body_paths=synchronized,
            recorded_bodies_without_source_colliders=noncollision, actual_observation_step=self.start_step)
        fingers = [self.meta["joint_names"].index(name) for name in world.GRIPPER_JOINT_NAMES]
        arm = [self.meta["joint_names"].index(f"panda_joint{j}") for j in range(1, 8)]
        payload_index = self.meta["task_object_names"].index("bowl")
        pose = self.data["task_object_states"][i, payload_index, :7]
        world.payload_pose = pose_matrix(pose)
        world.set_obstacle_pose("bowl", pose)
        world.gripper_joint_positions = self.data["q"][i, fingers].copy()
        world.gripper_joint_position = float(world.gripper_joint_positions[0])
        world.attachment = np.asarray(self.result["measured_tcp_to_bowl"], float).copy()
        world.set_phase("lift")
        if hazard_on:
            world.set_obstacle_pose(self.policy["hazard_name"], self.hazard_pose)
        close_end = int(np.flatnonzero(self.data["phase"] == "close")[-1])
        history = [{k: self.data[k][j] for k in ("physics_step", "phase", "tcp_pose_w", "task_object_states")}
                   for j in range(close_end, i + 1)]
        motion = lift_motion_evidence(history, payload_index)
        context = observe_transport_binding(world, kin, self.profile, physics_step=self.start_step,
            arm_q=self.data["q"][i, arm].copy(), measured_tcp=pose_matrix(self.data["tcp_pose_w"][i]),
            measured_payload=world.payload_pose.copy(), raw_contact_rows=self.contacts, lift_motion=motion)
        # JSON canonicalization resolves dataclass tuple/list representation only;
        # numeric observations, input hashes and source gates are exact.
        require(json.loads(json.dumps(context.evidence())) == self.result["observed_transport_binding"],
                "Reconstructed transport observation differs from retained original")
        apply_transport_binding(world, self.profile, context)
        world.set_phase("transport")
        target, source_target = transport_target(world, self.profile, context)
        require(np.array_equal(target, self.plan["target_tcp"]), "Source-rule target differs from the original fixed transport goal")
        require(np.array_equal(world.attachment, self.plan["measured_planning_attachment"]), "Original measured planning attachment differs")
        require(world.default_clearance == .002, "Original non-hazard 2 mm margin changed")
        q = np.asarray(context.arm_q)
        # This checks the actual common start with all source geometry. No
        # artificial connecting segment or replacing the initial waypoint.
        valid = world.valid(q, mode="planning", gripper_q=world.gripper_joint_positions)
        start = dict(passed=bool(valid), measurement=deepcopy(world.last_clearance), actual_q=q.tolist())
        return world, kin, context, target, source_target, start

    def evidence(self, world, kin, context, target, source_target, start):
        path_file = self.baseline / f"plan_{self.plan_number:02d}_transport.npz"
        with np.load(path_file, allow_pickle=False) as raw:
            path = raw["positions"].copy()
            require(np.array_equal(raw["start_q"], context.arm_q) and np.array_equal(raw["goal_tcp"], target),
                    "Original representative transport path does not bind the common query")
        baseline_metrics = path_metrics(path, kin, np.asarray(context.arm_q), target)
        actual = np.flatnonzero(self.data["phase"] == "transport")
        tcp = self.data["tcp_pose_w"][np.r_[self.index, actual]]
        return dict(task_id=TASK, variant=self.variant, baseline=str(self.baseline), baseline_input_sha256=self.baseline_inputs,
            baseline_provenance=self.provenance, hazard_pose=self.hazard_pose, hazard_pose_evidence=self.hazard_evidence,
            layout=self.layout, layout_sha256=sha(self.layout_path), safety_policy=self.policy, controls=self.controls,
            transport_start_physics_step=self.start_step, transport_start_array_row=self.index,
            observed_transport_binding=context.evidence(), source_target=source_target, target_tcp=target,
            actual_background_paths=self.meta["background_paths"],
            actual_background_poses=self.data["background_rigid_poses"][self.index],
            actual_background_sha256=digest(self.data["background_rigid_poses"][self.index].tolist()),
            background_sync=world.paired_background_sync,
            baseline_transport_path=dict(path=str(path_file), sha256=sha(path_file), metrics=baseline_metrics,
                actual_tcp_path_length_m=float(np.linalg.norm(np.diff(tcp[:, :3], axis=0), axis=1).sum()),
                actual_recorded_transport_duration_s=len(actual) / 120,
                distinction="Planned path duration excludes the original 0.3 s transport endpoint hold; actual phase includes it"),
            original_start_source_fcl=start, payload_source=world.payload_geometry.provenance,
            raw_lift_slip_preserved=self.result["lift_attachment_motion"], physical_execution=False, accepted=False)


def geometric_sample(world, robot, q, context, hazard_name):
    """Separate original triangle surfaces, solid hulls and conservative checks."""
    import fcl
    from kitchen_safety.geometry_audit import minimum_distance
    grip = dict(zip(world.GRIPPER_JOINT_NAMES, context.six_finger_q))
    pose = world.kin.fk(q) @ np.asarray(context.measured_attachment).reshape(4, 4)
    points = world.payload_geometry.vertices @ pose[:3, :3].T + pose[:3, 3]
    payload = [("source_bowl_triangles", fcl.CollisionObject(world.payload_triangle_model,
        fcl.Transform(pose[:3, :3], pose[:3, 3])), points.min(0), points.max(0))]
    hazards = [row for row in world.objects if world._matches(row[0], hazard_name)]
    require(hazards, "Nonempty actual hazard source collision geometry required")
    required = world.execution_clearances[hazard_name]
    source_robot = robot.shapes(q, grip)
    hulls = [row for row in world.shapes(q, grip, pose) if
             row[0].startswith(("robot_hull/", "gripper/", "payload/"))]
    require(hulls, "Nonempty conservative source hull representation required")
    result = dict(source_mesh=minimum_distance(source_robot + payload, hazards, required),
        source_robot_mesh=minimum_distance(source_robot, hazards, required),
        source_bowl_mesh=minimum_distance(payload, hazards, required),
        source_hull=minimum_distance(hulls, hazards, required),
        conservative_execution=world.clearance(q, mode="execution", gripper_q=grip, object_pose=pose,
                                               obstacle_filter=(hazard_name,)),
        conservative_planning=world.clearance(q, mode="planning", gripper_q=grip, object_pose=pose,
                                              obstacle_filter=(hazard_name,)))
    require(all(np.isfinite(row["distance_m"]) for row in result.values()), "Missing/nonfinite hazard geometry distance")
    return result


def summarize_geometry(rows):
    require(rows, "Complete nonempty geometry sampling required")
    names = ("source_mesh", "source_robot_mesh", "source_bowl_mesh", "source_hull",
             "conservative_execution", "conservative_planning")
    return {name: dict(passed=all(row[name]["valid"] for row in rows),
        violations=sum(not row[name]["valid"] for row in rows),
        intersections=sum(row[name]["intersecting"] for row in rows),
        minimum=min((dict(sample=row["sample"], time_s=row["time_s"], **row[name]) for row in rows),
                    key=lambda row: row["distance_m"])) for name in names}


def pair_gates(rows):
    result = []
    for seed in SEEDS:
        off = [r for r in rows if r["seed"] == seed and r["hazard_in_planning_world"] is False]
        on = [r for r in rows if r["seed"] == seed and r["hazard_in_planning_world"] is True]
        checks = dict(exactly_one_off_and_on=len(off) == len(on) == 1,
                      both_planned=len(off) == len(on) == 1 and all(r.get("success") is True for r in off + on))
        if checks["both_planned"]:
            a, b = off[0], on[0]
            checks.update(identical_fixed_query=a["query_sha256"] == b["query_sha256"] and
                a["metrics"]["fixed_query_passed"] and b["metrics"]["fixed_query_passed"],
                original_full_cuda_and_source=all(r["full_world_validation"]["valid"] and
                    r["full_world_validation"]["validation_complete"] and cuda_zero(r["dense_120hz_cuda"]) and
                    r["dense_120hz_all_source_passed"] for r in (a, b)),
                off_source_margin_risk=all(a["geometry"][k]["violations"] > 0 for k in ("source_mesh", "source_hull")),
                on_source_and_conservative_safe=all(b["geometry"][k]["passed"] for k in
                    ("source_mesh", "source_hull", "conservative_execution", "conservative_planning")))
        result.append(dict(seed=seed, checks=checks, passed=all(checks.values())))
    return result
