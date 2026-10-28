"""Read-only evidence helper for a FUTURE task-aware collection run.

Construct TaskRunProvenance before SimulationAppContext. Bind the composed
source environment before builder.make_registered/reset. Construct
TaskPredicateRecorder immediately after reset and record every physical sample,
then every completed action after termination predicates have been evaluated.
No function steps, sets state, invokes objects_settled, or changes predicates.
Existing episodes cannot be retrofitted with this missing runtime evidence.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import re

import numpy as np

from .collection_evidence import CollectionProvenance, LOCAL_CODE
from .startup_recorder import array, sha

ROOT = Path(__file__).resolve().parents[3]
ARENA = "third_party/IsaacLab-Arena/isaaclab_arena/"
TASK_CODE = tuple(sorted(set(LOCAL_CODE) | {
    "scripts/extensions/replicator/task_collect.py",
    "scripts/extensions/replicator/audit_task_source_relation.py",
    *["kitchen_safety/extensions/replicator/"+name+".py" for name in (
        "task_collection_evidence", "task_environment", "task_recorder", "task_contacts", "task_collision",
        "task_contact_policy", "task_curobo", "task_motion", "task_robot_spheres",
        "bowl_collision", "bowl_motion", "bowl_recorder", "bowl_controller_profile",
        "source_support_curobo", "source_support_query", "banana_controller_profile", "task_plan_diagnostics")],
    *[ARENA+name+".py" for name in ("environments/arena_world", "environments/arena_env_builder_cfg",
        "environments/isaaclab_arena_manager_based_env", "assets/object_reference", "tasks/predicates/spatial",
        "tasks/predicates/object_settling", "tasks/predicates/temporal", "progress_tracking/completion_criteria",
        "utils/bounding_box", "utils/usd/helpers", "relations/validation/pre_physics", "relations/relation_loss_strategies")],
    "third_party/IsaacLab-Arena/submodules/IsaacLab/source/isaaclab_physx/isaaclab_physx/sensors/contact_sensor/contact_sensor.py",
}))
BANANA_TASK = "kitchen_bench_replicator_g_shape_banana_bowl"
BOWL_TASK = "kitchen_bench_replicator_g_shape_bowl_sink"
BANANA_RECIPE = "kitchen_safety/extensions/replicator/banana_background_physics.py"
BANANA_GRASP_CANDIDATE = "kitchen_safety/extensions/replicator/banana_halfcenter_candidate.py"
BANANA_TRANSPORT_CANDIDATE = "kitchen_safety/extensions/replicator/banana_transport_height.py"
BASE_RECIPE = "kitchen_safety/extensions/replicator/background_physics.py"
STAND_SOURCE_KEY = "runtime/home/.cache/isaaclab_arena/usd/robot_on_stand/replicator_local_droid_0.8_1.08x0.91032.usd"
STAND_RUNTIME_PATH = "/home/user/.cache/isaaclab_arena/usd/robot_on_stand/replicator_local_droid_0.8_1.08x0.91032.usd"


def required_task_sources(task_id):
    """Task-local additions must not retroactively add banana code to bowl runs."""
    return tuple(sorted(set(TASK_CODE) | ({BANANA_RECIPE} if task_id == BANANA_TASK else set())))


def input_key(path, root=ROOT):
    candidate = Path(path)
    return str(candidate.relative_to(root)) if candidate.is_absolute() and candidate.is_relative_to(root) else str(candidate)


def known_old_bowl_stand_alias(entries, required_key, digest, task_id):
    """One historical bind-mount alias only; never search arbitrary equal hashes."""
    if task_id != BOWL_TASK or required_key != STAND_SOURCE_KEY:
        return False
    actual = [(key, row) for key, row in entries.items() if row["kind"] == "composed_robot_on_stand"]
    return bool(len(actual) == 1 and actual[0][0] == STAND_RUNTIME_PATH and
        actual[0][1]["path"] == STAND_RUNTIME_PATH and
        actual[0][1]["initial_sha256"] == actual[0][1]["checked_sha256"] == digest)


def require(value, message):
    if not value:
        raise ValueError(message)


def json_new(path, value):
    with Path(path).open("x") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


class TaskRunProvenance(CollectionProvenance):
    """Selected execution sources + explicit inputs, all captured before use."""
    def __init__(self, layout_path, *, root=ROOT, collector_path=None, input_paths=()):
        super().__init__(root, TASK_CODE)
        if collector_path is not None:
            self.capture(collector_path, "execution_source", snapshot=True)
        self.capture(layout_path, "source_layout_input", snapshot=True)
        for path in input_paths:
            self.capture(path, "controller_input", snapshot=True)
        self.layout = json.loads(Path(layout_path).read_text())
        self.task_id = self.layout["task_id"]
        self.required_execution_sources = required_task_sources(self.task_id)
        for path in set(self.required_execution_sources) - set(TASK_CODE):
            self.capture(path, "execution_source", snapshot=True)
        self.capture(ARENA+"../isaaclab_arena_environments/kitchen_bench/"+self.task_id+".yaml", "source_task_yaml", snapshot=True)
        self.capture(self.layout["kitchen_usd"], "source_kitchen")
        for item in self.layout["objects"].values():
            self.capture(item["usd_path"], "task_asset_root")
        for path in (
            "assets/nvidia/Assets/Isaac/6.1/Isaac/IsaacLab/Arena/assets/robot_library/droid/franka_robotiq_2f_85_flattened.usd",
            "assets/nvidia/Assets/Isaac/6.1/Isaac/IsaacLab/Arena/assets/object_library/srl_robolab_assets/robots/franka_stand_grey.usda",
        ):
            self.capture(path, "robot_asset_root")
        self.scene_bound = False

    def bind_grasp_candidate(self, profile_path):
        """Capture a selected banana candidate before App/Kit or profile use."""
        require(self.task_id == BANANA_TASK, "Half-center candidate is banana-only")
        require(not self.scene_bound, "Candidate must be captured before scene binding")
        self.capture(BANANA_GRASP_CANDIDATE, "execution_source", snapshot=True)
        self.capture(profile_path, "grasp_candidate_input", snapshot=True)
        key = input_key(Path(profile_path).resolve(), self.root)
        self.grasp_candidate_binding = dict(profile_path=key,
            profile_sha256=self.entries[key]["initial_sha256"],
            module_path=BANANA_GRASP_CANDIDATE,
            module_sha256=self.entries[BANANA_GRASP_CANDIDATE]["initial_sha256"])
        return dict(self.grasp_candidate_binding)

    def bind_transport_candidate(self, profile_path):
        """Capture this banana-only goal rule before App/Kit or input use."""
        require(self.task_id == BANANA_TASK, "Transport-height candidate is banana-only")
        require(not self.scene_bound, "Transport candidate must be captured before scene binding")
        self.capture(BANANA_TRANSPORT_CANDIDATE, "execution_source", snapshot=True)
        self.capture(profile_path, "transport_candidate_input", snapshot=True)
        key = input_key(Path(profile_path).resolve(), self.root)
        self.transport_candidate_binding = dict(profile_path=key,
            profile_sha256=self.entries[key]["initial_sha256"],
            module_path=BANANA_TRANSPORT_CANDIDATE,
            module_sha256=self.entries[BANANA_TRANSPORT_CANDIDATE]["initial_sha256"])
        return dict(self.transport_candidate_binding)

    def bind_environment(self, definition):
        """Call after scene assembly but BEFORE env construction/reset."""
        require(definition.resolved_task_layout == self.layout, "Constructed task layout differs from pre-initialization input")
        info = definition.background_physics_provenance
        self.capture(info["source"], "source_kitchen", expected_sha256=info["source_sha256"])
        recipe_path = info.get("recipe_source_path", BASE_RECIPE)
        if self.task_id == BANANA_TASK:
            require(input_key(recipe_path, self.root) == BANANA_RECIPE and "base_profile" in info,
                "Banana task requires its captured leaf and inherited profile")
        require(input_key(Path(recipe_path).resolve() if Path(recipe_path).is_absolute() else (self.root/recipe_path).resolve(), self.root)
            in self.entries, "Actual recipe was not captured before initialization")
        self.capture(recipe_path, "execution_source", snapshot=True, expected_sha256=info["recipe_sha256"])
        if "base_profile" in info:
            inherited = info["base_profile"]
            require(inherited["source_sha256"] == info["source_sha256"] and
                inherited["recipe_sha256"] == info["base_recipe_sha256"] and
                inherited["overlay_sha256"] == info["base_overlay_sha256"], "Inherited physical profile identity differs")
            require(input_key(Path(info["base_recipe_source_path"]).resolve(), self.root) in self.entries,
                "Inherited recipe was not captured before initialization")
            self.capture(info["base_recipe_source_path"], "execution_source", snapshot=True, expected_sha256=info["base_recipe_sha256"])
            self.capture(info["base_overlay_path"], "inherited_kitchen_overlay", expected_sha256=info["base_overlay_sha256"])
        self.capture(definition.scene.assets["kitchen"].usd_path, "composed_kitchen_overlay", expected_sha256=info["overlay_sha256"])
        source = definition.source_relation_audit
        self.capture(source["source_yaml"], "source_task_yaml", snapshot=True, expected_sha256=source["source_yaml_sha256"])
        for path, digest in source["assets_sha256"].items():
            self.capture(path, "source_relation_asset", expected_sha256=digest)
        self.capture(definition.embodiment.scene_config.robot.spawn.usd_path, "composed_robot_on_stand")
        self.background_physics = info
        self.scene_bound = True

    def finish_task(self, directory, final_physics_step):
        require(self.scene_bound, "Actual composed source scene was not bound before use")
        report = self.check(final_physics_step=int(final_physics_step))
        report.update(task_id=self.task_id, source_layout=self.layout, background_physics=self.background_physics,
            initialization_scope="TASK_CODE and input layout/source assets captured before SimulationAppContext; actual overlay bound before env creation/reset",
            required_execution_sources=list(self.required_execution_sources),
            scope="Selected runtime source and input start/end hashes, not process memory or all external binary/transitive USD dependencies.")
        if hasattr(self, "grasp_candidate_binding"):
            report["grasp_candidate_binding"] = self.grasp_candidate_binding
        if hasattr(self, "transport_candidate_binding"):
            report["transport_candidate_binding"] = self.transport_candidate_binding
        json_new(Path(directory)/"task_collection_provenance.json", report)
        return report


def sensor_identity(env, task):
    """Actual native sensed body inventory and resolved original filter target."""
    sensor = env.scene[task.contact_sensor_name]
    expressions = list(sensor.cfg.filter_prim_paths_expr)
    resolved = []
    for expression in expressions:
        pattern = expression.replace("{ENV_REGEX_NS}", "/World/envs/env_.*").replace("{ENV_NS}", "/World/envs/env_0")
        resolved.extend(str(prim.GetPath()) for prim in env.sim.stage.Traverse() if re.fullmatch(pattern, str(prim.GetPath())))
    sensed = list(sensor.body_physx_view.prim_paths)
    force = array(sensor.data.force_matrix_w)
    require(force.shape == (1, 1, 1, 3), "Original predicate requires one sensed body / one destination filter")
    require(len(sensed) == len(resolved) == 1, "Sensor target identity is ambiguous or missing")
    def exact(path):
        return path.replace("{ENV_REGEX_NS}", "/World/envs/env_0").replace("{ENV_NS}", "/World/envs/env_0")
    require(sensed == [exact(task.pick_up_object.get_prim_path())] and
        resolved == [exact(task.destination_location.get_prim_path())], "Actual sensor differs from original task roles")
    require(sensor.contact_view.filter_count == 1, "Native filter inventory differs from original task")
    return dict(contact_sensor_name=task.contact_sensor_name, configured_sensed_expression=sensor.cfg.prim_path,
        configured_filter_expressions=expressions, sensed_native_paths=sensed, resolved_filter_prim_paths=resolved,
        native_filter_count=int(sensor.contact_view.filter_count), force_matrix_shape=list(force.shape),
        actual_pick_up_path=exact(task.pick_up_object.get_prim_path()), actual_destination_path=exact(task.destination_location.get_prim_path()),
        scope="Read native sensed-body paths, configured filter expression resolved against the actual stage and native filter count; no inferred contact from proximity.")


class TaskPredicateRecorder:
    def __init__(self, env, definition):
        self.env, self.task = env, definition.task
        task = self.task
        self.pick, self.destination = task.pick_up_object.name, task.destination_location.name
        self.physics, self.actions = [], []
        bounds = {}
        for name in (self.pick, self.destination):
            box = env.arena_world.get_aabb_in_local_frame(name)
            low, high = array(box.min_point), array(box.max_point)
            require(low.shape == high.shape == (1, 3) and np.isfinite([low, high]).all() and np.all(high > low), "Invalid actual spawned predicate bounds")
            bounds[name] = dict(min=low[0].tolist(), max=high[0].tolist())
        self.metadata = dict(schema_version=1, task_id=definition.resolved_task_layout["task_id"],
            pick_up_object=self.pick, destination_location=self.destination,
            actual_spawned_local_bounds=bounds, sensor_identity=sensor_identity(env, task),
            thresholds={key: getattr(task, key) for key in ("force_threshold", "velocity_threshold", "support_cone_half_angle_rad", "placement_consecutive_steps")},
            reset_physics_step=int(env.sim.get_physics_step_count()), physics_dt_s=float(env.physics_dt), action_dt_s=float(env.step_dt),
            scope="Read original task config, runtime spawned bounds, actual filtered force and existing rest-recorder state. Never evaluates stateful objects_settled or changes termination state.")

    def read(self, phase, action_index, *, action_boundary):
        from isaaclab_arena.tasks.predicates.spatial import object_on_destination
        from isaaclab.managers import SceneEntityCfg
        task, env = self.task, self.env
        entry = env.object_initial_rest_pose_recorder._entries.get(self.pick)
        has_rest = bool(array(entry["settled"])[0]) if entry is not None else False
        rest = array(entry["position"])[0] if has_rest else np.zeros(3)
        pose = array(env.arena_world.get_pose_w(self.pick))[0]
        destination = array(env.arena_world.get_pose_w(self.destination))[0]
        velocity = array(env.arena_world.get_mean_linear_velocity_w(self.pick))[0]
        force = array(env.scene[task.contact_sensor_name].data.force_matrix_w)[0, 0, 0]
        on = bool(array(object_on_destination(env, object_cfg=SceneEntityCfg(self.pick),
            destination_cfg=SceneEntityCfg(self.destination), contact_sensor_cfg=task.contact_sensor_cfg,
            force_threshold=task.force_threshold, velocity_threshold=task.velocity_threshold,
            support_cone_half_angle_rad=task.support_cone_half_angle_rad))[0])
        # No call to objects_settled: it would mutate the original task's state.
        terms = {name: bool(array(env.termination_manager.get_term(name))[0]) for name in env.termination_manager.active_terms} if action_boundary else {}
        return dict(physics_step=int(env.sim.get_physics_step_count()), phase=str(phase), action_index=int(action_index),
            object_pose=pose, destination_pose=destination, object_centroid_world=array(env.arena_world.get_centroid_w(self.pick))[0],
            object_linear_velocity=velocity, destination_contact_force=force, has_original_rest_pose=has_rest,
            original_rest_position=rest, original_above_rest=bool(has_rest and pose[2] > rest[2]+.01),
            original_object_on_destination=on, terminal_success=bool(terms.get("success", False)),
            terminal_failure=bool(any(value for name, value in terms.items() if name != "success")))

    def sample_physics(self, phase, action_index):
        row = self.read(phase, action_index, action_boundary=False)
        require(not self.physics or row["physics_step"] == self.physics[-1]["physics_step"]+1, "Predicate physics coverage has a gap/duplicate")
        self.physics.append(row)

    def sample_action(self, phase, action_index):
        row = self.read(phase, action_index, action_boundary=True)
        require(not self.actions or row["action_index"] == self.actions[-1]["action_index"]+1, "Predicate action coverage has a gap/duplicate")
        self.actions.append(row)

    def finish(self, directory):
        directory = Path(directory)
        self.metadata["sensor_identity_at_end"] = sensor_identity(self.env, self.task)
        require(self.metadata["sensor_identity"] == self.metadata["sensor_identity_at_end"], "Actual sensor identity changed during collection")
        require(self.physics and self.physics[0]["physics_step"] == self.metadata["reset_physics_step"], "Missing reset predicate observation")
        files = {}
        for name, rows in [("physics", self.physics), ("actions", self.actions)]:
            path = directory/("task_predicate_"+name+".npz")
            require(rows, "Missing predicate "+name+" observations")
            if path.exists():
                raise FileExistsError(path)
            with path.open("xb") as stream:
                np.savez_compressed(stream, **{key: np.asarray([row[key] for row in rows]) for key in rows[0]})
            files[path.name] = sha(path)
        self.metadata["input_sha256"] = files
        json_new(directory/"task_predicate_metadata.json", self.metadata)
        return self.metadata


def audit_grasp_candidate_binding(report, metadata, entries, profile_inputs):
    """Require new candidate evidence only when this recorded run uses it."""
    candidate_rows = [(key, row) for key, row in entries.items() if row["kind"] == "grasp_candidate_input"]
    collector_text = entries.get("scripts/extensions/replicator/task_collect.py", {}).get("source_text", "")
    candidate_required = (metadata["task_id"] == BANANA_TASK and ".bind_grasp_candidate(" in collector_text)
    binding = metadata.get("grasp_candidate_binding")
    if not (candidate_required or candidate_rows or binding is not None or "grasp_candidate_binding" in report):
        return dict(required=False)
    require(metadata["task_id"] == BANANA_TASK and binding is not None and
        binding == report.get("grasp_candidate_binding"), "Actual grasp candidate binding missing/different")
    require(len(candidate_rows) == 1 and candidate_rows[0][0] == binding["profile_path"] and
        candidate_rows[0][1]["initial_sha256"] == binding["profile_sha256"], "Actual grasp candidate input missing/different")
    require(binding["module_path"] == BANANA_GRASP_CANDIDATE and BANANA_GRASP_CANDIDATE in entries and
        entries[BANANA_GRASP_CANDIDATE]["kind"] == "execution_source" and
        entries[BANANA_GRASP_CANDIDATE]["initial_sha256"] == binding["module_sha256"],
        "Actual grasp candidate module missing/different")
    candidate = json.loads(candidate_rows[0][1]["source_text"])
    require(candidate.get("schema_version") == 1 and candidate.get("task_id") == BANANA_TASK and
        candidate.get("candidate_id") == "contactcenter1_jaw0" and
        candidate.get("geometric_centroid_fraction_xy") == .5 and candidate.get("additional_yaw_rad") == 0. and
        candidate.get("additional_world_height_m") == 0. and candidate.get("baseline_and_variants_share_candidate") is True,
        "Actual grasp candidate differs from prepared finite selection")
    require(candidate.get("base_profile_sha256") in {row["initial_sha256"] for row in profile_inputs} and
        candidate.get("source_banana_sha256") == metadata["source_hashes"]["banana"],
        "Actual grasp candidate source/base profile differs")
    return dict(required=True, candidate_id=candidate["candidate_id"], binding=binding)


def audit_transport_candidate_binding(report, metadata, entries, profile_inputs):
    """New banana runs bind their transport rule; old recorded code stays valid."""
    rows = [(key, row) for key, row in entries.items() if row["kind"] == "transport_candidate_input"]
    collector = entries.get("scripts/extensions/replicator/task_collect.py", {}).get("source_text", "")
    required = metadata["task_id"] == BANANA_TASK and ".bind_transport_candidate(" in collector
    binding = metadata.get("transport_candidate_binding")
    if not (required or rows or binding is not None or "transport_candidate_binding" in report):
        return dict(required=False)
    require(metadata["task_id"] == BANANA_TASK and binding is not None and
        binding == report.get("transport_candidate_binding"), "Actual transport candidate binding missing/different")
    require(len(rows) == 1 and rows[0][0] == binding["profile_path"] and
        rows[0][1]["initial_sha256"] == binding["profile_sha256"], "Actual transport candidate input missing/different")
    require(binding["module_path"] == BANANA_TRANSPORT_CANDIDATE and BANANA_TRANSPORT_CANDIDATE in entries and
        entries[BANANA_TRANSPORT_CANDIDATE]["kind"] == "execution_source" and
        entries[BANANA_TRANSPORT_CANDIDATE]["initial_sha256"] == binding["module_sha256"],
        "Actual transport candidate module missing/different")
    candidate = json.loads(rows[0][1]["source_text"])
    expected = dict(schema_version=1, task_id=BANANA_TASK, candidate_id="retain_actual_lift_end_height",
        payload_root_z_rule="current_observed_payload_root_z", destination_xy_rule="current_original_bowl_root_xy",
        payload_orientation_rule="current_observed_payload_orientation",
        attachment_rule="unchanged_original_close_planning_attachment", additional_height_m=0.,
        source_clearance_m=.002, full_original_world_and_self_and_source_checks_required=True,
        baseline_and_variants_share_candidate=True)
    require(all(candidate.get(key) == value for key, value in expected.items()),
        "Actual transport candidate differs from prepared finite selection")
    bases = [row for row in profile_inputs if row["initial_sha256"] == candidate.get("base_profile_sha256")]
    require(len(bases) == 1 and candidate.get("source_object_sha256") == metadata["source_hashes"],
        "Actual transport candidate source/base profile differs")
    base = json.loads(bases[0]["source_text"])
    require(candidate.get("source_layout_sha256") == base.get("source_layout_sha256") and
        candidate.get("source_kitchen_sha256") == base.get("source_kitchen_sha256"),
        "Actual transport candidate layout/kitchen differs")
    return dict(required=True, candidate_id=candidate["candidate_id"], binding=binding)


def audit_provenance(path, metadata, final_physics_step):
    """Pure CPU start/end and archived-text check; no current-code equality claim."""
    report = json.loads(Path(path).read_text())
    require(report["task_id"] == metadata["task_id"] and report["source_layout"] == metadata["source_layout"], "Provenance task/layout differs")
    require(report["checked_physics_step"] == int(final_physics_step), "Provenance does not bind collection end")
    require(report["passed"] is True, "Collection provenance reported a failure")
    entries = report["entries"]
    required_sources = set(required_task_sources(metadata["task_id"]))
    require(required_sources <= set(entries), "Required task execution source snapshots missing")
    for key, item in entries.items():
        require(item["initial_sha256"] == item["checked_sha256"] == item.get("expected_sha256", item["initial_sha256"])
            and item["unchanged"] is True and item["matches_declared_source"] is True, "Source/input changed: "+key)
        if key in required_sources or item["kind"] in ("execution_source", "source_layout_input", "source_task_yaml", "controller_input", "grasp_candidate_input", "transport_candidate_input"):
            require(hashlib.sha256(item["source_text"].encode()).hexdigest() == item["initial_sha256"], "Archived source SHA differs: "+key)
    for key in ("source_sha256", "recipe_sha256", "overlay_sha256"):
        require(report["background_physics"][key] == metadata["background_physics"][key], "Profile identity differs: "+key)
    require(report["background_physics"] == metadata["background_physics"], "Full physical profile identity differs")
    physical = metadata["background_physics"]
    recipe_key = input_key(physical.get("recipe_source_path", BASE_RECIPE))
    if metadata["task_id"] == BANANA_TASK:
        require(recipe_key == BANANA_RECIPE and "base_profile" in physical,
            "Banana task requires its captured leaf and inherited profile")
    require(recipe_key in entries and entries[recipe_key]["kind"] == "execution_source" and
        entries[recipe_key]["initial_sha256"] == physical["recipe_sha256"], "Actual recipe source missing/different")
    if "base_profile" in physical:
        inherited = physical["base_profile"]
        require(inherited["source_sha256"] == physical["source_sha256"] and
            inherited["recipe_sha256"] == physical["base_recipe_sha256"] and
            inherited["overlay_sha256"] == physical["base_overlay_sha256"], "Inherited physical profile identity differs")
        for field, digest_field, kind in [("base_recipe_source_path", "base_recipe_sha256", "execution_source"),
                ("base_overlay_path", "base_overlay_sha256", "inherited_kitchen_overlay")]:
            key = input_key(physical[field])
            require(key in entries and entries[key]["kind"] == kind and entries[key]["initial_sha256"] == physical[digest_field],
                "Inherited physical input missing/different: "+key)
    # A green start/end result for the remaining entries cannot excuse omitted
    # source assets, original YAML, layout or the actual controller input.
    source = metadata["source_relations"]
    required_inputs = dict(source["assets_sha256"])
    required_inputs[source["source_yaml"]] = source["source_yaml_sha256"]
    require(required_inputs, "Required original source input inventory is empty")
    accepted_aliases = []
    for input_path, digest in required_inputs.items():
        key = input_key(input_path)
        if key in entries:
            require(entries[key]["initial_sha256"] == digest, "Required original input missing/different: "+key)
        else:
            require(known_old_bowl_stand_alias(entries, key, digest, metadata["task_id"]), "Required original input missing/different: "+key)
            accepted_aliases.append(dict(required_path=key, recorded_path=STAND_RUNTIME_PATH, sha256=digest,
                scope="Known repo runtime/home to container /home/user bind-mount alias; unique composed_robot_on_stand role"))
    layout_inputs = [row for row in entries.values() if row["kind"] == "source_layout_input"]
    require(layout_inputs and all(json.loads(row["source_text"]) == metadata["source_layout"] for row in layout_inputs),
        "Actual source layout input omitted or mismatched")
    profile_inputs = [row for row in entries.values() if row["kind"] == "controller_input"]
    matching_profiles = []
    for row in profile_inputs:
        try:
            profile = json.loads(row["source_text"])
        except (ValueError, TypeError):
            continue
        if profile.get("task_id") == metadata["task_id"]:
            matching_profiles.append(profile)
    require(len(matching_profiles) == 1, "Actual task controller profile input omitted/ambiguous")
    profile = matching_profiles[0]
    require(profile.get("source_layout_sha256") in {row["initial_sha256"] for row in layout_inputs} and
        profile.get("source_kitchen_sha256") == metadata["background_physics"]["source_sha256"],
        "Controller profile references different layout/kitchen inputs")
    if "source_object_sha256" in profile:
        require(profile["source_object_sha256"] == metadata["source_hashes"], "Controller profile object inventory differs")
    else:
        require(metadata["task_object_names"] == ["bowl"] and profile.get("source_bowl_sha256") == metadata["source_hashes"]["bowl"],
            "Controller profile original payload hash missing/different")
    # Only the new banana collector declares this extra input. Historical
    # banana/bowl runs retain their actual archived dependency inventories.
    candidate_audit = audit_grasp_candidate_binding(report, metadata, entries, profile_inputs)
    transport_audit = audit_transport_candidate_binding(report, metadata, entries, profile_inputs)
    require(any(row["kind"] == "composed_kitchen_overlay" and row["initial_sha256"] == metadata["background_physics"]["overlay_sha256"] for row in entries.values()), "Actual overlay input omitted")
    return dict(passed=True, execution_sources=len(required_sources), archived_entries=len(entries), sha256=sha(path),
                accepted_input_aliases=accepted_aliases, grasp_candidate=candidate_audit,
                transport_candidate=transport_audit)


def audit_predicate_evidence(directory, metadata, data, trajectory, result):
    """CPU identity, raw-state and original-predicate checks for the new schema."""
    from scipy.spatial.transform import Rotation
    from scripts.extensions.replicator.audit_task_source_relation import source
    from scripts.extensions.replicator.review_episode import source_thresholds
    from scripts.extensions.replicator.review_task_episode import raw_destination
    directory = Path(directory)
    info = json.loads((directory/"task_predicate_metadata.json").read_text())
    require(info.get("schema_version") == 1, "Unsupported runtime predicate schema")
    _, original = source(metadata["task_id"])
    params = original["task"]["subtasks"][0]["params"]
    pick, destination = params["pick_up_object"], params["destination_location"]
    require(info["task_id"] == metadata["task_id"] and info["pick_up_object"] == pick and
        info["destination_location"] == destination, "Original predicate task roles differ")
    defaults = source_thresholds()
    expected = {key: params.get(key, getattr(defaults, key)) for key in
        ("force_threshold", "velocity_threshold", "support_cone_half_angle_rad")}
    expected["placement_consecutive_steps"] = params.get("placement_consecutive_steps", 1)
    require(info["thresholds"] == expected, "Original success thresholds changed")
    sensor = info["sensor_identity"]
    root = "/World/envs/env_0/"
    references = {row["id"]: root+"kitchen/"+row["prim_path"] for row in original["object_references"]}
    destination_path = root+destination if destination in metadata["task_object_names"] else references[destination]
    require(sensor == info["sensor_identity_at_end"] and sensor["contact_sensor_name"] == "contact_sensor_"+pick
        and sensor["sensed_native_paths"] == [root+pick] and sensor["resolved_filter_prim_paths"] == [destination_path]
        and sensor["actual_pick_up_path"] == root+pick and sensor["actual_destination_path"] == destination_path
        and sensor["native_filter_count"] == 1 and sensor["force_matrix_shape"] == [1, 1, 1, 3],
        "Actual destination sensor identity differs from original task")
    require(info["physics_dt_s"] == metadata["physics_dt_s"] and info["action_dt_s"] == metadata["action_dt_s"]
        and info["reset_physics_step"] == metadata["reset_physics_step"], "Predicate recording time base differs")
    saved = {}
    for kind in ("physics", "actions"):
        name = "task_predicate_"+kind+".npz"
        require(sha(directory/name) == info["input_sha256"][name], "Predicate NPZ hash differs")
        with np.load(directory/name, allow_pickle=False) as values:
            saved[kind] = {key: values[key] for key in values.files}
    bounds = {}
    require(set(info["actual_spawned_local_bounds"]) == {pick, destination}, "Actual spawned bounds inventory differs")
    for name, box in info["actual_spawned_local_bounds"].items():
        value = np.asarray([box["min"], box["max"]])
        require(value.shape == (2, 3) and np.isfinite(value).all() and np.all(value[1] > value[0]), "Invalid actual spawned bounds")
        bounds[name] = value
    ids = metadata["task_object_names"].index(pick)
    object_states = data["task_object_states"][:, ids]
    if destination in metadata["task_object_names"]:
        destination_poses = data["task_object_states"][:, metadata["task_object_names"].index(destination), :7]
    else:
        matrices = data["task_reference_transforms"][:, metadata["task_reference_paths"].index(destination_path)]
        destination_poses = np.c_[matrices[:, :3, 3], Rotation.from_matrix(matrices[:, :3, :3]).as_quat()]
    evaluations = {}
    for kind, rows in saved.items():
        expected_steps = data["physics_step"] if kind == "physics" else trajectory["after_physics_step"]
        require(np.array_equal(rows["physics_step"], expected_steps), "Missing complete predicate "+kind+" timeline")
        indexes = np.searchsorted(data["physics_step"], expected_steps)
        for key, value in rows.items():
            require(len(value) == len(indexes) and (value.dtype.kind in "US" or np.isfinite(value).all()), "Malformed predicate field "+key)
        require(np.array_equal(rows["phase"], data["phase"][indexes]) and
            np.array_equal(rows["action_index"], data["action_index"][indexes]), "Predicate phase/action mismatch")
        for key, actual in [("object_pose", object_states[indexes, :7]), ("destination_pose", destination_poses[indexes]),
                ("object_linear_velocity", object_states[indexes, 7:10]),
                ("destination_contact_force", data["destination_contact_force"][indexes, 0, 0])]:
            require(rows[key].shape == actual.shape and np.max(np.abs(rows[key]-actual)) <= 1e-6,
                    "Predicate differs from actual physics field: "+key)
        centers = object_states[indexes, :3]+Rotation.from_quat(object_states[indexes, 3:7]).apply(bounds[pick].mean(axis=0))
        require(np.max(np.abs(rows["object_centroid_world"]-centers)) <= 1e-6, "Spawned centroid does not match measured pose/local bounds")
        values = [raw_destination(object_states[i], data["destination_contact_force"][i, 0, 0], bounds[pick].mean(axis=0),
            destination_poses[i], bounds[destination], expected["force_threshold"], expected["velocity_threshold"],
            expected["support_cone_half_angle_rad"]) for i in indexes]
        require(np.array_equal(rows["original_object_on_destination"], [row["passed"] for row in values]),
                "Original recorded final predicate differs from independent raw support/speed/spatial evaluation")
        evaluations[kind] = values
    actions = saved["actions"]
    resting = np.flatnonzero(actions["has_original_rest_pose"])
    require(len(resting), "Original predicate never recorded a settled pose")
    first = int(resting[0])
    require(actions["has_original_rest_pose"][first:].all(), "Original rest state was cleared during episode")
    rest = actions["original_rest_position"][first]
    require(np.max(np.abs(actions["original_rest_position"][first:]-rest)) <= 1e-7, "Original rest reference changed")
    raw_index = np.searchsorted(data["physics_step"], actions["physics_step"][first])
    require(np.linalg.norm(object_states[raw_index, 7:10]) < .01 and np.linalg.norm(object_states[raw_index, 10:13]) < .05
        and np.max(np.abs(rest-object_states[raw_index, :3])) <= 1e-6, "Original first resting pose lacks its actual low-speed state")
    above = actions["has_original_rest_pose"] & (actions["object_pose"][:, 2] > actions["original_rest_position"][:, 2]+.01)
    require(np.array_equal(above, actions["original_above_rest"]), "Original strict resting-height predicate differs")
    rise = np.flatnonzero(above)
    success = np.flatnonzero(actions["terminal_success"])
    require(len(rise) and len(success) and int(rise[0]) >= first and success[0] >= rise[0], "Original settle/lift/success temporal order absent")
    required = expected["placement_consecutive_steps"]
    end = int(success[0])
    require(end-required+1 >= rise[0] and actions["original_object_on_destination"][end-required+1:end+1].all(),
        "First success lacks original required consecutive destination samples")
    terminal = result["terminal"]["first_success"]
    require(terminal["physics_step"] == int(actions["physics_step"][end]) and terminal["terms"]["success"] is True,
            "First runtime termination does not match recorded predicate action")
    require(not actions["terminal_failure"].any() and result.get("original_task_success") is True and
        result.get("final_original_destination_predicate") is True and evaluations["physics"][-1]["passed"],
        "Original task did not finish with actual destination support/stability")
    return dict(passed=True, physics_samples=len(saved["physics"]["physics_step"]), actions=len(actions["physics_step"]),
        first_rest_physics_step=int(actions["physics_step"][first]), first_rise_physics_step=int(actions["physics_step"][rise[0]]),
        first_success_physics_step=int(actions["physics_step"][end]), final=evaluations["physics"][-1],
        input_sha256={name: sha(directory/name) for name in ("task_predicate_metadata.json", "task_predicate_physics.npz", "task_predicate_actions.npz")})
