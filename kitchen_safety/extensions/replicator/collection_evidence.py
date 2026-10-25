"""Read-only run provenance and absolute post-startup physics-step alignment.

No simulator imports, stepping, state writes, tolerance changes, or inferred
samples. Historical physics_states.npz stays unchanged.
"""
import hashlib
import json
from pathlib import Path

import numpy as np


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def audit_motion_alignment(prefix_steps, index, motion, *, world_ready_step, final_step):
    """Require the measured prefix and main streams to form one exact timeline."""
    prefix = np.asarray(prefix_steps)
    steps = np.asarray(index["physics_step"])
    phases, actions = np.asarray(index["phase"]), np.asarray(index["action_index"])
    n = len(steps)
    shape_ok = (steps.shape == phases.shape == actions.shape == (n,)
                and np.issubdtype(steps.dtype, np.integer)
                and np.issubdtype(actions.dtype, np.integer))
    prefix_ok = (prefix.ndim == 1 and len(prefix) > 0
                 and np.issubdtype(prefix.dtype, np.integer)
                 and np.all(np.diff(prefix) == 1))
    checks = dict(
        world_ready_reached=world_ready_step is not None,
        prefix_contiguous=bool(prefix_ok),
        prefix_ends_at_world_ready=bool(prefix_ok and world_ready_step is not None and prefix[-1] == world_ready_step),
        nonempty_main_physics_stream=bool(n),
        main_index_shape=bool(shape_ok),
        main_steps_contiguous=bool(shape_ok and np.all(np.diff(steps) == 1)),
        main_begins_after_prefix=bool(shape_ok and n and world_ready_step is not None and steps[0] == world_ready_step + 1),
        main_ends_at_collection_end=bool(shape_ok and n and steps[-1] == final_step),
        main_states_present=motion is not None,
        main_rows_match_index=False,
        main_phase_and_action_match=False,
        main_physics_check_matches=False,
    )
    if motion is not None:
        checks["main_rows_match_index"] = bool(shape_ok and all(len(motion[key]) == n for key in
                                                             ("q", "mustard", "phase", "action_index", "physics_check")))
        checks["main_phase_and_action_match"] = bool(np.array_equal(phases, motion["phase"])
                                                        and np.array_equal(actions, motion["action_index"]))
        checks["main_physics_check_matches"] = bool(np.array_equal(motion["physics_check"], np.arange(1, n + 1)))
    return dict(schema_version=1, passed=all(checks.values()), checks=checks,
                coverage=dict(prefix_samples=len(prefix), main_samples=n,
                              prefix_first_physics_step=int(prefix[0]) if len(prefix) else None,
                              prefix_last_physics_step=int(prefix[-1]) if len(prefix) else None,
                              world_ready_physics_step=world_ready_step,
                              main_first_physics_step=int(steps[0]) if n else None,
                              main_last_physics_step=int(steps[-1]) if n else None,
                              collection_final_physics_step=int(final_step)),
                scope="Absolute physics counters observed after each scene update. Prefix includes post-reset initial state; "
                      "main begins at the next actual substep after collision-world readiness. Main rows bind one-to-one "
                      "to physics_states.npz in its unchanged schema, including exact phase/action/physics_check. "
                      "No video-frame or missing-state inference; a missing main-state NPZ cannot pass this delivery gate.")


class MotionPhysicsIndex:
    def __init__(self):
        self.rows = []

    def append(self, physics_step, phase, action_index):
        self.rows.append((int(physics_step), str(phase), int(action_index)))

    def finish(self, output, startup_metadata, final_step):
        output = Path(output)
        path, report_path = output / "motion_physics_index.npz", output / "motion_physics_alignment.json"
        if path.exists() or report_path.exists():
            raise FileExistsError("Never overwrite motion physics evidence")
        index = dict(physics_step=np.asarray([x[0] for x in self.rows], dtype=np.int64),
                     phase=np.asarray([x[1] for x in self.rows], dtype=str),
                     action_index=np.asarray([x[2] for x in self.rows], dtype=np.int64))
        np.savez_compressed(path, **index)
        inputs = {path.name: sha256(path)}
        try:
            prefix_path = output / "startup_physics_states.npz"
            inputs[prefix_path.name] = sha256(prefix_path)
            with np.load(prefix_path, allow_pickle=False) as prefix:
                motion_path = output / "physics_states.npz"
                if motion_path.exists():
                    inputs[motion_path.name] = sha256(motion_path)
                    with np.load(motion_path, allow_pickle=False) as motion:
                        report = audit_motion_alignment(prefix["physics_step"], index, motion,
                            world_ready_step=startup_metadata.get("world_ready_physics_step"), final_step=final_step)
                else:
                    report = audit_motion_alignment(prefix["physics_step"], index, None,
                            world_ready_step=startup_metadata.get("world_ready_physics_step"), final_step=final_step)
        except Exception as error:
            report = dict(schema_version=1, passed=False, checks={"complete_valid_alignment_evidence": False}, error=repr(error))
        report["input_sha256"] = inputs
        report_path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        return report


# The collector's explicit local execution dependencies, including conditional
# branches. Package binaries/external libraries are versioned in result.json;
# this list does not claim to snapshot their complete source or loaded bytecode.
LOCAL_CODE = (
    "scripts/extensions/replicator/collect.py",
    *["kitchen_safety/extensions/replicator/" + name + ".py" for name in (
        "collection_evidence", "startup_recorder", "deferred_safety", "environment", "layout",
        "local_droid", "background_physics", "collision_planner", "curobo_backend", "overview")],
    *["kitchen_safety/" + name + ".py" for name in (
        "planning", "planning_checks", "curobo_backend", "grasp_geometry", "grasp_contacts",
        "placement_geometry", "placement_planner", "guided_transport", "transport_phases",
        "augmentation_geometry", "obstacle_library", "collection_presets", "dataset", "background_camera",
        "camera_device", "cpu_camera_copy", "step_profiling", "randomization")],
    *["third_party/IsaacLab-Arena/isaaclab_arena/" + name + ".py" for name in (
        "environments/arena_env_builder", "assets/background", "assets/object", "assets/object_base",
        "scene/scene", "embodiments/droid/droid", "embodiments/robot_on_stand_utils",
        "embodiments/gripper", "tasks/pick_and_place_task", "terms/events", "utils/usd/prim_tree")],
)


class CollectionProvenance:
    def __init__(self, root, code_paths=LOCAL_CODE):
        self.root, self.entries = Path(root).resolve(), {}
        for path in code_paths:
            self.capture(path, "execution_source", snapshot=True)

    def capture(self, path, kind, *, snapshot=False, expected_sha256=None):
        path = Path(path)
        path = (self.root / path).resolve() if not path.is_absolute() else path.resolve()
        try:
            key = str(path.relative_to(self.root))
        except ValueError:
            key = str(path)
        initial = sha256(path)
        if key not in self.entries:
            self.entries[key] = dict(path=str(path), kind=kind, initial_sha256=initial)
            if snapshot:
                text = path.read_text()
                if hashlib.sha256(text.encode()).hexdigest() != initial:
                    raise ValueError("Source changed during snapshot or non-UTF8/newline-normalized source: " + key)
                self.entries[key]["source_text"] = text
        if expected_sha256 is not None:
            entry = self.entries[key]
            if entry.get("expected_sha256", expected_sha256) != expected_sha256:
                raise ValueError("Conflicting source digest: " + key)
            entry["expected_sha256"] = expected_sha256
        return key

    def bind_scene(self, layout, background_physics, kitchen_path):
        if not background_physics:
            raise ValueError("Collector requires explicit background source/recipe/overlay provenance")
        info = background_physics
        self.capture(info["source"], "source_kitchen", expected_sha256=info["source_sha256"])
        self.capture("kitchen_safety/extensions/replicator/background_physics.py", "execution_source",
                     expected_sha256=info["recipe_sha256"])
        self.capture(kitchen_path, "composed_kitchen_overlay", expected_sha256=info["overlay_sha256"])
        self.capture("third_party/IsaacLab-Arena/isaaclab_arena_environments/kitchen_bench/" + layout["task_id"] + ".yaml",
                     "source_task_yaml", snapshot=True, expected_sha256=layout["source_yaml_sha256"])
        self.capture("assets/robolab/objects/hope/mustard_bottle.usd", "task_asset_root")
        self.capture("assets/robolab/objects/ycb/bowl.usd", "task_asset_root")
        for path in (
            "assets/nvidia/Assets/Isaac/6.1/Isaac/IsaacLab/Arena/assets/robot_library/droid/franka_robotiq_2f_85_flattened.usd",
            "assets/nvidia/Assets/Isaac/6.1/Isaac/IsaacLab/Arena/assets/object_library/srl_robolab_assets/robots/franka_stand_grey.usda",
        ):
            self.capture(path, "robot_asset_root")
        if layout.get("obstacle"):
            self.capture(layout["obstacle"]["usd_path"], "hazard_asset_root")
        self.background_physics = dict(info)

    def check(self, *, final_physics_step=None):
        entries = {}
        for key, initial in self.entries.items():
            row = dict(initial)
            try:
                row["checked_sha256"] = sha256(row["path"])
                row["unchanged"] = row["initial_sha256"] == row["checked_sha256"]
                row["matches_declared_source"] = row.get("expected_sha256", row["initial_sha256"]) == row["initial_sha256"]
            except OSError as error:
                row.update(checked_sha256=None, unchanged=False, matches_declared_source=False, error=repr(error))
            entries[key] = row
        return dict(schema_version=1, passed=bool(entries) and all(x["unchanged"] and x["matches_declared_source"] for x in entries.values()),
                    entries=entries, checked_physics_step=final_physics_step,
                    background_physics=getattr(self, "background_physics", None),
                    scope="Execution source snapshot before SimulationAppContext; selected input configurations captured before parsing; "
                          "actual source/recipe/overlay bound after scene assembly and before env creation/reset. All entries rehashed at collection end. "
                          "Includes explicit local execution dependencies and conditional branches; external package binaries, transitive USD dependencies "
                          "and process memory/bytecode are not fully snapshotted. Separate asset restoration and package evidence remain required.")

    def finish(self, output, final_physics_step):
        path = Path(output) / "collection_provenance.json"
        if path.exists():
            raise FileExistsError("Never overwrite run source provenance")
        report = self.check(final_physics_step=int(final_physics_step))
        path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        return report


def audit_saved_collection_evidence(episode):
    """Recompute alignment and verify archived source hashes, without a runtime.

Current working-tree source may legitimately have changed after collection;
the archived initial source and recorded final hash are the execution evidence.
"""
    episode = Path(episode)
    inputs = {}
    try:
        for name in ("startup_physics_states.npz", "startup_audit.json", "physics_states.npz",
                     "motion_physics_index.npz", "motion_physics_alignment.json", "collection_provenance.json"):
            inputs[name] = sha256(episode / name)
        startup = json.loads((episode / "startup_audit.json").read_text())
        saved = json.loads((episode / "motion_physics_alignment.json").read_text())
        provenance = json.loads((episode / "collection_provenance.json").read_text())
        with np.load(episode / "startup_physics_states.npz", allow_pickle=False) as prefix, \
             np.load(episode / "motion_physics_index.npz", allow_pickle=False) as index, \
             np.load(episode / "physics_states.npz", allow_pickle=False) as motion:
            alignment = audit_motion_alignment(prefix["physics_step"], index, motion,
                world_ready_step=startup["metadata"]["world_ready_physics_step"],
                final_step=saved["coverage"]["collection_final_physics_step"])
        rows = provenance["entries"]
        mismatches = []
        for key, row in rows.items():
            expected = row.get("expected_sha256", row["initial_sha256"])
            snapshot_ok = "source_text" not in row or hashlib.sha256(row["source_text"].encode()).hexdigest() == row["initial_sha256"]
            if not (row["initial_sha256"] == row["checked_sha256"] == expected and snapshot_ok
                    and row["unchanged"] and row["matches_declared_source"]):
                mismatches.append(key)
        kinds = {row["kind"] for row in rows.values()}
        recipe = "kitchen_safety/extensions/replicator/background_physics.py"
        info = provenance["background_physics"]
        source = [x for x in rows.values() if x["kind"] == "source_kitchen"]
        overlay = [x for x in rows.values() if x["kind"] == "composed_kitchen_overlay"]
        checks = dict(
            alignment_recomputed=alignment["passed"],
            alignment_matches_saved=saved["passed"] == alignment["passed"] and saved["checks"] == alignment["checks"]
                and saved["coverage"] == alignment["coverage"],
            alignment_input_hashes_match=all(saved["input_sha256"].get(name) == inputs[name] for name in
                ("startup_physics_states.npz", "physics_states.npz", "motion_physics_index.npz")),
            startup_npz_hash_matches=startup["input_sha256"]["startup_physics_states.npz"] == inputs["startup_physics_states.npz"],
            source_integrity_saved_and_verified=bool(provenance["passed"] and not mismatches),
            required_execution_source_snapshots=all(key in rows and "source_text" in rows[key] for key in LOCAL_CODE),
            source_recipe_overlay_present={"source_kitchen", "composed_kitchen_overlay", "source_task_yaml", "default_layout"}.issubset(kinds),
            recipe_hash_bound=rows[recipe].get("expected_sha256") == info["recipe_sha256"],
            source_hash_bound=len(source) == 1 and source[0].get("expected_sha256") == info["source_sha256"],
            overlay_hash_bound=len(overlay) == 1 and overlay[0].get("expected_sha256") == info["overlay_sha256"],
            final_physics_step_matches=provenance["checked_physics_step"] == alignment["coverage"]["collection_final_physics_step"],
        )
        report = dict(schema_version=1, passed=all(checks.values()), checks=checks,
                      alignment=alignment, provenance_mismatches=mismatches,
                      scope="CPU reconstruction of timeline and archived execution-source integrity only. "
                            "Does not rerun physical stability, collision safety, task success, video or asset dependency gates.")
    except Exception as error:
        report = dict(schema_version=1, passed=False, checks={"complete_saved_collection_evidence": False}, error=repr(error))
    report["input_sha256"] = inputs
    return report
