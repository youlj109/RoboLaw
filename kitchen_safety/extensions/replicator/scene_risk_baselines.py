"""Hash-bound, CPU-only inventory of recorded successful Replicator baselines.

This module never turns a successful diagnostic or neutral hold into a baseline.
Admission requires an existing independent acceptance report, exact raw-data
hashes, complete observed motion phases and original-source provenance. It does
not rerun physics, and acceptance here is not certification of a new hazard.
"""
from __future__ import annotations

import hashlib
import json
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

PREFIX = "kitchen_bench_replicator_"
INVENTORY = "reports/replicator_safety_upgrade/baselines.json"
ASSIGNMENT = "student_skills/robolaw-replicator/references/assignment.json"
REPORTS = "reports/replicator_execution_20261023"

# Explicit independently reviewed episodes, never newest-file / success-flag selection.
CANDIDATES = [
    dict(task_id=PREFIX+"g_shape_mustard_bowl", profile=profile,
         episode_path=f"datasets/avoidance_trials/replicator/base/{episode}",
         verification_path=f"{REPORTS}/g_mustard/delivery_manifest_v2_final01.json",
         actual_review_path=f"{REPORTS}/g_mustard/{review}_actual_review_complete.json",
         config_path="configs/extensions/replicator/g_shape_mustard.json",
         format="mustard_split_physics")
    for profile, episode, review in [("base", "a20261023_v2_base02", "v2_base02"),
                                     ("human", "a20261023_v2_humanbase01", "v2_humanbase01")]
] + [
    dict(task_id=PREFIX+"g_shape_bowl_sink", profile=profile,
         episode_path=f"datasets/replicator/g_shape_bowl_sink/base/{episode}",
         verification_path=f"{REPORTS}/g_bowl_sink/{review}_root_baseline_acceptance01.json",
         config_path="configs/extensions/replicator/g_shape_bowl_sink_source_layout.json",
         format="task_full_physics")
    for profile, episode, review in [("base", "attempt_04", "attempt04"),
                                     ("human", "human_matched_01", "human_matched01")]
]


class BaselineUnavailable(ValueError):
    """No complete, currently hash-verifiable successful baseline is available."""


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _relative(path):
    value = str(path)
    return value[len("/workspace/"):] if value.startswith("/workspace/") else value


def _read(root, path):
    return json.loads((Path(root)/_relative(path)).read_text())


def _json_hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def _npz(path):
    with np.load(path, allow_pickle=False) as data:
        return {key: data[key] for key in data.files}


def _required(checks, names):
    return all(checks.get(name) is True or
               isinstance(checks.get(name), dict) and checks[name].get("passed") is True
               for name in names)


def _raw_stream(root, spec, result):
    """Return unmodified observations in world xyz + xyzw convention.

    The installed Isaac version and these recorders use xyzw, including
    root_state_w. Do NOT apply the wxyz conversion used by older Isaac releases.
    """
    episode = Path(root)/spec["episode_path"]
    data = _npz(episode/"physics_states.npz")
    if spec["format"] == "mustard_split_physics":
        prefix = _npz(episode/"startup_physics_states.npz")
        index = _npz(episode/"motion_physics_index.npz")
        if str(data["pose_convention"]) != "world xyz + xyzw":
            raise ValueError("Unsupported mustard pose convention")
        if not np.array_equal(data["phase"], index["phase"]) or not np.array_equal(data["action_index"], index["action_index"]):
            raise ValueError("Main stream and absolute physics-step index disagree")
        if not np.array_equal(data["physics_check"], np.arange(1, len(data["q"])+1)):
            raise ValueError("Main physics check indices are not contiguous")
        if not np.array_equal(prefix["joint_names"], data["joint_names"]):
            raise ValueError("Prefix and motion joint identities disagree")
        merged = dict(q=np.concatenate([prefix["q"], data["q"]]),
                      payload_pose_xyzw=np.concatenate([prefix["mustard"][:, :7], data["mustard"]]),
                      phase=np.concatenate([prefix["phase"], data["phase"]]),
                      physics_step=np.concatenate([prefix["physics_step"], index["physics_step"]]),
                      action_index=np.concatenate([prefix["action_index"], data["action_index"]]),
                      attached=np.r_[np.zeros(len(prefix["q"]), dtype=bool), data["attached"]],
                      joint_names=data["joint_names"].tolist(), arm_ids=data["arm_ids"],
                      physics_dt_s=float(result["execution_configuration"]["physics_dt"]),
                      payload_name="mustard_bottle", payload_asset_path="assets/robolab/objects/hope/mustard_bottle.usd")
    else:
        meta = _read(root, f'{spec["episode_path"]}/physics_metadata.json')
        payload = "bowl" if spec["task_id"].endswith("bowl_sink") else ("banana" if "banana" in spec["task_id"] else "mustard_bottle")
        object_index = meta["task_object_names"].index(payload)
        names = meta["joint_names"]
        merged = dict(q=data["q"], payload_pose_xyzw=data["task_object_states"][:, object_index, :7],
                      phase=data["phase"], physics_step=data["physics_step"], action_index=data["action_index"],
                      attached=data["has_planning_attachment"], joint_names=names,
                      arm_ids=np.asarray([names.index(f"panda_joint{i}") for i in range(1, 8)]),
                      physics_dt_s=float(meta["physics_dt_s"]), payload_name=payload,
                      payload_asset_path=result["layout"]["objects"][payload]["usd_path"],
                      robot_body_pose_xyzw=data["robot_body_pose_w"], robot_body_names=meta["robot_body_names"],
                      tcp_pose_xyzw=data["tcp_pose_w"])
    merged["gripper_names"] = [name for i, name in enumerate(merged["joint_names"]) if i not in merged["arm_ids"]]
    merged["time_s"] = merged["physics_step"] * merged["physics_dt_s"]
    return merged


def inspect_stream(stream):
    """Recompute observed completeness; a diagnostic success flag cannot pass."""
    q, payload, phases, steps = [stream[k] for k in ("q", "payload_pose_xyzw", "phase", "physics_step")]
    n = len(steps)
    phase_set = set(phases.tolist())
    retreat = np.flatnonzero(phases == "retreat")
    release = np.flatnonzero(phases == "release")
    lift = np.flatnonzero(phases == "lift")
    end_settle = np.flatnonzero(np.isin(phases, ["settle", "final_settle"]))
    quat_norm = np.linalg.norm(payload[:, 3:7], axis=1) if payload.shape == (n, 7) else np.array([np.nan])
    checks = dict(
        complete_reset_physics=bool(n > 2 and np.array_equal(steps, np.arange(n))),
        full_actual_arm_and_gripper=bool(q.shape == (n, 13) and len(stream["joint_names"]) == 13 and len(stream["arm_ids"]) == 7),
        finite_actual_states=bool(np.isfinite(q).all() and np.isfinite(payload).all()),
        payload_orientation_valid=bool(np.isfinite(quat_norm).all() and np.max(np.abs(quat_norm-1)) < 1e-3),
        complete_operating_phases=bool({"pregrasp", "approach", "close", "lift", "lower", "release", "retreat"} <= phase_set),
        transfer_phase_present=bool({"above_bowl", "transport"} & phase_set),
        ordered_release_retreat_settle=bool(len(release) and len(retreat) and len(end_settle) and release[-1] < retreat[0] and retreat[-1] < end_settle[-1]),
        terminal_settle_duration=bool(len(retreat) and (n-1-retreat[-1])*stream["physics_dt_s"] >= 1.0),
        final_gripper_open=bool(abs(q[-1, stream["joint_names"].index("finger_joint")]) < .01),
        actual_payload_lift=bool(len(lift) and np.max(payload[lift, 2])-payload[0, 2] > .01),
    )
    return dict(passed=all(checks.values()), checks=checks, samples=n,
                duration_s=float((steps[-1]-steps[0])*stream["physics_dt_s"]),
                phase_counts=dict(Counter(phases.tolist())),
                first_physics_step=int(steps[0]), last_physics_step=int(steps[-1]))


def verify_candidate(root, spec, task):
    """Recheck acceptance bindings and observed full-episode data, CPU only."""
    root = Path(root)
    out = dict(spec, status="blocked_baseline", checks={}, errors=[], files={})
    files = out["files"]
    def bind(path, expected=None):
        key = _relative(path)
        if Path(key).is_absolute():
            raise ValueError(f"External path cannot be a local baseline binding: {key}")
        actual = sha256(root/key)
        if expected is not None and actual != expected:
            raise ValueError(f"Hash mismatch: {key}")
        files[key] = actual
        return actual
    try:
        acceptance = _read(root, spec["verification_path"])
        result_path = spec["episode_path"]+"/result.json"
        result = _read(root, result_path)
        if result.get("task_id") != task["id"] or result.get("variant") != "base":
            raise ValueError("Candidate is not the requested original-task no-hazard baseline")
        if result.get("diagnostic_only") or result.get("neutral_sequence_complete"):
            raise ValueError("Diagnostic/neutral records cannot be admitted as a successful baseline")
        if spec["format"] == "mustard_split_physics":
            gates = [f'baseline.{spec["profile"]}.{name}' for name in ("physical_and_rgb", "independent_actual", "sampled_visual", "complete_startup_physics", "absolute_motion_and_provenance")]
            if acceptance.get("passed") is not True or not _required(acceptance.get("checks", {}), gates):
                raise ValueError("Required baseline delivery gates missing")
            expected = {_relative(path): item["sha256"] for path, item in acceptance["artifacts"].items()}
            paths = [p for p in expected if p.startswith(spec["episode_path"]+"/") and not p.endswith((".mp4", ".h5", ".png", ".jpg"))]
            paths += [spec["actual_review_path"]]
            raw_review = _read(root, spec["actual_review_path"])
            if raw_review.get("passed") is not True or not all(v is True for v in raw_review["checks"].values()):
                raise ValueError("Independent actual-state review failed")
            if not all(result.get(k) is True for k in ("success", "arena_success", "final_on_destination")) or result.get("physical_failure_reasons"):
                raise ValueError("Original task result is not successful")
            provenance_name = "collection_provenance.json"
            out["controls"] = result["effective_controls"]
            out["execution_configuration"] = result["execution_configuration"]
            out["recorded_source_layout_sha256"] = result["source_task"]["layout_sha256"]
            out["actual_review_path"] = spec["actual_review_path"]
        else:
            required = ("complete_finite_reset_physics", "all_action_and_physics_identities", "matching_complete_neutral", "physical_pick_lift_release_and_original_success", "collector_completed_physical_trial", "raw_original_final_support_speed_geometry", "actual_spawned_predicate_geometry_and_sensor_identity", "complete_execution_provenance", "independent_visual_review")
            if acceptance.get("baseline_verified") is not True or acceptance.get("episode") != spec["episode_path"] or not _required(acceptance.get("checks", {}), required):
                raise ValueError("Required independent baseline acceptance gates missing")
            expected = {_relative(path): item["expected"] for path, item in acceptance["verified_input_hashes"].items()}
            paths = [p for p in expected if not p.endswith((".mp4", ".jpg", ".png"))]
            if not all(result.get(k) is True for k in ("physical_trial_passed", "original_task_success", "physical_sequence_complete", "final_original_destination_predicate")) or result.get("status") != "sequence_complete":
                raise ValueError("Original complete physical task result failed")
            if result.get("physics_state_setters_used") is not False or result.get("physical_weld_used") is not False:
                raise ValueError("Unverified physical state manipulation")
            provenance_name = "task_collection_provenance.json"
            out["controls"] = result["controller_profile"]
            out["controller_profile_sha256"] = result["controller_profile_sha256"]
            out["execution_configuration"] = result.get("runtime_threads", {})
            out["recorded_source_layout_sha256"] = result["controller_profile"]["source_layout_sha256"]
        # Raw physics and result must be bound by the independent report itself.
        physics_path = spec["episode_path"]+"/physics_states.npz"
        for required_path in (physics_path, result_path, spec["episode_path"]+"/"+provenance_name):
            if required_path not in expected:
                raise ValueError(f"Acceptance does not bind required input: {required_path}")
        bind(spec["verification_path"])
        for path in paths:
            bind(path, expected[path])
        bind(task["source_yaml"], task["source_sha256"])
        provenance = _read(root, spec["episode_path"]+"/"+provenance_name)
        if provenance.get("passed") is not True:
            raise ValueError("Collection provenance failed")
        kinds = {item.get("kind") for item in provenance["entries"].values()}
        if not {"source_task_yaml", "source_kitchen", "task_asset_root", "robot_asset_root", "execution_source"} <= kinds:
            raise ValueError("Provenance is missing required original source identities")
        archived_sources = {}
        external_recorded_assets = []
        for path, item in provenance["entries"].items():
            initial = item["initial_sha256"]
            if item.get("checked_sha256") != initial or item.get("unchanged") is not True:
                raise ValueError(f"Execution inputs changed during recording: {path}")
            if "source_text" in item:
                if hashlib.sha256(item["source_text"].encode()).hexdigest() != initial:
                    raise ValueError(f"Archived source text does not match recording: {path}")
                archived_sources[path] = initial
            # Current execution code can evolve; recorded text remains hash-bound.
            # Source geometry/layout/controller inputs cannot silently evolve.
            if item["kind"] not in ("execution_source",) and not Path(_relative(path)).is_absolute():
                bind(path, initial)
            elif Path(_relative(path)).is_absolute():
                external_recorded_assets.append(dict(path=path, sha256=initial, currently_rechecked=False))
        bind(spec["config_path"])
        layout = result["layout"]
        bind(layout["kitchen_usd"])
        stream = _raw_stream(root, spec, result)
        audit = inspect_stream(stream)
        if not audit["passed"]:
            raise ValueError("Actual observed completeness failed: "+", ".join(k for k,v in audit["checks"].items() if not v))
        bind(stream["payload_asset_path"])
        out.update(status="verified_baseline", source_yaml=task["source_yaml"], source_yaml_sha256=task["source_sha256"],
                   source_layout=layout, layout=layout, layout_content_sha256=_json_hash(layout),
                   physics_path=physics_path, result_path=result_path, seed=int(result["seed"]),
                   payload_name=stream["payload_name"], payload_asset_path=stream["payload_asset_path"],
                   physics_dt_s=stream["physics_dt_s"], observation_audit=audit,
                   archived_execution_sources=archived_sources, external_recorded_assets=external_recorded_assets,
                   checks=dict(independent_acceptance_present=True, exact_inputs_hash_bound=True,
                               original_source_yaml_matches=True, recorded_provenance_verified=True,
                               complete_observed_success_sequence=True),
                   limits=["Single recorded no-hazard baseline, not proof of all-path risk or safe replanning.",
                           "Original independent physics/visual acceptance is reused after input-hash and raw-completeness checks; physics is not rerun.",
                           "Archived execution source may differ from current code; external container cache assets are recorded provenance, not locally rehashed."])
    except (OSError, ValueError, KeyError, TypeError, IndexError) as exc:
        out["errors"].append(str(exc))
    return out


def discover_records(root):
    """List observed records; discovery alone never grants baseline admission."""
    root = Path(root)
    found = []
    for base in ("datasets/replicator", "datasets/avoidance_trials/replicator", REPORTS):
        for path in sorted((root/base).rglob("result.json")):
            try:
                result = json.loads(path.read_text())
            except (OSError, ValueError):
                continue
            task_id = result.get("task_id", "")
            if not task_id.startswith(PREFIX):
                continue
            state = "unreviewed_record"
            if result.get("neutral_sequence_complete") or "neutral" in result.get("status", ""):
                state = "neutral_only"
            elif result.get("diagnostic_only") or "diagnostic" in result.get("status", "") or "probe" in str(path):
                state = "diagnostic_only"
            elif result.get("variant") != "base":
                state = "hazard_variant_not_baseline"
            found.append(dict(task_id=task_id, episode_path=path.parent.relative_to(root).as_posix(),
                              record_type=state, original_task_success=result.get("original_task_success", result.get("arena_success")),
                              status=result.get("status"), eligible_without_independent_acceptance=False))
    return found


def build_inventory(root, candidate_specs=None):
    root = Path(root).resolve()
    assignment = _read(root, ASSIGNMENT)
    records = discover_records(root)
    entries = []
    for task in assignment["tasks"]:
        baselines = [verify_candidate(root, spec, task) for spec in (CANDIDATES if candidate_specs is None else candidate_specs) if spec["task_id"] == task["id"]]
        verified = [b for b in baselines if b["status"] == "verified_baseline"]
        selected = next((b for b in verified if b["profile"] == "base"), verified[0] if verified else None)
        entries.append(dict(task_id=task["id"], status="verified_baseline" if selected else "blocked_baseline",
                            source_yaml=task["source_yaml"], source_yaml_sha256=task["source_sha256"],
                            selected_baseline=selected["profile"] if selected else None, baselines=baselines,
                            records=[r for r in records if r["task_id"] == task["id"]],
                            reason=None if selected else "No complete independently accepted, currently hash-bound successful no-hazard episode."))
    return dict(schema_version=1, generated_utc=datetime.now(timezone.utc).isoformat(),
                assignment_sha256=sha256(root/ASSIGNMENT), selection_rule="Explicit independently accepted episodes; default base profile. No shortest-path cherry-picking.",
                summary=dict(total_tasks=len(entries), verified_baseline_tasks=sum(e["status"]=="verified_baseline" for e in entries),
                             blocked_baseline_tasks=sum(e["status"]=="blocked_baseline" for e in entries),
                             verified_baseline_profiles=sum(b["status"]=="verified_baseline" for e in entries for b in e["baselines"])),
                tasks=entries)


def load_baseline(root, task_id, profile="base", inventory_path=None):
    """Load one hash-verified actual stream. Raises on missing/stale evidence.

    Returned q includes all 13 actual joints. payload_pose_xyzw and optional
    robot_body_pose_xyzw are measured poses, not planned attachment transforms.
    physics_step/time_s cover the entire reset-to-final-settle stream.
    """
    root = Path(root).resolve()
    if not task_id.startswith(PREFIX):
        task_id = PREFIX+task_id
    inventory = _read(root, inventory_path or INVENTORY)
    if sha256(root/ASSIGNMENT) != inventory["assignment_sha256"]:
        raise BaselineUnavailable("Stale baseline assignment binding")
    task = next((t for t in inventory["tasks"] if t["task_id"] == task_id), None)
    candidate = next((b for b in (task or {}).get("baselines", []) if b["profile"] == profile and b["status"] == "verified_baseline"), None)
    if candidate is None:
        raise BaselineUnavailable(f"blocked_baseline: {task_id}/{profile}")
    for path, expected in candidate["files"].items():
        if sha256(root/path) != expected:
            raise BaselineUnavailable(f"Stale baseline binding: {path}")
    result = _read(root, candidate["result_path"])
    stream = _raw_stream(root, candidate, result)
    audit = inspect_stream(stream)
    if not audit["passed"]:
        raise BaselineUnavailable("Recorded observation completeness changed")
    stream.update(task_id=task_id, profile=profile, layout=candidate["layout"], seed=candidate["seed"],
                  controls=candidate["controls"], binding=candidate, observation_audit=audit)
    return stream
