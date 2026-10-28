"""Exact two-layout provenance audit for the additive bowl hazard collector.

The original generic failure is retained. A separately saved projection maps
the already captured canonical layout to its original-task role and the full
hazard layout to its additive role. No raw report, bytes, hash, runtime verdict,
or non-layout check is changed. This grants no physical-task acceptance.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

from .bowl_paired_query import TASK, require, sha, write


def projection(raw, metadata, result):
    require(raw["task_id"] == metadata["task_id"] == result["task_id"] == TASK, "Exact original bowl task required")
    full = metadata["source_layout"]
    require(raw["source_layout"] == result["layout"] == full and "obstacle" in full,
            "Raw actual additive layout identity differs")
    canonical = {k: v for k, v in full.items() if k != "obstacle"}
    policy = result["safety_policy"]
    require(policy["task_id"] == TASK and policy["variant"] == result["variant"] == full["obstacle"]["category"],
            "Actual additive policy/task/variant differs")
    entries = raw["entries"]
    layout_keys = [k for k, v in entries.items() if v["kind"] == "source_layout_input"]
    require(len(layout_keys) == 1, "Exactly one actual augmented layout input required")
    actual_key = layout_keys[0]
    original_key = "configs/extensions/replicator/g_shape_bowl_sink_source_layout.json"
    require(original_key in entries and actual_key != original_key, "Captured exact canonical source-layout input missing")
    actual, original = entries[actual_key], entries[original_key]
    require(original["kind"] == "hazard_policy_source_input", "Unrecognized canonical layout capture role")
    for key, row, expected in [(actual_key, actual, full), (original_key, original, canonical)]:
        require(json.loads(row["source_text"]) == expected and
            hashlib.sha256(row["source_text"].encode()).hexdigest() == row["initial_sha256"] == row["checked_sha256"] ==
            policy["asset_inputs"][key] and row["unchanged"] is True and row["matches_declared_source"] is True,
            "Exact layout bytes/hash or source-only-plus-obstacle relation differs")
    require(result["controller_profile"]["source_layout_sha256"] == policy["source_layout_sha256"] == original["initial_sha256"],
            "Controller must bind the captured unchanged canonical layout")
    for key, expected in policy["asset_inputs"].items():
        require(key in entries and entries[key]["initial_sha256"] == expected,
                "Additive hazard policy/asset input omitted or changed: " + key)
    projected = deepcopy(raw)
    projected["source_layout"] = canonical
    projected["entries"][actual_key]["kind"] = "independently_verified_additive_hazard_layout"
    projected["entries"][original_key]["kind"] = "source_layout_input"
    projected["derived_review_projection"] = dict(raw_report_modified=False,
        exact_changes=["source_layout interpreted as already captured canonical original-task layout",
            "actual full layout role independently checked as canonical plus exactly one obstacle",
            "captured canonical entry role mapped from hazard_policy_source_input to source_layout_input"],
        accepted=False)
    meta = deepcopy(metadata)
    meta["source_layout"] = canonical
    return projected, meta, dict(actual_augmented_layout_key=actual_key, canonical_layout_key=original_key,
        actual_layout_sha256=actual["initial_sha256"], canonical_layout_sha256=original["initial_sha256"],
        only_additive_field="obstacle", canonical_original_task_verified=True,
        additive_policy_assets_verified=len(policy["asset_inputs"]))


def audit(path, metadata, result, final_physics_step, output):
    from .task_collection_evidence import audit_provenance
    path, output = Path(path), Path(output)
    try:
        generic = dict(passed=True, report=audit_provenance(path, metadata, final_physics_step))
    except Exception as error:
        generic = dict(passed=False, error=repr(error))
    raw = json.loads(path.read_text())
    projected, meta, exact = projection(raw, metadata, result)
    require(not output.exists(), "Never overwrite a provenance projection")
    write(output, projected)
    projected_review = audit_provenance(output, meta, final_physics_step)
    return dict(passed=projected_review["passed"], raw_generic_review=generic,
        raw_provenance_sha256=sha(path), exact_additive_layout_mapping=exact,
        projected_report=str(output), projected_report_sha256=sha(output), projected_original_task_review=projected_review,
        raw_modified=False, accepted=False,
        scope="Independent explicit source-layout/additive-layout role mapping only; every other original provenance check retained. Not physical success or raw generic acceptance.")
