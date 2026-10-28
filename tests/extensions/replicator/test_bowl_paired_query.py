"""CPU contract tests: real recorded schema and fail-closed paired comparisons."""
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest

from kitchen_safety.extensions.replicator.bowl_paired_query import (
    ROOT, SEEDS, TASK, controls, cuda_zero, densify, pair_gates, require_hazard_reference,
    successful_record, transport_index, validate_seeds,
)


@pytest.fixture
def records():
    path = ROOT / "datasets/replicator/g_shape_bowl_sink/base/human_matched_01"
    return json.loads((path / "result.json").read_text()), json.loads((path / "physics_metadata.json").read_text())


def test_actual_matched_schema(records):
    result, meta = records
    successful_record(result, meta, "base")
    assert controls(result, meta, result["controller_profile"], "human")["padding_m"] == .015


def test_old_baseline_padding_is_evidence_not_fallback():
    path = ROOT / "datasets/replicator/g_shape_bowl_sink/base/attempt_04"
    result = json.loads((path / "result.json").read_text())
    meta = json.loads((path / "physics_metadata.json").read_text())
    assert "transport_object_padding_m" not in result
    assert controls(result, meta, result["controller_profile"], "robot")["padding_m"] == 0
    del next(p for p in result["plans"] if p["phase"] == "transport")["payload_geometry"]["padding_m"]
    with pytest.raises(KeyError):
        controls(result, meta, result["controller_profile"], "robot")


@pytest.mark.parametrize("key,value", [("task_id", "mustard_bottle"), ("physical_trial_passed", False),
    ("physical_sequence_complete", False), ("original_task_success", False),
    ("final_original_destination_predicate", False), ("physics_state_setters_used", True),
    ("physical_weld_used", True), ("status", "failed"), ("variant", "human")])
def test_incomplete_or_wrong_original_task_rejected(records, key, value):
    result, meta = records
    result[key] = value
    with pytest.raises(ValueError):
        successful_record(result, meta, "base")


@pytest.mark.parametrize("changed", ["padding", "nested_padding", "controller", "human_profile"])
def test_matched_controller_negative(records, changed):
    result, meta = records
    original = deepcopy(result["controller_profile"])
    if changed == "padding":
        result["transport_object_padding_m"] = 0
    elif changed == "nested_padding":
        next(p for p in result["plans"] if p["phase"] == "transport")["validation"]["transport_object_padding_m"] = 0
    elif changed == "controller":
        result["controller_profile"]["release_gap_m"] = .005
    else:
        result["control_profile"] = "base"
    with pytest.raises(ValueError):
        controls(result, meta, original, "human")


def boundary():
    return dict(plans=[dict(phase="transport", status="executed", start_physics_step=2)],
        observed_transport_binding=dict(physics_step=2)), dict(physics_step=np.arange(5),
        phase=np.array(["close", "lift", "lift", "transport", "transport"]))


def test_transport_uses_last_lift_actual_not_first_transport():
    result, data = boundary()
    assert transport_index(result, data)[0] == 2
    result["plans"][0]["start_physics_step"] = 3
    result["observed_transport_binding"]["physics_step"] = 3
    with pytest.raises(ValueError, match="final actual lift"):
        transport_index(result, data)


@pytest.mark.parametrize("which", ["missing", "duplicate", "not_executed", "stale_binding"])
def test_transport_boundary_negative(which):
    result, data = boundary()
    if which == "missing": result["plans"] = []
    if which == "duplicate": result["plans"] *= 2
    if which == "not_executed": result["plans"][0]["status"] = "failed"
    if which == "stale_binding": result["observed_transport_binding"]["physics_step"] = 1
    with pytest.raises(ValueError): transport_index(result, data)


def test_actual_boundary_2608(records):
    result, _ = records
    with np.load(ROOT / "datasets/replicator/g_shape_bowl_sink/base/human_matched_01/physics_states.npz") as raw:
        data = {k: raw[k] for k in ("physics_step", "phase")}
    assert transport_index(result, data)[0] == 2608


@pytest.mark.parametrize("seeds", [(42,43), (43,42,44), (42,42,44), (42,43,45), (42,43,44,45)])
def test_predeclared_order_no_seed_selection(seeds):
    with pytest.raises(ValueError): validate_seeds(seeds)


def test_declared_seeds():
    validate_seeds([42,43,44])


def test_dynamic_hazard_has_no_spawn_fallback():
    with pytest.raises(ValueError, match="no spawn fallback"):
        require_hazard_reference(dict(static=False), None)
    with pytest.raises(ValueError, match="Explicit"):
        require_hazard_reference({}, Path("existing"))
    require_hazard_reference(dict(static=True), None)
    require_hazard_reference(dict(static=False), Path("must_be_independently_audited"))


def test_original_eight_substeps_and_no_endpoint_splice():
    path = np.array([[.2]*7, [.3]*7, [.5]*7])
    dense = densify(path)
    assert dense.shape == (17,7)
    assert np.array_equal(dense[::8], path)
    assert dense[0,0] == .2


@pytest.mark.parametrize("path", [np.zeros((1,7)), np.zeros((3,6)), np.full((2,7), np.nan)])
def test_invalid_dense_path(path):
    with pytest.raises(ValueError): densify(path)


def test_cuda_valid_flag_cannot_excuse_nonzero_component():
    report = dict(valid=True, components={k:dict(nonzero_samples=[]) for k in
        ("joint_bound", "world_collision", "self_collision")})
    report["components"]["self_collision"]["nonzero_samples"] = [5]
    assert not cuda_zero(report)
    report["components"]["self_collision"]["nonzero_samples"] = []
    assert cuda_zero(report)
    assert not cuda_zero(dict(valid=True, components={}))
    del report["components"]["self_collision"]
    assert not cuda_zero(report)


def good_rows():
    rows=[]
    for seed in SEEDS:
        for on in (False, True):
            rows.append(dict(seed=seed, hazard_in_planning_world=on, success=True, query_sha256="same",
                metrics=dict(fixed_query_passed=True), full_world_validation=dict(valid=True,validation_complete=True),
                dense_120hz_cuda=dict(valid=True,components={k:dict(nonzero_samples=[]) for k in
                    ("joint_bound", "world_collision", "self_collision")}),
                dense_120hz_all_source_passed=True,
                geometry={k:dict(passed=on,violations=int(not on)) for k in
                    ("source_mesh","source_hull","conservative_execution","conservative_planning")}))
    return rows


def test_all_declared_pairs_required():
    assert all(p["passed"] for p in pair_gates(good_rows()))
    assert not all(p["passed"] for p in pair_gates(good_rows()[:-1]))
    duplicate=good_rows();duplicate.append(deepcopy(duplicate[0]))
    assert not pair_gates(duplicate)[0]["passed"]


@pytest.mark.parametrize("field", ["query", "self", "source", "no_risk", "on_planning", "failed"])
def test_pair_cannot_pass_incomplete_or_noncausal_evidence(field):
    rows=good_rows()
    if field=="query":rows[1]["query_sha256"]="changed"
    if field=="self":rows[1]["dense_120hz_cuda"]["components"]["self_collision"]["nonzero_samples"]=[3]
    if field=="source":rows[1]["dense_120hz_all_source_passed"]=False
    if field=="no_risk":rows[0]["geometry"]["source_mesh"]["violations"]=0
    if field=="on_planning":rows[1]["geometry"]["conservative_planning"]["passed"]=False
    if field=="failed":rows[1]["success"]=False
    assert not pair_gates(rows)[0]["passed"]
