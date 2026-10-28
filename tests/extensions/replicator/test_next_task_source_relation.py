"""CPU regression checks for preserving the next tasks' original relations."""
import copy
import importlib.util
from pathlib import Path

import pytest

pytest.importorskip("isaaclab_arena.relations.validation.pre_physics")

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location(
    "next_task_relation_audit", ROOT / "scripts/extensions/replicator/audit_task_source_relation.py")
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


@pytest.mark.parametrize("operation", ["banana_bowl", "bowl_sink"])
def test_every_original_relation_is_validated(operation):
    layout = audit.prepare(operation)
    report = audit.check_layout(layout)
    _, source = audit.source(layout["task_id"])
    assert report["initial_source_relations_passed"]
    assert [{key: row[key] for key in original} for row, original in zip(report["checks"], source["relations"])] == source["relations"]
    assert len(report["checks"]) == len(source["relations"])
    assert not report["execution_ready"]


def test_on_preserves_original_five_centimeter_edge_margin():
    layout = audit.prepare("banana_bowl")
    # Entire fruit still fits inside the source counter, but its left edge is
    # only ~6.5 mm from the edge, below the unchanged original 50 mm margin.
    layout["objects"]["banana"]["position"][0] = -2.85
    report = audit.check_layout(layout)
    check = next(row for row in report["checks"] if row["kind"] == "on" and row["subject"] == "banana")
    assert min(check["edge_margins_xy_m"]) > 0
    assert check["required_edge_margin_m"] == .05
    assert not check["passed"]
    assert not report["initial_source_relations_passed"]


def test_bowl_keeps_source_side_of_sink():
    layout = audit.prepare("bowl_sink")
    layout["objects"]["bowl"]["position"][1] = -.4
    report = audit.check_layout(layout)
    check = next(row for row in report["checks"] if row["kind"] == "next_to" and row["subject"] == "bowl")
    assert not check["passed"]
    assert not report["initial_source_relations_passed"]


def test_robot_yaw_is_not_relaxed():
    layout = audit.prepare("banana_bowl")
    layout["robot_yaw_deg"] += 1
    report = audit.check_layout(layout)
    assert not next(row for row in report["checks"] if row["kind"] == "rotate_around_solution")["passed"]


def test_sink_reference_cannot_be_replaced_with_spawned_object():
    layout = audit.prepare("bowl_sink")
    layout["objects"]["sink"] = copy.deepcopy(layout["objects"]["bowl"])
    with pytest.raises(ValueError, match="exactly the source task objects"):
        audit.check_layout(layout)
