#!/usr/bin/env python3
"""Diagnose actual states from an aborted action; never certify an episode.

The normal episode loader deliberately rejects incomplete actions. This separate
entry point accepts exactly one final partial action, validates its recorded
extent, and optionally runs the unchanged CUDA self-collision audit on only the
states that were actually saved. It never pads a state or changes a task gate.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from scripts.extensions.replicator.task_audit_sidecars import ROOT, sha

_GPU_LOCK = None


def validate_partial(metadata, data, trajectory):
    steps = data["physics_step"]
    if (steps.ndim != 1 or len(steps) < 2
            or steps[0] != metadata["reset_physics_step"]
            or np.any(np.diff(steps) != 1)):
        raise ValueError("Missing reset/continuous actual physics steps")
    for name, value in data.items():
        if (len(value) != len(steps)
                or (value.dtype.kind not in "US" and not np.isfinite(value).all())):
            raise ValueError("Nonfinite/misaligned actual state: " + name)
    if data["q"].shape != (len(steps), len(metadata["joint_names"])):
        raise ValueError("Actual joint/name dimensions differ")
    physics_dt = metadata["physics_dt_s"]
    decimation = round(metadata["action_dt_s"] / physics_dt)
    if (physics_dt <= 0 or decimation < 2
            or abs(decimation * physics_dt - metadata["action_dt_s"]) > 1e-9):
        raise ValueError("Invalid recorded action/physics periods")
    count = len(trajectory["actions"])
    if count < 1 or any(len(value) != count for value in trajectory.values()):
        raise ValueError("Empty/misaligned recorded actions")
    before, after = trajectory["before_physics_step"], trajectory["after_physics_step"]
    expected = steps[0] + np.arange(count) * decimation
    if not np.array_equal(before, expected) or not np.array_equal(after[:-1], expected[:-1] + decimation):
        raise ValueError("An earlier action is incomplete or has a step gap")
    duration = int(after[-1] - before[-1])
    if not 0 < duration < decimation or after[-1] != steps[-1]:
        raise ValueError("Expected exactly one final partial action ending at the last actual state")
    if len(steps) != (count - 1) * decimation + duration + 1:
        raise ValueError("Actual sample count differs from the recorded partial extent")
    if data["action_index"][0] != -1 or str(data["phase"][0]) != "reset":
        raise ValueError("Initial sample is not the recorded reset state")
    for index, (first, last) in enumerate(zip(before, after)):
        mask = (steps > first) & (steps <= last)
        if (not np.all(data["action_index"][mask] == index)
                or not np.all(data["phase"][mask] == trajectory["phase"][index])):
            raise ValueError("Recorded phase/action differs from actual physics samples")
    return dict(passed=True, reset_physics_step=int(steps[0]),
        final_actual_physics_step=int(steps[-1]), actual_physics_samples=len(steps),
        completed_actions=count - 1, partial_action_index=count - 1,
        partial_action_before_physics_step=int(before[-1]),
        partial_action_after_physics_step=int(after[-1]),
        partial_action_recorded_substeps=duration, required_substeps=decimation,
        unexecuted_substeps_not_inferred=decimation - duration,
        complete_episode=False, recorded_positions_modified=False)


def main():
    global _GPU_LOCK
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--cuda-self", action="store_true",
                        help="Use the canonical GPU lock and audit every actual q; no simulation")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Refuse to overwrite a diagnostic report")
    report = dict(schema_version=1, accepted=False, task_success=False,
        diagnostic_partial_episode=True, episode=str(args.episode),
        scope="Actual recorded prefix only. This report cannot satisfy any complete-episode acceptance gate.",
        kind="partial-self-collision" if args.cuda_self else "partial-preflight")
    try:
        metadata = json.loads((args.episode / "physics_metadata.json").read_text())
        with np.load(args.episode / "physics_states.npz", allow_pickle=False) as raw:
            data = {name: raw[name] for name in raw.files}
        with np.load(args.episode / "trajectory.npz", allow_pickle=False) as raw:
            trajectory = {name: raw[name] for name in raw.files}
        report["actual_extent"] = validate_partial(metadata, data, trajectory)
        from kitchen_safety.extensions.replicator.task_collection_evidence import audit_provenance
        report["provenance"] = audit_provenance(
            args.episode / "task_collection_provenance.json", metadata, int(data["physics_step"][-1]))
        if not report["provenance"]["passed"]:
            raise ValueError("Actual execution provenance failed")
        report["input_sha256"] = {name: sha(args.episode / name) for name in
            ("physics_states.npz", "physics_metadata.json", "trajectory.npz", "result.json",
             "task_collection_provenance.json")}
        report["audit_code_sha256"] = {str(path.relative_to(ROOT)): sha(path) for path in
            (Path(__file__).resolve(), ROOT / "scripts/extensions/replicator/task_audit_sidecars.py")}
        if args.cuda_self:
            import fcntl
            lock_path = ROOT / "reports/skill_trials/gpu.lock"
            if ROOT != Path("/workspace") or not lock_path.exists():
                raise RuntimeError("CUDA audit requires verified mount and canonical existing lock")
            _GPU_LOCK = lock_path.open("a+")
            fcntl.flock(_GPU_LOCK.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            from scripts.extensions.replicator.task_audit_sidecars import self_collision_audit
            report["actual_self_collision"] = self_collision_audit(metadata, data)
            report["passed"] = report["actual_self_collision"]["passed"]
        else:
            report["passed"] = True
            report["self_collision_checked"] = False
    except Exception as exc:
        report.update(passed=False, error=f"{type(exc).__name__}: {exc}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps({key: report[key] for key in ("kind", "passed", "accepted", "task_success")}))
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
