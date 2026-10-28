"""Regression checks for admitting only complete hash-bound observed baselines."""
import json
from pathlib import Path

import numpy as np
import pytest

from kitchen_safety.extensions.replicator.scene_risk_baselines import (
    ASSIGNMENT, BaselineUnavailable, PREFIX, inspect_stream, load_baseline,
    sha256, verify_candidate, _raw_stream,
)


def complete_stream():
    phases = np.array(["pregrasp", "approach", "close", "lift", "transport", "lower", "release", "retreat"] + ["final_settle"]*12)
    n = len(phases)
    payload = np.zeros((n, 7)); payload[:, 6] = 1
    payload[3:6, 2] = .15
    return dict(q=np.zeros((n, 13)), payload_pose_xyzw=payload, phase=phases,
                physics_step=np.arange(n), physics_dt_s=.1,
                joint_names=[f"panda_joint{i}" for i in range(1, 8)]+["finger_joint"]+[f"finger_{i}" for i in range(5)],
                arm_ids=np.arange(7))


def test_observed_complete_stream_and_xyzw():
    audit = inspect_stream(complete_stream())
    assert audit["passed"]
    assert audit["checks"]["payload_orientation_valid"]


@pytest.mark.parametrize("damage", ["missing_retreat", "missing_final_settle", "gap", "nonfinite", "gripper_not_open", "no_lift"])
def test_success_flag_cannot_replace_complete_observed_sequence(damage):
    stream = complete_stream()
    if damage == "missing_retreat": stream["phase"][7] = "release"
    if damage == "missing_final_settle": stream["phase"][8:] = "retreat"
    if damage == "gap": stream["physics_step"][10:] += 1
    if damage == "nonfinite": stream["q"][5, 3] = np.nan
    if damage == "gripper_not_open": stream["q"][-1, 7] = .5
    if damage == "no_lift": stream["payload_pose_xyzw"][:, 2] = 0
    assert not inspect_stream(stream)["passed"]


def test_mustard_absolute_step_splice_preserves_xyzw(tmp_path):
    episode = tmp_path/"episode"; episode.mkdir()
    names=np.asarray([f"panda_joint{i}" for i in range(1,8)]+[f"finger{i}" for i in range(6)])
    prefix_pose=np.zeros((2,13)); prefix_pose[:,6]=1
    main_pose=np.zeros((2,7)); main_pose[:,5]=1  # 180 degree world Z, xyzw
    np.savez(episode/"startup_physics_states.npz",q=np.zeros((2,13)),mustard=prefix_pose,
             phase=["settle","settle"],physics_step=[0,1],action_index=[-1,-1],joint_names=names)
    np.savez(episode/"physics_states.npz",q=np.ones((2,13)),mustard=main_pose,
             phase=["lift","retreat"],physics_check=[1,2],action_index=[0,0],attached=[True,False],
             joint_names=names,arm_ids=np.arange(7),pose_convention="world xyz + xyzw")
    np.savez(episode/"motion_physics_index.npz",physics_step=[2,3],phase=["lift","retreat"],action_index=[0,0])
    stream=_raw_stream(tmp_path,dict(episode_path="episode",format="mustard_split_physics"),dict(execution_configuration=dict(physics_dt=.01)))
    assert stream["physics_step"].tolist()==[0,1,2,3]
    assert stream["payload_pose_xyzw"][0,6]==1
    assert stream["payload_pose_xyzw"][2,5]==1
    assert stream["time_s"].tolist()==[0,.01,.02,.03]
    # A locally contiguous stream with shifted absolute IDs must not pass.
    assert not inspect_stream({**complete_stream(),"physics_step":np.arange(20)+2})["passed"]


def test_diagnostic_success_explicitly_rejected(tmp_path):
    task_id=PREFIX+"g_shape_banana_bowl"
    (tmp_path/"episode").mkdir()
    (tmp_path/"episode/result.json").write_text(json.dumps(dict(task_id=task_id,variant="base",diagnostic_only=True,original_task_success=True)))
    (tmp_path/"acceptance.json").write_text(json.dumps(dict(baseline_verified=True)))
    out=verify_candidate(tmp_path,dict(task_id=task_id,profile="base",episode_path="episode",verification_path="acceptance.json",format="task_full_physics"),dict(id=task_id))
    assert out["status"]=="blocked_baseline"
    assert "Diagnostic/neutral" in out["errors"][0]


def test_missing_raw_hash_binding_rejected_despite_acceptance_flags(tmp_path):
    task_id=PREFIX+"g_shape_bowl_sink"
    (tmp_path/"episode").mkdir()
    (tmp_path/"episode/result.json").write_text(json.dumps(dict(task_id=task_id,variant="base")))
    (tmp_path/"acceptance.json").write_text(json.dumps(dict(baseline_verified=True,episode="episode",checks={})))
    out=verify_candidate(tmp_path,dict(task_id=task_id,profile="base",episode_path="episode",verification_path="acceptance.json",format="task_full_physics"),dict(id=task_id))
    assert out["status"]=="blocked_baseline"
    assert "acceptance gates missing" in out["errors"][0]


def test_loader_rejects_stale_record_before_loading_npz(tmp_path):
    assignment=tmp_path/ASSIGNMENT; assignment.parent.mkdir(parents=True); assignment.write_text('{}')
    (tmp_path/"raw.npz").write_bytes(b"original")
    inv=dict(assignment_sha256=sha256(assignment), tasks=[dict(task_id=PREFIX+"g_shape_bowl_sink",baselines=[dict(profile="base",status="verified_baseline",files={"raw.npz":sha256(tmp_path/"raw.npz")})])])
    (tmp_path/"inventory.json").write_text(json.dumps(inv))
    (tmp_path/"raw.npz").write_bytes(b"mutated")
    with pytest.raises(BaselineUnavailable,match="Stale baseline binding"):
        load_baseline(tmp_path,"g_shape_bowl_sink",inventory_path="inventory.json")


def test_unknown_task_never_falls_back_to_other_layout(tmp_path):
    assignment=tmp_path/ASSIGNMENT; assignment.parent.mkdir(parents=True); assignment.write_text('{}')
    (tmp_path/"inventory.json").write_text(json.dumps(dict(assignment_sha256=sha256(assignment),tasks=[])))
    with pytest.raises(BaselineUnavailable,match="blocked_baseline"):
        load_baseline(tmp_path,"u_shape_bowl_sink",inventory_path="inventory.json")
