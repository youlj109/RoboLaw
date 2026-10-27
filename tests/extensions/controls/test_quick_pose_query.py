"""One-call dispatch and calibration tests; mock results are not planning evidence."""
import importlib.util
import sys
import json
import hashlib
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
import pytest
from curobo.types.base import TensorDeviceType
from kitchen_safety.curobo_backend import CuroboPlanner

source=Path(__file__).resolve().parents[3]/'scripts/extensions/controls/quick_plan_scene.py'
spec=importlib.util.spec_from_file_location('quick_pose_query',source)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


@pytest.mark.parametrize('mode',['joint_goal','pose_goal'])
def test_query_dispatches_once_with_one_attempt_and_no_goal_change(mode,monkeypatch):
    # Importing the pinned MotionGen class itself allocates a CUDA default Pose.
    # Mock only its settings container; real GPU execution is tested separately.
    monkeypatch.setitem(sys.modules,'curobo.wrap.reacher.motion_gen',
        SimpleNamespace(MotionGenPlanConfig=lambda **kwargs:SimpleNamespace(**kwargs)))
    calls=SimpleNamespace(plan_single=Mock(return_value='pose'),plan_single_js=Mock(return_value='joint'))
    offset=np.eye(4);offset[0,3]=.131
    planner=SimpleNamespace(args=TensorDeviceType(device='cpu'),motion_gen=calls,
        T_B_W=np.eye(4),T_ee_tcp=offset,_pose_list=CuroboPlanner._pose_list)
    target=np.eye(4);target[:3,3]=[1.,2.,3.];before=target.copy()
    result=module.motiongen_query_once(planner,np.zeros(7),target,np.ones(7),mode)
    method=calls.plan_single if mode=='pose_goal' else calls.plan_single_js
    other=calls.plan_single_js if mode=='pose_goal' else calls.plan_single
    assert method.call_count==1 and other.call_count==0
    assert method.call_args.args[2].max_attempts==1
    assert np.array_equal(target,before)
    if mode=='pose_goal':assert np.allclose(method.call_args.args[1].position.numpy(),[[.869,2.,3.]])
    assert result==('pose' if mode=='pose_goal' else 'joint')


@pytest.mark.parametrize('mutation',['none','missing_payload','changed_manifest'])
def test_missing_or_changed_human_payload_cannot_be_planned_as_empty_space(tmp_path,mutation):
    payload=tmp_path/'person.usdc';payload.write_bytes(b'hash fixture, not a USD model')
    manifest=tmp_path/'manifest.json';manifest.write_text(json.dumps({'generated_files':{
        'person.usdc':hashlib.sha256(payload.read_bytes()).hexdigest()}}))
    config={'human_pose_revision':{'manifest':'manifest.json',
        'manifest_sha256':hashlib.sha256(manifest.read_bytes()).hexdigest()}}
    if mutation=='missing_payload':payload.unlink()
    if mutation=='changed_manifest':manifest.write_text('{}')
    if mutation=='none':assert module.verify_human_asset_binding(tmp_path,config)['files_checked']==1
    else:
        with pytest.raises(ValueError):module.verify_human_asset_binding(tmp_path,config)
