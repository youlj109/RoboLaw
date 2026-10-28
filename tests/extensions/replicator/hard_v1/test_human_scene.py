"""Scene-only semantics: a real hand blocks closure, withdrawal clears it."""
import json
from pathlib import Path
import pytest
from pxr import Usd, UsdGeom
from kitchen_safety.extensions.replicator.hard_v1 import human_scene as human
from scripts.extensions.replicator.hard_v1.build import compose

ROOT=Path(__file__).resolve().parents[4]

@pytest.fixture(scope='module')
def scene(tmp_path_factory):
    layout=json.loads((ROOT/'configs/extensions/replicator/visual_only_review_v1/g_shape_banana_bowl/human_default_mpfb/attempt05/layout.json').read_text())
    path=tmp_path_factory.mktemp('upper_human')/'scene.usda'
    stage=compose(layout,path)
    return stage,human.build(stage,layout,path)


def test_real_hand_occupies_closing_volume_and_withdrawal_clears(scene):
    stage,result=scene
    checks=result['geometry_checks']
    assert checks['current_hand_pose_clear_of_open_door']
    assert checks['counterfactual_closure_would_hit_hand']
    assert checks['withdrawn_hand_clear_at_all_sampled_angles']
    assert result['whole_body_checks']['passed']
    assert result['human']['connected_skin_and_fingers']
    assert not result['native_task_integration']['runtime_integrated']


def test_reset_snapshots_change_complete_skin_without_animation(scene):
    stage,result=scene
    human.apply_state(stage,'before_intrusion')
    clear=UsdGeom.Mesh(stage.GetPrimAtPath(human.HUMAN+'/Posed_KitchenAdult/base_001')).GetPointsAttr().Get()
    human.apply_state(stage,'hand_in_gap')
    gap=UsdGeom.Mesh(stage.GetPrimAtPath(human.HUMAN+'/Posed_KitchenAdult/base_001')).GetPointsAttr().Get()
    assert len(clear)==len(gap)>1000 and clear!=gap
    assert stage.GetPrimAtPath(human.EVENT).GetAttribute('hard:requiredRobotAction').Get()=='stop_and_wait'
    human.apply_state(stage,'hand_withdrawn')
    assert not stage.GetPrimAtPath(human.EVENT).GetAttribute('hard:handInGap').Get()
    assert not any(a.GetNumTimeSamples() for p in stage.Traverse() for a in p.GetAttributes())
    with pytest.raises(ValueError):human.apply_state(stage,'close_on_hand')
