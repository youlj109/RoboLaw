import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[3]


@pytest.mark.parametrize('side,target_x', [('left', 2.08087763), ('right', 2.14557961)])
def test_hand_is_between_original_robot_and_button(side, target_x):
    config = json.loads((ROOT/'configs/extensions/controls'/
        (side+'_button_hand_between_draft03.json')).read_text())
    human = config['variants']['human']
    asset = ROOT/Path(human['asset']).parent
    pose = json.loads((asset/'pose.json').read_text())
    hand = Rotation.from_euler('z', human['yaw_deg'], degrees=True).apply(
        np.array(pose['after']['hand_r']).mean(axis=0))+human['position']
    assert abs(hand[0]-target_x)<.04
    assert config['robot_pose_xyzw'][1]<hand[1]<-.38513822422
    assert 1.05<hand[2]<1.25
    audit = json.loads((asset/'rebuild_geometry_audit.json').read_text())
    assert audit['passed'] and audit['checks']['source_wrist_frame_preserved']
    assert audit['checks']['feet_unchanged']


@pytest.mark.parametrize('side,previous', [
    ('left', 'left_button_anatomical_candidate01'),
    ('right', 'right_button_anatomical_candidate02'),
])
def test_draft_preserves_robot_controller_and_other_variants(side, previous):
    folder = ROOT/'configs/extensions/controls'
    old = json.loads((folder/(previous+'.json')).read_text())
    new = json.loads((folder/(side+'_button_hand_between_draft03.json')).read_text())
    for key in ('task_id', 'source_sha256', 'robot_pose_xyzw', 'prepress_planning'):
        assert old[key]==new[key]
    for role in ('property', 'robot'):
        assert old['variants'][role]==new['variants'][role]
    assert new['placement_candidate_selection']['human']['rule'].startswith('Explicit diagnostic pose')


@pytest.mark.parametrize('side,candidate,previous',[
    ('left','05','repaired_left_button'),('right','02','right_button_final_candidate'),
])
def test_final_hand_reaches_workspace_without_changing_task_policy(side,candidate,previous):
    folder=ROOT/'configs/extensions/controls'
    config=json.loads((folder/(side+'_button_between_candidate'+candidate+'.json')).read_text())
    old=json.loads((folder/(previous+'.json')).read_text())
    for key in ('task_id','source_sha256','robot_pose_xyzw','prepress_planning'):
        assert old[key]==config[key]
    for role in ('property','robot'):
        assert old['variants'][role]==config['variants'][role]
    spec=config['variants']['human']
    assert spec['position'][2]==0
    bones=json.loads((ROOT/Path(spec['asset']).parent/'pose.json').read_text())['after']
    rotation=Rotation.from_euler('z',spec['yaw_deg'],degrees=True)
    wrist=rotation.apply(bones['hand_r'][0])+spec['position']
    assert config['robot_pose_xyzw'][1]<wrist[1]<-.38513822422
    # The fingertips, not just the wrist marker, reach across the toaster front.
    tips=rotation.apply([bones[name][1] for name in ('index_03_r','middle_03_r','ring_03_r')])+spec['position']
    target_x=config.get('button_center_world_m',[2.08087763])[0]
    assert np.min(np.abs(tips[:,0]-target_x))<.16
    assert np.all((tips[:,2]>1.1)&(tips[:,2]<1.4))
