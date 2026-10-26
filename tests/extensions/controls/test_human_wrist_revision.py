import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[3]
ASSET=ROOT/'assets/extensions/controls/human_reach_anatomical_down01'


@pytest.mark.parametrize('asset_name',['human_reach_anatomical_down01','human_hand_between_middle01','human_hand_between_roll30'])
def test_wrist_follows_original_forearm_relative_frame(asset_name):
    asset=ROOT/'assets/extensions/controls'/asset_name
    pose=json.loads((asset/'pose.json').read_text())
    wrist=pose['wrist_structure']
    assert wrist['mode']=='follow-forearm'
    def rotation(name):
        q=wrist[name]
        return Rotation.from_quat([*q[1:],q[0]])
    error=rotation('source_relative_hand_quaternion_wxyz').inv()*rotation('posed_relative_hand_quaternion_wxyz')
    assert error.magnitude()<np.deg2rad(.001)
    def bend(bones):
        lower=np.subtract(bones['lowerarm_r'][1],bones['lowerarm_r'][0])
        hand=np.subtract(bones['hand_r'][1],bones['hand_r'][0])
        return np.arccos(np.clip(np.dot(lower,hand)/np.linalg.norm(lower)/np.linalg.norm(hand),-1,1))
    assert abs(bend(pose['before'])-bend(pose['after']))<np.deg2rad(.001)


@pytest.mark.parametrize('asset_name',['human_reach_anatomical_down01','human_hand_between_middle01','human_hand_between_roll30'])
def test_arm_lengths_shoulder_and_feet_preserved(asset_name):
    asset=ROOT/'assets/extensions/controls'/asset_name
    pose=json.loads((asset/'pose.json').read_text())
    assert pose['arm_frame']=='source-bend-plane'
    for name in ('upperarm_r','lowerarm_r'):
        lengths=[np.linalg.norm(np.subtract(pose[key][name][1],pose[key][name][0])) for key in ('before','after')]
        assert abs(lengths[0]-lengths[1])<1e-5
    assert np.max(np.abs(np.subtract(pose['before']['upperarm_r'][0],pose['after']['upperarm_r'][0])))<1e-5
    audit=json.loads((asset/'rebuild_geometry_audit.json').read_text())
    assert audit['passed'] and audit['checks']['feet_unchanged']
    assert audit['checks']['source_wrist_frame_preserved']
    assert audit['checks']['source_wrist_bend_preserved']


def test_button_revision_does_not_change_original_controls_or_other_hazards():
    for key,original in (('left','repaired_left_button'),('right','right_button_final_candidate')):
        folder=ROOT/'configs/extensions/controls'
        before=json.loads((folder/(original+'.json')).read_text())
        after=json.loads((folder/(key+'_button_anatomical_down01.json')).read_text())
        for name in ('task_id','source_sha256','robot_pose_xyzw','button_joint','button_joint_limits_m','prepress_planning'):
            assert before[name]==after[name]
        for role in ('property','robot'):
            assert before['variants'][role]==after['variants'][role]


@pytest.mark.parametrize('flat,role', [(False, 'human'), (True, 'human'), (True, 'property')])
def test_ranked_layout_selection_checks_flat_search_role(tmp_path, flat, role):
    original=tmp_path/'config.json'
    search=tmp_path/'search.json'
    output=tmp_path/'selected.json'
    original.write_text(json.dumps({'variants': {'human': {'asset': 'old'}}}))
    candidate={'spec': {'asset': 'new', 'position': [2.9, -.86, 0], 'yaw_deg': -30}}
    search.write_text(json.dumps({'variant': role,
        'candidates': [candidate] if flat else {'human': [candidate]}}))
    result=subprocess.run([sys.executable, str(ROOT/'scripts/extensions/controls/select_layout_candidates.py'),
        '--config', str(original), '--output', str(output), '--select', 'human', str(search), '0'],
        capture_output=True, text=True)
    if flat and role!='human':
        assert result.returncode!=0 and not output.exists()
        assert 'Candidate role mismatch' in result.stderr
    else:
        assert result.returncode==0, result.stderr
        assert json.loads(output.read_text())['variants']['human']==candidate['spec']
