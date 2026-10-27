"""Pre-contact candidate policy tests; no GPU, scene changes or path validation."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

SOURCE = Path(__file__).resolve().parents[3]/'scripts/extensions/controls/endpoint_candidates.py'
spec = importlib.util.spec_from_file_location('endpoint_candidates', SOURCE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.mark.parametrize('phase', ['prepress', 'pregrasp'])
def test_candidates_preserve_operation_axis_and_input(phase):
    target = np.eye(4)
    target[:3, :3] = Rotation.from_euler('z', 90, degrees=True).as_matrix()
    target[:3, 3] = [2., -1., 1.2]
    original = target.copy()
    rows = list(module.pose_variants(target, phase))
    assert rows[0][1]['strategy'] == 'redundant_ik_only'
    assert np.array_equal(rows[0][0], target)
    for pose, policy in rows:
        assert np.allclose(pose[:3, 0], target[:3, 0])
        delta = pose[:3, 3]-target[:3, 3]
        expected = np.zeros(3)
        expected[1 if phase == 'prepress' else 0] = -policy['extra_standoff_m']
        assert np.allclose(delta, expected)
        assert policy['actual_contact_target_changed'] is False
    assert np.array_equal(target, original)


class FakeTensor:
    def __init__(self, values):
        self.values = np.asarray(values)

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.values


class FakeKin:
    limits = np.asarray([[-3., 3.]]*7)

    def __init__(self):
        self.calls = 0

    def ik(self, pose, seed, attempts):
        assert attempts == 1
        self.calls += 1
        return np.full(7, self.calls*.1)


def planner(valid):
    return SimpleNamespace(args=SimpleNamespace(to_device=lambda x: x),
        motion_gen=SimpleNamespace(graph_planner=SimpleNamespace(
            mask_samples=lambda x: FakeTensor([valid]))))


def test_selects_first_eligible_without_any_path_planner():
    kin = FakeKin()
    selected, report = module.select_endpoint(kin, planner(True), np.zeros(7), np.ones(7),
                                             np.eye(4), 'prepress')
    assert kin.calls == 1
    assert selected['candidate_index'] == 0
    assert selected['policy']['strategy'] == 'redundant_ik_only'
    assert len(report['candidates']) == 1


def test_candidate_failure_is_bounded_without_motiongen_fallback():
    kin = FakeKin()
    selected, report = module.select_endpoint(kin, planner(False), np.zeros(7), np.ones(7),
                                             np.eye(4), 'pregrasp', max_candidates=3)
    assert selected is None
    assert kin.calls == 3
    assert len(report['candidates']) == 3
    assert report['budget_exhausted']


def test_time_budget_can_stop_before_any_ik():
    kin = FakeKin()
    selected, report = module.select_endpoint(kin, planner(True), np.zeros(7), np.ones(7),
                                             np.eye(4), 'prepress', budget_s=0)
    assert selected is None
    assert kin.calls == 0
    assert report['budget_exhausted']


def test_no_contact_phase_candidates():
    with pytest.raises(ValueError):
        list(module.pose_variants(np.eye(4), 'turn'))


def test_single_round_has_only_one_ik_call_and_one_change():
    kin = FakeKin()
    target = np.eye(4)
    target[:3, 3] = [5.17, -3.22, 1.26]
    original = target.copy()
    result = module.single_policy_endpoint(kin, target, np.ones(7), 'pregrasp',
                                           {'change':'standoff','value':.09})
    assert kin.calls == 1
    assert np.allclose(result['requested_tcp'][:3, 3], [5.08,-3.22,1.26])
    assert np.array_equal(result['requested_tcp'][:3, :3], target[:3, :3])
    assert result['policy']['axial_roll_delta_deg'] == 0
    assert np.array_equal(target, original)


def test_single_round_refuses_compound_or_unbounded_change():
    kin = FakeKin()
    for policy in ({'change':'standoff','value':.3},
                   {'change':'standoff','value':.09,'roll':30},
                   {'change':'roll','value':float('nan')}):
        with pytest.raises(ValueError):
            module.single_policy_endpoint(kin, np.eye(4), np.ones(7), 'pregrasp', policy)
    assert kin.calls == 0


def test_arrival_point_change_preserves_pose_and_only_solves_once():
    kin=FakeKin();target=np.eye(4);target[:3,3]=[5.17,-3.22609,1.26588]
    result=module.single_policy_endpoint(kin,target,np.ones(7),'pregrasp',
        {'change':'arrival_tcp_position','value':[5.26,-3.22609,1.26588]})
    assert kin.calls==1
    assert np.array_equal(result['requested_tcp'][:3,:3],target[:3,:3])
    assert np.allclose(result['requested_tcp'][:3,3],[5.26,-3.22609,1.26588])
    assert result['policy']['actual_contact_target_changed'] is False


def test_tilt_is_only_a_free_space_candidate_not_a_contact_axis_claim():
    pose,seed,policy=module.single_policy_parameters(np.eye(4),np.ones(7),'pregrasp',
        {'change':'pregrasp_tilt_y','value':30})
    assert np.allclose(pose[:3,3],0)
    assert np.allclose(pose[:3,:3],Rotation.from_euler('y',30,degrees=True).as_matrix())
    assert policy['requires_alignment_before_contact']
    with pytest.raises(ValueError):
        module.single_policy_parameters(np.eye(4),np.ones(7),'prepress',
            {'change':'pregrasp_tilt_y','value':30})


def test_yaw_changes_world_heading_only_and_solves_once():
    kin = FakeKin()
    target = np.eye(4)
    target[:3, :3] = Rotation.from_euler('y', 45, degrees=True).as_matrix()
    target[:3, 3] = [5.17, -3.22609, 1.26588]
    original = target.copy()
    seed = np.arange(7.)
    result = module.single_policy_endpoint(kin, target, seed, 'pregrasp',
        {'change': 'pregrasp_yaw_z', 'value': -30})
    expected = Rotation.from_euler('z', -30, degrees=True).as_matrix() @ target[:3, :3]
    assert np.allclose(result['requested_tcp'][:3, :3], expected)
    assert np.array_equal(result['requested_tcp'][:3, 3], target[:3, 3])
    assert np.array_equal(target, original)
    assert kin.calls == 1
    assert result['policy']['requires_alignment_before_contact']
    assert result['policy']['actual_contact_target_changed'] is False


@pytest.mark.parametrize('phase,value', [('prepress', 15), ('pregrasp', 61),
                                        ('pregrasp', float('nan'))])
def test_invalid_yaw_does_not_solve_ik(phase, value):
    kin = FakeKin()
    with pytest.raises(ValueError):
        module.single_policy_endpoint(kin, np.eye(4), np.zeros(7), phase,
            {'change': 'pregrasp_yaw_z', 'value': value})
    assert kin.calls == 0


def test_auxiliary_height_does_not_change_real_contact_or_orientation():
    target = np.eye(4)
    target[:3, 3] = [5.17, -3.22609, 1.26588]
    kin = FakeKin()
    result = module.single_policy_endpoint(kin, target, np.zeros(7), 'pregrasp',
        {'change': 'pregrasp_height', 'value': .06})
    assert kin.calls == 1
    assert np.allclose(result['requested_tcp'][:3, 3], [5.17, -3.22609, 1.32588])
    assert np.array_equal(result['requested_tcp'][:3, :3], target[:3, :3])
    assert result['policy']['actual_contact_target_changed'] is False
    assert result['policy']['approach_endpoint_changed'] is True
    assert result['policy']['requires_alignment_before_contact'] is True


@pytest.mark.parametrize('phase,value', [('prepress', .04), ('pregrasp', .061),
                                        ('pregrasp', 0), ('pregrasp', -.01)])
def test_auxiliary_height_is_bounded_and_precontact_only(phase, value):
    with pytest.raises(ValueError):
        module.single_policy_parameters(np.eye(4), np.zeros(7), phase,
            {'change': 'pregrasp_height', 'value': value})
