"""Regression gates for the autonomous knob and premature-button failures."""
import copy
import importlib.util
import json
from pathlib import Path

MODULE_PATH = Path(__file__).resolve().parents[3] / 'scripts/extensions/controls/review_contact_causality.py'
SPEC = importlib.util.spec_from_file_location('contact_causality', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def episode(tmp_path):
    names = ['sideL_button_cancel_joint', 'sideL_knob_doneness_joint', 'sideL_lever_joint',
             'sideR_button_cancel_joint', 'sideR_knob_doneness_joint', 'sideR_lever_joint']
    def state(q):
        return {'toaster_q': [q, 0, 0, 0, 0, 0], 'predicate': q > sum(MODULE.ORIGINAL_LIMITS_M) / 2}
    data = {'result': {'toaster_joint_names': names, 'button_joint_limits_m': MODULE.ORIGINAL_LIMITS_M,
                       'success': True, 'physical_object_setters_after_reset': False},
            'records': [{'before': state(0), 'after': state(0)},
                        {'before': state(0), 'after': state(.002)}],
            'contacts': [], 'audit': {'passed': True, 'physics_updates': []}}
    for index, (q, force) in enumerate([(0, 0), (0, 0), (.00005, .5), (.002, .5)]):
        data['audit']['physics_updates'].append({'action_index': index // 2,
            'toaster_q': state(q)['toaster_q'], 'toaster_dq': [0]*6,
            'button_contacts': {'finger': {'normal_force': [force]}}})
    def write(value):
        for name, contents in value.items():
            (tmp_path / (name + '.json')).write_text(json.dumps(contents))
        return MODULE.review(tmp_path)
    return data, write


def test_contact_then_motion_is_accepted(tmp_path):
    data, write = episode(tmp_path)
    assert write(data)['passed']


def test_autonomous_knob_is_rejected_even_when_button_successful(tmp_path):
    data, write = episode(tmp_path)
    data['audit']['physics_updates'][0]['toaster_q'][1] = 2.1
    result = write(data)
    assert result['checks']['original_task_succeeded']
    assert not result['checks']['non_target_joints_stationary']
    assert not result['passed']


def test_button_moves_before_contact_is_rejected(tmp_path):
    data, write = episode(tmp_path)
    data['audit']['physics_updates'][1]['toaster_q'][0] = .0002
    result = write(data)
    assert not result['checks']['precontact_target_stationary']
    assert not result['passed']


def test_same_substep_contact_and_success_is_explicit(tmp_path):
    data, write = episode(tmp_path)
    data['audit']['physics_updates'][2]['toaster_q'][0] = .002
    result = write(data)
    assert result['passed']
    assert result['ordering'] == 'same_substep'


def test_missing_substep_evidence_never_upgrades_to_pass(tmp_path):
    data, write = episode(tmp_path)
    data['audit']['physics_updates'] = []
    result = write(data)
    assert not result['checks']['physics_synchronous_evidence_complete']
    assert not result['passed']
