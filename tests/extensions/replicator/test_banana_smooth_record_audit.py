"""Adversarial joins/units/coverage tests using explicitly synthetic records."""
from copy import deepcopy
import math
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from kitchen_safety.extensions.replicator.banana_smooth_record_audit import (
    URDF, INTERFACE, audit_records,
)


def synthetic():
    names = [f'panda_joint{i}' for i in range(1, 8)] + ['finger_joint',
        'right_outer_knuckle_joint', 'left_inner_finger_joint', 'right_inner_finger_joint',
        'left_inner_finger_knuckle_joint', 'right_inner_finger_knuckle_joint']
    bounds = {j.get('name'): [float(j.find('limit').get('lower')), float(j.find('limit').get('upper'))]
              for j in ET.parse(URDF).getroot().findall('joint') if j.get('name') in names}
    q = np.tile([np.mean(bounds[name]) for name in names], (257, 1))
    q[:, 7] = .4
    dq = np.zeros_like(q)
    phase = np.array(['reset'] + ['warmup'] * 8 + ['lower'] * 8 + ['release'] * 240)
    indices = np.r_[-1, np.repeat(np.arange(32), 8)]
    data = dict(q=q, dq=dq, physics_step=np.arange(257), phase=phase, action_index=indices)
    meta = dict(joint_names=names, action_joint_names=names[:8], action_interface=INTERFACE,
                physics_dt_s=1 / 120, action_dt_s=1 / 15)
    targets = [.4 * (1-s)**3 * (1+3*s+6*s*s) for s in np.arange(1, 15) / 15] + [0.] * 16
    actions, rows, bound_rows = [], [], []
    for i in range(32):
        target = targets[i-2] if i >= 2 else 0.
        action = np.r_[q[0, :7], target].astype(np.float32).tolist()
        actions.append(dict(action=action, phase='warmup' if i == 0 else 'lower' if i == 1 else 'release',
            source_binary_intent=0., gripper_target_rad=target, action_interface=INTERFACE,
            before_physics_step=i*8, after_physics_step=(i+1)*8))
    for i in range(257):
        bound_rows.append(dict(physics_step=i, phase=phase[i], action_index=int(indices[i]),
            allowance_rad=1e-5, maximum_excess_rad=0., passed=True, violations=[], measured_q_modified=False))
        if i == 0:
            continue
        index = (i-1)//8
        action = actions[index]['action']
        target = np.r_[action, np.zeros(5)].tolist()
        checks = dict(raw_action_matches=True, gripper_processed_identity=True, staged_targets_match=True,
                      native_targets_match=True, finite_targets=True)
        rows.append(dict(before_physics_step=i-1, after_physics_step=i, phase=phase[i], action_index=index,
            raw_action_rad=action.copy(), processed_gripper_target_rad=[action[-1]],
            staged_joint_targets_rad=target.copy(), native_joint_targets_rad=target.copy(),
            checks=checks, passed=True, actual_q=q[i].tolist(), actual_dq=dq[i].tolist()))
    commands = dict(initial=dict(applied_interval=False, physics_step=0, q=q[0].tolist(), dq=dq[0].tolist()),
        unmatched_pending=False, passed=True, joint_names=names, controlled_joint_ids=list(range(8)), rows=rows)
    limits = dict(passed=True, allowance_rad=1e-5, rows=bound_rows)
    result = dict(action_interface=INTERFACE, release_start_physics_step=16,
        release_schedule=dict(initial_actual_q_rad=.4, initial_actual_dq_rad_s=0., actions=30,
            physics_steps=240, open_first_action=14, open_first_applied_physics_ordinal=113, targets_rad=targets),
        accepted=False, physical_trial_passed=False, task_success=False, diagnostic_only=True,
        command_recording=deepcopy(commands), actual_joint_bound_audit=deepcopy(limits))
    trajectory = dict(actions=np.asarray([a['action'] for a in actions]), phase=np.array([a['phase'] for a in actions]),
        before_physics_step=np.arange(32)*8, after_physics_step=np.arange(1,33)*8)
    return [meta, data, trajectory, result, actions, commands, limits]


class RecordAuditTests(unittest.TestCase):
    def test_synthetic_records_pass_without_task_acceptance(self):
        report = audit_records(*synthetic())
        self.assertTrue(report['passed'])
        self.assertFalse(report['accepted'])
        self.assertFalse(report['task_success'])

    def rejects(self, mutate):
        fixture = synthetic()
        mutate(fixture)
        with self.assertRaises((ValueError, IndexError)):
            audit_records(*fixture)

    def test_reject_shifted_native_interval(self):
        self.rejects(lambda x: x[5]['rows'][30].update(after_physics_step=30))

    def test_reject_actual_q_from_previous_step(self):
        self.rejects(lambda x: x[5]['rows'][30]['actual_q'].__setitem__(0, .1))

    def test_reject_native_target_mismatch_even_when_pass_flag_true(self):
        self.rejects(lambda x: x[5]['rows'][20]['native_joint_targets_rad'].__setitem__(7, .7))

    def test_reject_nonfinite_uncontrolled_target(self):
        self.rejects(lambda x: x[5]['rows'][20]['native_joint_targets_rad'].__setitem__(12, math.nan))

    def test_reject_omitted_command(self):
        self.rejects(lambda x: x[5]['rows'].pop())

    def test_reject_changed_joint_inventory(self):
        self.rejects(lambda x: x[5]['controlled_joint_ids'].__setitem__(7, 8))

    def test_reject_fake_bound_tolerance(self):
        self.rejects(lambda x: x[6].update(allowance_rad=.001))

    def test_reject_measured_bound_violation(self):
        self.rejects(lambda x: x[1]['q'].__setitem__((200, 7), -.00002))

    def test_reject_changed_release_schedule(self):
        self.rejects(lambda x: x[3]['release_schedule']['targets_rad'].__setitem__(10, .7))

    def test_reject_erased_initial_actual_velocity(self):
        self.rejects(lambda x: x[3]['release_schedule'].update(initial_actual_dq_rad_s=.2))

    def test_reject_broken_action_cadence(self):
        self.rejects(lambda x: x[2]['after_physics_step'].__setitem__(1, 15))

    def test_reject_unsupported_units(self):
        self.rejects(lambda x: x[0].update(action_interface='binary'))

    def test_reject_promotion(self):
        self.rejects(lambda x: x[3].update(accepted=True))


if __name__ == '__main__':
    unittest.main()
