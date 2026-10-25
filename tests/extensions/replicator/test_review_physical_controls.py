import hashlib
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from scripts.extensions.replicator.review_physical_controls import hazard_motion, mobility_measurements


def mobility_fixture():
    paths = ['/Fridge/door_left_01_obj_00', '/Fridge/door_right_01_obj_00', '/Fridge/body_01_obj_00']
    phases = ['neutral']*30+['known_external_torque']*30+['coasting_after_torque']*30
    physical = []
    for j in range(91):
        angle = min(max(j-30, 0), 30)/30
        pose = np.zeros((3, 7))
        pose[:, 3:] = Rotation.from_euler('z', np.array([15*angle, -12*angle, 0])[:, None], degrees=True).as_quat()
        physical.append(dict(physics_step=j, action_index=j-1, phase=phases[j-1] if j else 'reset_initial',
                             poses=pose.tolist(), velocities=np.zeros((3, 6)).tolist(), robot_q=[0.]*7))
    rows, calls = [], []
    for i in range(90):
        rows.append(dict(i=i, time_s=i/15, phase=phases[i], physics_step=i,
                         **{key: physical[i][key] for key in ['poses', 'velocities', 'robot_q']}))
        torque = np.zeros((3, 3), dtype=np.float32)
        if phases[i] == 'known_external_torque':
            torque[0, 2], torque[1, 2] = .3, -.3
        calls.append(dict(call_index=i, physics_step_before=i, action_index=i, phase=phases[i],
                          body_indices=[0, 1, 2], force_world_n=np.zeros((3, 3)).tolist(), torque_world_nm=torque.tolist(),
                          position_data=None, is_global=True))
    changes = {}
    poses = np.asarray([row['poses'] for row in rows])
    for name, index in [('left', 0), ('right', 1), ('housing', 2)]:
        angle = np.degrees((Rotation.from_quat(poses[:, index, 3:])*Rotation.from_quat(poses[0, index, 3:]).inv()).magnitude())
        changes[name] = dict(initial_hold_max_rotation_deg=float(angle[:31].max()),
                             rotation_at_pulse_end_deg=float(angle[60]), maximum_rotation_deg=float(angle.max()),
                             maximum_translation_m=0.)
    source = 'record actual native calls'
    digest = hashlib.sha256(source.encode()).hexdigest()
    states = dict(paths=paths, rows=rows, physical_rows=physical, reset_physics_step=0, final_physics_step=90)
    trace = dict(schema_version=1, paths=paths, target_body_indices=dict(left=0, right=1, housing=2), calls=calls,
                 reset_physics_step=0, final_physics_step=90, action_dt=1/15, physics_dt=1/15, decimation=1,
                 source_text=source, initial_source_sha256=digest, checked_source_sha256=digest)
    report = dict(schema_version=2, paths=paths, rows=rows, dt=1/15, physics_dt=1/15, force_applications=90,
                  source_sha256=digest, changes=changes)
    # Production uses i*dt, preserving its exact floating-point action clock.
    for row in rows:
        row['time_s'] = row['i']*report['dt']
    return states, trace, report


class HazardTests(unittest.TestCase):
    def test_reset_reference_rejects_accumulated_prefix_and_suffix_drift(self):
        poses = np.zeros((3, 7)); poses[:, 6] = 1; poses[:, 0] = [0, .0015, .003]
        self.assertTrue(hazard_motion(poses[:2])['passed'])
        self.assertTrue(hazard_motion(poses[1:])['passed'])
        self.assertFalse(hazard_motion(poses)['passed'])

    def test_strict_original_translation_boundary(self):
        poses = np.zeros((2, 7)); poses[:, 6] = 1; poses[1, 0] = .002
        self.assertFalse(hazard_motion(poses)['passed'])
        poses[1, 0] = np.nextafter(.002, 0)
        self.assertTrue(hazard_motion(poses)['passed'])

    def test_absolute_upright_and_nonfinite_rejected(self):
        poses = np.zeros((2, 7)); poses[:, 3:] = Rotation.from_euler('x', 3, degrees=True).as_quat()
        self.assertFalse(hazard_motion(poses)['passed'])
        poses[0, 0] = np.nan
        with self.assertRaises(ValueError): hazard_motion(poses)


class MobilityTests(unittest.TestCase):
    def test_real_trace_arithmetic(self):
        self.assertTrue(mobility_measurements(*mobility_fixture())['passed'])

    def test_missing_wrong_wrench_or_identity_fails(self):
        for field, value in [('physics_step_before', 19), ('body_indices', [1, 0, 2]),
                             ('phase', 'neutral'), ('torque_world_nm', [[0., 0., 0.]]*3)]:
            with self.subTest(field=field):
                states, trace, report = mobility_fixture(); trace['calls'][40][field] = value
                with self.assertRaises(ValueError): mobility_measurements(states, trace, report)
        states, trace, report = mobility_fixture(); trace['calls'].pop()
        with self.assertRaises(ValueError): mobility_measurements(states, trace, report)

    def test_green_claim_does_not_replace_raw_physics(self):
        states, trace, report = mobility_fixture()
        # Only an unobserved final physical substep moves the housing.
        states['physical_rows'][-1]['poses'][2][0] = .002
        self.assertFalse(mobility_measurements(states, trace, report)['passed'])
        states, trace, report = mobility_fixture(); report['changes']['left']['rotation_at_pulse_end_deg'] = 30.
        with self.assertRaises(ValueError): mobility_measurements(states, trace, report)

    def test_source_change_or_missing_new_trace_rejected(self):
        states, trace, report = mobility_fixture(); trace['source_text'] += ' changed'
        with self.assertRaises(ValueError): mobility_measurements(states, trace, report)
        states, trace, report = mobility_fixture(); report['schema_version'] = 1
        with self.assertRaises(ValueError): mobility_measurements(states, trace, report)


if __name__ == '__main__':
    unittest.main()
