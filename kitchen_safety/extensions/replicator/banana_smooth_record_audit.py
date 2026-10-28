"""Independent, read-only audit of schema04 radian commands and actual joint bounds.

This never imports the controller/schedule implementation and never promotes a
diagnostic. The recorded applied interval is joined to the following measured
state; source limits and all 13 independently measured joints remain unchanged.
"""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
URDF = ROOT / 'assets/curobo/droid/urdf/franka_robotiq_2f_85_zero.urdf'
URDF_SHA = '0aad7ada239f91360d57376247dc894e9c32a818957ee3099cbb6027ed18c2af'
INTERFACE = 'droid_arm7_and_gripper1_joint_position_radians_v1'


def require(condition, message):
    if not condition:
        raise ValueError(message)


def audit_records(metadata, data, trajectory, result, actions, commands, bounds):
    names = metadata['joint_names']
    controlled = [f'panda_joint{i}' for i in range(1, 8)] + ['finger_joint']
    ids = [names.index(name) for name in controlled]
    require(len(names) == len(set(names)) == 13, 'Complete independent actual joint inventory required')
    require(metadata['action_joint_names'] == controlled and commands['joint_names'] == names and
            commands['controlled_joint_ids'] == ids, 'Native/action joint inventory differs')
    require(metadata['action_interface'] == result['action_interface'] == INTERFACE,
            'Wrong action units/interface')
    require(metadata['action_dt_s'] == 1 / 15 and metadata['physics_dt_s'] == 1 / 120,
            'Original action and physics cadence differs')
    require(hashlib.sha256(URDF.read_bytes()).hexdigest() == URDF_SHA, 'Original URDF changed')
    source = {}
    for node in ET.parse(URDF).getroot().findall('joint'):
        limit = node.find('limit')
        if node.get('name') in names and limit is not None:
            require(node.get('name') not in source, 'Duplicate source joint')
            source[node.get('name')] = [float(limit.get('lower')), float(limit.get('upper'))]
    require(set(source) == set(names), 'Original bounds lack actual joints')
    limits = np.asarray([source[name] for name in names])
    steps = np.asarray(data['physics_step'])
    q, dq = np.asarray(data['q']), np.asarray(data['dq'])
    require(np.array_equal(steps, np.arange(len(steps))) and
            q.shape == dq.shape == (len(steps), 13) and
            np.isfinite(q).all() and np.isfinite(dq).all(), 'Actual states incomplete or nonfinite')
    require(len(actions) * 8 + 1 == len(steps) and len(commands['rows']) + 1 == len(steps) and
            len(bounds['rows']) == len(steps), 'Command, action or bound evidence coverage differs')
    require(commands['unmatched_pending'] is False and commands['passed'] is True and
            commands['initial']['applied_interval'] is False and commands['initial']['physics_step'] == 0 and
            np.array_equal(commands['initial']['q'], q[0]) and
            np.array_equal(commands['initial']['dq'], dq[0]), 'Unbound initial or pending command')
    excess = np.maximum(limits[:, 0] - q, q - limits[:, 1]).clip(min=0).max(axis=1)
    require(bounds['allowance_rad'] == 1e-5 and bounds['passed'] is True and
            np.all(excess <= 1e-5), 'Original actual joint bounds failed')
    for i, row in enumerate(bounds['rows']):
        require(row['physics_step'] == i and row['phase'] == str(data['phase'][i]) and
                row['action_index'] == int(data['action_index'][i]) and
                row['allowance_rad'] == 1e-5 and row['maximum_excess_rad'] == float(excess[i]) and
                row['passed'] is True and row['violations'] == [] and
                row['measured_q_modified'] is False, 'Saved actual bound row differs at ' + str(i))
    require(np.array_equal(trajectory['actions'], [a['action'] for a in actions]) and
            np.array_equal(trajectory['phase'], [a['phase'] for a in actions]), 'Action archive differs')
    release = np.flatnonzero(data['phase'] == 'release')
    require(len(release) == 240, 'Exactly 240 bounded release substeps required')
    boundary = int(result['release_start_physics_step'])
    require(np.array_equal(release, np.arange(boundary + 1, boundary + 241)) and
            boundary + 241 == len(steps) and boundary % 8 == 0 and
            str(data['phase'][boundary]) == 'lower', 'Release boundary or extent differs')
    initial_q, initial_dq = float(q[boundary, ids[-1]]), float(dq[boundary, ids[-1]])
    require(0 <= initial_q <= limits[ids[-1], 1], 'No clamping an out-of-range release start')
    # Independent polynomial form, not the collector's factored implementation.
    targets = [initial_q * (1 - 10 * ((i + 1) / 15) ** 3 +
               15 * ((i + 1) / 15) ** 4 - 6 * ((i + 1) / 15) ** 5)
               if i < 14 else 0. for i in range(30)]
    schedule = result['release_schedule']
    require(schedule['initial_actual_q_rad'] == initial_q and
            schedule['initial_actual_dq_rad_s'] == initial_dq and
            schedule['actions'] == 30 and schedule['physics_steps'] == 240 and
            schedule['open_first_action'] == 14 and schedule['open_first_applied_physics_ordinal'] == 113 and
            np.allclose(schedule['targets_rad'], targets, atol=2e-15, rtol=0), 'Schedule is not bound to actual boundary')
    for index, action in enumerate(actions):
        first, last = index * 8, (index + 1) * 8
        require(action['before_physics_step'] == int(trajectory['before_physics_step'][index]) == first and
                action['after_physics_step'] == int(trajectory['after_physics_step'][index]) == last and
                action['action_interface'] == INTERFACE, 'Action interval/interface differs')
        target = (targets[index - boundary // 8] if action['phase'] == 'release' else
                  math.pi / 4 if action['source_binary_intent'] == 1. else 0.)
        require(action['source_binary_intent'] in (0., 1.) and
                abs(action['gripper_target_rad'] - target) <= 2e-15 and
                action['action'][-1] == float(np.float32(target)), 'Changed release/binary target')
        require(np.all(data['phase'][first + 1:last + 1] == action['phase']) and
                np.all(data['action_index'][first + 1:last + 1] == index), 'Post-step action association differs')
    for i, row in enumerate(commands['rows'], 1):
        index = (i - 1) // 8
        action = np.asarray(actions[index]['action'])
        raw, processed = np.asarray(row['raw_action_rad']), np.asarray(row['processed_gripper_target_rad'])
        staged, native = np.asarray(row['staged_joint_targets_rad']), np.asarray(row['native_joint_targets_rad'])
        require(raw.shape == (8,) and processed.shape == (1,) and staged.shape == native.shape == (13,),
                'Incomplete command arrays')
        checks = dict(raw_action_matches=bool(np.array_equal(raw, action)),
            gripper_processed_identity=bool(processed[0] == action[-1]),
            staged_targets_match=bool(np.array_equal(staged[ids], action)),
            native_targets_match=bool(np.array_equal(native[ids], action)),
            finite_targets=all(np.isfinite(value).all() for value in (raw, processed, staged, native)))
        require(all(checks.values()) and checks == row['checks'] and row['passed'] is True and
                row['before_physics_step'] == i - 1 and row['after_physics_step'] == i and
                row['phase'] == str(data['phase'][i]) and row['action_index'] == index and
                np.array_equal(row['actual_q'], q[i]) and np.array_equal(row['actual_dq'], dq[i]),
                'Applied command/actual next-state mismatch at ' + str(i))
    require(commands == result['command_recording'] and bounds == result['actual_joint_bound_audit'],
            'Result and separate raw command/bound files differ')
    require(result['accepted'] is False and result['physical_trial_passed'] is False and
            result['task_success'] is False and result['diagnostic_only'] is True, 'Diagnostic cannot be promoted')
    return dict(passed=True, accepted=False, physical_trial_passed=False, task_success=False,
        measured_states=len(steps), applied_intervals=len(commands['rows']), actions=len(actions),
        release_steps=len(release), joint_names=names, original_urdf_sha256=URDF_SHA,
        maximum_original_joint_limit_excess_rad=float(excess.max()), allowance_rad=1e-5,
        actual_release_boundary_q_rad=initial_q, actual_release_boundary_dq_rad_s=initial_dq,
        actual_final_finger_q_rad=float(q[-1, ids[-1]]), independently_recomputed_targets_rad=targets,
        scope='Complete original13joint bounds and interval-bound raw/processed/staged/native commands. No CUDA self, physical acceptance, source penetration tolerance or actuator stability claim.')


def audit_episode(episode, metadata, data, trajectory, result):
    def read(name):
        return json.loads((Path(episode) / name).read_text())
    return audit_records(metadata, data, trajectory, result, read('actions.json'),
                         read('actual_command_targets.json'), read('actual_joint_bound_audit.json'))
