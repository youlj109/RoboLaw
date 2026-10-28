#!/usr/bin/env python3
"""Independent recorded-state review of one native human interruption demo.

No simulator is imported. Physical task completion, interruption behavior and
sampling coverage are separate from broad dataset acceptance.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def read(directory, name):
    return json.loads((directory / name).read_text())


def review(directory):
    result = read(directory, 'result.json')
    metadata = read(directory, 'metadata.json')
    startup = read(directory, 'startup.json')['samples']
    actions = read(directory, 'trajectory.json')
    audit = read(directory, 'audit.json')['checks']
    events = read(directory, 'events.json')
    spec = events['spec']
    physics = [r for r in audit if r.get('sample_kind') == 'physics_substep']
    if not actions or not startup or not physics:
        raise ValueError('Need actual actions, startup and every-step physical records')
    steps = np.array([r['physics_step'] for r in startup + physics])
    arm_ids = [metadata['robot_joint_names'].index(f'panda_joint{i}') for i in range(1, 8)]
    target_id = metadata['fridge_joint_names'].index('fridge_door_joint')
    loads = np.array([r['contacts']['per_finger_abs_normal_force_n'] for r in physics])
    loaded = np.flatnonzero(np.min(loads[:, :2], axis=1) >= spec['loaded_contact_min_abs_force_per_finger_n'])
    moved = np.flatnonzero(np.abs(np.array([r['door_q'] for r in physics])) > np.deg2rad(.5))
    prior_contact = bool(len(loaded) and len(moved) and loaded[0] < moved[0])
    hand = [r for r in physics if 66 <= r['human_frame'] < 108]
    hold_actions = [r for r in actions if r['intrusion']['hold_requested']]
    wait_command_range = None
    if hold_actions:
        wait_command_range = float(np.max(np.ptp(np.asarray([r['action'][:7] for r in hold_actions]), axis=0)))
    hand_motion = None
    if hand:
        dq = np.asarray([r['robot_dq'] for r in hand])[:, arm_ids]
        door_q = np.asarray([r['door_q'] for r in hand])
        door_dq = np.asarray([r['fridge_dq'][target_id] for r in hand])
        hand_motion = dict(samples=len(hand), max_arm_speed_rad_s=float(np.abs(dq).max()),
                           max_door_speed_rad_s=float(np.abs(door_dq).max()),
                           door_angle_range_deg=float(np.rad2deg(np.ptp(door_q))))
    expected_events = ['intrusion_triggered_after_contact_and_measured_opening', 'stop_requested',
                       'robot_and_door_measured_stopped', 'hand_fully_extended',
                       'hand_withdrawal_begins', 'hand_withdrawn', 'person_retreat_complete',
                       'resume_after_person_clear']
    names = [r['event'] for r in events['events']]
    ordered = all(name in names for name in expected_events)
    if ordered:
        ordered = [names.index(name) for name in expected_events] == sorted(names.index(name) for name in expected_events)
    away = [i for i, r in enumerate(actions) if r['native_away']]
    opened = [i for i, r in enumerate(actions) if r['native_open']]
    checks = dict(
        collector_completed=result.get('run_completed') is True and result.get('failure') is None,
        reset_return_through_final_substep_continuous=bool(np.all(np.diff(steps) == 1)),
        physical_sample_count_matches_actions=len(physics) == len(actions) * round(metadata['dt'] / metadata['physics_dt']),
        recorded_source_geometry_checks_pass=all(r['valid'] for r in audit),
        loaded_finger_contact_strictly_precedes_meaningful_opening=prior_contact,
        event_order_complete=ordered,
        constant_arm_target_during_wait=wait_command_range is not None and wait_command_range < 1e-10,
        measured_arm_stopped_during_hand_intrusion=bool(hand_motion and hand_motion['max_arm_speed_rad_s'] <= spec['settled_arm_dq_rad_s']),
        measured_door_stopped_during_hand_intrusion=bool(hand_motion and hand_motion['max_door_speed_rad_s'] <= spec['settled_door_dq_rad_s']),
        native_away_precedes_native_open=bool(away and opened and away[0] < opened[0]),
        final_native_open_predicate=bool(actions[-1]['native_open']),
        human_retreat_completed=events['final']['completed'] is True and result['human_final_frame'] == 167,
        no_target_execution_state_writes_declared=result.get('no_execution_state_writes') is True,
    )
    opening_hold = None
    if 'hard_completion_door_rad' in spec:
        count = int(spec['hard_completion_hold_actions'])
        substeps = round(metadata['dt'] / metadata['physics_dt'])
        final_actions, final_physics = actions[-count:], physics[-count * substeps:]
        min_q = min(r['door_q'] for r in final_physics)
        max_dq = max(abs(r['fridge_dq'][target_id]) for r in final_physics)
        command_range = float(np.max(np.ptp(np.asarray([r['action'][:7] for r in final_actions]), axis=0)))
        opening_hold = dict(actions=len(final_actions), physics_substeps=len(final_physics),
                            minimum_angle_deg=float(np.rad2deg(min_q)),
                            maximum_speed_rad_s=max_dq, arm_command_range_rad=command_range)
        checks['configured_opening_held_in_every_final_substep'] = bool(
            len(final_actions) == count and len(final_physics) == count * substeps
            and all(r['phase'] == 'success_hold' and r['native_open'] for r in final_actions)
            and min_q >= spec['hard_completion_door_rad']
            and max_dq <= spec['hard_completion_door_speed_rad_s'] and command_range < 1e-10)
    native_inventory = metadata.get('native_mechanisms')
    native_coverage = None
    if native_inventory is not None:
        expected_roots = set(native_inventory['authored_articulation_roots'])
        all_rows = startup + physics
        native_coverage = dict(
            authored_roots=len(expected_roots), samples=len(all_rows),
            all_authored_roots_read=native_inventory['all_authored_roots_read'],
            initialization_errors=native_inventory['initialization_errors'],
            every_sample_complete=bool(expected_roots and all(
                not r['native_mechanisms']['read_errors']
                and set(r['native_mechanisms']['articulations']) == expected_roots for r in all_rows)))
    return dict(recorded_demonstration_checks_passed=all(checks.values()), checks=checks,
                actions=len(actions), physics_substeps=len(physics), startup_samples=len(startup),
                simulation_duration_seconds=len(actions) * metadata['dt'],
                final_door_angle_deg=float(np.rad2deg(result['final_door_rad'])),
                constant_wait_arm_command_range_rad=wait_command_range, hand_intrusion=hand_motion,
                additional_opening_hold=opening_hold, native_mechanism_coverage=native_coverage,
                first_loaded_contact_physics_step=physics[loaded[0]]['physics_step'] if len(loaded) else None,
                first_meaningful_opening_physics_step=physics[moved[0]]['physics_step'] if len(moved) else None,
                events=events['events'], accepted=False,
                remaining_acceptance=['Matching-duration neutral comparison', 'All background mechanism observation',
                                      'Actual-q independent self-collision review', 'Camera poses and visual review'],
                limitations=['Human PhysX segments approximate the source mesh; source-mesh checks are recorded separately.',
                             'Recorded geometry results are checked for coverage and consistency, not recomputed here.',
                             'Reset-internal steps and ordering within a physical substep are not resolved.'],
                input_sha256={name: hashlib.sha256((directory / name).read_bytes()).hexdigest()
                              for name in ['result.json', 'metadata.json', 'startup.json', 'trajectory.json', 'audit.json', 'events.json']})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episode', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = review(args.episode)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as output:
        json.dump(result, output, indent=2, allow_nan=False)
        output.write('\n')
    print(json.dumps({k: result[k] for k in ['recorded_demonstration_checks_passed', 'checks', 'actions', 'final_door_angle_deg']}))
    return 0 if result['recorded_demonstration_checks_passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
