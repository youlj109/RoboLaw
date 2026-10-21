"""Independently validate lime contact records, all trajectory/RGB arrays and videos.

Native success evidence and the collector's physics-rate audit remain required.
This validator does not rerun PhysX or certify continuous-time contact forces.
"""
import argparse
import hashlib
import io
import json
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

import h5py
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation

from kitchen_safety.dataset import (
    CAMERA_NAMES, camera_pose_errors, camera_poses_from_body_states, decode_video)
from kitchen_safety.extensions.lightwheel_pick_place.contacts import LimeGraspContacts
from kitchen_safety.extensions.lightwheel_pick_place.environment import SOURCE, TASK_ID


def validate(out):
    result = json.loads((out / 'result.json').read_text())
    checks, details = {}, {}
    checks['original_task_success'] = (result.get('success') is True
        and result.get('terminal_terms', {}).get('success') is True
        and result.get('final_live_success') is True)
    checks['current_destination_predicate'] = result.get('final_on_destination') is True
    destination_forces = np.asarray(result.get('final_destination_contact_force_matrix_w_n', []), dtype=float)
    checks['current_destination_contact_evidence'] = (destination_forces.shape == (1, 1, 1, 3)
        and bool(np.isfinite(destination_forces).all()))
    # This source YAML leaves the native task defaults unchanged: 0.1 N,
    # pi/4 upward support cone, and speed strictly below 0.003 m/s.
    force = destination_forces.reshape(-1, 3)[0] if checks['current_destination_contact_evidence'] else None
    checks['current_upward_support_force'] = (force is not None
        and np.linalg.norm(force) >= .1 and force[2] > 0
        and force[2] >= np.linalg.norm(force) * np.cos(np.pi / 4))
    checks['source_task_identity'] = (result.get('task') == TASK_ID
        and result.get('source_yaml_sha256') == hashlib.sha256(SOURCE.read_bytes()).hexdigest())
    checks['no_physics_attachment'] = result.get('simulation_attachment') is False
    checks['collector_no_error'] = not result.get('error')
    records = json.loads((out / 'contacts.json').read_text())
    contacts = LimeGraspContacts.__new__(LimeGraspContacts)
    contacts.records = records
    body_range = [-.001, .061]
    close = contacts.summarize(body_range, 'close', last_actions=5)
    lift = contacts.summarize(body_range, 'lift', last_actions=5)
    carried = contacts.carried_region(body_range)
    details['recomputed_contacts'] = {'close': close, 'lift': lift, 'carried': carried}
    checks['physical_close_contacts'] = close['sampled_actions'] == 5 and close['both_fingers_near_body']
    checks['physical_lift_contacts'] = lift['sampled_actions'] == 5 and lift['both_fingers_on_body']
    checks['carried_contacts_on_body'] = carried['passed']

    with h5py.File(out / 'trajectory.h5') as trajectory, h5py.File(out / 'images.h5') as images:
        n = len(trajectory['action'])
        dt = float(trajectory.attrs['dt'])
        phases = [value.decode() if isinstance(value, bytes) else str(value)
                  for value in trajectory['phase'][:]]
        checks['array_shapes'] = (n > 0 and trajectory['action'].shape == (n, 8)
            and len(trajectory['time']) == n and len(phases) == n)
        checks['result_frame_count'] = result.get('frames') == n
        checks['timeline'] = dt > 0 and np.allclose(trajectory['time'][:], np.arange(n) * dt)
        required = ('q', 'dq', 'tcp', 'tcp_quat_xyzw', 'lime', 'wooden_bowl',
                    'robot_body_pose_w', 'robot_contact_forces')
        if result['variant'] == 'property':
            required += ('hazard_pose',)
        arrays, shapes, finite = {}, {}, {}
        for group in ('obs', 'next_obs'):
            for name in sorted(set(required) | set(trajectory[group].keys())):
                key = group + '/' + name
                if key not in trajectory:
                    shapes[key], finite[key] = False, False
                    continue
                value = trajectory[key][:]
                arrays[key] = value
                shapes[key] = value.ndim >= 2 and len(value) == n
                finite[key] = bool(np.isfinite(value).all())
        checks['all_state_array_lengths'] = all(shapes.values())
        checks['finite'] = bool(np.isfinite(trajectory['action'][:]).all()
                               and np.isfinite(trajectory['time'][:]).all() and all(finite.values()))
        details['state_shapes'] = {key: list(value.shape) for key, value in arrays.items()}
        if not checks['all_state_array_lengths']:
            raise ValueError('Required state arrays are missing or do not match the action frame count')
        checks['state_dimensions'] = all(
            arrays[group + '/' + name].shape[1:] == shape
            for group in ('obs', 'next_obs') for name, shape in {
                'q': (len(result['robot_joint_names']),),
                'dq': (len(result['robot_joint_names']),), 'tcp': (3,),
                'tcp_quat_xyzw': (4,), 'lime': (13,), 'wooden_bowl': (13,),
                'robot_body_pose_w': (len(result['robot_body_names']), 7),
                'robot_contact_forces': (len(result['robot_body_names']), 3),
            }.items())
        checks['state_continuity'] = all(np.allclose(arrays['next_obs/' + name][:-1],
            arrays['obs/' + name][1:], atol=1e-6, rtol=1e-6)
            for name in ('q', 'lime', 'wooden_bowl', 'tcp', 'robot_body_pose_w'))
        lift_indices = [i for i, phase in enumerate(phases) if phase == 'lift']
        actual_lift = (float(arrays['next_obs/lime'][lift_indices[-1], 2])
                      - float(result['initial_lime_pose'][2])) if lift_indices else None
        details['measured_lift_m'] = actual_lift
        checks['physical_lift'] = (actual_lift is not None and actual_lift > .08
            and result.get('lift_m', 0) > .08 and checks['physical_lift_contacts']
            and abs(actual_lift - result['lift_m']) < 1e-5)
        finger = result['robot_joint_names'].index('finger_joint')
        checks['released_at_end'] = n > 0 and abs(arrays['next_obs/q'][-1, finger]) < .05
        checks['current_destination_low_speed'] = n > 0 and np.linalg.norm(arrays['next_obs/lime'][-1, 7:10]) < .003
        checks['final_state_matches_result'] = n > 0 and 'final_lime' in result and np.allclose(
            arrays['next_obs/lime'][-1], result.get('final_lime', []), atol=1e-6, rtol=1e-6)

        contact_indices, local_errors, contact_valid = [], [], True
        for row in records:
            index = row.get('action_index')
            if not isinstance(index, int) or not 0 <= index < n or row.get('phase') != phases[index]:
                contact_valid = False
                continue
            contact_indices.append(index)
            pose = arrays['next_obs/lime'][index, :7]
            rotation = Rotation.from_quat(pose[3:7]).as_matrix()
            for name in contacts.fingers:
                contact = row['fingers'][name]
                fields = {key: np.asarray(contact[key], dtype=float) for key in (
                    'points_world_m', 'points_lime_local_m', 'normal_forces_n',
                    'normals_world', 'separations_m')}
                count = contact['count']
                contact_valid &= all(len(value) == count and np.isfinite(value).all()
                                     for value in fields.values())
                if count:
                    expected = (fields['points_world_m'].reshape(-1, 3) - pose[:3]) @ rotation
                    error = float(np.max(np.linalg.norm(
                        expected - fields['points_lime_local_m'].reshape(-1, 3), axis=1)))
                    local_errors.append(error)
        expected_indices = {i for i, phase in enumerate(phases)
                            if phase in ('close', 'lift', 'transfer', 'lower')}
        checks['contact_action_alignment'] = (contact_valid
            and len(contact_indices) == len(set(contact_indices))
            and expected_indices.issubset(set(contact_indices))
            and max(local_errors, default=0.) < 1e-5)
        details['contact_local_position_max_error_m'] = max(local_errors, default=None)
        checks['physics_rate_audit'] = result.get('physics_safety_checks', 0) >= n * 8
        audit = json.loads((out / 'full_world_audit.json').read_text())
        measurements = audit.get('measurements', [])
        audit_counts = np.zeros(n, dtype=int)
        last_measurement = {}
        audit_valid = bool(measurements)
        arm_names = [f'panda_joint{i}' for i in range(1, 8)]
        arm_ids = [result['robot_joint_names'].index(name) for name in arm_names]
        finger_names = set(result['robot_joint_names']) - set(arm_names)
        for item in measurements:
            index = item.get('action')
            if not isinstance(index, int) or not 0 <= index < n or item.get('phase') != phases[index]:
                audit_valid = False
                continue
            audit_counts[index] += 1
            last_measurement[index] = item
            numeric = [item.get(key, float('nan')) for key in ('distance_m', 'required_m', 'slack_m')]
            joints = np.asarray(item.get('q_arm', []), dtype=float)
            pose = np.asarray(item.get('lime_pose', []), dtype=float)
            gripper = item.get('gripper_q', {})
            audit_valid &= (item.get('valid') is True and item.get('obstacle') is not None
                and np.isfinite(numeric).all() and numeric[2] >= 0
                and joints.shape == (7,) and np.isfinite(joints).all()
                and pose.shape == (7,) and np.isfinite(pose).all()
                and set(gripper) == finger_names and np.isfinite(list(gripper.values())).all())
        checks['full_world_audit'] = (audit.get('passed') is True and audit_valid
            and len(measurements) >= n * 8 and bool((audit_counts >= 8).all()))
        checks['full_world_audit_state_alignment'] = len(last_measurement) == n and all(
            np.allclose(item['q_arm'], arrays['next_obs/q'][index, arm_ids], atol=1e-6, rtol=1e-6)
            and np.allclose(item['lime_pose'], arrays['next_obs/lime'][index, :7], atol=1e-6, rtol=1e-6)
            and all(abs(angle - arrays['next_obs/q'][index, result['robot_joint_names'].index(name)]) < 1e-6
                    for name, angle in item['gripper_q'].items())
            for index, item in last_measurement.items())
        details['full_world_audit'] = {'measurements': len(measurements),
            'minimum_checks_per_action': int(audit_counts.min()) if n else 0,
            'maximum_checks_per_action': int(audit_counts.max()) if n else 0,
            'scope': audit.get('scope')}
        if 'lift_support_contact_model' in result:
            from audit_source_support import audit as audit_source_support
            support = audit_source_support(out)
            (out / 'source_support_audit.json').write_text(json.dumps(support, indent=2))
            checks['measured_source_support_bounds'] = support.get('passed') is True
            details['source_support'] = {key: value for key, value in support.items() if key != 'measurements'}
        plans = [item for item in result.get('plans', []) if item.get('executed', True)]
        checks['all_plans_valid'] = bool(plans) and all(
            item.get('validation', {}).get('valid') is True
            and item.get('approach_lookahead', {'valid': True}).get('valid') is True for item in plans)

        metadata = json.loads((out / 'cameras.json').read_text())
        checks['frame_alignment'] = int(images.attrs['frames']) == n and all(
            len(images[name]['jpeg']) == n and len(images[name]['position_world']) == n
            and len(images[name]['quaternion_world_ros_xyzw']) == n for name in CAMERA_NAMES)
        expected = camera_poses_from_body_states(arrays['obs/robot_body_pose_w'],
                                                result['robot_body_names'], metadata)
        details['cameras'] = {}
        jpeg_sizes_ok = True
        for name in CAMERA_NAMES:
            for blob in images[name]['jpeg']:
                with Image.open(io.BytesIO(blob.tobytes())) as frame:
                    frame.load()
                    jpeg_sizes_ok &= frame.size == (metadata[name]['width'], metadata[name]['height'])
            positions = images[name]['position_world'][:]
            quaternions = images[name]['quaternion_world_ros_xyzw'][:]
            errors = camera_pose_errors(positions, quaternions, expected[name])
            details['cameras'][name] = errors
            checks['camera_finite_' + name] = bool(np.isfinite(positions).all() and np.isfinite(quaternions).all())
        checks['jpeg_decode_and_dimensions'] = jpeg_sizes_ok
        checks['camera_world_poses'] = all(item['position_max_error_m'] < .001
            and item['rotation_max_error_deg'] < .1 for item in details['cameras'].values())
        details['videos'] = {name: decode_video(out / (name + '.mp4'), n,
            expected_dt=dt, expected_size=(1920, 360) if name == 'all_cameras' else
            (metadata[name]['width'], metadata[name]['height']))
            for name in (*CAMERA_NAMES, 'all_cameras')}
        checks['videos'] = all(value == n for value in details['videos'].values())
        if result['variant'] == 'human':
            checks['human_overview_present'] = isinstance(result.get('overview_camera'), dict)
        if result.get('overview_camera'):
            spec = result['overview_camera']
            extra = out / 'overview'
            extra_metadata = json.loads((extra / 'cameras.json').read_text())
            alignment = json.loads((extra / 'frame_alignment.json').read_text())['measurements']
            sync = len(alignment) == n
            for i, item in enumerate(alignment):
                sync &= (i < n and item.get('action_index') == i
                    and item.get('primary_frame_index') == i and item.get('overview_frame_index') == i
                    and item.get('phase') == phases[i]
                    and isinstance(item.get('physics_step_before'), int)
                    and item.get('physics_step_before') == item.get('physics_step_after'))
            times = np.array([item.get('simulation_time_s', float('nan')) for item in alignment])
            checks['overview_frame_synchronization'] = (sync and np.isfinite(times).all()
                and np.allclose(times - times[0], trajectory['time'][:], atol=1e-6, rtol=1e-6))
            overview_ok = set(extra_metadata) == {'overview_camera'}
            camera = extra_metadata['overview_camera']
            with h5py.File(extra / 'images.h5') as extra_images:
                group = extra_images['overview_camera']
                overview_ok &= (int(extra_images.attrs['frames']) == n
                    and abs(float(extra_images.attrs['dt']) - dt) < 1e-9
                    and len(group['jpeg']) == n and group['position_world'].shape == (n, 3)
                    and group['quaternion_world_ros_xyzw'].shape == (n, 4)
                    and (camera['width'], camera['height']) == (1280, 720))
                for blob in group['jpeg']:
                    with Image.open(io.BytesIO(blob.tobytes())) as frame:
                        frame.load()
                        overview_ok &= frame.size == (1280, 720)
                positions = group['position_world'][:]
                quaternions = group['quaternion_world_ros_xyzw'][:]
                ros_rotation = (Rotation.from_quat(spec['quaternion_opengl_xyzw'])
                                * Rotation.from_quat([1, 0, 0, 0]))
                expected_overview = {'position_world': np.tile(spec['position_world'], (n, 1)),
                    'quaternion_world_ros_xyzw': np.tile(ros_rotation.as_quat(), (n, 1))}
                overview_pose = camera_pose_errors(positions, quaternions, expected_overview)
                overview_ok &= (np.isfinite(positions).all() and np.isfinite(quaternions).all()
                    and overview_pose['position_max_error_m'] < .001
                    and overview_pose['rotation_max_error_deg'] < .1)
            checks['overview_jpeg_arrays_and_pose'] = overview_ok
            root_video = out / 'overview_camera.mp4'
            overview_frames = decode_video(root_video, n, expected_dt=dt, expected_size=(1280, 720))
            panel_frames = decode_video(extra / 'all_cameras.mp4', n,
                expected_dt=dt, expected_size=(640, 360))
            checks['overview_videos'] = (overview_frames == n and panel_frames == n
                and hashlib.sha256(root_video.read_bytes()).digest()
                    == hashlib.sha256((extra / 'overview_camera.mp4').read_bytes()).digest())
            details['overview'] = {'frames': overview_frames, 'panel_frames': panel_frames,
                'pose_errors': overview_pose, 'alignment_records': len(alignment),
                'scope': 'Supplementary fixed camera captured before the same actions. '
                         'Original three policy camera checks remain required. '
                         'Human/task visibility requires separate visual review.'}
        if result['variant'] == 'property':
            poses = arrays['next_obs/hazard_pose']
            initial = arrays['obs/hazard_pose'][0]
            displacement = float(np.linalg.norm(poses[:, :3] - initial[:3], axis=1).max())
            ups = Rotation.from_quat(poses[:, 3:7]).apply(np.tile([0., 0., 1.], (n, 1)))
            initial_up = Rotation.from_quat(initial[3:7]).apply([0., 0., 1.])
            tilt = float(np.rad2deg(np.arccos(np.clip(ups @ initial_up, -1, 1))).max())
            details['property_max_displacement_m'] = displacement
            details['property_max_tilt_deg'] = tilt
            checks['property_stable'] = displacement < .01 and tilt < 5.

    if result['variant'] != 'base':
        clearances = json.loads((out / 'clearances.json').read_text())
        checks['hazard_clearance'] = (len(clearances) == result.get('clearance_checks')
            and len(clearances) >= n * 8 and all(item.get('valid') is True
            and item.get('obstacle') is not None
            and np.isfinite(item.get('distance_m', float('nan')))
            and item.get('slack_m', -1) >= 0 for item in clearances)
            and result.get('minimum_clearance', {}).get('valid') is True)
    return {'passed': all(checks.values()), 'checks': {key: bool(value) for key, value in checks.items()},
            'details': details, 'frames': n, 'dt': dt,
            'scope': 'Independent raw contact, RGB, state and video validation. Native '
                     'success and collector physics-rate audit are preserved; this does not rerun PhysX.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episode', type=Path)
    args = parser.parse_args()
    try:
        report = validate(args.episode)
    except Exception as exc:
        report = {'passed': False, 'error': f'{type(exc).__name__}: {exc}'}
    report['runtime_versions'] = {}
    for name in ('numpy', 'scipy', 'h5py', 'Pillow'):
        try:
            report['runtime_versions'][name] = version(name)
        except PackageNotFoundError:
            report['runtime_versions'][name] = None
    (args.episode / 'validation.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
