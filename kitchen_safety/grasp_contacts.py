"""Measure actual finger/bottle contacts in bottle coordinates during collection."""
import numpy as np
from scipy.spatial.transform import Rotation


def _numpy(value):
    value = value.torch if hasattr(value, 'torch') else value
    if hasattr(value, 'detach'):
        return value.detach().cpu().numpy().copy()
    return np.asarray(value.numpy() if hasattr(value, 'numpy') else value).copy()


class BottleGraspContacts:
    """Read PhysX contact points; this does not create forces or attachments."""

    fingers = ('left_inner_finger', 'right_inner_finger')

    def __init__(self, physics_sim_view, physics_dt, *, env_root='/World/envs/env_0', env_origin=None):
        self.physics_dt = physics_dt
        self.env_origin = np.zeros(3) if env_origin is None else np.asarray(env_origin, dtype=float)
        if self.env_origin.shape != (3,) or not np.isfinite(self.env_origin).all():
            raise ValueError('Environment origin must be a finite xyz vector')
        self.views = {
            name: physics_sim_view.create_rigid_contact_view(
                [f'{env_root}/Robot/Gripper/Robotiq_2F_85/{name}'],
                filter_patterns=[[f'{env_root}/mustard_bottle']],
                max_contact_data_count=4096)
            for name in self.fingers
        }
        self.records = []

    def sample(self, phase, action_index, bottle_pose):
        rotation = Rotation.from_quat(bottle_pose[3:7]).as_matrix()
        row = {'phase': phase, 'action_index': action_index, 'fingers': {}}
        for name, view in self.views.items():
            forces, points, normals, separations, counts, starts = map(
                _numpy, view.get_contact_data(dt=self.physics_dt))
            count, start = int(counts.reshape(-1)[0]), int(starts.reshape(-1)[0])
            if start + count > len(points):
                raise RuntimeError('Finger/bottle contact buffer was truncated')
            indices = slice(start, start + count)
            points = points[indices].reshape(-1, 3)
            local = (points - self.env_origin - np.asarray(bottle_pose[:3])) @ rotation
            row['fingers'][name] = {
                'count': count,
                'points_world_m': points.tolist(),
                'points_bottle_local_m': local.tolist(),
                'normal_forces_n': forces[indices].reshape(-1).tolist(),
                'separations_m': separations[indices].reshape(-1).tolist(),
            }
        self.records.append(row)
        return row

    def summarize(self, body_z_range, phase, last_actions=10, near_surface_m=.00025):
        """Check both fingers in each sampled action, then summarize the window.

        Closing a supported bottle need not create a positive normal load on
        both fingers. ``both_fingers_near_body`` checks actual contact points
        within 0.25 mm of the body surface. ``both_fingers_on_body`` retains the
        positive-load requirement, for use after lifting. Each boolean requires
        both sides together in every sampled action of the requested window;
        contacts from different actions cannot be combined into a valid grasp.
        """
        if last_actions < 1 or near_surface_m < 0:
            raise ValueError('Contact window must be positive and near-surface distance nonnegative')
        rows = [row for row in self.records if row['phase'] == phase][-last_actions:]
        frames, aggregate = [], {name: {'points': [], 'forces': [], 'separations': [],
                                       'touching_frames': 0} for name in self.fingers}
        for row in rows:
            frame = {'action_index': row['action_index'], 'fingers': {}}
            for name in self.fingers:
                contact = row['fingers'][name]
                points = np.asarray(contact['points_bottle_local_m'], dtype=float).reshape(-1, 3)
                forces = np.asarray(contact['normal_forces_n'], dtype=float).reshape(-1)
                separations = np.asarray(contact['separations_m'], dtype=float).reshape(-1)
                if not len(points) == len(forces) == len(separations) == contact['count']:
                    raise RuntimeError('Finger contact array counts do not agree')
                if not all(np.isfinite(value).all() for value in (points, forces, separations)):
                    raise RuntimeError('Non-finite finger contact data')
                inside = (points[:, 2] >= body_z_range[0]) & (points[:, 2] <= body_z_range[1])
                loaded = forces > .01
                near = separations <= near_surface_m
                frame['fingers'][name] = {
                    'loaded_contact_count': int(loaded.sum()),
                    'near_contact_count': int(near.sum()),
                    'all_loaded_contacts_on_body': bool(loaded.any() and inside[loaded].all()),
                    'all_near_contacts_on_body': bool(near.any() and inside[near].all()),
                }
                aggregate[name]['points'].extend(points.tolist())
                aggregate[name]['forces'].extend(forces.tolist())
                aggregate[name]['separations'].extend(separations.tolist())
                aggregate[name]['touching_frames'] += int(contact['count'] > 0)
            frame['both_fingers_near_body'] = all(item['all_near_contacts_on_body']
                                                   for item in frame['fingers'].values())
            frame['both_fingers_loaded_on_body'] = all(item['all_loaded_contacts_on_body']
                                                        for item in frame['fingers'].values())
            frames.append(frame)
        fingers = {}
        for name, collected in aggregate.items():
            points = np.asarray(collected['points'], dtype=float).reshape(-1, 3)
            forces = np.asarray(collected['forces'], dtype=float)
            separations = np.asarray(collected['separations'], dtype=float)
            loaded, near = forces > .01, separations <= near_surface_m
            z = points[:, 2]
            inside = (z >= body_z_range[0]) & (z <= body_z_range[1])
            fingers[name] = {
                'touching_frames': collected['touching_frames'],
                'loaded_contact_count': int(loaded.sum()),
                'loaded_z_range_m': [float(z[loaded].min()), float(z[loaded].max())] if loaded.any() else None,
                'loaded_force_sum_n': float(forces[loaded].sum()),
                'all_loaded_contacts_on_body': bool(loaded.any() and inside[loaded].all()),
                'near_contact_count': int(near.sum()),
                'near_z_range_m': [float(z[near].min()), float(z[near].max())] if near.any() else None,
                'all_near_contacts_on_body': bool(near.any() and inside[near].all()),
            }
        return {
            'phase': phase, 'sampled_actions': len(rows),
            'body_local_z_range_m': list(body_z_range),
            'near_surface_distance_m': float(near_surface_m),
            'positive_load_threshold_n': .01,
            'source': 'PhysX inner-finger contacts transformed by the measured bottle pose',
            'window_rule': 'Both fingers must satisfy the same condition simultaneously in every sampled action.',
            'fingers': fingers, 'frames': frames,
            'simultaneous_near_body_actions': sum(frame['both_fingers_near_body'] for frame in frames),
            'simultaneous_loaded_body_actions': sum(frame['both_fingers_loaded_on_body'] for frame in frames),
            'both_fingers_near_body': bool(frames and all(frame['both_fingers_near_body'] for frame in frames)),
            'both_fingers_on_body': bool(frames and all(frame['both_fingers_loaded_on_body'] for frame in frames)),
        }

    def carried_region(self, body_z_range):
        """Check loaded contact locations over the recorded carried motion.

        Dynamic motion can unload one finger. Do not require bilateral force
        at every motion sample; check where every measured loaded contact is.
        The separate settled lift check establishes bilateral support.
        """
        phases = ('lift', 'transfer_front', 'transfer', 'above_bowl', 'lower')
        loaded_count = 0
        outside = []
        extrema = {}
        for row in self.records:
            if row['phase'] not in phases:
                continue
            for finger, contact in row['fingers'].items():
                for point, force in zip(contact['points_bottle_local_m'], contact['normal_forces_n']):
                    if force <= .01:
                        continue
                    loaded_count += 1
                    bounds = extrema.setdefault(finger, [point[2], point[2]])
                    bounds[0], bounds[1] = min(bounds[0], point[2]), max(bounds[1], point[2])
                    if not body_z_range[0] <= point[2] <= body_z_range[1]:
                        outside.append({'action_index': row['action_index'], 'phase': row['phase'],
                                        'finger': finger, 'point_bottle_local_m': point,
                                        'normal_force_n': force})
        return {'scope': 'Loaded contact locations at recorded action samples, not continuous-time force certification',
                'body_local_z_range_m': list(body_z_range), 'loaded_contact_count': loaded_count,
                'loaded_z_range_by_finger_m': extrema, 'outside_body_contacts': outside,
                'passed': bool(loaded_count and not outside)}
