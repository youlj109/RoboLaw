"""Repair stale camera extrinsics from recorded pre-action body poses and mounts.

Reads only completed episodes. --apply preserves original pose datasets inside
each repaired camera group and replaces only its world pose arrays. RGB JPEGs,
camera mounts, intrinsics, trajectory states, and physical results are untouched.
The replacement is derived metadata, not a newly measured renderer pose.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys

import h5py
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from kitchen_safety.dataset import CAMERA_NAMES, camera_pose_errors, camera_poses_from_body_states

BACKUP = 'original_pose_before_refresh_fix'
SOURCE = 'Derived from same-frame trajectory.h5 obs/robot_body_pose_w and cameras.json fixed mount; camera-to-world ROS xyzw'


def repair_episode(root, apply=False):
    root = Path(root)
    result = json.loads((root / 'result.json').read_text())
    metadata = json.loads((root / 'cameras.json').read_text())
    with h5py.File(root / 'trajectory.h5', 'r') as trajectory:
        frames = len(trajectory['action'])
        if frames <= 0 or result['steps'] != frames:
            raise ValueError('Episode must have a completed, nonempty trajectory matching result.json')
        expected = camera_poses_from_body_states(
            trajectory['obs/robot_body_pose_w'][:],
            json.loads(trajectory.attrs['robot_body_names']), metadata)
    report = {'path': str(root), 'apply_requested': apply, 'frames': frames,
              'created_at': datetime.now(timezone.utc).isoformat(), 'cameras': {},
              'source': SOURCE,
              'limits': 'Reconstructed from saved nominal mount, not renderer temporal state; no pixel-identical replay claim.'}
    with h5py.File(root / 'images.h5', 'r+' if apply else 'r') as images:
        if images.attrs['frames'] != frames:
            raise ValueError('Image and completed trajectory counts disagree')
        for name in CAMERA_NAMES:
            group = images[name]
            before = camera_pose_errors(group['position_world'][:], group['quaternion_world_ros_xyzw'][:], expected[name])
            needs_repair = before['position_max_error_m'] >= .001 or before['rotation_max_error_deg'] >= .1
            item = {'parent_body': expected[name]['parent_body'], 'before_errors': before,
                    'needs_repair': needs_repair, 'repaired': False,
                    'already_has_original_backup': BACKUP in group}
            if needs_repair and apply:
                # Nested groups preserve the top-level three-camera interface.
                # Existing backup arrays survive reruns or interrupted repairs.
                backup = group.require_group(BACKUP)
                backup.attrs['reason'] = 'CameraCfg.update_latest_camera_pose=False returned initialization pose while wrist moved'
                if 'original_world_pose_source' not in backup.attrs:
                    backup.attrs['original_world_pose_source'] = group.attrs.get('world_pose_source', 'legacy Camera.data initialization pose')
                for key in ('position_world', 'quaternion_world_ros_xyzw'):
                    if key not in backup:
                        group.copy(key, backup, name=key)
                images.flush()
                for key in ('position_world', 'quaternion_world_ros_xyzw'):
                    if group[key].shape != expected[name][key].shape:
                        raise ValueError(f'{name}: invalid pose dimensions, not safe to repair')
                    group[key][:] = expected[name][key]
                group.attrs['world_pose_source'] = SOURCE
                group.attrs['world_pose_repair_at'] = report['created_at']
                group.attrs['original_pose_backup_group'] = BACKUP
                item['repaired'] = True
                item['backup_path'] = f'images.h5/{name}/{BACKUP}'
                item['after_errors'] = camera_pose_errors(group['position_world'][:],
                                                          group['quaternion_world_ros_xyzw'][:], expected[name])
                images.flush()
            report['cameras'][name] = item
    report['passed'] = all(not item['needs_repair'] or item['repaired'] for item in report['cameras'].values())
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episodes', nargs='+')
    parser.add_argument('--apply', action='store_true', help='Back up and replace erroneous world pose arrays only')
    parser.add_argument('--report', required=True)
    args = parser.parse_args()
    results = []
    for episode in args.episodes:
        try:
            result = repair_episode(episode, args.apply)
        except Exception as error:
            result = {'path': episode, 'passed': False, 'error': f'{type(error).__name__}: {error}'}
        results.append(result)
    report = Path(args.report)
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(json.dumps(results, indent=2) + '\n')
    print(json.dumps(results, indent=2))
    return 0 if all(result['passed'] for result in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
