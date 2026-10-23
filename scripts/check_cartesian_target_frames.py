"""Check control-target frame semantics without starting physics or risk queries."""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.task_cartesian_policy import pose_matrix, resolve_target_pose


def main():
    reference = pose_matrix([.4, -.2, .9, *Rotation.from_euler('xyz', [.2, -.4, .7]).as_quat()])
    base = pose_matrix([-.1, .3, .8, *Rotation.from_euler('z', -1.57).as_quat()])
    quaternion = Rotation.from_euler('xyz', [.3, .5, -.2]).as_quat().tolist()
    offset = [.02, -.01, .12]
    count = 0
    for name, matrix in [('world', np.eye(4)), ('reference', reference), ('robot_base', base)]:
        target = dict(reference='current_tcp', offset_m=offset, rotation_reference=name, rotation_xyzw=quaternion)
        actual = resolve_target_pose(target, reference, base, 'robolaw-cartesian-keyframes-2')
        np.testing.assert_allclose(actual[:3, 3], reference[:3, 3] + reference[:3, :3] @ offset)
        np.testing.assert_allclose(actual[:3, :3], matrix[:3, :3] @ Rotation.from_quat(quaternion).as_matrix())
        count += 1
    old = dict(reference='current_tcp', offset_m=offset, rotation_world_xyzw=quaternion)
    new = dict(reference='current_tcp', offset_m=offset, rotation_reference='world', rotation_xyzw=quaternion)
    np.testing.assert_array_equal(resolve_target_pose(old, reference, base, 'robolaw-cartesian-keyframes-1'),
                                  resolve_target_pose(new, reference, base, 'robolaw-cartesian-keyframes-2'))
    invalid = [dict(new, rotation_reference='camera'), dict(new, rotation_xyzw=[0, 0, 0, 0]),
               dict(new, rotation_xyzw=[0, 0, 0, 2]), dict(new, rotation_xyzw=[0, 0, float('nan'), 1]),
               dict(new, offset_m=[0, 0]), dict(new, offset_m=[0, 0, float('inf')]),
               dict(new, rotation_world_xyzw=quaternion), old]
    for target in invalid:
        try:
            resolve_target_pose(target, reference, base, 'robolaw-cartesian-keyframes-2')
        except ValueError:
            continue
        raise AssertionError('Invalid target was accepted: ' + repr(target))
    print(json.dumps(dict(passed=True, orientation_frames_checked=count, legacy_world_orientation_preserved=True,
                          invalid_targets_rejected=len(invalid), physics_started=False, risk_queries=0)))


if __name__ == '__main__':
    main()
