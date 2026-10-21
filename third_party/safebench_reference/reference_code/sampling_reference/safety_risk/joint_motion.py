"""Unit-aware robot rest evidence, separate from the frozen risk features."""
import math


def joint_speed_summary(velocities, channels):
    angular, linear, seen, errors = [], [], set(), []
    expected = {(name, i) for name, row in velocities.items()
                for i in range(len(row))} if all(isinstance(r, (list, tuple)) for r in velocities.values()) else set()
    for c in channels:
        key = (c['robot_name'], c['local_dof_index'])
        row = velocities.get(key[0])
        value = row[key[1]] if isinstance(row, (list, tuple)) and key[1] < len(row) else None
        if key in seen:
            errors.append('duplicate_channel')
        seen.add(key)
        if type(value) not in (int, float) or not math.isfinite(value):
            errors.append('missing_velocity')
            continue
        if c.get('joint_type_verified') is not True:
            errors.append('unverified_joint_type')
        elif c.get('joint_type') == 'revolute':
            angular.append(abs(value))
        elif c.get('joint_type') == 'prismatic':
            linear.append(abs(value))
        else:
            errors.append('unsupported_joint_type')
    if not expected or expected != seen:
        errors.append('incomplete_joint_axis')
    complete = not errors
    return {'complete': complete, 'errors': sorted(set(errors)),
            'max_angular_speed_rad_s': max(angular, default=0.0) if complete else None,
            'max_linear_speed_m_s': max(linear, default=0.0) if complete else None}
