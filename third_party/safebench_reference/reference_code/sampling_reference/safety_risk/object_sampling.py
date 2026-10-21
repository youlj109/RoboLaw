"""Post-physics rigid-object samples, with explicit units and quaternion order."""
import math


def _vector(value, count):
    values = [float(x) for x in value]
    if len(values) != count or not all(math.isfinite(x) for x in values):
        raise ValueError('invalid finite vector')
    return values


def sample_object(obj):
    # Isaac RigidPrim returns WXYZ; exported raw state uses XYZW.
    result = dict(translation=None, orientation=None, linear_velocity=None, angular_velocity=None)
    try:
        position, wxyz = obj.get_world_pose()
        position, wxyz = _vector(position, 3), _vector(wxyz, 4)
        result.update(translation=position, orientation=wxyz[1:] + wxyz[:1])
    except Exception:
        pass
    for field, method in [('linear_velocity', 'get_linear_velocity'),
                          ('angular_velocity', 'get_angular_velocity')]:
        try:
            result[field] = _vector(getattr(obj, method)(), 3)
        except Exception:
            pass
    return result


def project_object_samples(frames):
    output = {'object_pose_gt': {}, 'object_velocity_gt': {}, 'object_angular_velocity_gt': {}}
    names = sorted({name for frame in frames for name in frame})
    for name in names:
        samples = [frame.get(name) or {} for frame in frames]
        output['object_pose_gt'][name] = {
            'translation_per_step': [s.get('translation') for s in samples],
            'orientation_per_step': [s.get('orientation') for s in samples],
            'source': 'PhysXDataCollector.post_physics_world_state',
            'coordinate_frame': 'world', 'quaternion_order': 'xyzw'}
        output['object_velocity_gt'][name] = [s.get('linear_velocity') for s in samples]
        output['object_angular_velocity_gt'][name] = [s.get('angular_velocity') for s in samples]
    output['post_physics_state_provenance'] = {
        'phase': 'after_action_and_physics', 'coordinate_frame': 'world',
        'translation_unit': 'm', 'linear_velocity_unit': 'm/s',
        'angular_velocity_unit': 'rad/s', 'quaternion_order': 'xyzw',
        'missing_measurement': None, 'velocity_source': 'RigidPrim runtime velocity getters'}
    return output
