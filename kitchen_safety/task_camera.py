"""Validate explicit offline overview cameras; never online observations."""
import math
from kitchen_safety.task_contract import digest


def validate_camera_config(config, contract, raw_sha256):
    if (config.get('schema_version') != 'robolaw-native-camera-1' or
            config.get('task_sha256') != digest(contract)):
        raise ValueError('Camera configuration must bind this exact task contract')
    if not raw_sha256 or config.get('source_raw_sha256') != raw_sha256:
        raise ValueError('World-coordinate cameras must bind this exact raw episode')
    cameras = config.get('cameras')
    if not isinstance(cameras, list) or len(cameras) != 3:
        raise ValueError('Exactly three offline overview cameras are required')
    for camera in cameras:
        for key in ('eye', 'look_at'):
            value = camera.get(key)
            if (not isinstance(value, list) or len(value) != 3 or
                    any(type(v) not in (int, float) or not math.isfinite(v) for v in value)):
                raise ValueError('Camera vectors must contain three finite world-metre values')
        delta = [a-b for a,b in zip(camera['eye'],camera['look_at'])]
        if sum(v*v for v in delta) < 1e-6 or delta[0]**2+delta[1]**2 < 1e-8:
            raise ValueError('Degenerate camera view or look-at up direction')
        focal = camera.get('focal_length_mm')
        if type(focal) not in (int,float) or not math.isfinite(focal) or not 1 <= focal <= 200:
            raise ValueError('Explicit focal length must be between 1 and 200 mm')
    return config
