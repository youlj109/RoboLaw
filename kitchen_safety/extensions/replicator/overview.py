"""Supplementary fixed scene camera; original three DROID cameras stay unchanged."""

OVERVIEW_CAMERA_NAME = 'overview_camera'
OVERVIEW_SIZE = (1280, 720)
# Live probe03 shows the person, robot, bottle and bowl together. Stay below the
# original room ceiling at z=2.701 m; do not hide kitchen geometry for a camera.
OVERVIEW_EYE = (1.6, 1.7, 2.60)
OVERVIEW_TARGET = (.30, -.25, .85)


def configure_overview(scene_cfg, *, eye=None, target=None):
    """Add a static, separately recorded overview before the Arena scene is built."""
    import numpy as np
    from scipy.spatial.transform import Rotation
    from isaaclab.sensors import CameraCfg
    from isaaclab.sim import PinholeCameraCfg

    eye = np.asarray(OVERVIEW_EYE if eye is None else eye, dtype=float)
    target = np.asarray(OVERVIEW_TARGET if target is None else target, dtype=float)
    if eye.shape != (3,) or target.shape != (3,) or not np.isfinite([eye, target]).all():
        raise ValueError('Overview eye and target must be finite 3D positions')
    z = eye - target
    if np.linalg.norm(z) < 1e-6:
        raise ValueError('Overview eye and target must differ')
    z /= np.linalg.norm(z)
    x = np.cross([0., 0., 1.], z)
    if np.linalg.norm(x) < 1e-6:
        raise ValueError('Overview must not look parallel to world vertical')
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    quaternion = Rotation.from_matrix(np.column_stack([x, y, z])).as_quat()
    setattr(scene_cfg, OVERVIEW_CAMERA_NAME, CameraCfg(
        prim_path='{ENV_REGEX_NS}/SafetyOverviewCamera',
        width=OVERVIEW_SIZE[0], height=OVERVIEW_SIZE[1], data_types=['rgb'],
        spawn=PinholeCameraCfg(focal_length=18., focus_distance=3.,
                              horizontal_aperture=36., vertical_aperture=20.25),
        offset=CameraCfg.OffsetCfg(pos=tuple(eye), rot=tuple(quaternion), convention='opengl'),
    ))
    return {
        'name': OVERVIEW_CAMERA_NAME,
        'position_world': eye.tolist(),
        'target_world': target.tolist(),
        'quaternion_opengl_xyzw': quaternion.tolist(),
        'width': OVERVIEW_SIZE[0], 'height': OVERVIEW_SIZE[1],
        'purpose': 'Supplementary scene overview; not a replacement for any original DROID policy camera.',
    }
