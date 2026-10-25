"""Supplementary fixed overview; the original three DROID cameras are unchanged."""
def configure_overview(scene_cfg, *, eye=None, target=None):
    import numpy as np
    from scipy.spatial.transform import Rotation
    from isaaclab.sensors import CameraCfg
    from isaaclab.sim import PinholeCameraCfg
    eye=np.array([2.65,-2.50,2.75] if eye is None else eye,dtype=float);target=np.array([3.85,-.50,.95] if target is None else target,dtype=float)
    z=(eye-target)/np.linalg.norm(eye-target);x=np.cross([0.,0.,1.],z);x/=np.linalg.norm(x);y=np.cross(z,x)
    quat=Rotation.from_matrix(np.column_stack([x,y,z])).as_quat()
    scene_cfg.overview_camera=CameraCfg(prim_path='{ENV_REGEX_NS}/SafetyOverviewCamera',
        width=1280,height=720,data_types=['rgb'],
        spawn=PinholeCameraCfg(focal_length=18.,focus_distance=3.,horizontal_aperture=36.,vertical_aperture=20.25),
        offset=CameraCfg.OffsetCfg(pos=tuple(eye),rot=tuple(quat),convention='opengl'))
    return {'name':'overview_camera','position_world':eye.tolist(),'target_world':target.tolist(),
        'quaternion_opengl_xyzw':quat.tolist(),'purpose':'Supplementary scene overview; not a replacement for any original DROID policy camera.'}
