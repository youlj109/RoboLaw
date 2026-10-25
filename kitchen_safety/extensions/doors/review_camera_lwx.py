"""Supplementary synchronized real RGB camera; native three-camera data unchanged."""
import json
from pathlib import Path

import numpy as np

from kitchen_safety.extensions.doors.video import DoorCameraWriter

NAME = 'visible_review_camera'


def add_review_camera(cfg, options):
    from isaaclab.sensors import CameraCfg
    from isaaclab import sim as sim_utils
    from scipy.spatial.transform import Rotation
    pos = np.asarray(options['position_world_m'], dtype=float)
    target = np.asarray(options['look_at_world_m'], dtype=float)
    if pos.shape != (3,) or target.shape != (3,) or not np.isfinite([pos, target]).all():
        raise ValueError('Finite camera position and look-at point required')
    back = pos-target
    back /= np.linalg.norm(back)
    right = np.cross([0., 0., 1.], back)
    if np.linalg.norm(right) < .01:
        raise ValueError('Camera cannot look along world up')
    right /= np.linalg.norm(right)
    up = np.cross(back, right)
    rot = Rotation.from_matrix(np.column_stack([right, up, back])).as_quat()
    camera = CameraCfg(
        prim_path='{ENV_REGEX_NS}/VisibleReviewCamera',
        height=720, width=1280, data_types=['rgb'],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=2.1, focus_distance=28.,
            horizontal_aperture=5.376, vertical_aperture=3.024),
        offset=CameraCfg.OffsetCfg(pos=tuple(pos), rot=tuple(rot), convention='opengl'))
    setattr(cfg.scene, NAME, camera)
    return dict(options, quaternion_xyzw=rot.tolist(), sensor_name=NAME,
                scope='Supplementary real RGB sensor synchronized before the same action; no collision or physical state changes. Native cameras retained.')


class ReviewCameraWriter:
    def __init__(self, out, scene, names, dt):
        self.native = DoorCameraWriter(out, scene, names, dt)
        supplement = Path(out)/'supplementary_observer'
        supplement.mkdir(exist_ok=False)
        self.observer = DoorCameraWriter(supplement, scene, [NAME], dt)
        self.names = self.native.names
        self.dt = dt
        self.out = Path(out)

    def warm_up(self, sim):
        result = self.native.warm_up(sim)
        extra = self.observer.warm_up(sim)
        result['supplementary_observer_warmup'] = extra
        result['passed'] = bool(result['passed'] and extra['passed'] and not extra['physics_advanced'])
        result['physics_advanced'] = bool(result['physics_advanced'] or extra['physics_advanced'])
        return result

    def write(self, phase):
        self.native.write(phase)
        self.observer.write(phase)
        if self.native.frames != self.observer.frames:
            raise RuntimeError('Supplementary observer frame alignment failed')

    def close(self):
        try:
            self.native.close()
        finally:
            self.observer.close()
        from kitchen_safety.dataset import decode_video
        report = dict(native_frames=self.native.frames, observer_frames=self.observer.frames,
                      same_action_alignment=self.native.frames == self.observer.frames,
                      native_camera_names=list(self.native.names), supplementary_camera_name=NAME,
                      observer_video=decode_video(self.observer.out/(NAME+'.mp4'), self.observer.frames, expected_dt=self.dt)
                      if self.observer.frames else None)
        (self.out/'supplementary_observer_alignment.json').write_text(json.dumps(report, indent=2)+'\n')
