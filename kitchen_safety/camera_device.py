"""Opt-in GPU camera buffers with CPU PhysX; import after SimulationApp starts."""

from copy import deepcopy
from dataclasses import fields

import torch
import warp as wp

from isaaclab.sensors.camera.camera import Camera
from isaaclab.sensors.camera.camera_cfg import CameraCfg
from isaaclab.utils import configclass


class OutputDeviceCamera(Camera):
    """Use one explicit device for all camera state, views and image buffers.

    SensorBase assigns ``_device`` before allocating its masks/timestamps.
    Intercept that assignment, rather than changing device after its buffers
    exist.  Camera then passes this same device to FrameView, CameraRenderSpec,
    annotators and CameraData.  The global simulation device is untouched.
    """

    @property
    def _device(self):
        return self._camera_output_device

    @_device.setter
    def _device(self, simulation_device):
        self._camera_output_device = str(self.cfg.output_device or simulation_device)

    def reset(self, env_ids=None, env_mask=None):
        # InteractiveScene supplies indices on the physics device.  Explicit
        # Warp masks must also be moved: SensorBase otherwise passes them
        # directly to a camera-device kernel, and Camera reuses that mask for
        # its pose/frame update after the base reset.
        if isinstance(env_ids, wp.array):
            env_ids = wp.to_torch(env_ids)
        if isinstance(env_ids, torch.Tensor) and str(env_ids.device) != self.device:
            env_ids = env_ids.to(device=self.device)
        if env_mask is not None and str(env_mask.device) != self.device:
            env_mask = env_mask.to(device=self.device)
        return super().reset(env_ids=env_ids, env_mask=env_mask)


@configclass
class OutputDeviceCameraCfg(CameraCfg):
    class_type: type = OutputDeviceCamera
    output_device: str | None = None


def with_camera_output_device(camera_cfg: CameraCfg, output_device: str | None):
    """Return a GPU-buffer camera config, preserving its authored camera fields.

    Use before constructing the scene, with CPU physics.  ``None`` returns the
    input unchanged.  ``output_device`` is a declared dataclass field so it
    survives Isaac Lab's ``cfg.copy()`` / ``dataclasses.replace`` during sensor
    construction.  Camera mounting, optics, resolution and render settings are
    copied without changes.
    """
    if output_device is None:
        return camera_cfg
    device = torch.device(output_device)
    if device.type != "cuda":
        raise ValueError("Camera output override requires a CUDA device; use None for the original configuration")
    values = {
        field.name: deepcopy(getattr(camera_cfg, field.name))
        for field in fields(CameraCfg)
        if field.init and field.name != "class_type"
    }
    return OutputDeviceCameraCfg(**values, output_device=str(device))
