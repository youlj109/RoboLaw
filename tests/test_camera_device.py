"""Device-selection/config-copy tests without importing Kit or starting CUDA.

The thin parent fixture represents the checked SensorBase ordering: copy cfg,
assign simulation device, then allocate camera-owned state.  Real Torch/Warp
CPU indices exercise reset conversion; cross-device transfers are mocked.
"""

from copy import deepcopy
from dataclasses import dataclass, field, fields, replace
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

import torch
import warp as wp


@dataclass
class ParentCameraCfg:
    class_type: object = None
    prim_path: str = "/World/envs/env_0/Robot/wrist_camera"
    width: int = 1280
    height: int = 720
    data_types: list = field(default_factory=lambda: ["rgb"])
    offset: object = field(default_factory=lambda: {"pos": (0.011, -0.031, -0.074)})
    spawn: object = field(default_factory=lambda: {"focal_length": 2.8})
    renderer_cfg: object = field(default_factory=lambda: {"renderer_type": "isaac_rtx"})

    def copy(self):
        return replace(self)


class ParentCamera:
    def __init__(self, cfg):
        self.cfg = cfg.copy()

    def initialize_from_cpu_physics(self):
        self._device = "cpu"
        self.allocated_devices = [self._device] * 6

    @property
    def device(self):
        return self._device

    def reset(self, env_ids=None, env_mask=None):
        self.reset_received = (env_ids, env_mask)
        return "parent_reset"


def load_module_without_kit():
    modules = {
        name: ModuleType(name) for name in (
            "isaaclab", "isaaclab.sensors", "isaaclab.sensors.camera",
            "isaaclab.sensors.camera.camera", "isaaclab.sensors.camera.camera_cfg", "isaaclab.utils",
        )
    }
    modules["isaaclab.sensors.camera.camera"].Camera = ParentCamera
    modules["isaaclab.sensors.camera.camera_cfg"].CameraCfg = ParentCameraCfg
    modules["isaaclab.utils"].configclass = dataclass
    path = Path(__file__).resolve().parents[1] / "kitchen_safety" / "camera_device.py"
    spec = importlib.util.spec_from_file_location("camera_device_without_kit", path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


DEVICE_MODULE = load_module_without_kit()


class CameraDeviceTests(unittest.TestCase):
    def test_none_is_exact_noop_and_gpu_config_preserves_camera_fields(self):
        original = ParentCameraCfg()
        snapshot = deepcopy(original)
        self.assertIs(DEVICE_MODULE.with_camera_output_device(original, None), original)
        cfg = DEVICE_MODULE.with_camera_output_device(original, "cuda:0")
        self.assertEqual(original, snapshot)
        self.assertEqual(cfg.output_device, "cuda:0")
        self.assertIs(cfg.class_type, DEVICE_MODULE.OutputDeviceCamera)
        for item in fields(ParentCameraCfg):
            if item.name != "class_type":
                self.assertEqual(getattr(cfg, item.name), getattr(original, item.name))
        self.assertIsNot(cfg.offset, original.offset)
        cfg.data_types.append("rgba")
        self.assertEqual(original.data_types, ["rgb"])

    def test_declared_device_survives_cfg_copy_before_initialization(self):
        cfg = DEVICE_MODULE.with_camera_output_device(ParentCameraCfg(), "cuda:0")
        copied = cfg.copy().copy()
        self.assertEqual(copied.output_device, "cuda:0")
        camera = DEVICE_MODULE.OutputDeviceCamera(copied)
        camera.initialize_from_cpu_physics()
        self.assertEqual(camera.device, "cuda:0")
        self.assertEqual(camera.allocated_devices, ["cuda:0"] * 6)
        # A fresh initialization must make the same selection.
        camera.initialize_from_cpu_physics()
        self.assertEqual(camera.device, "cuda:0")

    def test_unset_output_device_uses_original_simulation_device(self):
        camera = DEVICE_MODULE.OutputDeviceCamera(DEVICE_MODULE.OutputDeviceCameraCfg())
        camera.initialize_from_cpu_physics()
        self.assertEqual(camera.device, "cpu")
        self.assertEqual(camera.reset(), "parent_reset")
        self.assertEqual(camera.reset_received, (None, None))

    def test_reset_moves_cpu_indices_and_mask_before_parent_reset(self):
        cfg = DEVICE_MODULE.with_camera_output_device(ParentCameraCfg(), "cuda:0")
        camera = DEVICE_MODULE.OutputDeviceCamera(cfg)
        camera.initialize_from_cpu_physics()
        indices = torch.tensor([0], device="cpu")
        mask = wp.ones(1, dtype=wp.bool, device="cpu")
        gpu_indices, gpu_mask = object(), object()
        with patch.object(indices, "to", return_value=gpu_indices) as move_indices:
            with patch.object(mask, "to", return_value=gpu_mask) as move_mask:
                result = camera.reset(indices, mask)
        self.assertEqual(result, "parent_reset")
        self.assertEqual(camera.reset_received, (gpu_indices, gpu_mask))
        move_indices.assert_called_once_with(device="cuda:0")
        move_mask.assert_called_once_with(device="cuda:0")

    def test_same_device_reset_preserves_buffers_and_python_indices(self):
        camera = DEVICE_MODULE.OutputDeviceCamera(DEVICE_MODULE.OutputDeviceCameraCfg())
        camera.initialize_from_cpu_physics()
        indices = torch.tensor([0], device="cpu")
        mask = wp.ones(1, dtype=wp.bool, device="cpu")
        with patch.object(indices, "to", side_effect=AssertionError("unnecessary copy")):
            with patch.object(mask, "to", side_effect=AssertionError("unnecessary copy")):
                camera.reset(indices, mask)
        self.assertIs(camera.reset_received[0], indices)
        self.assertIs(camera.reset_received[1], mask)
        camera.reset([0])
        self.assertEqual(camera.reset_received, ([0], None))

    def test_warp_cpu_indices_are_accepted(self):
        camera = DEVICE_MODULE.OutputDeviceCamera(DEVICE_MODULE.OutputDeviceCameraCfg())
        camera.initialize_from_cpu_physics()
        indices = wp.array([0], dtype=wp.int32, device="cpu")
        camera.reset(indices)
        torch.testing.assert_close(camera.reset_received[0], torch.tensor([0], dtype=torch.int32))


if __name__ == "__main__":
    unittest.main()
