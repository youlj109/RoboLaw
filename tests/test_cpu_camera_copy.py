"""Real Warp CPU allocations; no Isaac/Kit application or GPU simulation."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import warp as wp

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kitchen_safety.cpu_camera_copy import install_cpu_rgb_copy


class Annotator:
    def __init__(self, output):
        self.output = output
        self.calls = 0

    def get_data(self):
        self.calls += 1
        return self.output


class Renderer:
    def __init__(self):
        self.original_calls = []

    def render(self, data):
        self.original_calls.append(data)
        return "original"

    def read_output(self, data, camera):
        return "unchanged"


def make_data(source, *, key="rgba", height=3, width=5):
    rgba = wp.zeros((1, height, width, 4), dtype=wp.uint8, device="cpu")
    rgb = wp.array(ptr=rgba.ptr, shape=(1, height, width, 3),
                   strides=(height * width * 4, width * 4, 4, 1),
                   dtype=wp.uint8, device="cpu", copy=False)
    return SimpleNamespace(
        spec=SimpleNamespace(device="cpu", view_count=1,
                             cfg=SimpleNamespace(height=height, width=width,
                                                 data_types=[key], isp_cfg=None)),
        output_data={"rgba": SimpleNamespace(warp=rgba), "rgb": SimpleNamespace(warp=rgb)},
        ppisp_pipeline=None,
        annotators={key: Annotator(source)},
        renderer_info={},
    )


class CpuCameraCopyTests(unittest.TestCase):
    def setUp(self):
        self.ensure = patch("kitchen_safety.cpu_camera_copy._ensure_render_update").start()
        self.addCleanup(patch.stopall)

    def test_rgba_all_pixels_alpha_and_existing_rgb_alias(self):
        source = np.arange(3 * 5 * 4, dtype=np.uint8).reshape(3, 5, 4)
        data = make_data(source)
        rgba, rgb = (data.output_data[key].warp for key in ("rgba", "rgb"))
        # Capture views before installation to verify existing consumers see
        # writes; merely replacing a ProxyArray would fail this check.
        old_rgba_view, old_rgb_view = np.asarray(rgba), np.asarray(rgb)
        renderer = Renderer()
        read_output = renderer.read_output
        metadata = install_cpu_rgb_copy(renderer)
        self.assertIs(install_cpu_rgb_copy(renderer), metadata)
        renderer.render(data)
        np.testing.assert_array_equal(old_rgba_view[0], source)
        np.testing.assert_array_equal(old_rgb_view[0], source[..., :3])
        self.assertTrue(np.shares_memory(old_rgba_view, old_rgb_view))
        self.assertEqual(renderer.read_output, read_output)
        self.assertEqual(metadata["fast_calls"], 1)
        self.assertEqual(data.annotators["rgba"].calls, 1)
        self.ensure.assert_called_once_with()
        self.assertEqual(renderer.original_calls, [])

    def test_rgb_strided_destination_preserves_alpha(self):
        source = np.arange(3 * 5 * 3, dtype=np.uint8).reshape(3, 5, 3)
        data = make_data(source, key="rgb")
        np.asarray(data.output_data["rgba"].warp).fill(219)
        renderer = Renderer()
        install_cpu_rgb_copy(renderer)
        renderer.render(data)
        observed = data.output_data["rgba"].warp.numpy()[0]
        np.testing.assert_array_equal(observed[..., :3], source)
        np.testing.assert_array_equal(observed[..., 3], np.full((3, 5), 219, dtype=np.uint8))

    def test_cpu_warp_flattened_and_noncontiguous_numpy_inputs(self):
        expected = np.arange(60, dtype=np.uint8).reshape(3, 5, 4)
        padded = np.empty((3, 5, 8), dtype=np.uint8)
        padded[..., ::2] = expected
        inputs = [expected.reshape(-1), wp.array(expected, device="cpu"), padded[..., ::2]]
        for source in inputs:
            with self.subTest(type=type(source).__name__):
                data = make_data(source)
                renderer = Renderer()
                install_cpu_rgb_copy(renderer)
                renderer.render(data)
                np.testing.assert_array_equal(data.output_data["rgba"].warp.numpy()[0], expected)
                self.assertEqual(renderer.original_calls, [])

    def test_dictionary_info_and_empty_frame_clears_existing_allocation(self):
        info = {"frame": 8}
        data = make_data({"data": np.arange(60, dtype=np.uint8), "info": info})
        renderer = Renderer()
        metadata = install_cpu_rgb_copy(renderer)
        renderer.render(data)
        self.assertIs(data.renderer_info["rgba"], info)
        data.annotators["rgba"].output = np.empty(0, dtype=np.uint8)
        renderer.render(data)
        self.assertFalse(np.any(data.output_data["rgba"].warp.numpy()))
        self.assertFalse(np.any(data.output_data["rgb"].warp.numpy()))
        self.assertEqual(metadata["copied_buffers"], 1)
        self.assertEqual(metadata["empty_buffers"], 1)
        self.assertEqual(data.annotators["rgba"].calls, 2)

    def test_unsupported_configurations_call_original_before_annotator_read(self):
        changes = {
            "device": lambda d: setattr(d.spec, "device", "cuda:0"),
            "view_count": lambda d: setattr(d.spec, "view_count", 2),
            "ppisp": lambda d: setattr(d, "ppisp_pipeline", object()),
            "isp_config": lambda d: setattr(d.spec.cfg, "isp_cfg", object()),
            "depth": lambda d: d.spec.cfg.data_types.append("depth"),
            "output_shape": lambda d: setattr(d.spec.cfg, "height", 4),
            "unallocated": lambda d: setattr(d, "output_data", None),
            "multiple_annotators": lambda d: d.annotators.update(rgb=Annotator(np.zeros(45, dtype=np.uint8))),
        }
        for name, change in changes.items():
            with self.subTest(name=name):
                data = make_data(np.zeros(60, dtype=np.uint8))
                annotator = data.annotators["rgba"]
                change(data)
                renderer = Renderer()
                metadata = install_cpu_rgb_copy(renderer)
                self.assertEqual(renderer.render(data), "original")
                self.assertIs(renderer.original_calls[0], data)
                self.assertEqual(annotator.calls, 0)
                self.assertEqual(metadata["fallback_calls"], 1)
        self.ensure.assert_not_called()

    def test_unusual_source_delegates_same_frame_without_second_annotator_read(self):
        data = make_data(np.arange(60, dtype=np.float32))
        original_annotator = data.annotators["rgba"]
        renderer = Renderer()
        metadata = install_cpu_rgb_copy(renderer)
        self.assertEqual(renderer.render(data), "original")
        delegated = renderer.original_calls[0]
        self.assertIs(delegated.annotators["rgba"].get_data(), original_annotator.output)
        self.assertIs(data.annotators["rgba"], original_annotator)
        self.assertIs(delegated.output_data, data.output_data)
        self.assertEqual(original_annotator.calls, 1)
        self.assertEqual(metadata["fallback_reasons"], {"source_layout": 1})

    def test_annotator_error_propagates_unchanged(self):
        data = make_data(np.zeros(60, dtype=np.uint8))
        failure = RuntimeError("annotator failure")
        data.annotators["rgba"].get_data = lambda: (_ for _ in ()).throw(failure)
        renderer = Renderer()
        install_cpu_rgb_copy(renderer)
        with self.assertRaises(RuntimeError) as raised:
            renderer.render(data)
        self.assertIs(raised.exception, failure)
        self.assertEqual(renderer.original_calls, [])


if __name__ == "__main__":
    unittest.main()
