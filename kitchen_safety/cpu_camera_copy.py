"""CPU RGB copy for a single-view Isaac RTX renderer, without vendor patches.

Install on an initialized renderer instance.  Rendering/annotator freshness and
``read_output`` remain owned by Isaac Lab; only the single-image tiled copy is
replaced.  In particular, RGB may be a strided alias of an RGBA Warp allocation.
"""

from copy import copy
from functools import wraps
from types import MethodType

import numpy as np
import warp as wp


def _ensure_render_update():
    # Import only when a real render is requested, after SimulationApp exists.
    from isaaclab_physx.renderers.isaac_rtx_renderer_utils import ensure_isaac_rtx_render_update

    ensure_isaac_rtx_render_update()


class _CachedAnnotator:
    """Replay a fetched value when an unusual input requires the vendor path."""

    def __init__(self, value):
        self.value = value

    def get_data(self):
        return self.value


def install_cpu_rgb_copy(renderer):
    """Wrap ``renderer.render`` once and return its live, JSON-safe counters.

    Only CPU, single-view uint8 RGB/RGBA without PPISP uses the copy shortcut.
    All other configurations delegate to the original bound method.  The normal
    Isaac RGB annotator is named ``rgba`` and writes both outputs through their
    shared allocation.  No output arrays are replaced and no device is changed.
    """
    existing = getattr(renderer, "_cpu_rgb_copy_metadata", None)
    if existing is not None:
        return existing
    original = renderer.render
    metadata = {
        "installed": True,
        "implementation": "numpy_cpu_single_view_rgb_v1",
        "fast_calls": 0,
        "copied_buffers": 0,
        "empty_buffers": 0,
        "fallback_calls": 0,
        "fallback_reasons": {},
    }

    def fallback(reason, render_data, cached=None):
        metadata["fallback_calls"] += 1
        reasons = metadata["fallback_reasons"]
        reasons[reason] = reasons.get(reason, 0) + 1
        if cached is not None:
            # The source format is known only after get_data().  Preserve one
            # real annotator read and the same frame when delegating this case.
            render_data = copy(render_data)
            render_data.annotators = {cached[0]: _CachedAnnotator(cached[1])}
        return original(render_data)

    @wraps(original)
    def render(self, render_data):
        spec = render_data.spec
        outputs = render_data.output_data
        if spec is None or outputs is None:
            return fallback("unallocated", render_data)
        if str(spec.device) != "cpu":
            return fallback("device", render_data)
        if spec.view_count != 1:
            return fallback("view_count", render_data)
        if render_data.ppisp_pipeline is not None or getattr(spec.cfg, "isp_cfg", None) is not None:
            return fallback("ppisp", render_data)
        rgb_types = {"rgb", "rgba"}
        if not set(spec.cfg.data_types).issubset(rgb_types) or not set(outputs).issubset(rgb_types):
            return fallback("output_types", render_data)
        # Isaac's RGB/RGBA requests both map to one RGBA annotator.  Multiple
        # annotators retain the vendor's fetch/write ordering unchanged.
        if len(render_data.annotators) != 1:
            return fallback("annotator_count", render_data)
        data_type, annotator = next(iter(render_data.annotators.items()))
        if data_type not in rgb_types or data_type not in outputs:
            return fallback("annotator_type", render_data)
        target = outputs[data_type].warp
        channels = 4 if data_type == "rgba" else 3
        expected_shape = (1, spec.cfg.height, spec.cfg.width, channels)
        if not target.device.is_cpu or target.dtype != wp.uint8 or tuple(target.shape) != expected_shape:
            return fallback("output_layout", render_data)

        _ensure_render_update()
        output = annotator.get_data()
        if isinstance(output, dict):
            source = output["data"]
            render_data.renderer_info[data_type] = output["info"]
        else:
            source = output
        if source.size == 0:
            target.zero_()
            metadata["fast_calls"] += 1
            metadata["empty_buffers"] += 1
            return

        if isinstance(source, np.ndarray):
            # wp.array(numpy) in the original path also packs noncontiguous
            # NumPy input.  Keep that logical ordering, without the Warp copy.
            source_np = np.ascontiguousarray(source)
        elif isinstance(source, wp.array) and source.device.is_cpu and source.is_contiguous:
            source_np = np.asarray(source)
        else:
            return fallback("source_type_or_device", render_data, (data_type, output))
        if source_np.dtype != np.uint8 or source_np.size != int(np.prod(expected_shape)):
            return fallback("source_layout", render_data, (data_type, output))

        # np.asarray uses Warp's CPU __array_interface__, including byte
        # strides: RGB writes skip alpha and remain visible through RGBA/Torch.
        np.copyto(np.asarray(target), source_np.reshape(expected_shape), casting="no")
        metadata["fast_calls"] += 1
        metadata["copied_buffers"] += 1

    renderer._cpu_rgb_copy_original_render = original
    renderer._cpu_rgb_copy_metadata = metadata
    renderer.render = MethodType(render, renderer)
    return metadata
