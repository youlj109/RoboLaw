"""Keep original cameras and fail promptly when RTX returns a uniform image."""
import json
import numpy as np
from kitchen_safety.dataset import CameraWriter, numpy as array


class DoorCameraWriter(CameraWriter):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.rgb_quality = []

    def write(self, phase):
        super().write(phase)
        spans = {}
        for name in self.names:
            rgb = array(self.scene[name].data.output['rgb'])[0, ::8, ::8, :3]
            spans[name] = int(np.ptp(rgb, axis=(0, 1)).max())
        self.rgb_quality.append({'frame': self.frames - 1, 'phase': phase,
                                 'spatial_channel_spans': spans})
        if min(spans.values()) <= 2:
            raise RuntimeError(f'Uniform camera output at frame {self.frames - 1}: {spans}')

    def close(self):
        super().close()
        (self.out / 'rgb_quality.json').write_text(json.dumps({
            'passed': bool(self.rgb_quality) and all(
                min(r['spatial_channel_spans'].values()) > 2 for r in self.rgb_quality),
            'sample_stride': 8, 'frames': self.rgb_quality}, indent=2))
