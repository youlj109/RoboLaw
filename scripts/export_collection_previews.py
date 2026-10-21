"""Export all three RGB previews and a labelled montage from recorded JPEGs.

Example: bash scripts/arena_python.sh scripts/export_collection_previews.py EPISODE
No simulator is started. Complete, decoded outputs are reused. Incomplete or
corrupt previews are rebuilt in temporary files before replacing final outputs.
"""
import argparse
import io
import json
from pathlib import Path
import sys
import tempfile
import time

import h5py
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from kitchen_safety.dataset import (CAMERA_NAMES, RGB_ALIGNMENT, close_encoders,
                                   decode_video, preview_panel, video_encoder)


def export_episode(root, *, render_root=None):
    root = Path(root)
    image_root = Path(render_root) if render_root is not None else root
    started = time.perf_counter()
    item = {'path': str(root), 'passed': False, 'skipped_existing': False}
    with h5py.File(root / 'trajectory.h5', 'r') as trajectory:
        frames = len(trajectory['action'])
        phases = trajectory['phase'].asstr()[:]
        dt = float(trajectory.attrs['dt'])
    if frames <= 0 or len(phases) != frames or not np.isfinite(dt) or dt <= 0:
        raise ValueError('Expected a non-empty trajectory, matching phases, and positive dt')
    if render_root is not None:
        rendered = json.loads((image_root / 'render_report.json').read_text())
        if Path(rendered['source_episode']).resolve() != root.resolve():
            raise ValueError('Rendered images reference a different source episode')
        if (rendered.get('completed') is not True or rendered.get('complete_trajectory') is not True
                or rendered.get('source_obs_indices') != list(range(frames))):
            raise ValueError('Previews require the complete rendered source trajectory in action order')
        item['render_output'] = str(image_root)
    cameras = json.loads((image_root / 'cameras.json').read_text())
    if set(cameras) != set(CAMERA_NAMES):
        raise ValueError(f'Expected the original three cameras: {CAMERA_NAMES}')
    sizes = {name: (int(cameras[name]['width']), int(cameras[name]['height']))
             for name in CAMERA_NAMES}
    if any(width <= 0 or height <= 0 or width % 2 or height % 2
           for width, height in sizes.values()):
        raise ValueError('H.264 previews require positive even camera dimensions')
    sizes['all_cameras'] = (640 * len(CAMERA_NAMES), 360)

    with h5py.File(image_root / 'images.h5', 'r') as images:
        if set(images) != set(CAMERA_NAMES):
            raise ValueError(f'images.h5 must contain all three original cameras: {CAMERA_NAMES}')
        if images.attrs.get('alignment') != RGB_ALIGNMENT:
            raise ValueError('Unsupported or missing RGB/action alignment')
        if images.attrs.get('frames') != frames or not np.isclose(images.attrs['dt'], dt, rtol=0, atol=1e-10):
            raise ValueError('Image frame count or dt disagrees with trajectory')
        for name in CAMERA_NAMES:
            if len(images[name]['jpeg']) != frames:
                raise ValueError(f'{name}: image and action counts differ')
        order = tuple(json.loads(images.attrs.get('camera_order', json.dumps(CAMERA_NAMES))))
        if len(order) != 3 or set(order) != set(CAMERA_NAMES):
            raise ValueError('Recorded camera order does not contain the original three cameras')
        item['frames'] = frames
        try:
            decoded = {f'{name}.mp4': decode_video(image_root / f'{name}.mp4', frames, size, dt)
                       for name, size in sizes.items()}
        except (OSError, ValueError, RuntimeError, StopIteration):
            decoded = None
        if decoded is not None:
            item.update(passed=True, skipped_existing=True, video_decoded_frames=decoded,
                        elapsed_seconds=round(time.perf_counter() - started, 3))
            return item

        with tempfile.TemporaryDirectory(prefix='.collection_previews_', dir=image_root) as temporary:
            temporary = Path(temporary)
            encoders = {}
            try:
                for name, (width, height) in sizes.items():
                    encoders[name] = video_encoder(temporary / f'{name}.mp4', width, height, dt)
                for index, phase in enumerate(phases):
                    panels = []
                    for name in order:
                        with Image.open(io.BytesIO(images[name]['jpeg'][index].tobytes())) as image:
                            image.load()
                            if image.mode != 'RGB' or image.size != sizes[name]:
                                raise ValueError(f'{name} frame {index}: RGB dimensions disagree with metadata')
                            encoders[name].stdin.write(np.asarray(image).tobytes())
                            panels.append(preview_panel(image, name, phase))
                    encoders['all_cameras'].stdin.write(np.concatenate(panels, axis=1).tobytes())
            finally:
                close_encoders(encoders)
            # No final output is replaced until all four new videos fully decode.
            decoded = {f'{name}.mp4': decode_video(temporary / f'{name}.mp4', frames, size, dt)
                       for name, size in sizes.items()}
            for name in sizes:
                (temporary / f'{name}.mp4').replace(image_root / f'{name}.mp4')
    item.update(passed=True, video_decoded_frames=decoded,
                elapsed_seconds=round(time.perf_counter() - started, 3))
    return item


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episodes', nargs='+')
    parser.add_argument('--report')
    parser.add_argument('--render-output', type=Path,
                        help='RGB directory for one separate physical episode; previews are written beside these images')
    args = parser.parse_args()
    if args.render_output is not None and len(args.episodes) != 1:
        parser.error('--render-output requires exactly one source episode')
    results = []
    for episode in args.episodes:
        started = time.perf_counter()
        try:
            item = export_episode(episode, render_root=args.render_output)
        except Exception as error:
            item = {'path': episode, 'passed': False,
                    'error': f'{type(error).__name__}: {error}',
                    'elapsed_seconds': round(time.perf_counter() - started, 3)}
        results.append(item)
        print(json.dumps(item, ensure_ascii=False), flush=True)
    if args.report:
        report = Path(args.report)
        report.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(mode='w', dir=report.parent, prefix=report.name + '.',
                                         suffix='.tmp', delete=False) as stream:
            json.dump(results, stream, indent=2, ensure_ascii=False)
            stream.write('\n')
            temporary = Path(stream.name)
        temporary.replace(report)
    return 0 if all(item['passed'] for item in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
