#!/usr/bin/env python3
"""Extract a bounded sample of actual synchronized H5 JPEGs; never infer success."""
import argparse
import hashlib
import io
import json
from pathlib import Path

import h5py
import numpy as np
from PIL import Image, ImageDraw


def make(episode, output):
    episode, output = Path(episode), Path(output)
    if output.exists() or output.with_suffix('.json').exists():
        raise FileExistsError('Preserve prior montage evidence')
    raw = dict(np.load(episode/'trajectory.npz', allow_pickle=False))
    phases = raw['phase']
    frames = [int(np.flatnonzero(phases == phase)[-1]) for phase in
              ('initial_settle', 'close', 'lift', 'transport', 'release', 'retreat', 'final_settle')]
    release = np.flatnonzero(phases == 'release')
    frames = sorted(set(frames+[int(release[0]), int(release[min(3, len(release)-1)])]))
    cameras = [('images.h5', 'external_camera'), ('images.h5', 'external_camera_2'),
               ('images.h5', 'wrist_camera'), ('overview/images.h5', 'overview_camera')]
    width, height = 420, 263
    canvas = Image.new('RGB', (width*len(cameras), height*len(frames)), (18, 23, 29))
    draw = ImageDraw.Draw(canvas)
    records = []
    for col, (filename, camera) in enumerate(cameras):
        with h5py.File(episode/filename, 'r') as handle:
            for row, frame in enumerate(frames):
                encoded = handle[camera]['jpeg'][frame].tobytes()
                image = Image.open(io.BytesIO(encoded)).convert('RGB')
                image.thumbnail((width, 235))
                canvas.paste(image, (col*width, row*height+27))
                step, phase = int(raw['before_physics_step'][frame]), str(phases[frame])
                draw.text((col*width+4, row*height+6), f'{camera} f{frame} step{step} {phase}', fill='white')
                records.append(dict(frame=frame, physics_step=step, phase=phase, camera=camera,
                                    jpeg_sha256=hashlib.sha256(encoded).hexdigest()))
    output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output, quality=94)
    result = dict(accepted=False, episode=str(episode), frames=records,
        output_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
        input_sha256={name: hashlib.sha256((episode/name).read_bytes()).hexdigest()
                      for name in ('trajectory.npz', 'images.h5', 'overview/images.h5')},
        generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        scope='Resized original H5 JPEGs only; exact frame/physics-step identities retained. No final unseen state or successful task inference.')
    with output.with_suffix('.json').open('x') as stream:
        json.dump(result, stream, indent=2)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episode', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = make(args.episode, args.output)
    print(json.dumps(dict(output=str(args.output), selected_frames=sorted({r['frame'] for r in report['frames']}))))
