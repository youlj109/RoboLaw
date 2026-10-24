"""Extract actual recorded combined-camera frames; visual review remains pending."""
import argparse
import hashlib
import json
from pathlib import Path

import imageio_ffmpeg
from PIL import Image, ImageDraw


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episode', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    manifest_path = args.output.with_suffix('.manifest.json')
    if args.output.exists() or manifest_path.exists():
        parser.error('Refusing to replace preview evidence')
    rows_path = args.episode / 'trajectory.json'
    rows = json.loads(rows_path.read_text())
    if not rows:
        parser.error('No recorded trajectory')
    indices = {0, len(rows) - 1}
    for phase in ['pregrasp', 'grasp_handle', 'pull_door']:
        matching = [i for i, row in enumerate(rows) if row['phase'] == phase]
        if not matching:
            parser.error('Missing recorded phase: ' + phase)
        indices.add(matching[len(matching) // 2])
    indices = sorted(indices)
    video = args.episode / 'all_cameras.mp4'
    reader = imageio_ffmpeg.read_frames(str(video), pix_fmt='rgb24')
    selected = {}
    try:
        metadata = next(reader)
        width, height = metadata['size']
        for index, raw in enumerate(reader):
            if index in indices:
                selected[index] = Image.frombytes('RGB', (width, height), raw)
            if index >= indices[-1]:
                break
    finally:
        reader.close()
    if set(selected) != set(indices):
        raise ValueError('Video ended before the requested recorded frames')
    canvas = Image.new('RGB', (width, (height + 24) * len(indices)), (20, 20, 20))
    draw = ImageDraw.Draw(canvas)
    for row, index in enumerate(indices):
        text = f"frame {index} | {rows[index]['phase']} | door {rows[index]['next_obs']['door_q']:.5f}"
        draw.text((8, row * (height + 24) + 5), text, fill='white')
        canvas.paste(selected[index], (0, row * (height + 24) + 24))
    canvas.save(args.output, quality=88)
    sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    manifest = dict(episode=str(args.episode), image=str(args.output),
                    image_sha256=sha(args.output), video_sha256=sha(video),
                    trajectory_sha256=sha(rows_path), actual_frames=indices,
                    implementation_sha256=sha(Path(__file__)), review_pending=True,
                    scope='Actual recorded keyframes only; extraction is not visual review or safety acceptance.')
    manifest_path.write_text(json.dumps(manifest, indent=2) + '\n')
    print('NATIVE_KEYFRAMES', json.dumps(manifest), flush=True)


if __name__ == '__main__':
    main()
