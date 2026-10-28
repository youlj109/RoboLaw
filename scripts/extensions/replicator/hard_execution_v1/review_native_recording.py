#!/usr/bin/env python3
"""Decode native demo videos and make timestamped review sheets, without Kit.

This verifies file integrity and exposes frames for human review. It cannot
establish physical task success or infer contact from an image.
"""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def review(path, out, count):
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise ValueError(f'Cannot open {path}')
    reported = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    if reported < 1 or fps <= 0:
        raise ValueError(f'Invalid video metadata: {path}')
    selected = set(np.linspace(0, reported - 1, min(count, reported)).round().astype(int))
    frames, tiles, shape = 0, [], None
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if shape is None:
            shape = tuple(frame.shape)
        if tuple(frame.shape) != shape:
            raise ValueError(f'Frame dimensions changed at {frames}: {path}')
        if frames in selected:
            rgb = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            rgb.thumbnail((640, 360))
            tile = Image.new('RGB', (640, 390), '#131820')
            tile.paste(rgb, ((640 - rgb.width) // 2, 30))
            ImageDraw.Draw(tile).text((10, 8), f'{path.name} | frame {frames} | {frames / fps:.2f} s', fill='white')
            tiles.append(tile)
        frames += 1
    capture.release()
    if frames != reported:
        raise ValueError(f'Decoded {frames}, expected {reported}: {path}')
    columns = 3
    sheet = Image.new('RGB', (columns * 640, ((len(tiles) + columns - 1) // columns) * 390), '#131820')
    for index, tile in enumerate(tiles):
        sheet.paste(tile, ((index % columns) * 640, (index // columns) * 390))
    sheet.save(out)
    return dict(path=str(path.resolve()), sha256=digest(path), decoded_frames=frames,
                fps=fps, duration_seconds=frames / fps, width=shape[1], height=shape[0],
                sheet=str(out.resolve()), full_decode_passed=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episode', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--samples', type=int, default=12)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    videos = sorted(args.episode.rglob('*.mp4'))
    if not videos:
        raise ValueError('No native MP4 files found')
    rows = []
    for video in videos:
        stem = '_'.join(video.relative_to(args.episode).with_suffix('').parts)
        rows.append(review(video, args.output / f'{stem}.jpg', args.samples))
    report = dict(episode=str(args.episode.resolve()), videos=rows,
                  physical_success_inferred_from_video=False,
                  scope='Full sequential decode and sampled visual review sheets; task and contact evidence are separate.')
    (args.output / 'video_review.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(dict(videos=len(rows), frames=[row['decoded_frames'] for row in rows], output=str(args.output))))


if __name__ == '__main__':
    main()
