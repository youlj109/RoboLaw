#!/usr/bin/env python3
"""Verify the revised human clip and combine it with the unchanged v1 clips."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import subprocess

import cv2

ROOT = Path(__file__).resolve().parents[4]
OUT = ROOT / 'exports/hard_demo_v2'
REPORT = ROOT / 'reports/doors_hard_demo_v2'
FFMPEG = ROOT / '.venv-arena/lib/python3.12/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2'


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    verification = read(OUT / '01_human_walk.verification.json')
    source = read(REPORT / 'source.json')
    geometry = read(REPORT / 'geometry.json')
    render = read(REPORT / 'frames.render.json')
    assert verification['decoded_frames'] == render['frame_count'] == geometry['sample_count'] == 168
    assert geometry['all_discrete_geometry_checks_passed']
    assert sha(ROOT / source['scene']) == source['scene_sha256'] == render['source_scene_sha256'] == geometry['scene_sha256']
    for inputs in [source['direct_source_dependency_sha256'], geometry['input_layers_sha256']]:
        for path, expected in inputs.items():
            assert sha(ROOT / path) == expected, path
    assert sha(ROOT / verification['video']) == verification['sha256']
    human = dict(id='human', video=verification['video'], sha256=verification['sha256'],
                 duration_seconds=14., decoded_frames=168,
                 verification='exports/hard_demo_v2/01_human_walk.verification.json',
                 scene=source['scene'], scene_sha256=source['scene_sha256'],
                 person_asset=source['human_asset'], person_sha256=sha(ROOT / source['human_asset']),
                 animation_export='reports/doors_hard_demo_v2/animation_final.json.gz',
                 animation_export_sha256=sha(REPORT / 'animation_final.json.gz'),
                 render_report='reports/doors_hard_demo_v2/frames.render.json',
                 evidence=['reports/doors_hard_demo_v2/source.json', 'reports/doors_hard_demo_v2/geometry.json',
                           'reports/doors_hard_demo_v2/independent_motion_review.json'])
    clips = [human]
    previous = read(ROOT / 'exports/hard_demo_v1/manifest.json')
    for name in ['property', 'water']:
        clip = next(c for c in previous['clips'] if c['id'] == name)
        assert sha(ROOT / clip['video']) == clip['sha256']
        assert sha(ROOT / clip['scene']) == clip['scene_sha256']
        clips.append({**clip, 'unchanged_from_v1': True})
    concat = OUT / 'concat.txt'
    concat.write_text(''.join("file '" + str((ROOT / c['video']).resolve()) + "'\n" for c in clips))
    combined = OUT / 'three_demos.mp4'
    if combined.exists():
        raise FileExistsError(combined)
    subprocess.run([str(FFMPEG), '-hide_banner', '-loglevel', 'error', '-n', '-f', 'concat', '-safe', '0',
                    '-i', str(concat), '-c', 'copy', '-movflags', '+faststart', str(combined)], check=True)
    cap = cv2.VideoCapture(str(combined))
    fps, count = cap.get(cv2.CAP_PROP_FPS), 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        assert frame.shape[:2] == (890, 960)
        count += 1
    cap.release()
    assert count == 360 and abs(fps - 12.) < .001
    manifest = dict(revision='v2 human walking and anatomical arm animation',
                    scope='Scripted scene videos with a stationary robot; no robot trajectories or policy/physics acceptance.',
                    clips=clips,
                    combined=dict(video=str(combined.relative_to(ROOT)), sha256=sha(combined),
                                  duration_seconds=count / fps, fps=fps, decoded_frames=count, width=960, height=890),
                    checks=dict(all_frames_decoded=True, scene_and_dependency_hashes_match=True,
                                all_168_human_frames_geometry_checked=True, property_water_unchanged=True),
                    evidence_sha256={p: sha(ROOT / p) for p in human['evidence']},
                    supersedes_human='exports/hard_demo_v1/01_human_open_fridge.mp4')
    (OUT / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(manifest['combined'], indent=2))


if __name__ == '__main__':
    main()
