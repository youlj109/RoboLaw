"""Independently replay CPU mobility evidence and fully decode its saved videos.

This adds a new review sidecar without rewriting the original probe or raw data.
"""
import argparse
import json
from pathlib import Path

from kitchen_safety.dataset import decode_video
from scripts.extensions.replicator.review_physical_controls import audit_mobility, require, sha

VIDEO_NAMES = ('external_camera.mp4', 'external_camera_2.mp4', 'wrist_camera.mp4',
               'all_cameras.mp4', 'overview/overview_camera.mp4')


def review(directory):
    directory = Path(directory)
    probe = json.loads((directory/'mobility_probe.json').read_text())
    raw = audit_mobility(directory)
    require(raw['passed'], 'Raw mobility arithmetic failed')
    videos = {}
    for name in VIDEO_NAMES:
        require(probe['decoded_frames'][Path(name).stem] == 90, 'Original mobility video count differs')
        path = directory/name
        digest = sha(path)
        frames = decode_video(path, 90, expected_dt=probe['dt'])
        require(sha(path) == digest, 'Video changed during independent decode')
        videos[name] = dict(frames=frames, dt=probe['dt'], sha256=digest)
    return dict(schema_version=1, passed=True, mobility=str(directory), raw_physical_review=raw, videos=videos,
                input_sha256={name: sha(directory/name) for name in
                              ['mobility_probe.json', 'recorded_states.json', 'wrench_calls.json', *VIDEO_NAMES]},
                review_code_sha256={str(path): sha(path) for path in [Path(__file__).resolve(),
                    Path(__file__).with_name('review_physical_controls.py').resolve()]},
                scope='Independent raw-state/native-command arithmetic and complete video decoding; no automatic semantic visual approval.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mobility', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    try:
        data = review(args.mobility)
    except (OSError, ValueError, KeyError, TypeError, IndexError, StopIteration, RuntimeError) as error:
        data = dict(passed=False, error=f'{type(error).__name__}: {error}')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(data, stream, indent=2); stream.write('\n')
    print(json.dumps({k: v for k, v in data.items() if k not in ['input_sha256', 'review_code_sha256']}, indent=2))
    return 0 if data['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
