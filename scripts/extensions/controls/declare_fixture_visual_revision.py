"""Declare one pose-only fixture revision for image review, never acceptance."""
import argparse
import hashlib
import json
from pathlib import Path


def declare(source, output, position, yaw, reason):
    import math
    if output.exists():
        raise FileExistsError(output)
    if len(position) != 3 or not all(math.isfinite(v) for v in [*position, yaw]):
        raise ValueError('Expected a finite xyz position and yaw')
    config = json.loads(source.read_text())
    config['variants']['robot']['position'] = position
    config['variants']['robot']['yaw_deg'] = yaw
    config['visual_revision'] = dict(source=str(source),
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(), variant='robot',
        reason=reason, scope='Pose-only image-review candidate. Original asset, robot, '
            'task and other variants retained. No numeric geometry screening.',
        accepted_for_collection=False,
        warning='Earlier physical and safety acceptance does not transfer to this layout.')
    x = position[0]
    config['visual_review_views'] = dict(
        fixture_front=([2.12, -1.6, 1.55], [2.12, -.42, 1.12]),
        fixture_top=([2.12, -.5, 2.4], [2.12, -.5, 1.05]),
        fixture_detail=([x + .65 if x > 2.12 else x - .35, -1.2, 1.65],
                        [2.12, -.45, 1.15]))
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(config, indent=2) + '\n')
    return config


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--position', type=float, nargs=3, required=True)
    parser.add_argument('--yaw', type=float, required=True)
    parser.add_argument('--reason', required=True)
    args = parser.parse_args()
    declare(args.source, args.output, args.position, args.yaw, args.reason)
    print('UNREVIEWED_FIXTURE_POSE', args.output)
