"""Freeze a fresh visual layout revision without changing historical files."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--variant', choices=['human', 'property', 'robot'], required=True)
    p.add_argument('--position', type=float, nargs=3)
    p.add_argument('--yaw', type=float)
    p.add_argument('--asset')
    p.add_argument('--view', nargs=7, action='append', metavar=('NAME', 'EX', 'EY', 'EZ', 'TX', 'TY', 'TZ'))
    p.add_argument('--reason', required=True)
    a = p.parse_args()
    source = a.source.read_bytes()
    config = json.loads(source)
    entry = config['variants'][a.variant]
    if a.position is not None:
        entry['position'] = a.position
    if a.yaw is not None:
        entry['yaw_deg'] = a.yaw
    if a.asset is not None:
        entry['asset'] = a.asset
    for view in a.view or []:
        values = [float(v) for v in view[1:]]
        config.setdefault('visual_review_views', {})[view[0]] = [values[:3], values[3:]]
    config['visual_revision'] = {
        'source': str(a.source), 'source_sha256': hashlib.sha256(source).hexdigest(),
        'variant': a.variant, 'reason': a.reason, 'scope': 'Visual review only',
        'accepted_for_collection': False,
        'warning': 'Historical physical/safety acceptance does not transfer to this revision.',
    }
    a.output.parent.mkdir(parents=True, exist_ok=True)
    with a.output.open('x') as stream:
        json.dump(config, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    print(a.output)


if __name__ == '__main__':
    main()
