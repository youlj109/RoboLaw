#!/usr/bin/env python3
"""Verify constructed hard scenes and frozen provenance without simulation."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(Path(__file__).parent))
from build import sha, validate
from pxr import Usd


def verify(report_path):
    report = json.loads(Path(report_path).read_text())
    results = []
    for entry in report['entries']:
        scene = ROOT / entry['scene']
        assert sha(scene) == entry['scene_sha256'], f'Scene changed: {scene}'
        for path, expected in entry['dependencies_sha256'].items():
            assert sha(ROOT / path) == expected, f'Dependency changed: {path}'
        stage = Usd.Stage.Open(str(scene))
        checks = validate(stage)
        states = {name: validate(Usd.Stage.Open(str(ROOT / path)))
                  for name, path in entry['state_previews'].items()}
        results.append(dict(category=entry['category'], scene=entry['scene'], checks=checks,
                            state_checks=states, hashes_match=True))
    return dict(scope='CPU composition, provenance and no-trajectory verification only', entries=results)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report')
    args = parser.parse_args()
    print(json.dumps(verify(args.report), ensure_ascii=False, indent=2))
