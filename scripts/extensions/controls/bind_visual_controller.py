"""Rebind a known controller to a newly reviewed human asset without tuning it."""
import argparse
import hashlib
import json
from pathlib import Path

from kitchen_safety.extensions.controls.execution_contract import (
    controller_signature, frozen_task_differences)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--visual', type=Path, required=True)
    parser.add_argument('--controller', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--round', type=int, required=True)
    args = parser.parse_args()
    visual = json.loads(args.visual.read_text())
    source = json.loads(args.controller.read_text())
    candidate = json.loads(args.controller.read_text())
    candidate['variants']['human'] = visual['variants']['human']
    for key in ('visual_revision', 'human_pose_revision'):
        candidate[key] = visual[key]
    if frozen_task_differences(visual, candidate):
        raise ValueError('Controller differs from the new frozen task/layout beyond declared control references')
    if controller_signature(source) != controller_signature(candidate):
        raise ValueError('Rebinding must not tune the known controller')
    digest = hashlib.sha256(args.visual.read_bytes()).hexdigest()
    candidate['execution_revision'] = dict(source=str(args.visual), source_sha256=digest,
        layout_modified=False, scope='Controller retained; binding to new explicitly image-reviewed scene.')
    candidate['benchmark_execution_round'] = dict(round=args.round, source=str(args.controller),
        scope='One attempt after authorized visual arm revision; controller unchanged, '
              'original task/goal/margins retained. No historical physical acceptance inherited.')
    candidate['quality_recheck'] = dict(visual_config=str(args.visual), visual_config_sha256=digest,
        scope='New reviewed visual scene; every physical and safety gate remains required.')
    args.output_dir.mkdir(parents=True, exist_ok=False)
    config_path = args.output_dir / '07_oven_temperature_human.json'
    config_path.write_text(json.dumps(candidate, indent=2) + '\n')
    mapping = {'07_oven_temperature_human': str(config_path)}
    (args.output_dir / 'overrides.json').write_text(json.dumps(mapping, indent=2) + '\n')
    print(config_path)


if __name__ == '__main__':
    main()
