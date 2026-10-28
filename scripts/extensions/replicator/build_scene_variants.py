#!/usr/bin/env python3
"""Generate all assigned Replicator scenes; CPU geometry only."""
import argparse
from kitchen_safety.extensions.replicator.scene_variant_builder import build


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', action='append', help='Optional exact assigned task ID')
    parser.add_argument('--repair-initial-robot', help='Read-only failure audit listing the scenes to regenerate')
    args = parser.parse_args()
    result = build(args.task,args.repair_initial_robot)
    return int(any(row['status'] in ('configuration_failed','failed') for row in result['scenes']))


if __name__ == '__main__':
    raise SystemExit(main())
