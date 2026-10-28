#!/usr/bin/env python3
"""Recompute recording integrity; explicitly leave full native acceptance pending."""
import argparse
import json
from pathlib import Path

from kitchen_safety.extensions.replicator.collection_evidence import audit_saved_collection_evidence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episode', required=True, type=Path)
    parser.add_argument('--report', required=True, type=Path)
    args = parser.parse_args()
    report = audit_saved_collection_evidence(args.episode)
    report.update(native_stability_verified=False, native_task_success_verified=False, accepted=False,
                  remaining=['Exact-placement matched neutral native stability',
                             'Independent native task/contact/release/terminal-state review',
                             'Actual scene and trajectory visual review'])
    with args.report.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(report), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
