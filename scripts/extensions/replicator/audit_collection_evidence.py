"""CPU-only recheck of absolute physics timeline and archived run provenance."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from kitchen_safety.extensions.replicator.collection_evidence import audit_saved_collection_evidence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episode', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output exists; never overwrite evidence')
    report = audit_saved_collection_evidence(args.episode)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(passed=report['passed'], output=str(args.output), checks=report['checks'])))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
