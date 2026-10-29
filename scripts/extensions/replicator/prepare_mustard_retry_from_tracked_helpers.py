#!/usr/bin/env python3
"""Reproduce the round3 mustard CPU selection using tracked helper code.

The frozen prepare_round3_mustard.py and its historical source SHA remain
unchanged. This entry point replaces only the location of identical diagnostic
functions with the checked-in helper. Source assets and bound historical input
reports still need restoring; they are data inputs, not missing Python code.

Use new --report and --config paths. No GPU, Kit, physics, or layout editing.
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from scripts.extensions.replicator import prepare_round3_mustard as preparation


def main():
    if '--help' not in sys.argv and ('--report' not in sys.argv or '--config' not in sys.argv):
        raise SystemExit('Specify new --report and --config paths; frozen outputs are retained.')
    preparation.DIAGNOSTIC = ROOT/'scripts/extensions/replicator/planning_failure_cpu_geometry.py'
    preparation.main()


if __name__ == '__main__': main()
