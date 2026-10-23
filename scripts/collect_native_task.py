"""Native task policy + task-bound raw-only safety recording.

The policy owns commands and semantic phases; the safety recorder owns only
immutable evidence. No risk evaluation/rendering runs in this process.
"""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scripts.inspect_risk_task import main

if __name__=='__main__':
    if '--policy-factory' not in sys.argv:
        raise SystemExit('Supply --policy-factory kitchen_safety.<module>:<factory>; no generic task-success policy is assumed')
    raise SystemExit(main())
