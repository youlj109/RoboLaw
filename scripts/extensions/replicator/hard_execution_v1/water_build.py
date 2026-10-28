"""Build/audit passive water execution assets without Kit/CUDA."""
import argparse
import json
from kitchen_safety.extensions.replicator.hard_execution_v1.water_mechanism import build_short_turn, inspect

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check-only", action="store_true")
    args=parser.parse_args()
    if not args.check_only:
        build_short_turn()
    result=inspect()
    print(json.dumps(result, indent=2))
    return 0 if result["passed"] else 2

if __name__=="__main__":
    raise SystemExit(main())
