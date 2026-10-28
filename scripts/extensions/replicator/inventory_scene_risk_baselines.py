#!/usr/bin/env python3
"""Inventory complete real baselines without Isaac/GPU imports."""
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from kitchen_safety.extensions.replicator.scene_risk_baselines import build_inventory, INVENTORY


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=ROOT)
    parser.add_argument("--output", type=Path, default=Path(INVENTORY))
    parser.add_argument("--candidate-specs", type=Path, help="JSON list of additional explicitly accepted baseline specifications")
    args = parser.parse_args()
    from kitchen_safety.extensions.replicator.scene_risk_baselines import CANDIDATES
    specs = None
    if args.candidate_specs:
        extra = json.loads(args.candidate_specs.read_text())
        if not isinstance(extra, list):
            parser.error("--candidate-specs must contain a JSON list")
        specs = CANDIDATES + extra
    result = build_inventory(args.repo, candidate_specs=specs)
    output = args.output if args.output.is_absolute() else args.repo/args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix+".tmp")
    temporary.write_text(json.dumps(result, indent=2, ensure_ascii=False)+"\n")
    temporary.replace(output)
    print(json.dumps(dict(output=str(output), **result["summary"]), ensure_ascii=False))
    for task in result["tasks"]:
        for baseline in task["baselines"]:
            if baseline["errors"]:
                print(task["task_id"], baseline["profile"], baseline["errors"])


if __name__ == "__main__":
    main()
