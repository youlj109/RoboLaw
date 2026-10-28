#!/usr/bin/env python3
"""CPU-only: freeze one timing-only retry candidate for each of 84 failures."""
import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from kitchen_safety.extensions.replicator.physical_retry_plan import prepare_plan,sha


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--first-pass-manifest',type=Path,default=ROOT/'exports/replicator_154_physical_demos_20261028/manifest.json')
    parser.add_argument('--camera-retake-manifest',type=Path,default=ROOT/'exports/replicator_u_banana_camera_retakes_20261028/manifest.json')
    parser.add_argument('--observations',type=Path,default=ROOT/'configs/extensions/replicator/physical_execution_154_20261028.json')
    parser.add_argument('--output',type=Path,default=ROOT/'reports/replicator_physical_retry_round1_20261028/trajectories')
    parser.add_argument('--plan',type=Path,default=ROOT/'configs/extensions/replicator/physical_retry_round1_20261028.json')
    args=parser.parse_args()
    plan=prepare_plan(ROOT,args.first_pass_manifest,args.camera_retake_manifest,args.observations,args.output,args.plan)
    print(json.dumps(dict(plan=str(args.plan),sha256=sha(args.plan),summary=plan['summary'],gpu_launched=False),ensure_ascii=False))


if __name__=='__main__': main()
