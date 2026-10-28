#!/usr/bin/env python3
"""CPU preparation only; never launches Kit/GPU or collects data."""
import argparse,json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from kitchen_safety.extensions.replicator.mustard_family_preparation import TASKS,prepare


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task-id',choices=TASKS,required=True)
    parser.add_argument('--layout',type=Path,required=True)
    parser.add_argument('--geometry-report',type=Path,required=True)
    parser.add_argument('--initial-robot-report',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    report=prepare(args.task_id,args.layout,args.geometry_report,args.initial_robot_report)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(task_id=args.task_id,cpu_geometry_prepared=report['cpu_geometry_prepared'],
        source_supports=report['profile']['source_support_collider_paths'],
        physical_recipe_topology=report['profile']['physical_profile']['topology_applicable'],
        collector_implemented=False,execution_ready=False,output=str(args.output))),flush=True)
    return 0


if __name__=='__main__':raise SystemExit(main())
