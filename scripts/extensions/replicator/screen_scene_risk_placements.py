#!/usr/bin/env python3
"""Initialize all 154 entries and screen verified baselines using CPU source geometry."""
import argparse
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from kitchen_safety.extensions.replicator.scene_risk_placement import initialize_reports,run_task,REPORTS,CONFIGS


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task',action='append',help='Assigned full task id or short task name; repeatable')
    parser.add_argument('--variant',action='append',help='Exact asset id; repeatable')
    parser.add_argument('--initialize-only',action='store_true')
    parser.add_argument('--skip-init',action='store_true')
    parser.add_argument('--only-failed',action='store_true',help='Preserve already passing rows; repair only failed/pending entries')
    parser.add_argument('--inventory','--baseline-inventory',dest='inventory',type=Path)
    parser.add_argument('--output',type=Path,default=REPORTS)
    parser.add_argument('--config-output',type=Path,default=CONFIGS)
    parser.add_argument('--max-candidates',type=int,default=1800)
    parser.add_argument('--sample-count',type=int,default=96)
    parser.add_argument('--no-resume',action='store_true')
    parser.add_argument('--resume',action='store_true',help='Resume successful unchanged entries (default)')
    args=parser.parse_args()
    if args.max_candidates<1 or args.sample_count<2:parser.error('Positive candidates and at least two measured samples required')
    if not args.skip_init:initialize_reports(args.output)
    if args.initialize_only:return 0
    if args.task:
        tasks=[t if t.startswith('kitchen_bench_replicator_') else 'kitchen_bench_replicator_'+t for t in args.task]
    else:
        import json
        tasks=[t['id'] for t in json.loads((ROOT/'student_skills/robolaw-replicator/references/assignment.json').read_text())['tasks']]
    for task in tasks:
        run_task(task,output=args.output,config_output=args.config_output,inventory_path=args.inventory,
            max_candidates=args.max_candidates,sample_count=args.sample_count,resume=not args.no_resume,variants_filter=args.variant,only_failed=args.only_failed)
    return 0

if __name__=='__main__':raise SystemExit(main())
