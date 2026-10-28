#!/usr/bin/env python3
"""Repair reviewed countertop candidates without GPU, baselines or simulation."""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from kitchen_safety.extensions.replicator.visual_tabletop_placement import build,OUTPUT,REPORTS

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=OUTPUT)
    parser.add_argument('--reports',type=Path,default=REPORTS)
    parser.add_argument('--max-candidates',type=int,default=3000)
    parser.add_argument('--task',action='append')
    args=parser.parse_args()
    if not 1<=args.max_candidates<=5000:parser.error('Candidate limit must be 1..5000')
    build(output=args.output,reports=args.reports,max_candidates=args.max_candidates,task_filter=args.task)

if __name__=='__main__':main()
