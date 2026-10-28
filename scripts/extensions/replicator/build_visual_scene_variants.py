#!/usr/bin/env python3
"""Build 154 visual-guided configs; repair human/clamp using CPU source geometry."""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from kitchen_safety.extensions.replicator.visual_guided_scene_placement import build,OUTPUT,REPORTS

def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--task',action='append');p.add_argument('--output',type=Path,default=OUTPUT)
 p.add_argument('--reports',type=Path,default=REPORTS);p.add_argument('--max-candidates',type=int,default=6000)
 p.add_argument('--no-resume',action='store_true');p.add_argument('--starting-manifest',type=Path)
 p.add_argument('--approach-guides',action='store_true');p.add_argument('--asset-overrides',type=Path);a=p.parse_args()
 tasks=[t if t.startswith('kitchen_bench_replicator_') else 'kitchen_bench_replicator_'+t for t in a.task] if a.task else None
 build(tasks,output=a.output,reports=a.reports,max_candidates=a.max_candidates,resume=not a.no_resume,starting_manifest=a.starting_manifest,approach_guides=a.approach_guides,asset_overrides=a.asset_overrides)
 return 0
if __name__=='__main__':raise SystemExit(main())
