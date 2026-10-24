"""Prepare one declared task/risk variant; never start collection or evaluation."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from kitchen_safety.task_variant_bundle import prepare

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--task-id',required=True);p.add_argument('--variant-id',required=True)
    p.add_argument('--placement',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--suite',type=Path,default=ROOT/'configs/risk_task_suite/suite.json')
    p.add_argument('--library',type=Path,default=ROOT/'configs/obstacle_library.json')
    p.add_argument('--human-assets',type=Path,default=ROOT/'configs/risk_human_assets.json')
    a=p.parse_args();result=prepare(ROOT,a.suite,a.library,a.human_assets,a.task_id,a.variant_id,a.placement,a.output)
    print(json.dumps(result,ensure_ascii=False));return 0
if __name__=='__main__':raise SystemExit(main())
