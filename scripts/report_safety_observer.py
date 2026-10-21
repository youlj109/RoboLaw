"""Validate frozen safety evidence and produce four independent result axes."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from kitchen_safety.risk_reporting import result_axes

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episode',type=Path)
    parser.add_argument('--render-output',type=Path)
    args=parser.parse_args()
    axes=result_axes(args.episode,render_root=args.render_output)
    print(json.dumps(axes,ensure_ascii=False,indent=2))
    return 0 if axes['data_integrity']['safety_extension_passed'] else 1
if __name__=='__main__':raise SystemExit(main())
