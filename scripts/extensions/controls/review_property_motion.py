"""Save an independent, immutable review of recorded property root motion."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0,str(ROOT))
from kitchen_safety.extensions.controls.property_motion import property_motion_review


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('episode',type=Path)
    args=p.parse_args()
    output=args.episode/'property_motion_review.json'
    if output.exists():raise FileExistsError(output)
    result_file=args.episode/'result.json'
    audit_file=args.episode/'audit.json'
    report=property_motion_review(json.loads(result_file.read_text()),json.loads(audit_file.read_text())['physics_updates'])
    report['input_sha256']={p.stem:hashlib.sha256(p.read_bytes()).hexdigest() for p in (result_file,audit_file)}
    output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    raise SystemExit(0 if report['passed'] else 2)


if __name__=='__main__':main()
