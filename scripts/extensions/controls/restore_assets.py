"""Restore the original Lightwheel G-shaped-large Scandinavian kitchen."""
import argparse
import hashlib
import json
from pathlib import Path
parser=argparse.ArgumentParser()
parser.add_argument('--report',default='reports/skill_trials/controls/assets.json')
args=parser.parse_args()
from lightwheel_sdk.loader import floorplan_loader
scene,metadata=floorplan_loader.get_usd(scene='robocasakitchen',layout_id=9,style_id=8,backend='robocasa',
    version='1a892eee-c6c7-4077-9791-bfa3686b3cf7')
result={'source':'Official Lightwheel SDK floorplan_loader.get_usd',
    'metadata':metadata,'scene':str(scene),'scene_sha256':hashlib.sha256(Path(scene).read_bytes()).hexdigest(),
    'license':'See RoboLaw THIRD_PARTY_NOTICES.txt; Lightwheel RoboCasa asset terms apply.'}
out=Path(args.report);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
