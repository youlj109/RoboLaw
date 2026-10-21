#!/usr/bin/env python3
"""Check the eleven published scene entry points without starting Isaac Sim."""
import argparse
import json
from pathlib import Path
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument("--require-restored", action="store_true", help="Also require NVIDIA assets restored from upstream")
args=parser.parse_args()
ROOT=Path(__file__).resolve().parents[1]
ids=('property_tall_glass','property_wine_glass','property_glass_bottle','property_ceramic_mug','property_vase','property_tablet_stand','robot_steel_clamp','robot_projecting_crossbar','robot_narrow_opening','robot_sharp_equipment_housing')
entries=[]
for group in ('property','robot'):
    entries+=json.loads((ROOT/f'assets/obstacle_library/{group}/catalog.json').read_text())['assets']
actual={x['id'] for x in entries}
assert len(entries)==10 and actual==set(ids),(len(entries),sorted(actual^set(ids)))
required=['scenes/base.usda','scenes/human_mpfb.usda','assets/kitchen/scene.usd','assets/people/mpfb_corridor/collidable.usda','assets/people/mpfb_corridor/person.usdc','assets/curobo/droid/tcp_calibration.json','assets/planning/placement_guides.json']
required += [x['usd_path'] for x in entries]
required += [f"scenes/obstacle_library/{identifier}.usda" for identifier in ids]
required += [f"configs/obstacle_library/{identifier}.json" for identifier in ids]
external=[x['path'] for x in json.loads((ROOT/'configs/external_assets.json').read_text())['assets'] if x['required']]
if args.require_restored: required += external
missing=[p for p in required if not (ROOT/p).is_file()]
print(json.dumps({'scene_count':11,'require_restored':args.require_restored,'external_files_to_restore':len(external),'missing_files':missing,'passed':not missing},ensure_ascii=False,indent=2))
raise SystemExit(bool(missing))
