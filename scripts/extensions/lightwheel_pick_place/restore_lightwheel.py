"""Restore the source registry's exact Lightwheel floorplan and distractor objects."""
from pathlib import Path
import json
from lightwheel_sdk.loader import floorplan_loader,object_loader
ROOT=Path(__file__).resolve().parents[3]
scene,metadata=floorplan_loader.get_usd(scene='robocasakitchen',layout_id=3,style_id=8,backend='robocasa')
rows={};provenance={'floorplan':{'path':str(scene),'metadata':metadata},'objects':{}}
for name,kwargs in [('broccoli',dict(registry_type='objects',file_name='Broccoli010',file_type='USD')),
                    ('sweet_potato',dict(registry_type='objects',file_name='SweetPotato005',file_type='USD'))]:
    path,asset,meta=object_loader.acquire_by_registry(**kwargs)
    rows[name]={'name':name,'path':str(path)}
    provenance['objects'][name]={'path':str(path),'asset_name':asset,'metadata':meta}
p=ROOT/'assets/extensions/lightwheel_pick_place/downloads/Isaac/IsaacLab/Arena/assets/object_library/srl_robolab_assets/objects'
for reg,name,rel in [('lime01_fruits_veggies_robolab','lime','fruits_veggies/lime01.usd'),
 ('wooden_bowl_hot3d_robolab','wooden_bowl','hot3d/wooden_bowl.usd'),
 ('red_bell_pepper_objaverse_robolab','red_bell_pepper','objaverse/red_bell_pepper.usd')]:
    rows[reg]={'name':name,'path':str(p/rel)}
(ROOT/'configs/extensions/lightwheel_pick_place/asset_paths.json').write_text(json.dumps(rows,indent=2))
out=ROOT/'reports/skill_trials/lightwheel_pick_place';out.mkdir(exist_ok=True,parents=True)
(out/'lightwheel_assets.json').write_text(json.dumps(provenance,indent=2))
print('Restored floorplan and both original distractor objects.')
