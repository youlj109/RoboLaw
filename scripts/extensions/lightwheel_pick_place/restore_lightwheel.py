"""Restore Lightwheel floorplans and object USD paths for B01/B02 tasks."""
from pathlib import Path
import json
from lightwheel_sdk.loader import floorplan_loader, object_loader

ROOT = Path(__file__).resolve().parents[3]
OBJ = ROOT / "assets/extensions/lightwheel_pick_place/downloads/Isaac/IsaacLab/Arena/assets/object_library/srl_robolab_assets/objects"
OUT = ROOT / "reports/skill_trials/lightwheel_pick_place"
CFG = ROOT / "configs/extensions/lightwheel_pick_place/asset_paths.json"

# layout_id / style_id from Arena lightwheel_kitchen_factory.py
FLOORPLANS = (
    ("l_shaped_scandinavian", 3, 8),
    ("u_shaped_with_island_farmhouse2", 7, 10),
)

# registry_name -> (local name, relative usd under OBJ)
OBJECTS = (
    ("lime01_fruits_veggies_robolab", "lime", "fruits_veggies/lime01.usd"),
    ("wooden_bowl_hot3d_robolab", "wooden_bowl", "hot3d/wooden_bowl.usd"),
    ("red_bell_pepper_objaverse_robolab", "red_bell_pepper", "objaverse/red_bell_pepper.usd"),
    ("lemon_01_fruits_veggies_robolab", "lemon", "fruits_veggies/lemon_01.usd"),
    ("orange_01_fruits_veggies_robolab", "orange", "fruits_veggies/orange_01.usd"),
    ("apple_01_objaverse_robolab", "apple", "objaverse/apple_01.usd"),
    ("bowl_ycb_robolab", "bowl", "ycb/bowl.usd"),
    ("plate_large_vomp_robolab", "plate", "vomp/plate_large/plate_large.usd"),
)

LIGHTWHEEL_DISTRACTORS = (
    ("broccoli", dict(registry_type="objects", file_name="Broccoli010", file_type="USD")),
    ("sweet_potato", dict(registry_type="objects", file_name="SweetPotato005", file_type="USD")),
)


def main():
    rows = {}
    provenance = {"floorplans": {}, "objects": {}}
    if CFG.exists():
        rows = json.loads(CFG.read_text())

    for label, layout_id, style_id in FLOORPLANS:
        scene, metadata = floorplan_loader.get_usd(
            scene="robocasakitchen", layout_id=layout_id, style_id=style_id, backend="robocasa"
        )
        provenance["floorplans"][label] = {"path": str(scene), "metadata": metadata, "layout_id": layout_id, "style_id": style_id}
        print("FLOORPLAN", label, scene, flush=True)

    for name, kwargs in LIGHTWHEEL_DISTRACTORS:
        path, asset, meta = object_loader.acquire_by_registry(**kwargs)
        rows[name] = {"name": name, "path": str(path)}
        provenance["objects"][name] = {"path": str(path), "asset_name": asset, "metadata": meta}
        print("DISTRACTOR", name, path, flush=True)

    missing = []
    for reg, name, rel in OBJECTS:
        path = OBJ / rel
        if not path.exists():
            missing.append(str(path))
            print("MISSING_OBJECT", path, flush=True)
            continue
        rows[reg] = {"name": name, "path": str(path)}
        provenance["objects"][reg] = {"path": str(path), "relative": rel}
        print("OBJECT", reg, path, flush=True)

    CFG.write_text(json.dumps(rows, indent=2) + "\n")
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "lightwheel_assets.json").write_text(json.dumps(provenance, indent=2) + "\n")
    if missing:
        print("INCOMPLETE missing=", len(missing), flush=True)
        raise SystemExit(2)
    print("Restored B01+B02 floorplans and object path map.", flush=True)


if __name__ == "__main__":
    main()
