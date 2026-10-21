"""Download the two original Arena props and preserve their relative dependencies."""
from pathlib import Path
from urllib.parse import urljoin
import shutil
import requests
from pxr import UsdUtils

ROOT = Path(__file__).resolve().parents[1]
BASE = 'https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1/Isaac/IsaacLab/Arena/assets/object_library/srl_robolab_assets/'
seen = set()

def fetch(url, path):
    if url in seen:
        return
    seen.add(url)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        response = requests.get(url, timeout=90)
        response.raise_for_status()
        path.write_bytes(response.content)
    print(path.relative_to(ROOT), flush=True)
    if path.suffix in ('.usd', '.usda', '.usdc'):
        layers, assets, payloads = UsdUtils.ExtractExternalReferences(str(path))
        for dep in list(layers) + list(assets) + list(payloads):
            if dep in ('OmniPBR.mdl', 'OmniGlass.mdl', 'OmniSurface.mdl'):
                continue  # Isaac Sim's built-in MDL search path.
            if dep.startswith(('http:', 'https:', 'omniverse:')):
                raise RuntimeError(f'Absolute dependency needs explicit mapping: {dep}')
            fetch(urljoin(url, dep), path.parent / dep)

if __name__ == '__main__':
    for item in ['objects/hope/mustard_bottle.usd', 'objects/ycb/bowl.usd']:
        fetch(urljoin(BASE, item), ROOT / 'assets/robolab' / item)
    for item in [
        'Assets/Isaac/6.1/Isaac/IsaacLab/Arena/assets/robot_library/droid/franka_robotiq_2f_85_flattened.usd',
        'Assets/Isaac/6.1/Isaac/IsaacLab/Arena/assets/object_library/srl_robolab_assets/robots/franka_stand_grey.usda',
    ]:
        fetch('https://omniverse-content-staging.s3-us-west-2.amazonaws.com/'+item,ROOT/'assets/nvidia'/item)
    kitchen = Path.home() / '.cache/lightwheel_sdk/floorplan/robocasa-robocasakitchen-1-1'
    if not kitchen.exists():
        from lightwheel_sdk.loader import floorplan_loader
        kitchen=Path(floorplan_loader.get_usd(scene='robocasakitchen',layout_id=1,style_id=1,backend='robocasa')[0]).parent
    shutil.copytree(kitchen, ROOT / 'assets/kitchen', dirs_exist_ok=True)
