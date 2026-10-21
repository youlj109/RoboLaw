"""Load the locally authored safety obstacles without changing the shared task."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def load_assets():
    assets = []
    for category in ('people', 'property', 'robot'):
        path = ROOT / 'assets/obstacle_library' / category / 'catalog.json'
        if path.exists():
            assets.extend(json.loads(path.read_text())['assets'])
    identifiers = [asset['id'] for asset in assets]
    if len(identifiers) != len(set(identifiers)):
        raise ValueError('Duplicate obstacle identifiers in the asset catalogs')
    return assets


def get_asset(identifier):
    for asset in load_assets():
        if asset['id'] == identifier:
            return asset
    raise ValueError(f'Unknown obstacle: {identifier}')


def resolve_layout(identifier='', jitter=0., seed=0):
    """Resolve the actual scene description for both execution and its saved record."""
    layout = json.loads((ROOT/'configs/layout.json').read_text())
    if jitter:
        import numpy as np
        rng = np.random.default_rng(seed)
        for key in ('mustard_position', 'bowl_position'):
            layout[key][:2] = (np.array(layout[key][:2])+rng.uniform(-jitter,jitter,2)).tolist()
    if identifier:
        entry = get_asset(identifier)
        layout['obstacle'] = entry
        category = entry['category']
        if category == 'human':
            layout['human'].update(asset=entry['usd_path'],position=entry['position'],yaw_deg=entry.get('yaw_deg',0))
        elif category == 'property':
            layout['property'] = dict(glass_asset=entry['usd_path'],glass_position=entry['position'],
                                      glass_yaw_deg=entry.get('yaw_deg',0))
        else:
            layout['robot'].update(barrier_asset=entry['usd_path'],barrier_position=entry['position'],
                                  barrier_yaw_deg=entry.get('yaw_deg',0))
        # Every library configuration uses the same goal-directed transport.
        # The original default scenes keep their existing expert settings.
        layout.setdefault('planning',{}).setdefault('transfer_mode_by_variant',{})[category]='goal'
    return layout
