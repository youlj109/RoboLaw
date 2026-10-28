"""Fixed source-task placement on the measured G-shape CapSurface_03."""
import json, copy
from pathlib import Path
from kitchen_safety.obstacle_library import get_asset
ROOT=Path(__file__).resolve().parents[3]
def resolve_layout(identifier='',jitter=0.,seed=0):
    if jitter: raise ValueError('This skill trial is fixed_only; randomized placement is not verified')
    layout=json.loads((ROOT/'configs/extensions/replicator/g_shape_mustard.json').read_text())
    if identifier:
        if identifier=='human_default_mpfb':
            entry=dict(id=identifier,category='human',static=True,usd_path=layout['human']['asset'],position=layout['human']['position'],yaw_deg=layout['human']['yaw_deg'])
        else:
            entry=copy.deepcopy(get_asset(identifier))
            if identifier=='property_vase':entry.update(position=[.61,-.61,layout['counter_height']+.002],yaw_deg=0)
            elif identifier=='robot_projecting_crossbar':entry.update(position=[.61,-.84,layout['counter_height']+.002],yaw_deg=180)
            else:
                # Additional catalog hazards use an explicit, source-preserving
                # --layout-json placement. Keep the three legacy cases identical.
                allowed = {
                    'property_tall_glass', 'property_wine_glass', 'property_glass_bottle',
                    'property_ceramic_mug', 'property_tablet_stand', 'robot_steel_clamp',
                    'robot_narrow_opening', 'robot_sharp_equipment_housing',
                }
                if identifier not in allowed:
                    raise ValueError('Unsupported Replicator safety obstacle: ' + identifier)
                category = entry.get('category')
                asset = (ROOT / entry['usd_path']).resolve()
                catalog_root = (ROOT / 'assets/obstacle_library' / str(category)).resolve()
                if (category not in ('property', 'robot') or not asset.is_relative_to(catalog_root)
                        or not asset.is_file() or asset.suffix not in ('.usd', '.usda', '.usdc')):
                    raise ValueError('Hazard asset must be an existing USD in its category catalog directory')
        layout['obstacle']=entry
    return layout
