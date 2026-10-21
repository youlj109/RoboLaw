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
            else:raise ValueError('Only representative vase/crossbar/person are configured for this trial')
        layout['obstacle']=entry
    return layout
