"""Plot the recorded bottle transport against its endpoint-to-endpoint segment."""
import argparse
import json
from pathlib import Path

import h5py
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from pxr import Usd, UsdGeom

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('episode')
p.add_argument('--output', required=True)
args = p.parse_args()
root = Path(__file__).resolve().parents[1]
episode = Path(args.episode)
r = json.loads((episode/'result.json').read_text())
with h5py.File(episode/'trajectory.h5') as f:
    phases = f['phase'].asstr()[:]
    indices = np.flatnonzero(np.isin(phases, ['transfer_front', 'transfer', 'above_bowl']))
    if not len(indices):
        raise ValueError('No transport states')
    a, b = int(indices[0]), int(indices[-1])
    xyz = np.vstack([f['obs/mustard'][a:b+1, :3], f['next_obs/mustard'][b, :3]])
    tcp = np.vstack([f['obs/tcp'][a:b+1], f['next_obs/tcp'][b]])
start, end = xyz[0], xyz[-1]
d = end[:2]-start[:2]
u = np.clip((xyz[:, :2]-start[:2])@d/(d@d), 0, 1)
line = start + u[:, None]*(end-start)
offset = xyz[:, :2]-start[:2]
lateral = np.abs(d[0]*offset[:, 1]-d[1]*offset[:, 0])/np.linalg.norm(d)
summary = dict(variant=r['variant'], episode=str(episode), first_frame=a, last_frame=b,
               max_bottle_lateral_deviation_m=float(lateral.max()),
               max_bottle_height_above_direct_segment_m=float((xyz[:, 2]-line[:, 2]).max()),
               max_tcp_height_m=float(tcp[:, 2].max()),
               scope='Recorded transport only; geometric illustration, not a collision test.')
fig, axes = plt.subplots(1, 2, figsize=(12, 4.7), constrained_layout=True)
for ax, dims, title in zip(axes, [(0, 1), (0, 2)], ['Top view', 'Side view']):
    i, j = dims
    ax.plot([start[i], end[i]], [start[j], end[j]], '--', color='#c64a43', lw=2,
            label='Direct bottle-center segment')
    ax.plot(xyz[:, i], xyz[:, j], color='#186cb2', lw=2.8, label='Executed bottle-center path')
    ax.scatter(start[i], start[j], c='#16824b', s=65, zorder=6, label='Lifted start')
    ax.scatter(end[i], end[j], c='#6b2fa0', marker='*', s=130, zorder=6, label='Above bowl')
    ax.set(xlabel='World X (m)', ylabel='World '+('Y' if j == 1 else 'Z')+' (m)', title=title)
    ax.grid(alpha=.2)
    ax.set_aspect('equal', adjustable='box')
if r['variant'] == 'property':
    x, y, z = r['layout']['property']['glass_position']
    asset = root/r['layout']['property'].get('glass_asset', 'assets/safety/glass.usda')
    stage = Usd.Stage.Open(str(asset))
    bb = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]).ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
    height = float(bb.GetMax()[2]-bb.GetMin()[2])
    axes[0].add_patch(Circle((x, y), .039, facecolor='#e5a440', alpha=.55, label='Glass footprint'))
    axes[1].add_patch(Rectangle((x-.039, z), .078, height, facecolor='#e5a440', alpha=.55, label='Glass silhouette'))
    axes[1].set_ylim(min(z-.015, xyz[:, 2].min()-.04), xyz[:, 2].max()+.05)
    axes[0].set_ylim(min(y-.10, xyz[:, 1].min()-.07), max(y+.10, xyz[:, 1].max()+.07))
else:
    from audit_collection_clearance import hazard_stage
    stage, _ = hazard_stage(r['layout'], r['variant'])
    cache = UsdGeom.XformCache()
    vertices = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        pts = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get())
        t = np.asarray(cache.GetLocalToWorldTransform(prim)).T
        pts = pts@t[:3, :3].T + t[:3, 3]
        mask = ((pts[:, 0] > 1.85) & (pts[:, 0] < 2.7) &
                (pts[:, 1] > -.85) & (pts[:, 1] < -.1) &
                (pts[:, 2] > 1.) & (pts[:, 2] < 1.38))
        vertices.append(pts[mask])
    points = np.concatenate(vertices)[::3]
    for ax, j in zip(axes, [1, 2]):
        ax.scatter(points[:, 0], points[:, j], s=2, c='#e5a440', alpha=.18,
                   label='Nearby obstacle surface (projection)')
for ax in axes:
    ax.legend(loc='best', fontsize=8)
fig.suptitle(r['variant'].capitalize()+' | recorded transport, no prescribed lateral waypoints')
out = Path(args.output)
out.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(out.with_suffix('.png'), dpi=170)
out.with_suffix('.json').write_text(json.dumps(summary, indent=2)+'\n')
print(json.dumps(summary, indent=2))
