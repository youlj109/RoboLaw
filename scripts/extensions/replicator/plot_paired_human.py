"""Plot paired bottle routes against the actual nearby human collision surface."""
import argparse,json
from pathlib import Path
import numpy as np
from pxr import UsdGeom
from audit import hazard_stage
from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld
from kitchen_safety.planning import DroidKinematics
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from scipy.spatial import ConvexHull
from kitchen_safety.extensions.replicator.source_geometry import SourceBottleGeometry
p=argparse.ArgumentParser();p.add_argument('paired',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();r=json.loads((a.paired/'paired_plans.json').read_text());layout=json.loads(Path(r['layout']).read_text());stage,_=hazard_stage(layout,'human',layout['obstacle']);kin=DroidKinematics(np.asarray(layout['robot_position'])+[0,0,layout['stand_height']],layout['robot_yaw_deg']);w=CollisionWorld(stage,kin)
source=SourceBottleGeometry(); bottle_vertices=np.concatenate([v for _,_,v in source.parts]);off=np.load(a.paired/'seed_43_off.npz');on=np.load(a.paired/'seed_43_on.npz');worst=next(x for x in r['rows'] if x['seed']==43 and not x['human_in_planning_world'])['source_mesh_minimum']['sample'];u=worst/8;lo=int(u);q=(1-(u-lo))*off['q'][lo]+(u-lo)*off['q'][min(lo+1,len(off['q'])-1)];pose_off=kin.fk(q)@off['attachment'];centers=(on['tcp']@on['attachment'])[:,:3,3];pose_on=(on['tcp']@on['attachment'])[np.argmin(np.abs(centers[:,0]-pose_off[0,3]))];
fig,axes=plt.subplots(1,2,figsize=(13,5.6))
for ax,dimensions,labels in [(axes[0],[0,1],('x (m)','y (m)')),(axes[1],[0,2],('x (m)','z (m)'))]:
 for human_vertices,faces in w.meshes.values():
  triangles=human_vertices[faces];center=triangles.mean(1);mask=(center[:,0]>.35)&(center[:,0]<.9)&(center[:,1]>-.8)&(center[:,1]<-.2)&(center[:,2]>.94)&(center[:,2]<1.3)
  if mask.any():ax.add_collection(PolyCollection(triangles[mask][:,:,dimensions],facecolor='#7a8794',edgecolor='none',alpha=.35))
 for row in r['rows']:
  if not row['success']:continue
  d=np.load(a.paired/row['path_file']);tcp=d['tcp'];attachment=d['attachment'];bottle=(tcp@attachment)[:,:3,3];on=row['human_in_planning_world'];ax.plot(bottle[:,dimensions[0]],bottle[:,dimensions[1]],color='#147eb3' if on else '#c12f44',alpha=.8,lw=1.8,label=('Human on: safe detour' if on else 'Human off: offline unsafe proposal') if row['seed']==42 else None)
 for pose,color,label in [(pose_off,'#c12f44','Carried bottle at risk'),(pose_on,'#147eb3','Bottle on higher route')]:
  points=(bottle_vertices@pose[:3,:3].T+pose[:3,3])[:,dimensions];outline=points[ConvexHull(points).vertices];ax.fill(outline[:,0],outline[:,1],facecolor=color,edgecolor=color,alpha=.22,label=label)
 ax.set(xlabel=labels[0],ylabel=labels[1]);ax.grid(alpha=.2);ax.legend(fontsize=9);ax.autoscale();ax.set_aspect('equal',adjustable='datalim')
axes[0].set_title('Top: actual source human collision surface nearby');axes[1].set_title('Side: predicted carried-bottle root routes')
off_rows=[x for x in r['rows'] if x['success'] and not x['human_in_planning_world']];on_rows=[x for x in r['rows'] if x['success'] and x['human_in_planning_world']];length=lambda rows:f"{min(x['tcp_path_length_m'] for x in rows):.3f}–{max(x['tcp_path_length_m'] for x in rows):.3f}";fig.suptitle(f"Same measured start, target and controls; seeds {' / '.join(map(str,r['seeds']))}\nHuman off {length(off_rows)} m TCP; human on {length(on_rows)} m TCP",fontsize=12)
off_clear=[x['source_mesh_minimum']['distance_m']*1000 for x in off_rows];on_clear=min(x['source_mesh_minimum']['distance_m']*1000 for x in on_rows);fig.text(.5,.015,f'Geometry-only planning counterfactuals; unsafe paths were never executed. Source mesh clearance: off {min(off_clear):.2f}–{max(off_clear):.2f} mm, on >={on_clear:.2f} mm (required 20 mm).',ha='center',fontsize=9)
fig.tight_layout(rect=[0,.04,1,.91]);fig.savefig(a.output,dpi=170);plt.close(fig)
