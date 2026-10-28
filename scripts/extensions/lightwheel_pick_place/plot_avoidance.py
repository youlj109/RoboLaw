"""Export actual-trajectory and geometric counterfactual evidence as PNG/PDF."""
import argparse,json
from pathlib import Path
import h5py,numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pxr import Usd
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
from kitchen_safety.extensions.lightwheel_pick_place.task_config import episode_spec,pick_object_name,destination_object_name

p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True);p.add_argument('--episode',type=Path,required=True);p.add_argument('--audit',type=Path);p.add_argument('--output-dir',type=Path);a=p.parse_args();output=a.output_dir or a.episode;output.mkdir(parents=True,exist_ok=True)
r=json.loads((a.episode/'result.json').read_text());audit=json.loads((a.audit or a.episode/'avoidance_audit.json').read_text());b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:]).as_euler('xyz',degrees=True)[2]);spec=episode_spec(r);pick=pick_object_name(r,spec);dest=destination_object_name(r,spec);w=CollisionWorld(Usd.Stage.Open(str(a.episode/'scene.usda')),kin,pick_object=pick,destination_object=dest,source_surface_suffix=spec.source_surface_suffix,movable_objects=tuple(n for n in spec.rigid_objects if n!=pick));name={'human':'person','property':'glass','robot':'barrier'}[r['variant']];w.set_obstacle_pose(name,audit['hazard_initial_pose'])
parts=[v[::max(1,len(v)//1200)] for n,(v,faces) in w.meshes.items() if w._matches(n,name)]
for n,obj,lo,hi in w.objects:
    if w._matches(n,name) and n not in w.meshes:
        parts.append(np.array(np.meshgrid(*zip(lo,hi))).reshape(3,-1).T)
points=np.concatenate(parts)
traces={}
for label,ep in [('Baseline counterfactual',a.baseline),('Replanned physical run',a.episode)]:
    with h5py.File(ep/'trajectory.h5') as f:
        q=f['next_obs/q'][:,:7];traces[label]={'q':q,'tcp':f['next_obs/tcp'][:],'lime':f['next_obs/lime'][:,:3],'phase':f['phase'].asstr()[:]}
        traces[label]['elbow']=np.array([kin.links(v)['/panda/panda_link4'][:3,3] for v in q])
fig,axes=plt.subplots(2,2,figsize=(13,9),constrained_layout=True)
for ax,(u,v,title) in zip(axes[0],[(0,1,'Top view: actual elbow, TCP and carried fruit paths'),(1,2,'Side view: actual elbow, TCP and carried fruit paths')]):
    ax.scatter(points[:,u],points[:,v],s=2,c='.65',alpha=.4,label='Added mesh / primitive bounds')
    for label,color in [('Baseline counterfactual','#c13b27'),('Replanned physical run','#007ea8')]:
        for key,style in [('elbow','-'),('tcp','--'),('lime',':')]:
            path=traces[label][key];ax.plot(path[:,u],path[:,v],style,color=color,lw=1.7,label=label+' / '+key)
    ax.set(title=title,xlabel='XYZ'[u]+' (m)',ylabel='XYZ'[v]+' (m)');ax.axis('equal');ax.grid(alpha=.2)
axes[0,0].legend(fontsize=7,loc='best')
phases=list(dict.fromkeys([str(v) for v in traces['Baseline counterfactual']['phase']]))
ax=axes[1,0]
for key,label,color in [('baseline_counterfactual','Counterfactual baseline','#c13b27'),('augmented_actual','Measured augmented run','#007ea8')]:
    data=audit[key]['measurements'];xs=[];ys=[]
    for index,phase in enumerate(phases):
        values=[x for x in data if x['phase']==phase]
        xs.extend(index+np.linspace(0,1,len(values),endpoint=False));ys.extend(x['distance_m']*1000 for x in values)
    ax.plot(xs,ys,color=color,lw=1,label=label)
ax.axhline(audit['baseline_counterfactual']['required_m']*1000,color='black',ls='--',label='Unchanged execution margin')
ax.set_xticks(np.arange(len(phases))+.5,phases,rotation=50,ha='right',fontsize=7);ax.set(ylabel='Hazard surface distance (mm)',xlabel='Progress normalized within each named phase',ylim=(0,200),title='120 Hz geometry: clearance detail (0–200 mm window)');ax.grid(alpha=.2);ax.legend(fontsize=8)
ax=axes[1,1]
names=[];differences=[]
for phase,row in audit['path_comparison']['phases'].items():
    names.append(phase);differences.append(row['metrics']['q']['max_normalized_phase_difference'])
ax.bar(names,differences,color='#007ea8');ax.set(ylabel='Max joint-vector difference (rad)',title='Actual joint paths, normalized per free-motion phase');ax.grid(axis='y',alpha=.2)
fig.suptitle(r['variant']+' avoidance: same reset layout, seed and grasp; original task success',fontsize=13)
for suffix in ('png','pdf'):fig.savefig(output/('path_comparison.'+suffix),dpi=160)
print(output/'path_comparison.png')
