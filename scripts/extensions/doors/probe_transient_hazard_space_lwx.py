"""Finite source-geometry diagnostic for transient obstruction outside later door/robot volume."""
import argparse, hashlib, itertools, json
from pathlib import Path
import fcl
import numpy as np
from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from kitchen_safety.extensions.doors.task_spheres import initial_spheres
from kitchen_safety.planning import transform
from scripts.extensions.doors.physical_hazard_geometry import physical_robot_shapes
from scripts.extensions.doors.search_avoidance_layouts import clearance
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--baseline',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
m=json.loads((a.baseline/'metadata.json').read_text());rows=json.loads((a.baseline/'trajectory.json').read_text())
_,kin,w,g=reconstruct(m,json.loads(Path(m['variant_config']).read_text()))
pre=[i for i,r in enumerate(rows) if r['phase']=='pregrasp']
if not pre:raise RuntimeError('No actual pregrasp trajectory')
def robot(i):
 q=np.asarray(rows[i]['next_obs']['q']);f={n:q[m['robot_joint_names'].index(n)] for n in w.GRIPPER_JOINT_NAMES}
 return physical_robot_shapes(w.shapes(q[:7],f))
endpoints=robot(0)+robot(pre[-1])+robot(len(rows)-1)
late=list(range(pre[-1],len(rows),6))+[len(rows)-1]
later_shapes=[s for i in late for s in robot(i)]
end=dict(m,initial_q=rows[pre[-1]]['next_obs']['q']);spheres=initial_spheres(m,kin)+initial_spheres(end,kin)
centers=np.asarray([s[2] for s in spheres]);radii=np.asarray([s[3] for s in spheres]);points=[]
for i in pre[5:-5:5]:
 for name,obj,lo,hi in robot(i):
  for weight in itertools.product([.15,.5,.85],repeat=3):
   pos=lo+(hi-lo)*np.asarray(weight);gap=float(np.min(np.linalg.norm(centers-pos,axis=1)-radii))
   if gap<.023:continue
   point=fcl.CollisionObject(fcl.Sphere(.001),fcl.Transform(pos))
   if not fcl.collide(point,obj,fcl.CollisionRequest(),fcl.CollisionResult()):continue
   shapes=[('finite_sample',point,pos-.001,pos+.001)]
   physical=clearance(shapes,endpoints,.023,full=True);gap=min(gap,physical['distance_m'])
   if gap<.023:continue
   fixed=clearance(shapes,w.objects,.023,full=True);gap=min(gap,fixed['distance_m'])
   if gap<.023:continue
   later=clearance(shapes,later_shapes,.023,full=True);gap=min(gap,later['distance_m'])
   if gap<.023:continue
   points.append({'action':i,'robot_part':name,'point':pos.tolist(),'sphere_endpoint_clearance_m':float(np.min(np.linalg.norm(centers-pos,axis=1)-radii)),'endpoint_source_clearance_m':physical['distance_m'],'initial_environment_clearance_m':fixed['distance_m'],'later_robot_clearance_m':later['distance_m'],'minimum_clearance_m':gap,'door_sweep_clearance_m':float('inf'),'_shape':shapes})
 print('TRANSIENT_SOURCE_PROGRESS',i,'points',len(points),flush=True)
initial=transform(m['initial_door_pose'][:3],m['initial_door_pose'][3:]);door_indices=sorted(set([0,*range(0,len(rows),6),len(rows)-1]))
for i in door_indices:
 pose=rows[i]['next_obs']['door_body_pose'];delta=transform(pose[:3],pose[3:])@np.linalg.inv(initial)
 for body in g['moving_bodies']:w.move_body(body,delta)
 door=[s for s in w.objects if any(s[0].startswith(b+'/') for b in g['moving_bodies'])]
 for item in points:
  detail=clearance(item['_shape'],door,.023,full=True)
  if detail['distance_m']<item['door_sweep_clearance_m']:item['door_sweep_clearance_m']=detail['distance_m'];item['nearest_door_action']=i;item['nearest_door_part']=detail.get('hazard_part')
  item['minimum_clearance_m']=min(item['minimum_clearance_m'],detail['distance_m'])
for item in points:item.pop('_shape')
points.sort(key=lambda v:v['minimum_clearance_m'],reverse=True)
report={'baseline':str(a.baseline),'baseline_trajectory_sha256':hashlib.sha256((a.baseline/'trajectory.json').read_bytes()).hexdigest(),'scope':'Finite 1 mm source-sphere samples inside transient measured robot shapes. Original endpoint planner spheres plus source endpoints, sampled later robot, initial environment and sampled moving door. Old endpoint/later joint postures are held fixed in this diagnostic; a new same-Cartesian-target IK posture may have different feasibility. No finite asset dimensions, support, continuous sweep, paired planning or physical acceptance. No surviving point does not prove impossibility.','sampling':{'pregrasp_stride':5,'local_bbox_weights':[.15,.5,.85],'later_robot_stride':6,'door_stride':6,'point_sphere_radius_m':.001},'sampled_candidate_points':len(points),'points_above_clearance':{str(c):sum(v['minimum_clearance_m']>c for v in points) for c in [.023,.063,.103]},'max_combined_clearance_m':points[0]['minimum_clearance_m'] if points else None,'best_samples':points[:120]}
a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='best_samples'}),flush=True)
