"""Compare measured lime collision meshes/convex hull with its carried OBB, CPU only."""
import json
from pathlib import Path
import numpy as np,h5py
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,Gf
from kitchen_safety.planning import DroidKinematics,transform
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
root=Path(__file__).resolve().parents[3];episode=root/'datasets/skill_trials/lightwheel_pick_place/base_attempt08';r=json.loads((episode/'result.json').read_text())
with h5py.File(episode/'trajectory.h5') as f:
 pose=f['next_obs/lime'][-1,:7];q=f['next_obs/q'][-1]
s=Usd.Stage.Open(str(episode/'scene.usda'));s.SetEditTarget(s.GetSessionLayer());x=UsdGeom.Xformable(s.GetPrimAtPath('/World/envs/env_0/lime'));x.ClearXformOpOrder();x.AddTransformOp().Set(Gf.Matrix4d(transform(pose[:3],pose[3:7]).T.tolist()))
base=r['robot_base_pose'];kin=DroidKinematics(base[:3],Rotation.from_quat(base[3:7]).as_euler('xyz',degrees=True)[2]);world=CollisionWorld(s,kin)
limes=[item for item in world.objects if world._matches(item[0],'lime')];vertices=np.concatenate([world.meshes[name][0] for name,_,_,_ in limes]);hull,hull_vertices=world._convex(vertices);hull_obj=world.fcl.CollisionObject(hull)
T=transform(pose[:3],pose[3:7]);center=T[:3,3]+T[:3,:3]@world.object_center;radius=np.abs(T[:3,:3])@(world.object_size/2)
box=('old_carried_obb',world.fcl.CollisionObject(world.fcl.Box(*world.object_size),world.fcl.Transform(T[:3,:3],center)),center-radius,center+radius)
report={'measured_lime_pose':pose.tolist(),'source_collider_count':len(limes),'source_mesh_bounds':{'low':vertices.min(0).tolist(),'high':vertices.max(0).tolist()},'hull_bounds':{'low':hull_vertices.min(0).tolist(),'high':hull_vertices.max(0).tolist()},'carried_obb_bounds':{'low':box[2].tolist(),'high':box[3].tolist()},'distances':[]}
for name,obj,lo,hi in world.objects:
 if not ('/counter_main_main_group/top_geometry_right' in name or '/plant_main_group/' in name):continue
 ds=[float(world.fcl.distance(source,obj,world.fcl.DistanceRequest(),world.fcl.DistanceResult())) for _,source,_,_ in limes]
 report['distances'].append({'obstacle':name,'source_mesh_raw_distance_m':min(ds),'convex_hull_raw_distance_m':float(world.fcl.distance(hull_obj,obj,world.fcl.DistanceRequest(),world.fcl.DistanceResult())),'carried_obb_raw_distance_m':float(world.fcl.distance(box[1],obj,world.fcl.DistanceRequest(),world.fcl.DistanceResult())),'obstacle_top_z':float(hi[2])})
report['lift_sweeps']=[]
for dy in [0.,-.03,-.06,-.10]:
    invalid=[];minimum=1.
    for k,t in enumerate(np.linspace(0,1,82)):
        hull_obj.setTranslation(np.array([0.,dy*t,.16*t]))
        for name,obj,lo,hi in world.objects:
            if '/plant_main_group/' not in name:continue
            d=float(world.fcl.distance(hull_obj,obj,world.fcl.DistanceRequest(),world.fcl.DistanceResult()));minimum=min(minimum,d)
            if d<.002:invalid.append({'index':k,'fraction':float(t),'obstacle':name,'raw_distance_m':d})
    report['lift_sweeps'].append({'end_delta':[0.,dy,.16],'minimum_raw_plant_distance_m':minimum,'invalid_samples':invalid})
print(json.dumps(report,indent=2));(root/'reports/skill_trials/lightwheel_pick_place/geometry_probe_carried_lime.json').write_text(json.dumps(report,indent=2))
