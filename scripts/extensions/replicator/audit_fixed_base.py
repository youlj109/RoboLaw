"""Read-only CPU audit of initial hazards versus source stand/link0 colliders.

Uses actual composed USD collider points and authored mesh approximations,
without the moving-arm CollisionWorld or its /Robot exclusion rule.
"""
import argparse,hashlib,itertools,json
from pathlib import Path
import fcl,h5py,numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from pxr import Gf,Usd,UsdGeom,UsdPhysics
ROOT=Path(__file__).resolve().parents[3]
ASSEMBLY=ROOT/'runtime/home/.cache/isaaclab_arena/usd/robot_on_stand/replicator_local_droid_0.8_1.08x0.91032.usd'

def matrix(pose):
 out=np.eye(4);out[:3,:3]=Rotation.from_quat(pose[3:]).as_matrix();out[:3,3]=pose[:3];return out

def convex(points):
 hull=ConvexHull(points);faces=hull.simplices.copy()
 normals=np.cross(points[faces[:,1]]-points[faces[:,0]],points[faces[:,2]]-points[faces[:,0]])
 rev=np.einsum('ij,ij->i',normals,hull.equations[:,:3])<0;faces[rev]=faces[rev][:,[0,2,1]]
 remap=np.full(len(points),-1,dtype=np.int32);remap[hull.vertices]=np.arange(len(hull.vertices));faces=remap[faces]
 packed=np.c_[np.full(len(faces),3,dtype=np.int32),faces].ravel()
 return fcl.Convex(points[hull.vertices],len(faces),packed)

def collider(prim,world,visible=False):
 name=str(prim.GetPath());approx=UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() if prim.HasAPI(UsdPhysics.MeshCollisionAPI) else None
 if visible: approx=None
 info=dict(path=name,type=prim.GetTypeName(),approximation=approx,world_transform=world.tolist())
 if prim.IsA(UsdGeom.Mesh):
  mesh=UsdGeom.Mesh(prim);points=np.asarray(mesh.GetPointsAttr().Get(),dtype=float);points=points@world[:3,:3].T+world[:3,3]
  counts=mesh.GetFaceVertexCountsAttr().Get();indices=np.asarray(mesh.GetFaceVertexIndicesAttr().Get());faces=[];start=0
  for count in counts:
   poly=indices[start:start+count];start+=count
   faces.extend([poly[0],poly[i],poly[i+1]] for i in range(1,count-1))
  faces=np.asarray(faces,dtype=np.int32)
  if approx=='convexHull':geom=convex(points);semantics='Solid convex hull of authored collision-mesh vertices'
  elif approx in (None,'none','meshSimplification'):
   geom=fcl.BVHModel();geom.beginModel(len(points),len(faces));geom.addSubModel(points,faces);geom.endModel();semantics='Authored collision triangles'
  else:raise ValueError(f'Unhandled mesh approximation {name}: {approx}')
  obj=fcl.CollisionObject(geom);info.update(vertices=len(points),triangles=len(faces),bounds=[points.min(0).tolist(),points.max(0).tolist()],semantics=semantics)
 elif prim.IsA(UsdGeom.Cube):
  h=float(UsdGeom.Cube(prim).GetSizeAttr().Get())/2;points=np.array(list(itertools.product([-h,h],repeat=3)));points=points@world[:3,:3].T+world[:3,3]
  obj=fcl.CollisionObject(convex(points));info.update(bounds=[points.min(0).tolist(),points.max(0).tolist()],semantics='Exact transformed USD cube solid')
 elif prim.IsA(UsdGeom.Cylinder):
  schema=UsdGeom.Cylinder(prim);axis={'X':0,'Y':1,'Z':2}[str(schema.GetAxisAttr().Get())];scale=np.linalg.norm(world[:3,:3],axis=0);rot=world[:3,:3]/scale
  assert np.allclose(rot.T@rot,np.eye(3),atol=1e-6)
  radial=[i for i in range(3) if i!=axis];assert np.isclose(scale[radial[0]],scale[radial[1]],atol=1e-6)
  radius=float(schema.GetRadiusAttr().Get())*scale[radial[0]];height=float(schema.GetHeightAttr().Get())*scale[axis]
  alignment=Rotation.align_vectors(np.eye(3)[axis][None],np.array([[0.,0.,1.]]))[0].as_matrix()
  obj=fcl.CollisionObject(fcl.Cylinder(radius,height),fcl.Transform(rot@alignment,world[:3,3]));info.update(radius_m=radius,height_m=height,semantics='Exact USD cylinder solid with authored scale and axis')
 else:raise ValueError(f'Unhandled collider {name}: {prim.GetTypeName()}')
 return obj,info

def selected(p,visible):
 if visible:
  return p.IsA(UsdGeom.Gprim) and UsdGeom.Imageable(p).ComputeVisibility()!='invisible' and UsdGeom.Imageable(p).ComputePurpose() in ('default','render')
 return p.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()

def fixed_colliders(base_pose,visible=False):
 stage=Usd.Stage.Open(str(ASSEMBLY));assert stage and not stage.GetCompositionErrors()
 root=stage.GetPrimAtPath('/panda/panda_link0');cache=UsdGeom.XformCache();source=np.asarray(cache.GetLocalToWorldTransform(root)).T;adjust=matrix(base_pose)@np.linalg.inv(source)
 rows=[]
 for p in Usd.PrimRange(root,Usd.TraverseInstanceProxies()):
  if selected(p,visible):
   rows.append(collider(p,adjust@np.asarray(cache.GetLocalToWorldTransform(p)).T,visible=visible))
 assert len(rows)==2, [i[1]['path'] for i in rows]
 return rows

def hazard_colliders(entry,pose,visible=False):
 stage=Usd.Stage.CreateInMemory();root=UsdGeom.Xform.Define(stage,'/World/Hazard');root.GetPrim().GetReferences().AddReference(str(ROOT/entry['usd_path']));root.ClearXformOpOrder();root.AddTranslateOp().Set(Gf.Vec3d(*map(float,pose[:3])));q=pose[3:];root.AddOrientOp().Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,q[:3]))));cache=UsdGeom.XformCache();rows=[]
 for p in Usd.PrimRange(root.GetPrim(),Usd.TraverseInstanceProxies()):
  if selected(p,visible):rows.append(collider(p,np.asarray(cache.GetLocalToWorldTransform(p)).T,visible=visible))
 assert rows
 return rows

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--episode',action='append');parser.add_argument('--report',default=str(ROOT/'reports/skill_trials/replicator/fixed_base_audit.json'));args=parser.parse_args()
 paths=[Path(x) for x in args.episode] if args.episode else [ROOT/'datasets/skill_trials/replicator'/v/a for v,a in [('human','attempt_01'),('property','attempt_03'),('robot','attempt_02')]]
 records=[]
 for out in paths:
  variant,attempt=out.parent.name,out.name;result=json.loads((out/'result.json').read_text());entry=result['obstacle']
  with h5py.File(out/'trajectory.h5') as f:
   names=json.loads(f.attrs['robot_body_names']);base_pose=f['obs/robot_body_pose_w'][0,names.index('panda_link0')]
   measured_hazard=f['obs/hazard_pose'][0] if 'hazard_pose' in f['obs'] else None
  fixed=fixed_colliders(base_pose)
  authored=np.r_[entry['position'],Rotation.from_euler('z',entry.get('yaw_deg',0),degrees=True).as_quat()]
  pose_sets=[('authored_initial',authored)]
  if measured_hazard is not None:pose_sets.append(('first_recorded_physics_pose',measured_hazard))
  audits=[]
  geometries={}
  for mode in ('authored_collision','visible_surface'):
   fixed=fixed_colliders(base_pose,visible=mode=='visible_surface')
   geometries[mode]=[x[1] for x in fixed]
   for pose_source,pose in pose_sets:
    hazard=hazard_colliders(entry,pose,visible=mode=='visible_surface');pairs=[]
    for body,body_info in fixed:
     for item,item_info in hazard:
      contacts=fcl.CollisionResult();count=fcl.collide(body,item,fcl.CollisionRequest(num_max_contacts=1,enable_contact=True),contacts)
      raw=float(fcl.distance(body,item,fcl.DistanceRequest(),fcl.DistanceResult()))
      pairs.append(dict(fixed=body_info['path'],hazard=item_info['path'],intersecting=bool(count),distance_m=max(0.,raw),raw_fcl_distance=raw))
    audits.append(dict(geometry_mode=mode,pose_source=pose_source,hazard_pose=pose.tolist(),hazard_colliders=len(hazard),pairs_checked=len(pairs),intersecting_pair_count=sum(x['intersecting'] for x in pairs),minimum_pair=min(pairs,key=lambda x:x['distance_m']),intersections=[x for x in pairs if x['intersecting']],hazard_geometry=[x[1] for x in hazard]))
  records.append(dict(variant=variant,attempt=attempt,base_pose_world_xyz_xyzw=base_pose.tolist(),fixed_geometry=geometries,checks=audits,passed=all(x['intersecting_pair_count']==0 for x in audits)))
 report=dict(passed=all(x['passed'] for x in records),scope='Read-only CPU initial hazard versus actual composed DROID stand and fixed panda_link0 collider geometry. No moving-arm replay. Collision audit: solid convexHull according to USD, exact primitives, triangle meshes; no PhysX cooked-hull tolerance claim. Additional visible geometry audit: all non-proxy visible primitives and actual mesh triangles, checking surface intersection; solid collision checks cover containment where colliders have solid semantics.',assembly=str(ASSEMBLY.relative_to(ROOT)),assembly_sha256=hashlib.sha256(ASSEMBLY.read_bytes()).hexdigest(),episodes=records)
 p=Path(args.report);p.write_text(json.dumps(report,indent=2)+'\n')
 for episode_path,record in zip(paths,records):
  sidecar={k:v for k,v in report.items() if k not in ('episodes','passed')}
  sidecar.update(record)
  target=episode_path/'fixed_base_hazard_audit.json'
  target.write_text(json.dumps(sidecar,indent=2)+'\n')
 print(json.dumps(dict(passed=report['passed'],episodes=[dict(variant=x['variant'],passed=x['passed'],checks=[{k:c[k] for k in ['geometry_mode','pose_source','pairs_checked','intersecting_pair_count','minimum_pair']} for c in x['checks']]) for x in records]),indent=2))
 raise SystemExit(0 if report['passed'] else 1)

if __name__=='__main__':main()
