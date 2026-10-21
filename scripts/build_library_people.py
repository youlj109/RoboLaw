"""Build six textured MPFB people, collision wrappers and catalogue (no Kit required).

Run: .venv/bin/python scripts/build_library_people.py
The selected MakeHuman system assets are CC0. Downloaded source assets stay next to
this library; existing projects/assets/cameras are never modified.
"""
import argparse, concurrent.futures, json, math, os, struct, subprocess, zlib
from pathlib import Path
import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'assets/obstacle_library/people'
PERSONS=['medium_shirt','compact_sport','broad_workwear']
SOURCE_URL='https://files.makehumancommunity.org/asset_packs/makehuman_system_assets/makehuman_system_assets_cc0.zip'
CAMERA2=np.array([2.77,-.88,1.46])

def fetch_sources():
 import urllib.request
 index=json.loads((ROOT/'assets/people/downloads/asset_manifest.json').read_text())
 prefixes=('clothes/female_sportsuit01/','clothes/male_worksuit01/','hair/ponytail01/','skins/young_asian_female/')
 items=[x for x in index if x['name'].startswith(prefixes) and not x['name'].endswith('/')]
 def fetch(x):
  p=OUT/'source_library'/x['name'];p.parent.mkdir(parents=True,exist_ok=True)
  if p.exists() and p.stat().st_size==x['size']:return
  request=urllib.request.Request(SOURCE_URL,headers={'Range':f"bytes={x['offset']}-{x['offset']+x['compressed']+1024}"})
  with urllib.request.urlopen(request,timeout=90) as response:
   if response.status!=206:raise RuntimeError('Source server did not return requested byte range')
   data=response.read()
  header=struct.unpack('<4s5H3L2H',data[:30]);start=30+header[-2]+header[-1]
  packed=data[start:start+x['compressed']];raw=zlib.decompress(packed,-15) if header[3]==8 else packed
  if len(raw)!=x['size']:raise RuntimeError(f"Incomplete source download: {x['name']}")
  p.write_bytes(raw)
 with concurrent.futures.ThreadPoolExecutor(max_workers=6) as pool:list(pool.map(fetch,items))

def triangles_of(mesh,cache):
 points=np.asarray(mesh.GetPointsAttr().Get(),dtype=float)
 matrix=np.asarray(cache.GetLocalToWorldTransform(mesh.GetPrim()))
 points=(np.column_stack((points,np.ones(len(points))))@matrix)[:,:3]
 ids=np.asarray(mesh.GetFaceVertexIndicesAttr().Get(),dtype=np.int64)
 counts=np.asarray(mesh.GetFaceVertexCountsAttr().Get(),dtype=np.int64)
 tris=[];start=0
 for count in counts:
  face=ids[start:start+count];start+=count
  for j in range(1,count-1):tris.append([face[0],face[j],face[j+1]])
 return points,np.asarray(tris,dtype=np.int64)

def ray_intersections(tris,origin,direction):
 # Vectorized Moller-Trumbore: de-duplicate shared triangle-edge crossings.
 e1=tris[:,1]-tris[:,0];e2=tris[:,2]-tris[:,0];p=np.cross(direction,e2);det=np.einsum('ij,ij->i',e1,p)
 valid=np.abs(det)>1e-10;inv=np.zeros_like(det);inv[valid]=1/det[valid]
 tv=origin-tris[:,0];u=np.einsum('ij,ij->i',tv,p)*inv;q=np.cross(tv,e1)
 v=q@direction*inv;t=np.einsum('ij,ij->i',e2,q)*inv
 values=np.sort(t[valid&(u>=0)&(v>=0)&(u+v<=1)&(t>1e-7)])
 return values[np.r_[True,np.diff(values)>1e-5]] if len(values) else values

def wrap_and_measure(folder):
 from trimesh.triangles import closest_point
 metadata=json.loads((folder/'pose.json').read_text())
 stage=Usd.Stage.CreateNew(str(folder/'collidable.usda'))
 UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
 person=UsdGeom.Xform.Define(stage,'/Person').GetPrim();stage.SetDefaultPrim(person);person.GetReferences().AddReference('person.usdc')
 cache=UsdGeom.XformCache();all_points=[];triangle_sets=[];mesh_records=[];texture_paths=[];materials=0
 for prim in stage.Traverse():
  if prim.IsA(UsdGeom.Mesh):
   mesh=UsdGeom.Mesh(prim);mesh.CreateDoubleSidedAttr(True)
   # Static triangle collisions preserve the real hand and sleeve silhouette.
   UsdPhysics.CollisionAPI.Apply(prim);UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('none')
   points,faces=triangles_of(mesh,cache);all_points.append(points);triangle_sets.append(points[faces])
   material,_=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
   mesh_records.append({'path':str(prim.GetPath()),'vertices':len(points),'triangles':len(faces),'bound_material':str(material.GetPath()) if material else None,'collision':'none'})
  if prim.IsA(UsdShade.Material):materials+=1
  if prim.IsA(UsdShade.Shader):
   shader=UsdShade.Shader(prim)
   if shader.GetIdAttr().Get()=='UsdPreviewSurface' and any(n in str(prim.GetPath()) for n in ('short02','ponytail01','eyebrow','BrownEyes')):shader.CreateInput('opacityThreshold',Sdf.ValueTypeNames.Float).Set(.5)
   for inp in shader.GetInputs():
    value=inp.Get()
    if isinstance(value,Sdf.AssetPath) and value.path:
     resolved=value.resolvedPath or str((folder/value.path).resolve())
     texture_paths.append({'path':value.path,'resolved':resolved,'exists':Path(resolved).exists()})
 stage.GetRootLayer().Save()
 points=np.concatenate(all_points);low=points.min(axis=0);high=points.max(axis=0)
 angle=math.radians(metadata['yaw_deg']);R=np.array([[math.cos(angle),-math.sin(angle),0],[math.sin(angle),math.cos(angle),0],[0,0,1]])
 camlocal=(CAMERA2-np.array(metadata['position']))@R
 mindistance=float('inf');inside=[]
 for record,tris in zip(mesh_records,triangle_sets):
  # Three skew rays reduce edge/near-tangent ambiguity in this point-inside check.
  votes=[]
  for direction in [np.array([1.,.173,.389]),np.array([.271,1.,.473]),np.array([.193,.317,1.])]:
   direction/=np.linalg.norm(direction);votes.append(len(ray_intersections(tris,camlocal,direction))%2==1)
  if sum(votes)>=2:inside.append(record['path'])
  cp=closest_point(tris,np.broadcast_to(camlocal,(len(tris),3)));mindistance=min(mindistance,float(np.linalg.norm(cp-camlocal,axis=1).min()))
 validation={'meters_per_unit':UsdGeom.GetStageMetersPerUnit(stage),'up_axis':str(UsdGeom.GetStageUpAxis(stage)),'mesh_count':len(mesh_records),'material_count':materials,'meshes':mesh_records,'textures':texture_paths,'all_textures_exist':all(x['exists'] for x in texture_paths),'all_meshes_have_materials':all(x['bound_material'] for x in mesh_records),'bounds_min_m':low.tolist(),'bounds_max_m':high.tolist(),'dimensions_m':(high-low).tolist(),'fixed_external_camera_2_world':CAMERA2.tolist(),'camera2_inside_meshes':inside,'camera2_min_surface_distance_m':mindistance,'camera2_point_check_passed':not inside,'simulation_validated':False}
 (folder/'validation.json').write_text(json.dumps(validation,ensure_ascii=False,indent=2)+'\n')
 entry={'id':metadata['id'],'label':metadata['label'],'category':'human','usd_path':str((folder/'collidable.usda').relative_to(ROOT)),'position':metadata['position'],'yaw_deg':metadata['yaw_deg'],'static':True,'dimensions':(high-low).tolist(),'nominal_standing_height_m':metadata['nominal_standing_height_m'],'keypoints':{k:metadata[k] for k in ['wrist_local','wrist_world','hand_tip_local','elbow_local','shoulder_local']},'provenance':{'generator':'MPFB 2.0.18 / MakeHuman Community','source_url':SOURCE_URL,'asset_license':'CC0 (MakeHuman system asset pack and generated mesh)','code_license':'GPL-3.0-or-later (MPFB)','clothing':metadata['clothing'],'hair':metadata['hair'],'macro_parameters':metadata['macro_parameters'],'generator_script':'scripts/build_library_people.py'},'notes':['Textured static adult; hand aligned with unchanged bottle-to-bowl corridor.','All camera mounts, intrinsics and task endpoints are unchanged.','Different body macro parameters and garment geometry, not only recoloring.','Static triangle collisions; no human biomechanics or injury model.'],'validation_path':str((folder/'validation.json').relative_to(ROOT))}
 print('VALIDATED',entry['id'],'camera2_distance',round(mindistance,3),'inside',inside,flush=True)
 return entry

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--person',choices=PERSONS);parser.add_argument('--skip-blender',action='store_true');parser.add_argument('--blender',default='/home/user/.local/opt/blender-4.5.4-linux-x64/blender');args=parser.parse_args()
 OUT.mkdir(parents=True,exist_ok=True)
 if not args.skip_blender:
  fetch_sources()
  for person in [args.person] if args.person else PERSONS:
   subprocess.run([args.blender,'-b','-t','6','--python',str(ROOT/'scripts/build_library_people_blender.py'),'--','--person',person],cwd=ROOT,check=True)
 entries=[]
 for person in PERSONS:
  for pose in ['palm_down','raised_wrist']:
   folder=OUT/person/pose
   if (folder/'person.usdc').exists():entries.append(wrap_and_measure(folder))
 (OUT/'catalog.json').write_text(json.dumps({'assets':entries},ensure_ascii=False,indent=2)+'\n')
 (OUT/'SOURCES.txt').write_text('MPFB/MakeHuman obstacle people\n=============================\nGenerated with MPFB 2.0.18, same locally installed version as assets/people.\nCode: https://github.com/makehumancommunity/mpfb2 (GPL-3.0-or-later).\nSystem asset pack: '+SOURCE_URL+'\nAssets: CC0; upstream .mhclo/.mhmat license metadata preserved in source_library.\nShared source assets: assets/people/mpfb_library (skin, hair, shoes, eyes and eyebrows).\nNew source assets: source_library (sportsuit, worksuit, ponytail and female skin).\nLocal work: three adult body macro configurations; two reaching poses each; normalized metric height; portable materials; static USD mesh colliders.\nRebuild: .venv/bin/python scripts/build_library_people.py\nCamera mounts/intrinsics are never modified by this builder.\nThe point-inside camera check is geometric only; rendered visibility is validated separately.\n')
 print('CATALOG',len(entries),flush=True)
if __name__=='__main__':main()
