"""Independent actual-pose check of measured, source-only support classifications.

Reads source USD collision vertices directly; minimum height of their convex
hull equals the minimum vertex height. No planning box/sphere proxy is used.
"""
import argparse,json
from pathlib import Path
import h5py,numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,UsdPhysics
from kitchen_safety.extensions.lightwheel_pick_place.collision import _primitive_world_bounds

from kitchen_safety.extensions.lightwheel_pick_place.source_support import near_support_slab

def save_report(report,path):
 """Preserve historical evidence; a changed recomputation gets a new sidecar."""
 path=Path(path);payload=json.dumps(report,indent=2)
 if path.exists() and path.read_text()==payload:return path
 candidate=path;index=1
 while True:
  try:
   with candidate.open('x') as stream:stream.write(payload)
   return candidate
  except FileExistsError:
   candidate=path.with_name(f'{path.stem}.recheck{index:02d}{path.suffix}');index+=1

from kitchen_safety.extensions.lightwheel_pick_place.task_config import episode_spec,pick_object_name,initial_pick_pose,h5_pose_dataset

def audit(episode):
 r=json.loads((episode/'result.json').read_text());model=r['lift_support_contact_model'];source=r['source_support_contact_model'];stage=Usd.Stage.Open(str(episode/'scene.usda'));cache=UsdGeom.XformCache();spec=episode_spec(r);pick=pick_object_name(r,spec);root=stage.GetPrimAtPath('/World/envs/env_0/'+pick);inverse=np.linalg.inv(np.array(cache.GetLocalToWorldTransform(root)).T);parts=[]
 for p in Usd.PrimRange(root,Usd.TraverseInstanceProxies()):
  if not p.IsA(UsdGeom.Mesh) or not p.HasAPI(UsdPhysics.CollisionAPI) or not UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get():continue
  mat=inverse@np.array(cache.GetLocalToWorldTransform(p)).T;v=np.array(UsdGeom.Mesh(p).GetPointsAttr().Get());parts.append(v@mat[:3,:3].T+mat[:3,3])
 vertices=np.concatenate(parts);surface=stage.GetPrimAtPath(source['surface']);mat=np.array(cache.GetLocalToWorldTransform(surface)).T;bounds=_primitive_world_bounds(surface,mat)
 if bounds is None:
  assert surface.IsA(UsdGeom.Mesh)
  v=np.array(UsdGeom.Mesh(surface).GetPointsAttr().Get());v=v@mat[:3,:3].T+mat[:3,3];bounds=(v.min(0),v.max(0))
 top=float(bounds[1][2]);anchor=np.array(source['source_resting_xy_m']);tolerance=.0005
 def world_vertices(pose):return vertices@Rotation.from_quat(pose[3:7]).as_matrix().T+pose[:3]
 def minimum(pose):return float(world_vertices(pose)[:,2].min())
 with h5py.File(episode/'trajectory.h5') as f:
  phases=[x.decode() for x in f['phase'][:]];first=phases.index('lift');held=f['obs/'+h5_pose_dataset(f['obs'],pick,r)][first,:7];initial=np.array(initial_pick_pose(r)[:7])
 held_min=minimum(held);initial_min=minimum(initial);floor=held_min-tolerance;initial_floor=floor
 if model.get('mode')=='native_rest_until_detached':
  floor=initial_min-tolerance;detached=False;rows=[];contacts_valid=True
  initial_contact=source.get('initial_source_contacts',{})
  checks={'source_identity':source['surface']==(spec.source_surface_abs if spec.source_surface_abs else '/World/envs/env_0/kitchen'+spec.source_surface_suffix),
   'native_source_calibration':abs(initial_min-source['source_resting_min_z_m'])<1e-6 and np.allclose(source['source_resting_root_m'],initial[:3],atol=1e-7,rtol=0),
   'bounded_contact_region':source['source_resting_extra_tolerance_m']==tolerance and source['source_resting_max_xy_displacement_m']==.05 and source['detach_clearance_m']==.002,
   'initial_physical_support':initial_contact.get('count',0)>0,
   'held_not_below_native_rest':held_min>=floor,
   'lift_calibration_matches':abs(model['initial_lower_bound_m']-floor)<1e-6 and model['jitter_tolerance_m']==tolerance and model['max_xy_displacement_m']==.05}
  for i,item in enumerate(json.loads((episode/'full_world_audit.json').read_text())['measurements']):
   pose=np.array(item['lime_pose']);z=minimum(pose);xy=float(np.linalg.norm(pose[:2]-anchor));contact=item.get('source_contact',{});count=contact.get('count',-1)
   arrays={key:np.asarray(contact.get(key,[]),dtype=float) for key in ('points_world_m','normals_world','normal_forces_n','separations_m')}
   contact_ok=(isinstance(count,int) and count>=0 and arrays['points_world_m'].size==3*count and arrays['normals_world'].size==3*count and arrays['normal_forces_n'].size==count and arrays['separations_m'].size==count and all(np.isfinite(x).all() for x in arrays.values()))
   if count>0 and contact_ok:
    points=arrays['points_world_m'].reshape(-1,3);normals=arrays['normals_world'].reshape(-1,3)
    contact_ok=bool(np.all(points[:,:2]>=bounds[0][:2]-.006) and np.all(points[:,:2]<=bounds[1][:2]+.006) and np.all(np.abs(points[:,2]-top)<=.006) and np.allclose(np.linalg.norm(normals,axis=1),1,atol=1e-4))
   contacts_valid &= contact_ok
   phase=item['phase'];near=near_support_slab(hull_min_z=z,hull_max_z=float(world_vertices(pose)[:,2].max()),surface_bottom=float(bounds[0][2]),surface_top=top)
   # On the actual lift/approach column, only the exact source region can
   # receive an exception. Once detached, it cannot be re-enabled by a fall.
   region=xy<.05 and z>=floor-1e-8
   # Early attempt03 additionally bounded an arbitrary asset origin. Preserve
   # that recorded policy when auditing history, but newer samples use the
   # physically relevant collision hull, whose bottom already bounds support.
   if source.get('root_z_gate',True):region=bool(region and pose[2]>=initial[2]-tolerance-1e-8)
   allowed=phase in ('pregrasp','approach','close','lift') and not detached and region
   in_source_xy=bool(np.all(pose[:2]>=bounds[0][:2]) and np.all(pose[:2]<=bounds[1][:2]))
   permitted=not (near and in_source_xy) or allowed
   next_detached=detached or (z>=top+.002 and count==0)
   state_match=item.get('source_detached_before')==detached and item.get('source_detached_after')==next_detached
   height_match=abs(item.get('source_hull_min_z_m',float('inf'))-z)<1e-7
   bound_match=phase!='lift' or abs(item.get('source_lift_floor_z_m',float('inf'))-floor)<1e-7
   rows.append({'physics_index':i,'action':item['action'],'phase':phase,'actual_minimum_z_m':z,'native_rest_lower_bound_m':floor,'source_xy_displacement_m':xy,'source_contact_count':count,'contacts_valid':contact_ok,'detached_before':detached,'detached_after':next_detached,'recorded_state_matches':state_match and height_match and bound_match,'support_valid':permitted})
   detached=next_detached
  checks['raw_exact_source_contacts_valid']=contacts_valid;checks['recorded_contact_state_sequence']=all(x['recorded_state_matches'] for x in rows);checks['native_support_and_permanent_detach']=all(x['support_valid'] for x in rows);checks['actually_detached']=detached
  return {'passed':bool(all(checks.values())),'checks':{k:bool(v) for k,v in checks.items()},'mode':model['mode'],'root_z_gate':source.get('root_z_gate',True),'source_surface_top_m':top,'initial_rest_minimum_z_m':initial_min,'held_minimum_z_m':held_min,'source_lower_bound_m':floor,'measurements':rows,'scope':'Independent raw USD vertices and 120 Hz measured contacts/poses. Exact source-only native-rest hull floor, 50 mm xy, permanent revoke after >=2 mm separation and zero contacts. Old root-origin policy is honored only when recorded. No human/property/barrier margin changes.'}
 checks={'source_identity':source['surface']==(spec.source_surface_abs if spec.source_surface_abs else '/World/envs/env_0/kitchen'+spec.source_surface_suffix),
  'source_rest_matches':abs(initial_min-source['source_resting_min_z_m'])<1e-6 and np.allclose(anchor,initial[:2],atol=1e-7,rtol=0),
  'held_not_lower_than_original':held_min>=initial_min-tolerance,
  'model_calibration_matches':abs(held_min-model['measured_closed_grasp_min_z_m'])<1e-6 and abs(floor-model['initial_lower_bound_m'])<1e-6 and model['jitter_tolerance_m']==tolerance and model['max_xy_displacement_m']==.05 and np.allclose(model['xy_anchor_m'],anchor,atol=1e-7,rtol=0)}
 rows=[]
 for i,item in enumerate(json.loads((episode/'full_world_audit.json').read_text())['measurements']):
  phase=item['phase']
  if phase not in ('pregrasp','approach','close','lift'):continue
  pose=np.array(item['lime_pose']);z=minimum(pose);near_source=near_support_slab(hull_min_z=z,hull_max_z=float(world_vertices(pose)[:,2].max()),surface_bottom=float(bounds[0][2]),surface_top=top);xy=float(np.linalg.norm(pose[:2]-anchor));bound=(floor if phase=='lift' else initial_min-tolerance);recorded=item.get('source_lift_floor_z_m');bound_matches=(phase!='lift' or (recorded is not None and abs(recorded-floor)<1e-7));permitted=(not near_source or (z>=bound-1e-8 and xy<.05))
  rows.append({'physics_index':i,'action':item['action'],'phase':phase,'actual_minimum_z_m':z,'expected_lower_bound_m':bound,'recorded_lift_lower_bound_m':recorded,'near_source_support':near_source,'source_xy_displacement_m':xy,'bound_matches':bound_matches,'support_valid':permitted})
  if phase=='lift':floor=max(floor,z-tolerance)
 checks['physics_bound_sequence']=all(x['bound_matches'] for x in rows);checks['source_support_geometry']=all(x['support_valid'] for x in rows);checks['lift_samples']=sum(x['phase']=='lift' for x in rows)>=8
 return {'passed':bool(all(checks.values())),'checks':{k:bool(v) for k,v in checks.items()},'source_surface_top_m':top,'initial_rest_minimum_z_m':initial_min,'held_minimum_z_m':held_min,'initial_lift_lower_bound_m':initial_floor,'final_lift_lower_bound_m':floor,'measurements':rows,'scope':'Independent direct USD collision vertices and actual physics poses; exact source surface only. Verifies recorded bound before each update and monotonic tightening, not generic relaxed collision clearance.'}

def main():
 p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('--output',type=Path);a=p.parse_args()
 try:report=audit(a.episode)
 except Exception as exc:report={'passed':False,'error':f'{type(exc).__name__}: {exc}'}
 target=save_report(report,a.output or a.episode/'source_support_audit.json');print(json.dumps({'report_path':str(target),**{k:v for k,v in report.items() if k!='measurements'}},indent=2));return 0 if report['passed'] else 2
if __name__=='__main__':raise SystemExit(main())
