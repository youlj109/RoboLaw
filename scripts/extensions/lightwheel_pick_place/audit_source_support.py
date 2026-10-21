"""Independent actual-pose check of measured monotonic source support bounds.

Reads source USD collision vertices directly; minimum height of their convex
hull equals the minimum vertex height. No planning box/sphere proxy is used.
"""
import argparse,json
from pathlib import Path
import h5py,numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,UsdPhysics
from kitchen_safety.extensions.lightwheel_pick_place.collision import _primitive_world_bounds

def audit(episode):
 r=json.loads((episode/'result.json').read_text());model=r['lift_support_contact_model'];source=r['source_support_contact_model'];stage=Usd.Stage.Open(str(episode/'scene.usda'));cache=UsdGeom.XformCache();root=stage.GetPrimAtPath('/World/envs/env_0/lime');inverse=np.linalg.inv(np.array(cache.GetLocalToWorldTransform(root)).T);parts=[]
 for p in Usd.PrimRange(root,Usd.TraverseInstanceProxies()):
  if not p.IsA(UsdGeom.Mesh) or not p.HasAPI(UsdPhysics.CollisionAPI) or not UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get():continue
  mat=inverse@np.array(cache.GetLocalToWorldTransform(p)).T;v=np.array(UsdGeom.Mesh(p).GetPointsAttr().Get());parts.append(v@mat[:3,:3].T+mat[:3,3])
 vertices=np.concatenate(parts);surface=stage.GetPrimAtPath(source['surface']);mat=np.array(cache.GetLocalToWorldTransform(surface)).T;bounds=_primitive_world_bounds(surface,mat)
 if bounds is None:
  assert surface.IsA(UsdGeom.Mesh)
  v=np.array(UsdGeom.Mesh(surface).GetPointsAttr().Get());v=v@mat[:3,:3].T+mat[:3,3];bounds=(v.min(0),v.max(0))
 top=float(bounds[1][2]);anchor=np.array(source['source_resting_xy_m']);tolerance=.0005
 def minimum(pose):return float((vertices@Rotation.from_quat(pose[3:7]).as_matrix().T+pose[:3])[:,2].min())
 with h5py.File(episode/'trajectory.h5') as f:
  phases=[x.decode() for x in f['phase'][:]];first=phases.index('lift');held=f['obs/lime'][first,:7];initial=np.array(r['initial_lime_pose'][:7])
 held_min=minimum(held);initial_min=minimum(initial);floor=held_min-tolerance;initial_floor=floor
 checks={'source_identity':source['surface']=='/World/envs/env_0/kitchen/counter_main_main_group/top_geometry_right',
  'source_rest_matches':abs(initial_min-source['source_resting_min_z_m'])<1e-6 and np.allclose(anchor,initial[:2],atol=1e-7,rtol=0),
  'held_not_lower_than_original':held_min>=initial_min-tolerance,
  'model_calibration_matches':abs(held_min-model['measured_closed_grasp_min_z_m'])<1e-6 and abs(floor-model['initial_lower_bound_m'])<1e-6 and model['jitter_tolerance_m']==tolerance and model['max_xy_displacement_m']==.05 and np.allclose(model['xy_anchor_m'],anchor,atol=1e-7,rtol=0)}
 rows=[]
 for i,item in enumerate(json.loads((episode/'full_world_audit.json').read_text())['measurements']):
  phase=item['phase']
  if phase not in ('pregrasp','approach','close','lift'):continue
  pose=np.array(item['lime_pose']);z=minimum(pose);near_source=z<top+.002;xy=float(np.linalg.norm(pose[:2]-anchor));bound=(floor if phase=='lift' else initial_min-tolerance);recorded=item.get('source_lift_floor_z_m');bound_matches=(phase!='lift' or (recorded is not None and abs(recorded-floor)<1e-7));permitted=(not near_source or (z>=bound-1e-8 and xy<.05))
  rows.append({'physics_index':i,'action':item['action'],'phase':phase,'actual_minimum_z_m':z,'expected_lower_bound_m':bound,'recorded_lift_lower_bound_m':recorded,'near_source_support':near_source,'source_xy_displacement_m':xy,'bound_matches':bound_matches,'support_valid':permitted})
  if phase=='lift':floor=max(floor,z-tolerance)
 checks['physics_bound_sequence']=all(x['bound_matches'] for x in rows);checks['source_support_geometry']=all(x['support_valid'] for x in rows);checks['lift_samples']=sum(x['phase']=='lift' for x in rows)>=8
 return {'passed':bool(all(checks.values())),'checks':{k:bool(v) for k,v in checks.items()},'source_surface_top_m':top,'initial_rest_minimum_z_m':initial_min,'held_minimum_z_m':held_min,'initial_lift_lower_bound_m':initial_floor,'final_lift_lower_bound_m':floor,'measurements':rows,'scope':'Independent direct USD collision vertices and actual physics poses; exact source surface only. Verifies recorded bound before each update and monotonic tightening, not generic relaxed collision clearance.'}

def main():
 p=argparse.ArgumentParser();p.add_argument('episode',type=Path);a=p.parse_args()
 try:report=audit(a.episode)
 except Exception as exc:report={'passed':False,'error':f'{type(exc).__name__}: {exc}'}
 (a.episode/'source_support_audit.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='measurements'},indent=2));return 0 if report['passed'] else 2
if __name__=='__main__':raise SystemExit(main())
