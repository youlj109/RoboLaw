"""B03 raw-mesh/source-contact audit, with independent support state per banana."""
import argparse,json
from pathlib import Path
import fcl,h5py,numpy as np
from pxr import Usd,UsdGeom,UsdPhysics
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
from kitchen_safety.extensions.lightwheel_pick_place.task_config import episode_spec


def meshes(stage,root,cache):
    inv=np.linalg.inv(np.asarray(cache.GetLocalToWorldTransform(root)).T);parts=[]
    for prim in Usd.PrimRange(root,Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh) or not prim.HasAPI(UsdPhysics.CollisionAPI) or not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():continue
        mesh=UsdGeom.Mesh(prim);mat=inv@np.asarray(cache.GetLocalToWorldTransform(prim)).T;v=np.asarray(mesh.GetPointsAttr().Get(),float)@mat[:3,:3].T+mat[:3,3];ix=np.asarray(mesh.GetFaceVertexIndicesAttr().Get(),int);faces=[];offset=0
        for count in mesh.GetFaceVertexCountsAttr().Get():
            poly=ix[offset:offset+count];faces.extend([[poly[0],poly[j],poly[j+1]] for j in range(1,count-1)]);offset+=count
        parts.append((v,np.asarray(faces,np.int32)))
    if not parts:raise ValueError('No source collision mesh: '+str(root.GetPath()))
    return parts


def audit(ep):
    r=json.loads((ep/'result.json').read_text());spec=episode_spec(r);stage=Usd.Stage.Open(str(ep/'scene.usda'));cache=UsdGeom.XformCache();surface=stage.GetPrimAtPath(spec.source_surface_abs);source_parts=meshes(stage,surface,cache);plate_models=[]
    for v,faces in source_parts:
        model=fcl.BVHModel();model.beginModel(len(v),len(faces));model.addSubModel(v,faces);model.endModel();plate_models.append(model)
    builder=CollisionWorld.__new__(CollisionWorld);builder.fcl=fcl;vertices={};banana_models={}
    for pick in spec.pick_objects:
        v=np.concatenate([v for v,f in meshes(stage,stage.GetPrimAtPath('/World/envs/env_0/'+pick),cache)]);banana_models[pick],vertices[pick]=builder._convex(v)
    models={spec.pick_object:r['source_support_contact_model'],**r.get('extra_source_support_models',{})};detached={n:False for n in spec.pick_objects};seen={n:0 for n in spec.pick_objects};supported={n:False for n in spec.pick_objects};rows=[];checks={'all_pick_models':set(models)==set(spec.pick_objects)}
    with h5py.File(ep/'trajectory.h5') as h:
        plate=h['next_obs/plate'][:];ranges=r['pick_action_ranges'];initial_plate=np.array(r['initial_object_states']['plate'][:7]);movement=np.linalg.norm(plate[:,:3]-initial_plate[:3],axis=1);angles=Rotation.from_quat(plate[:,3:7]).inv()*Rotation.from_quat(initial_plate[3:7]);checks['source_plate_stable']=bool(movement.max()<=.001 and np.rad2deg(angles.magnitude()).max()<=1.)
        states={n:h['next_obs/'+n][:] for n in spec.pick_objects}
        calibrated={}
        for pick in spec.pick_objects:
            initial=np.array(r['initial_object_states'][pick][:7]) if pick==spec.pick_object else h['obs/'+pick][ranges[pick]['start'],:7]
            cloud=vertices[pick]@Rotation.from_quat(initial[3:7]).as_matrix().T+initial[:3]
            minimum=float(cloud[:,2].min());calibrated[pick]={'floor':minimum-.0005,'anchor':initial[:2]}
            checks['native_calibration_'+pick]=bool(abs(minimum-models[pick]['source_resting_min_z_m'])<1e-6 and np.allclose(initial[:3],models[pick]['source_resting_root_m'],atol=1e-6,rtol=0))
    for index,item in enumerate(json.loads((ep/'full_world_audit.json').read_text())['measurements']):
        pick=item.get('pick_object');action=item['action'];phase=item['phase']
        if pick not in models:checks['all_pick_labels']=False;continue
        seen[pick]+=1;model=models[pick];pose=np.asarray(item['lime_pose'],float);rotation=Rotation.from_quat(pose[3:7]).as_matrix();points=vertices[pick]@rotation.T+pose[:3];minimum=float(points[:,2].min());floor=calibrated[pick]['floor'];anchor=calibrated[pick]['anchor'];xy=float(np.linalg.norm(pose[:2]-anchor));source_contact=item['source_contact'];count=source_contact.get('count',-1);before=detached[pick];top=float(model.get('source_surface_top_m',r['source_support_contact_model']['surface_top_m']));after=bool(before or (minimum>=top+.002 and count==0));region=phase in ('pregrasp','approach','close','lift') and not before and xy<.05 and minimum>=floor-1e-8
        contact_ok=isinstance(count,int) and count>=0
        for key,width in [('points_world_m',3),('normals_world',3),('normal_forces_n',1),('separations_m',1)]:
            values=np.asarray(source_contact.get(key,[]),float);contact_ok=bool(contact_ok and values.size==count*width and np.isfinite(values).all())
        native_calibration=False
        if r['source_support_contact_model']['mode']=='native_contact_calibrated_until_detached':
            normals=np.asarray(source_contact.get('normals_world',[]),float).reshape(-1,3)
            contact_ok=bool(contact_ok and (count==0 or (min(source_contact['separations_m'])>=-.0005 and np.allclose(np.linalg.norm(normals,axis=1),1.,atol=1e-4))))
            native_calibration=bool(contact_ok and count>0 and not before and phase in ('pregrasp','approach','close','lift') and min(source_contact['separations_m'])>=-.0005 and np.allclose(np.linalg.norm(normals,axis=1),1.,atol=1e-4))
            if native_calibration:calibrated[pick]['floor']=minimum-.0005
            floor=calibrated[pick]['floor']
            contact_ok=bool(contact_ok and item.get('source_calibration_native_contact')==native_calibration and abs(item.get('source_support_calibrated_min_z_m',float('inf'))-(floor+.0005))<1e-6)
            region=phase in ('pregrasp','approach','close','lift') and not before and xy<.05 and minimum>=floor-1e-8
        supported[pick]|=bool(count>0 and not before and phase in ('approach','pregrasp','close'))
        banana=fcl.CollisionObject(banana_models[pick],fcl.Transform(rotation,pose[:3]));plate_pose=np.asarray(item.get('rigid_poses',{}).get('plate',plate[action,:7]),float);plate_rotation=Rotation.from_quat(plate_pose[3:7]).as_matrix();distance=float('inf');intersecting=False
        for plate_model in plate_models:
            obj=fcl.CollisionObject(plate_model,fcl.Transform(plate_rotation,plate_pose[:3]));raw=float(fcl.distance(banana,obj,fcl.DistanceRequest(),fcl.DistanceResult()));hit=raw<0 or (raw==0 and bool(fcl.collide(banana,obj,fcl.CollisionRequest(),fcl.CollisionResult())));distance=min(distance,max(0.,raw));intersecting|=hit
        source_clear=not intersecting and distance>=.002
        aligned=(ranges[pick]['start']<=action<ranges[pick]['end'] and abs(minimum-item['source_hull_min_z_m'])<1e-6 and item['source_detached_before']==before and item['source_detached_after']==after)
        row={'physics_index':index,'pick':pick,'action':action,'phase':phase,'raw_hull_min_z_m':minimum,'native_floor_m':floor,'source_distance_m':distance,'source_intersecting':intersecting,'bounded_source_exception':bool(region),'contacts_valid':bool(contact_ok),'state_aligned':bool(aligned),'passed':bool(contact_ok and aligned and (source_clear or region))};rows.append(row);detached[pick]=after
    checks.update(all_picks_measured=all(seen.values()),initial_physical_support=all(supported.values()),all_picks_detached=all(detached.values()),geometry_and_contact_state=bool(rows and all(x['passed'] for x in rows)))
    return {'passed':bool(all(checks.values())),'checks':checks,'measurements':rows,'scope':'Independent raw USD banana convex hulls and concave plate meshes, per-banana native rest floors, raw exact-source contacts, permanent detach, and measured plate states at each recorded action. The plate must remain within 1 mm / 1 degree; intra-action source pose is resolved at 15 Hz while banana/contact measurements remain at 120 Hz. All external safety margins are unchanged.'}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();report=audit(a.episode)
    with a.output.open('x') as f:json.dump(report,f,indent=2)
    print({k:v for k,v in report.items() if k!='measurements'});raise SystemExit(0 if report['passed'] else 2)
