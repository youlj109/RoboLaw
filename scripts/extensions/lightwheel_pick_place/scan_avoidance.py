"""CPU-only path-driven placement scan; never execute a dangerous counterfactual.

Require source geometry, fixed stand, initial and contact/placement states to be
clear. Select only hazards that violate an unchanged execution margin along the
measured obstacle-free pregrasp/transfer path. Full physics-rate replay follows
selection; this sparse scan is a candidate filter, not final acceptance.
"""
import argparse,json,itertools,time
from pathlib import Path
import fcl,h5py,numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,UsdPhysics,Gf
from kitchen_safety.geometry_audit import static_collision_shapes
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld

ROOT=Path(__file__).resolve().parents[3]

def fixed_shapes(stage):
    cache=UsdGeom.XformCache();out=[]
    for p in Usd.PrimRange(stage.GetPrimAtPath('/World/envs/env_0/Robot/panda_link0'),Usd.TraverseInstanceProxies()):
        if not p.HasAPI(UsdPhysics.CollisionAPI) or not UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get():continue
        if not p.IsA(UsdGeom.Mesh):raise RuntimeError('Unsupported fixed collider '+str(p.GetPath()))
        mesh=UsdGeom.Mesh(p);v=np.asarray(mesh.GetPointsAttr().Get(),dtype=float);t=np.asarray(cache.GetLocalToWorldTransform(p)).T;v=v@t[:3,:3].T+t[:3,3]
        ix=np.asarray(mesh.GetFaceVertexIndicesAttr().Get());faces=[];offset=0
        for n in mesh.GetFaceVertexCountsAttr().Get():
            poly=ix[offset:offset+n];offset+=n
            faces.extend([[poly[0],poly[j],poly[j+1]] for j in range(1,n-1)])
        model=fcl.BVHModel();model.beginModel(len(v),len(faces));model.addSubModel(v,np.asarray(faces,dtype=np.int32));model.endModel()
        out.append((str(p.GetPath()),fcl.CollisionObject(model),v.min(0),v.max(0)))
    return out

def minimum(shapes,hazards,margin,cutoff=.08):
    lo=np.array([s[2] for s in shapes]);hi=np.array([s[3] for s in shapes]);hl=np.array([s[2] for s in hazards]);hh=np.array([s[3] for s in hazards])
    broad=np.linalg.norm(np.maximum(np.maximum(lo[:,None]-hh[None],hl[None]-hi[:,None]),0),axis=2)
    pairs=sorted((float(broad[i,j]),i,j) for i,j in zip(*np.nonzero(broad<=cutoff)))
    best={'valid':True,'distance_m':cutoff,'slack_m':cutoff-margin,'required_m':margin,'robot_part':None,'obstacle':None,'intersecting':False}
    for bound,i,j in pairs:
        if bound>best['distance_m']:break
        raw=float(fcl.distance(shapes[i][1],hazards[j][1],fcl.DistanceRequest(),fcl.DistanceResult()))
        hit=raw<0 or (raw==0 and bool(fcl.collide(shapes[i][1],hazards[j][1],fcl.CollisionRequest(),fcl.CollisionResult())))
        d=max(0.,raw)
        if d<=best['distance_m']:
            best={'valid':not hit and d>=margin,'distance_m':d,'slack_m':d-margin,'required_m':margin,'robot_part':shapes[i][0],'obstacle':hazards[j][0],'intersecting':hit}
            if hit:break
    return best

def initial_intersections(source,hazards,*,intended_support_roots=()):
    sl=np.array([s[2] for s in source]);sh=np.array([s[3] for s in source]);bad=[];support=[]
    for hn,ho,hl,hh in hazards:
        bounds=np.linalg.norm(np.maximum(np.maximum(sl-hh,hl-sh),0),axis=1)
        for i in np.flatnonzero(bounds<.003):
            sn,so,lo,hi=source[i]
            raw=float(fcl.distance(ho,so,fcl.DistanceRequest(),fcl.DistanceResult()))
            hit=raw<0 or (raw==0 and bool(fcl.collide(ho,so,fcl.CollisionRequest(),fcl.CollisionResult())))
            if not hit:continue
            exact_support=any(sn==root or sn.startswith(root.rstrip('/')+'/') for root in intended_support_roots)
            intended=(exact_support and hl[2]>=hi[2]-.006) or ('/floor' in sn.lower() and hl[2]>=hi[2]-.006) or ('/counter_' in sn and hl[2]>=hi[2]-.006)
            (support if intended else bad).append({'hazard':hn,'source':sn,'raw_distance_m':raw})
            if bad:return bad,support
    return bad,support

def main():
    p=argparse.ArgumentParser();p.add_argument('--baseline',default='datasets/skill_trials/lightwheel_pick_place/base_attempt12');p.add_argument('--variant',choices=['human','property','robot'],required=True);p.add_argument('--output',required=True);p.add_argument('--stride',type=int,default=8);p.add_argument('--candidates',help='Optional JSON list of xyz/yaw candidate dicts');p.add_argument('--asset',help='Explicit alternate added hazard asset, original assets unchanged');p.add_argument('--endpoints-only',action='store_true',help='Require initial/final states only; contact phases may need a new collision-free redundant IK branch');a=p.parse_args()
    ep=Path(a.baseline);r=json.loads((ep/'result.json').read_text());b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2])
    from kitchen_safety.extensions.lightwheel_pick_place.task_config import episode_spec
    task=episode_spec(r);pick=task.pick_object
    spec=json.loads(task.default_variants.read_text())[a.variant]
    if a.asset:spec['asset']=a.asset
    stage=Usd.Stage.Open(str(ep/'scene.usda'));stage.SetEditTarget(stage.GetSessionLayer());x=UsdGeom.Xform.Define(stage,'/World/envs/env_0/'+spec['name']);x.GetPrim().GetReferences().AddReference(str(ROOT/spec['asset']));x.AddTranslateOp().Set(Gf.Vec3d(0,0,0));x.AddRotateZOp().Set(0.)
    world=CollisionWorld(stage,kin,pick_object=pick,destination_object=task.destination_object,source_surface_suffix=task.source_surface_suffix,movable_objects=tuple(n for n in task.rigid_objects if n!=pick)+('glass',))
    for name,state in r['initial_object_states'].items():world.set_obstacle_pose(name,state[:7])
    fixed=fixed_shapes(stage);source=[v for v in world.objects if not world._matches(v[0],spec['name'])]+fixed
    with h5py.File(ep/'trajectory.h5') as f:
        q=f['next_obs/q'][:];lime=f['next_obs/'+pick][:,:7];phases=f['phase'].asstr()[:]
        pick_poses={name:f['next_obs/'+name][:,:7] for name in (task.pick_objects or (pick,))}
        initial_q=f['obs/q'][0];initial_lime=f['obs/'+pick][0,:7]
    joints=r['robot_joint_names'];arm=[joints.index('panda_joint'+str(i)) for i in range(1,8)];fingers=[joints.index(n) for n in world.GRIPPER_JOINT_NAMES]
    selected=sorted(set([0,len(q)-1,*range(0,len(q),a.stride),*[i for i in range(1,len(q)) if phases[i]!=phases[i-1]],*[i-1 for i in range(1,len(q)) if phases[i]!=phases[i-1]]]))
    shapes={}
    for i in selected:
        active=next((name for name,bounds in r.get('pick_action_ranges',{}).items() if bounds['start']<=i<bounds['end']),pick)
        if world.pick_object!=active:world.bind_pick(active)
        shapes[i]=world.shapes(q[i,arm],dict(zip(world.GRIPPER_JOINT_NAMES,q[i,fingers])),pick_poses[active][i])
    if world.pick_object!=pick:world.bind_pick(pick)
    shapes[-1]=world.shapes(initial_q[arm],dict(zip(world.GRIPPER_JOINT_NAMES,initial_q[fingers])),initial_lime)
    movable=('pregrasp','transfer')
    endpoint={-1,*[i for i in selected if phases[i] not in movable],*[i for i in selected if i==len(q)-1 or (i>0 and phases[i]!=phases[i-1]) or (i+1<len(q) and phases[i]!=phases[i+1])]}
    if a.endpoints_only:endpoint={-1,0,len(q)-1}
    path=[i for i in selected if i not in endpoint]
    if a.candidates:candidates=json.loads(Path(a.candidates).read_text())
    elif a.variant=='human':candidates=[{'position':[float(x),float(y),0.],'yaw_deg':float(yaw)} for x,y,yaw in itertools.product(np.arange(3.05,4.91,.15),np.arange(-1.95,-.64,.15),range(0,360,30))]
    else:candidates=[{'position':[float(x),float(y),.922],'yaw_deg':float(yaw)} for x,y,yaw in itertools.product(np.arange(3.4,4.51,.05),np.arange(-.55,-.09,.05),([0] if a.variant=='property' else [0,90,180,270]))]
    start=time.perf_counter();rows=[];counts={'total':len(candidates),'source_or_fixed_intersection':0,'endpoint_violation':0,'unconstrained':0,'effective_candidates':0};margin=world.execution_clearances[spec['name']]
    for k,candidate in enumerate(candidates):
        pose=[*candidate['position'],*Rotation.from_euler('z',candidate.get('yaw_deg',0),degrees=True).as_quat()];world.set_obstacle_pose(spec['name'],pose)
        # Live construction uses world AABBs for primitive colliders. Pose
        # updates otherwise rotate their old boxes into tighter OBBs; using
        # those here would overstate feasibility for non-zero-yaw candidates.
        hazards=[(name,obj if name in world.meshes else fcl.CollisionObject(fcl.Box(*np.maximum(hi-lo,1e-4)),fcl.Transform((lo+hi)/2)),lo,hi)
                 for name,obj,lo,hi in world.objects if world._matches(name,spec['name'])]
        x.GetOrderedXformOps()[0].Set(Gf.Vec3d(*candidate['position']));x.GetOrderedXformOps()[1].Set(float(candidate.get('yaw_deg',0)))
        exact_hazards=static_collision_shapes(stage,'/World/envs/env_0/'+spec['name']) if a.variant=='robot' else hazards
        bad,support=initial_intersections(source,hazards)
        if bad:
            counts['source_or_fixed_intersection']+=1;print('SOURCE_REJECT',json.dumps({'candidate':candidate,'failures':bad}),flush=True);continue
        end_rows=[]
        for i in sorted(endpoint):
            v=minimum(shapes[i],hazards,margin)
            if not v['valid']:break
            end_rows.append({'action':i,**v})
        else:
            risks=[]
            for i in path:
                v=minimum(shapes[i],exact_hazards,margin)
                if not v['valid']:risks.append({'action':i,'phase':str(phases[i]),**v})
            if risks:
                counts['effective_candidates']+=1;row={'variant':a.variant,'spec':{**spec,**candidate},'risk_samples':len(risks),'minimum':min(risks,key=lambda z:z['slack_m']),'risk_records':risks,'endpoint_minimum':min(end_rows,key=lambda z:z['slack_m']),'support_contacts':support};rows.append(row)
                print('CANDIDATE',json.dumps({k:v for k,v in row.items() if k not in ('risk_records','support_contacts')}),flush=True)
            else:counts['unconstrained']+=1
            continue
        counts['endpoint_violation']+=1
    result={'baseline':str(ep),'scope':'CPU sparse measured trajectory candidate filter; primitive world-AABBs match live conservative execution geometry after yaw changes; no dangerous simulation. Exact primitive risk replay and physical replanning remain mandatory.','selected_indices':selected,'counts':counts,'elapsed_s':time.perf_counter()-start,'candidates':rows}
    Path(a.output).parent.mkdir(parents=True,exist_ok=True);Path(a.output).write_text(json.dumps(result,indent=2));print('COMPLETE',json.dumps(counts),flush=True)
if __name__=='__main__':main()
