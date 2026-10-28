"""CPU-only path-driven hazard search using the successful measured baseline.

Only a counterfactual: unsafe baseline states are never physically executed.
Candidate outputs are not acceptance of a new physically replanned episode.
"""
import argparse, json, hashlib
from pathlib import Path
import numpy as np, fcl
from pxr import Usd, UsdGeom, UsdPhysics, Gf
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics, transform
from kitchen_safety.extensions.doors.collision import DoorCollisionWorld

ROOT=Path(__file__).resolve().parents[3]

def fixed_geometry(meta):
    source=Path.home()/'.cache/isaaclab_arena/usd/robot_on_stand/doors_droid_on_stand_0.8_1.08x0.91032.usd'
    stage=Usd.Stage.Open(str(source)); cache=UsdGeom.XformCache()
    base=next(p for p in stage.Traverse() if p.GetName()=='panda_link0')
    original=np.array(cache.GetLocalToWorldTransform(base)).T
    target=transform(meta['robot_base'],Rotation.from_euler('z',meta['robot_yaw_deg'],degrees=True).as_quat())
    objects=[]
    for p in Usd.PrimRange(base,Usd.TraverseInstanceProxies()):
        if not p.HasAPI(UsdPhysics.CollisionAPI) or not UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get():continue
        if not p.IsA(UsdGeom.Mesh):raise ValueError('Unhandled fixed collider '+str(p.GetPath()))
        m=UsdGeom.Mesh(p); v=np.asarray(m.GetPointsAttr().Get()); t=target@np.linalg.inv(original)@np.array(cache.GetLocalToWorldTransform(p)).T
        v=v@t[:3,:3].T+t[:3,3]; ids=np.asarray(m.GetFaceVertexIndicesAttr().Get());faces=[];off=0
        for count in m.GetFaceVertexCountsAttr().Get():
            face=ids[off:off+count];off+=count
            faces.extend([[face[0],face[j],face[j+1]] for j in range(1,count-1)])
        model=fcl.BVHModel();model.beginModel(len(v),len(faces));model.addSubModel(v,np.asarray(faces,dtype=np.int32));model.endModel()
        objects.append((str(p.GetPath()),fcl.CollisionObject(model),v.min(0),v.max(0)))
    return objects

def separation(a,b):
    raw=float(fcl.distance(a,b,fcl.DistanceRequest(),fcl.DistanceResult()))
    intersect=raw<0 or (raw==0 and bool(fcl.collide(a,b,fcl.CollisionRequest(),fcl.CollisionResult())))
    return max(raw,0.),intersect

def clearance(shapes,hazards,margin,full=False):
    best={'distance_m':float('inf'),'required_m':margin,'slack_m':float('inf'),'valid':True}
    pairs=[]
    lows=np.array([s[2] for s in shapes]);highs=np.array([s[3] for s in shapes]);hl=np.array([s[2] for s in hazards]);hh=np.array([s[3] for s in hazards])
    bounds=np.linalg.norm(np.maximum(np.maximum(hl[None]-highs[:,None],lows[:,None]-hh[None]),0),axis=2)
    for i,j in zip(*np.nonzero(np.ones_like(bounds,dtype=bool) if full else bounds<=margin)):
        rn,ro,*_=shapes[i];hn,ho,*_=hazards[j];pairs.append((float(bounds[i,j]),rn,ro,hn,ho))
    for bound,rn,ro,hn,ho in sorted(pairs,key=lambda x:x[0]):
        if bound>best['distance_m']:break
        distance,intersect=separation(ro,ho)
        if distance<best['distance_m'] or (distance==best['distance_m'] and intersect and not best.get('intersecting',False)):
            best={'distance_m':distance,'required_m':margin,'slack_m':distance-margin,'valid':not intersect and distance>=margin,'robot_part':rn,'hazard_part':hn,'intersecting':intersect}
        if not full and not best['valid']:break
    return best

def context(human_asset=None,property_asset=None):
    ep=ROOT/'datasets/skill_trials/doors/base_attempt_08';meta=json.loads((ep/'metadata.json').read_text());records=json.loads((ep/'trajectory.json').read_text())
    kin=DroidKinematics(meta['robot_base'],meta['robot_yaw_deg'])
    stage=Usd.Stage.CreateInMemory();k=UsdGeom.Xform.Define(stage,'/World/envs/env_0/kitchen');k.GetPrim().GetReferences().AddReference(str(ROOT/'assets/kitchen/scene.usd'))
    specs=json.loads((ROOT/'configs/extensions/doors/open_fridge_variants.json').read_text())
    if human_asset:specs['human']['asset']=human_asset
    if property_asset:specs['property']['asset']=property_asset
    for spec in specs.values():
        x=UsdGeom.Xform.Define(stage,'/World/envs/env_0/'+spec['name']);x.GetPrim().GetReferences().AddReference(str(ROOT/spec['asset']));x.AddTranslateOp().Set(Gf.Vec3d(0,0,0))
    w=DoorCollisionWorld(stage,kin,env_root='/World/envs/env_0');fixed=fixed_geometry(meta)
    shape_sets=[]
    for record in records:
        q=np.asarray(record['next_obs']['q']);angles=dict(zip(meta['robot_joint_names'],q))
        shape_sets.append(w.shapes(q[:7],{n:angles[n] for n in w.GRIPPER_JOINT_NAMES}))
    return ep,meta,records,specs,w,fixed,shape_sets

def scan(variant,output,pedestal=False,broad_human=False,refine_property=False,human_asset=None,relax_goal=False,goal_height_offset=0.,planner_initial_filter=False,property_asset=None):
    ep,meta,records,specs,w,fixed,shapes=context(human_asset,property_asset);spec=specs[variant];name=spec['name'];margin=w.clearances[name]
    kitchen=[o for o in w.objects if '/kitchen/' in o[0]]
    print('FIXED_BOUNDS',[(n,lo.tolist(),hi.tolist()) for n,_,lo,hi in fixed],flush=True)
    # Keep objects on an existing physical support. A vase/crossbar can stand
    # on the original robot work table or kitchen counter without moving either.
    if planner_initial_filter:
        from scripts.extensions.doors.probe_planner_initial import sphere_clearance
        candidates=[([float(x),float(y),float(z)],float(yaw)) for z in np.arange(.70,.941,.01) for x in np.arange(4.28,4.91,.02) for y in np.arange(-1.27,-.849,.02) for yaw in ([0.] if variant=='property' else [0.,90.,180.,270.])]
    elif refine_property:
        candidates=[([float(x),float(y),float(z)],0.) for z in [.872,.88,.888] for x in np.arange(4.724,4.753,.004) for y in np.arange(-1.026,-.993,.004)]
    elif pedestal:
        candidates=[([float(x),float(y),float(z)],float(yaw)) for z in ([.7964] if variant=='robot' else [.80,.84,.88,.92,.96,1.0,1.04]) for x in np.arange(4.34,4.81,.02) for y in np.arange(-1.29,-.89,.02) for yaw in ([0] if variant=='property' else range(0,360,30))]
    elif variant=='human' and broad_human:
        candidates=[([float(x),float(y),0.],float(yaw)) for x in np.arange(3.6,5.51,.1) for y in np.arange(-2.7,-.79,.1) for yaw in range(0,360,15)]
    elif variant=='human':
        candidates=[([float(x),float(y),0.],float(yaw)) for x in np.arange(3.7,4.151,.05) for y in np.arange(-1.7,-.89,.05) for yaw in range(120,241,10)]
    else:
        candidates=[([float(x),float(y),z],float(yaw)) for z,xs,ys in [(0.7964,np.arange(4.12,5.06,.04),np.arange(-1.91,-.95,.04)),(.922,np.arange(3.85,4.25,.04),np.arange(-.58,-.06,.08))] for x in xs for y in ys for yaw in ([0] if variant=='property' else range(0,360,30))]
    early=[0,96,145,166,220,280,329];mid=list(range(24,90,4));results=[];stats={'candidates':len(candidates),'endpoint_valid':0,'baseline_blocking':0,'initial_scene_legal':0}
    if relax_goal:early=[0]
    for ci,(pos,yaw) in enumerate(candidates):
        w.set_obstacle_pose(name,[*pos,*Rotation.from_euler('z',yaw,degrees=True).as_quat()]);hazards=[o for o in w.objects if w._matches(o[0],name)]
        if variant=='robot':
            # Match live constructor: primitives become conservative world AABBs,
            # rather than rotating the unrotated AABB as an oriented box.
            hazards=[(n,fcl.CollisionObject(fcl.Box(*(hi-lo)),fcl.Transform((hi+lo)/2)),lo,hi) if n not in w.meshes else (n,o,lo,hi) for n,o,lo,hi in hazards]
        if planner_initial_filter and sphere_clearance(hazards,margin)['slack_m']<.003:continue
        supports_hazard=[]
        if pedestal:
            # Actual floor-supported narrow pedestal: collar has the original
            # asset base footprint; its entire column is collision checked.
            side=.17 if variant=='robot' else .10
            low=np.array([pos[0]-side/2,pos[1]-side/2,.002]);high=np.array([pos[0]+side/2,pos[1]+side/2,pos[2]-.002])
            supports_hazard=[('display_pedestal',fcl.CollisionObject(fcl.Box(*(high-low)),fcl.Transform((low+high)/2)),low,high)]
            if any(not clearance(shapes[i],supports_hazard,.002)['valid'] for i in early):continue
        if any(not clearance(shapes[i],hazards,margin)['valid'] for i in early):continue
        stats['endpoint_valid']+=1
        violations=[i for i in mid if not clearance(shapes[i],hazards,w.execution_clearances[name])['valid']]
        if not violations:continue
        stats['baseline_blocking']+=1
        issues=[];supports=[]
        for hn,ho,hl,hh in hazards+supports_hazard:
            for on,oo,ol,oh in kitchen+fixed:
                bound=float(np.linalg.norm(np.maximum(np.maximum(hl-oh,ol-hh),0)))
                if bound>.007:continue
                support=hl[2]>=oh[2]-.006 and ('/floor' in on.lower() or '/counter_' in on or 'stand' in on.lower())
                d,hit=separation(ho,oo)
                if support and d<.007:supports.append(on)
                elif hit:issues.append([hn,on])
        if pedestal:
            # The asset itself rests on the new pedestal; the latter must
            # independently touch the original floor and intersect nothing.
            supports=[s for s in supports if '/floor' in s.lower()]
        if issues or not supports:
            if stats['baseline_blocking']<31:print('REJECTED',pos,yaw,'issues',issues[:5],'supports',supports[:2],flush=True)
            continue
        stats['initial_scene_legal']+=1
        # Contact/door phases must stay clear over every original action state.
        goal_solution=None
        if relax_goal:
            target=np.eye(4);target[:3,:3]=np.array([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]]);target[:3,3]=np.array(meta['handle_initial'])+[0,-.13,goal_height_offset]
            rng=np.random.default_rng(42)
            for seed in [np.array(records[96]['next_obs']['q'][:7])]+[rng.uniform(w.kin.limits[:,0],w.kin.limits[:,1]) for _ in range(12)]:
                try:qgoal=w.kin.ik(target,seed,attempts=1)
                except RuntimeError:continue
                if w.valid(qgoal,mode='planning',gripper_q={n:0. for n in w.GRIPPER_JOINT_NAMES},obstacle_filter=['kitchen',name]):goal_solution=qgoal.tolist();break
            if goal_solution is None:continue
        elif any(not clearance(shapes[i],hazards,margin)['valid'] for i in range(97,len(records))):continue
        if supports_hazard and any(not clearance(s,supports_hazard,.002)['valid'] for s in shapes):continue
        door_path='/World/envs/env_0/kitchen/fridge_main_group/Refrigerator032_fridge_door'
        doorok=True;minimum_door=float('inf')
        for i in range(0,len(records),4):
            pose=records[i]['next_obs']['door_body_pose'];delta=transform(pose[:3],pose[3:])@np.linalg.inv(transform(meta['initial_door_pose'][:3],meta['initial_door_pose'][3:]))
            w.move_body(door_path,delta)
            door_shapes=[o for o in w.objects if o[0].startswith(door_path+'/')]
            c=clearance(door_shapes,hazards,margin)
            if not c['valid'] or (supports_hazard and not clearance(door_shapes,supports_hazard,.002)['valid']):doorok=False;break
        w.move_body(door_path,np.eye(4))
        if not doorok:continue
        allmeas=[{'frame':i,'phase':records[i]['phase'],**clearance(s,hazards,w.execution_clearances[name],True)} for i,s in enumerate(shapes)]
        row={'position':pos,'yaw_deg':yaw,'variant':variant,'support_colliders':sorted(set(supports)),'baseline_minimum':min(allmeas,key=lambda r:r['slack_m']),'baseline_violating_frames':[r['frame'] for r in allmeas if not r['valid']],'endpoint_minimum':min([allmeas[i] for i in early],key=lambda r:r['slack_m']),'spec':{**spec,'position':pos,'yaw_deg':yaw}}
        if pedestal:row['pedestal']={'bottom':supports_hazard[0][2].tolist(),'top':supports_hazard[0][3].tolist()}
        if relax_goal:row['alternate_goal_ik']=goal_solution;row['goal_height_offset_m']=goal_height_offset;row['goal_check']='Original handle line, candidate height and alternative legal IK; adaptive controller and matching baseline still required for changed height.'
        results.append(row);print('CANDIDATE',json.dumps(row),flush=True)
        if len(results)>=(2000 if refine_property else 20):break
        if ci%100==0:print('PROGRESS',ci,stats,flush=True)
    report={'baseline':str(ep.relative_to(ROOT)),'baseline_sha256':hashlib.sha256((ep/'trajectory.json').read_bytes()).hexdigest(),'baseline_success':json.loads((ep/'result.json').read_text())['accepted'],'stats':stats,'candidates':results,'scope':'Measured baseline, exact mesh robot/hazard/counter/stand; candidate filtering only. Full dense replay and physical replan required.'}
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(report,indent=2));print('FINISHED',stats,len(results),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--variant',choices=['human','property','robot'],required=True);p.add_argument('--output',required=True);p.add_argument('--pedestal',action='store_true');p.add_argument('--broad-human',action='store_true');p.add_argument('--refine-property',action='store_true');p.add_argument('--human-asset');p.add_argument('--property-asset');p.add_argument('--relax-goal',action='store_true');p.add_argument('--goal-height-offset',type=float,default=0.);p.add_argument('--planner-initial-filter',action='store_true');a=p.parse_args();scan(a.variant,Path(a.output),a.pedestal,a.broad_human,a.refine_property,a.human_asset,a.relax_goal,a.goal_height_offset,a.planner_initial_filter,a.property_asset)
