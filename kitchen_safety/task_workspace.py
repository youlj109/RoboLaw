"""Freeze task work regions and filter PT candidates before mesh queries.

Bounds select applicable geometry only. They are never numerical risk distances.
Free/moving assets are tested again from each offline measured pose; raw grasp,
support and RS contact evidence are independent of PT applicability.
"""
import ast
from collections import defaultdict, deque
from copy import deepcopy
import hashlib
import itertools
import json
from pathlib import Path
import numpy as np
from kitchen_safety.task_contract import digest

VERSION='robolaw-task-workspace-1'

def under(path,root):return path==root or path.startswith(root.rstrip('/')+'/')
def union(boxes):
    boxes=list(boxes)
    if not boxes:raise ValueError('Empty workspace geometry')
    return [np.min([b[0] for b in boxes],axis=0).tolist(),np.max([b[1] for b in boxes],axis=0).tolist()]
def corners(box):return np.asarray(list(itertools.product(*zip(*box))),float)
def overlap(a,b):return bool(np.all(np.asarray(a[0])<=b[1]) and np.all(np.asarray(b[0])<=a[1]))
def expanded(box,amount):return [(np.asarray(box[0])-amount).tolist(),(np.asarray(box[1])+amount).tolist()]

def load_policy(root,contract):
    tid=contract['task_id']
    if Path(tid).name!=tid or tid in ('.','..'):raise ValueError('Invalid workspace task identity')
    path=Path(root)/'configs/risk_workspace_policies'/(tid+'.json')
    if not path.exists():return None
    raw=path.read_bytes();request=json.loads(raw)
    if (request.get('version')!=VERSION or request.get('task_id')!=tid
            or request.get('task_sha256')!=digest(contract) or not request.get('authorization')
            or request.get('dynamic_membership')!='offline_each_measured_pose'
            or request.get('scope')!='PT_environment_features_and_rules'):
        raise ValueError('Workspace policy must bind the current task and offline motion semantics')
    return dict(request=request,source_file_sha256=hashlib.sha256(raw).hexdigest(),source_relative_path=str(path.relative_to(root)))

def materialize(contract,inventory,bindings,state,meshes,stage,envelope,profile):
    """CPU-only scene preparation; no risk query, risk feature or grade."""
    from pxr import UsdGeom
    from scipy.spatial.transform import Rotation
    from kitchen_safety.hs_geometry import transform
    from kitchen_safety.task_binding import physical_joint_bindings
    request=envelope['request'];cache={};local={};world={};by={e['path']:e for e in inventory['colliders']}
    robot=bindings['robot_root'];human_roots=bindings.get('human_roots',[])
    actors=defaultdict(list)
    for e in inventory['colliders']:
        key=e['fingerprint']
        if key not in cache:
            with np.load(Path(meshes)/(key+'.npz'),allow_pickle=False) as data:
                v=np.ascontiguousarray(data['vertices'],dtype=np.float64);f=np.ascontiguousarray(data['faces'],dtype=np.int32)
            if hashlib.sha256(v.tobytes()+f.tobytes()).hexdigest()!=key:raise ValueError('Workspace mesh hash changed')
            cache[key]=[v.min(0).tolist(),v.max(0).tolist()]
        local[e['path']]=cache[key]
        if e['dynamic']:
            pose=transform(state['actor_states_world'][e['actor']]);v=corners(cache[key])@pose[:3,:3].T+pose[:3,3]
            world[e['path']]=[v.min(0).tolist(),v.max(0).tolist()]
        else:world[e['path']]=cache[key]
        if e.get('actor'):actors[e['actor']].append(e['path'])
    joints,_=physical_joint_bindings(inventory)
    # Only unbreakable world-fixed graphs authorize static exclusion. Kinematic
    # bodies and free props are always retained as potentially entering later.
    fixed=set(bindings.get('fixed_graph_evidence',{}).get('physx_fixed_base_roots',[]))
    edges=defaultdict(set)
    for j in joints:
        a=j['attributes']
        if j['enabled'] and j['type']=='PhysicsFixedJoint' and all(float(a.get('physics:'+x,'nan'))>=1e30 for x in ('breakForce','breakTorque')):
            left=j['body0'];right=j['body1']
            if len(left)==len(right)==1:edges[left[0]].add(right[0]);edges[right[0]].add(left[0])
            elif len(left)+len(right)==1:fixed.update(left+right)
    todo=list(fixed)
    while todo:
        for child in edges.get(todo.pop(),()):
            if child not in fixed:fixed.add(child);todo.append(child)
    xcache=UsdGeom.XformCache();frames={}
    def frame(path):
        if path not in frames:
            prim=stage.GetPrimAtPath(path)
            if not prim:raise ValueError('Missing joint frame '+path)
            frames[path]=np.asarray(xcache.GetLocalToWorldTransform(prim)).T
        return frames[path]
    def joint_frame(j):
        authored=next(x for x in inventory['joints'] if x['path']==j['path'])
        attr=j['attributes'];p=np.asarray(ast.literal_eval(attr.get('physics:localPos0','(0,0,0)')),float)
        q=np.asarray(ast.literal_eval(attr.get('physics:localRot0','(1,0,0,0)')),float)
        rotation=Rotation.from_quat([*q[1:],q[0]]).as_matrix()
        axis=np.eye(3)[:,{'X':0,'Y':1,'Z':2}[attr.get('physics:axis','X')]]
        if authored['body0']:
            m=frame(authored['body0'][0]);p=m[:3,:3]@p+m[:3,3];axis=m[:3,:3]@rotation@axis
            owner=j['body0'][0]
            if owner in state['actor_states_world']:
                old=frame(owner);u,_,vt=np.linalg.svd(old[:3,:3]);r=u@vt
                if np.linalg.det(r)<0:u[:,-1]*=-1;r=u@vt
                measured=transform(state['actor_states_world'][owner]);p=measured[:3,:3]@r.T@(p-old[:3,3])+measured[:3,3];axis=measured[:3,:3]@r.T@axis
        else:axis=rotation@axis
        if not np.isfinite(p).all() or not np.isfinite(axis).all() or np.linalg.norm(axis)<1e-12:raise ValueError('Invalid joint workspace frame')
        return p,axis/np.linalg.norm(axis)
    def sweep(box,j):
        if j['type']=='PhysicsFixedJoint':return box
        p,axis=joint_frame(j);attr=j['attributes']
        if j['type']=='PhysicsRevoluteJoint':
            # Propagate spheres per collider, rather than rotating already
            # unioned AABBs. The triangle inequality bounds every joint angle
            # without the artificial diagonal growth of recursive boxes.
            radius=(np.linalg.norm(np.asarray(box['center'])-p)+box['radius']) if isinstance(box,dict) else np.linalg.norm(corners(box)-p,axis=1).max()
            return dict(center=p.tolist(),radius=float(radius))
        if j['type']=='PhysicsPrismaticJoint':
            low=float(attr['physics:lowerLimit']);high=float(attr['physics:upperLimit'])
            if not np.isfinite([low,high]).all() or high<low:raise ValueError('Unbounded slider workspace')
            # The current measured position can be anywhere in the authored interval.
            if isinstance(box,dict):return dict(center=box['center'],radius=box['radius']+high-low)
            extent=np.abs(axis)*(high-low)
            return [(np.asarray(box[0])-extent).tolist(),(np.asarray(box[1])+extent).tolist()]
        raise ValueError('Unsupported free joint workspace '+j['path'])
    children=defaultdict(list)
    for j in joints:
        if j['enabled'] and len(j['body0'])==len(j['body1'])==1:children[j['body0'][0]].append(j)
    def tree(root,allowed,seen):
        seen.add(root);boxes=[world[p] for p in actors.get(root,[]) if root not in bindings.get('fixed_robot_actor_paths',[])]
        for j in children.get(root,[]):
            child=j['body1'][0]
            if child not in allowed or child in seen:continue
            child_box=tree(child,allowed,seen)
            boxes.extend(sweep(b,j) for b in child_box)
        return boxes
    def bound(shape):
        return expanded([shape['center'],shape['center']],shape['radius']) if isinstance(shape,dict) else shape
    robot_actors={a for a in actors if under(a,robot)};seen=set();robot_boxes=[];gaps=[]
    try:
        for anchor in bindings.get('fixed_robot_actor_paths',[]):
            box=tree(anchor,robot_actors,seen)
            robot_boxes.extend(bound(b) for b in box)
        if seen!=robot_actors or not robot_boxes:raise ValueError('Robot graph has unbound movable bodies')
    except (ValueError,KeyError,StopIteration) as exc:gaps.append(str(exc))
    moving=[]
    for op in contract['operations']:
        if 'articulation_id' not in op:continue
        root=bindings['asset_roots'][op['articulation_id']]
        commanded=[j for j in joints if j['enabled'] and under(j['path'],root) and j['path'].rsplit('/',1)[-1]==op['joint_name']]
        try:
            if len(commanded)!=1:raise ValueError('Ambiguous commanded articulation joint')
            j=commanded[0];allowed=set(bindings['manipulated_actor_paths'][op['articulation_id']]);visited=set()
            box=tree(j['body1'][0],allowed,visited)
            if visited!=allowed or not box:raise ValueError('Manipulated link graph is incomplete')
            moving.extend(bound(sweep(b,j)) for b in box)
        except (ValueError,KeyError,StopIteration) as exc:gaps.append(str(exc))
    distance=profile['pt_hs_inspired']['objects'].values();warning=0.
    for obj in distance:
        d=obj['distance'];vR=d['robot_approach_bound_mps'];vO=d['object_approach_bound_mps']
        warning=max(warning,(vR+vO)*d['reaction_s']+vO*d['stop_s']+d['brake_travel_m']+d['contact_clearance_m']+d['robot_uncertainty_m']+d['object_uncertainty_m']+d['warning_band_m'])
    payload_extra=0.
    for aid in contract['roles']['payload_ids']:
        paths=[p for p in by if under(p,bindings['asset_roots'][aid])]
        if paths:payload_extra=max(payload_extra,float(np.linalg.norm(np.diff(np.asarray(union(local[p] for p in paths)),axis=0))))
    volumes=[expanded(b,warning+payload_extra) for b in robot_boxes]+[expanded(b,warning) for b in moving]
    supports=[]
    for op in contract['operations']:
        for aid in op.get('expected_support_ids',[]):
            paths=bindings.get('surface_bindings',{}).get(aid,[])
            if paths:supports.append(union(world[p] for p in paths))
    manual=request.get('manual_support_roots',[])
    if manual:
        supports=[union(world[p] for p in by if under(p,inventory['env_root']+'/'+root)) for root in manual]
    excluded=[inventory['env_root']+'/'+p for p in request.get('excluded_asset_roots',[])]
    role_roots=[bindings['asset_roots'][aid] for aid in sorted(set(contract['roles']['payload_ids']+contract['roles']['destination_ids']+contract['roles']['articulated_target_ids']))]
    result=dict(version=VERSION,task_id=contract['task_id'],task_sha256=digest(contract),inventory_sha256=digest(inventory),
        request=deepcopy(request),request_sha256=digest(request),source_file_sha256=envelope['source_file_sha256'],
        env_root=inventory['env_root'],robot_root=robot,human_roots=list(human_roots),work_volumes=volumes,support_regions=supports,
        family=contract['family'],manual_static_containment=bool(manual),excluded_asset_roots=excluded,role_roots=role_roots,
        whole_body_role_roots=[bindings['asset_roots'][aid] for aid in sorted(set(contract['roles']['payload_ids']+contract['roles']['destination_ids']))],
        uncertainty_gaps=gaps,unbounded=bool(gaps),PT_warning_padding_m=warning,associated_payload_extent_m=payload_extra,
        robot_envelope_method='per-collider triangle-inequality spheres for complete joint rotations and finite slider intervals over the physical spanning tree',
        RS='independent full robot joint/self-collision scope',HS='existing task-bound potentially entering humans retained; no initial-pose absence inference',
        colliders={p:dict(actor=e.get('actor'),dynamic=e['dynamic'],immutable=(not e['dynamic'] or e.get('actor') in fixed),
                        local_bounds=local[p],initial_world_bounds=world[p]) for p,e in by.items()})
    result['policy_sha256']=digest(result)
    return result

class Workspace:
    def __init__(self,policy):
        self.policy=policy;self.rows=policy['colliders'];self.static={};self.corners={};self.aliases={};self.groups={}
        for path,e in self.rows.items():
            self.corners[path]=corners(e['local_bounds'])
            if e['immutable']:self.static[path]=self._eligible(path,e['initial_world_bounds'])
        for root in policy.get('whole_body_role_roots',[]):
            paths=[p for p in self.rows if under(p,root)]
            for path in paths:self.groups[path]=paths
            if all(p in self.static for p in paths):
                selected=any(self.static[p] for p in paths)
                for path in paths:self.static[path]=selected
            else:
                for path in paths:self.static.pop(path,None)
    def _eligible(self,path,box):
        p=self.policy
        if any(under(path,r) for r in p['excluded_asset_roots']):return False
        if p['unbounded']:return True # Missing proof never excludes a candidate.
        if not any(overlap(box,v) for v in p['work_volumes']):return False
        if p['family']!='pick_and_place' or any(under(path,r) for r in p['role_roots']):return True
        if not p['support_regions']:return True
        for region in p['support_regions']:
            above=box[0][2]>=region[1][2]-.003
            if p['manual_static_containment'] and self.rows[path]['immutable']:
                xy=all(region[0][i]-.003<=box[0][i]<=box[1][i]<=region[1][i]+.003 for i in (0,1))
            else:xy=all(box[0][i]<=region[1][i]+.003 and box[1][i]>=region[0][i]-.003 for i in (0,1))
            if above and xy:return True
        return False
    def eligible(self,path,poses,cache=None):
        group=self.groups.get(path)
        if group and path not in self.static:
            key=('whole_body',tuple(group))
            if cache is not None and key in cache:return cache[key]
            result=any(self._single(p,poses,cache) for p in group)
            if cache is not None:cache[key]=result
            return result
        return self._single(path,poses,cache)
    def _single(self,path,poses,cache=None):
        if path in self.static:return self.static[path]
        if cache is not None and path in cache:return cache[path]
        from kitchen_safety.hs_geometry import transform
        e=self.rows[path]
        try:
            pose=transform(poses[e['actor']]);v=self.corners[path]@pose[:3,:3].T+pose[:3,3]
            result=self._eligible(path,[v.min(0),v.max(0)])
        except (KeyError,ValueError,TypeError):result=True # Query later reports unknown.
        if cache is not None:cache[path]=result
        return result
    def allows_contact(self,record,poses,source_paths,cache):
        a,b=record['collider0'],record['collider1']
        def known(path):
            if path in self.rows:return path
            if path not in self.aliases:self.aliases[path]=max((p for p in self.rows if under(path,p)),key=len,default=path)
            return self.aliases[path]
        a,b=known(a),known(b)
        if a in source_paths and b in self.rows and not under(b,self.policy['robot_root']):return self.eligible(b,poses,cache)
        if b in source_paths and a in self.rows and not under(a,self.policy['robot_root']):return self.eligible(a,poses,cache)
        if a not in self.rows or b not in self.rows:return True # Missing identity is not exclusion proof.
        return False

def runtime_omission(engine,relation,poses,cache,operation_remaining):
    contacts=relation.get('contact_target_colliders',relation['target_colliders'])
    bad_contact=sorted(p for p in contacts if not engine.eligible(p,poses,cache))
    bad_distance=sorted(p for p in relation['target_colliders'] if not engine.eligible(p,poses,cache))
    if not bad_contact and not bad_distance:return None
    return dict(version=VERSION,policy_sha256=engine.policy['policy_sha256'],
        excluded_contact_colliders=bad_contact,excluded_distance_colliders=bad_distance,
        fully_outside=not(set(contacts)-set(bad_contact)),
        fully_distance_omitted=bool(relation['query_distance'] and not(set(operation_remaining)-set(bad_distance))))

def validate_omission(relation,value,scope,operation_remaining=None):
    policy=scope['bindings'].get('workspace_policy')
    if (not isinstance(value,dict) or not policy or value.get('version')!=VERSION
            or value.get('policy_sha256')!=policy['policy_sha256']):raise ValueError('Unbound workspace omission')
    allowed_contact=set(relation.get('contact_target_colliders',relation['target_colliders']))
    for field,allowed in [('excluded_contact_colliders',allowed_contact),('excluded_distance_colliders',set(relation['target_colliders']))]:
        paths=value.get(field)
        if not isinstance(paths,list) or len(paths)!=len(set(paths)) or not set(paths)<=allowed:raise ValueError('Workspace omission escapes relation surfaces')
    if value.get('fully_outside') is not (not(allowed_contact-set(value['excluded_contact_colliders']))):raise ValueError('Invalid full workspace omission')
    if operation_remaining is not None and value.get('fully_distance_omitted') is not bool(relation['query_distance'] and not(set(operation_remaining)-set(value['excluded_distance_colliders']))):
        raise ValueError('Workspace and operation distance omissions disagree')
    return sorted(set(operation_remaining if operation_remaining is not None else relation['target_colliders'])-set(value['excluded_distance_colliders']))

def target_selection(policy,targets):
    engine=Workspace(policy);rows=[];env=policy['env_root']
    for target,entries in sorted(targets.items()):
        paths=sorted(e['path'] for e in entries)
        selected=[p for p in paths if not any(under(p,r) for r in policy['excluded_asset_roots'])
                  and (not policy['colliders'][p]['immutable'] or engine.static[p])]
        if selected:rows.append(dict(target=target[len(env)+1:],colliders=[p[len(env)+1:] for p in paths],risk_colliders=[p[len(env)+1:] for p in selected]))
    if not rows:raise ValueError('Workspace contains no PT targets')
    return dict(version='robolaw-authorized-pt-target-subset-1',task_id=policy['task_id'],task_sha256=policy['task_sha256'],
        scope='all_PT_features_and_rules',active_phases=['all'],authorization=policy['request']['authorization'],included_targets=rows,
        selection_basis=dict(kind=VERSION,workspace_policy_sha256=policy['policy_sha256'],dynamic_membership='offline_each_measured_pose'))
