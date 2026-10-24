"""Offline source-time HS/PT/RS evidence for native task contracts.

Each payload owns a tracker; each moving articulation keeps its true identity.
No inherited scene omissions and no predictions substituted for measured state.
"""
from collections import deque
from copy import deepcopy
import numpy as np
from kitchen_safety.task_scope import under
from kitchen_safety.hs_geometry import transform
from kitchen_safety.hs_dynamic import relation as hs_relation
from kitchen_safety.grasp_evidence import GraspEvidence
from kitchen_safety.rs_evidence import FaultEvidence
from kitchen_safety.pt_assets import support_evidence


def contact_step(packet,dt):
    pairs={}
    for r in packet.get('contacts',[]):
        key=tuple(sorted((r['actor0'],r['actor1'])))
        value=pairs.setdefault(key,dict(bodyA=key[0],bodyB=key[1],vector=np.zeros(3),contacts=[]))
        value['vector']+=(1 if r['actor0']==key[0] else -1)*np.sum([p['impulse_vector_ns'] for p in r['points']],axis=0)
        value['contacts'].append(r)
    for v in pairs.values():
        impulse=float(np.linalg.norm(v.pop('vector')));v.update(impulse_ns=impulse,force_n=impulse/dt)
    return dict(pairs=list(pairs.values()),coverage=packet.get('contact_coverage') is True,
        physics_step=packet['physics_step'])


def matches(record,source_paths,target_paths):
    def inside(p,paths):return any(under(p,x) for x in paths)
    return ((inside(record['collider0'],source_paths) and inside(record['collider1'],target_paths)) or
            (inside(record['collider1'],source_paths) and inside(record['collider0'],target_paths)))


def robot_joint_features(robot, joint_names, controlled_names):
    """Validate independent measured channels before computing scalar extrema.

    A missing position channel does not erase measured torque, or vice versa.
    In particular, Python min/max must not silently ignore a nonfinite joint.
    """
    margin=ratio=None;errors=[]
    try:
        if not controlled_names or len(set(controlled_names))!=len(controlled_names):
            raise ValueError('Controlled joint mapping is empty or duplicated')
        if len(set(joint_names))!=len(joint_names):raise ValueError('Joint identity is duplicated')
        ids=[joint_names.index(name) for name in controlled_names]
    except Exception as exc:return margin,ratio,['robot_joint_mapping: '+repr(exc)]
    def channel(key,shape):
        raw=robot.get(key)
        if raw is None:raise ValueError('Missing '+key)
        values=np.asarray(raw,dtype=float)
        if values.shape!=shape:raise ValueError('Invalid shape for '+key)
        selected=values[ids]
        if not np.isfinite(selected).all():raise ValueError('Nonfinite '+key)
        return selected
    try:
        q=channel('q',(len(joint_names),))
        limits=channel('joint_pos_limits',(len(joint_names),2))
        if np.any(limits[:,0]>=limits[:,1]):raise ValueError('Invalid joint position bounds')
        margin=float(np.minimum(q-limits[:,0],limits[:,1]-q).min())
    except Exception as exc:errors.append('joint_limit_margin: '+repr(exc))
    try:
        torque=channel('applied_torque',(len(joint_names),))
        effort=channel('joint_effort_limits',(len(joint_names),))
        if np.any(effort<=0):raise ValueError('Invalid effort limit')
        ratio=float((np.abs(torque)/effort).max())
    except Exception as exc:errors.append('joint_torque_ratio: '+repr(exc))
    return margin,ratio,errors


def self_collision_features(rows, rs_scope, windows):
    """No collision requires the exact frozen pair set, not all([]).

    Known positive intersections/contacts survive missing negative checks.
    Invalid or unbound geometry rows do not constitute physical evidence.
    """
    bodies=set(rs_scope['body_paths']);errors=[];hits=set();observed=set()
    def pair(value):
        if not isinstance(value,(list,tuple)) or len(value)!=2 or not all(isinstance(p,str) for p in value):
            raise ValueError('Malformed robot body pair')
        result=tuple(sorted(value))
        if result[0]==result[1] or not set(result)<=bodies:raise ValueError('Unbound robot body pair')
        return result
    expected={pair(p) for p in rs_scope['evaluated_pairs']}
    allowed={pair(p) for p in rs_scope['allowed_pairs']}
    if len(expected)!=len(rs_scope['evaluated_pairs']) or expected&allowed:
        raise ValueError('Frozen RS scope has duplicate or conflicting pairs')
    # A scope must account for every distinct robot body pair, either checked
    # or explicitly allowed by its frozen structural evidence.
    import itertools
    if expected|allowed!={tuple(p) for p in itertools.combinations(sorted(bodies),2)}:
        errors.append('frozen_robot_pair_scope_incomplete')
    if rs_scope.get('joint_graph_complete') is not True:errors.append('robot_joint_graph_incomplete')
    for row in rows:
        try:key=pair(row['pair'])
        except (KeyError,TypeError,ValueError) as exc:
            errors.append('invalid_self_geometry_pair: '+str(exc));continue
        if key not in expected:
            errors.append('unexpected_self_geometry_pair: '+repr(key));continue
        if key in observed:errors.append('duplicate_self_geometry_pair: '+repr(key))
        observed.add(key)
        if row.get('intersection') is True:hits.add(key)
        if row.get('complete') is not True or type(row.get('intersection')) is not bool:
            errors.append('incomplete_self_geometry_pair: '+repr(key))
    missing=sorted(expected-observed)
    if missing:errors.append('missing_self_geometry_pairs')
    for window in windows:
        for value in window['pairs']:
            try:key=pair([value['bodyA'],value['bodyB']])
            except (KeyError,TypeError,ValueError):continue
            if key not in allowed:hits.add(key)
    complete=not errors
    return (True if hits else False if complete else None),hits,dict(
        coverage_complete=complete,expected_pair_count=len(expected),observed_pair_count=len(observed),
        missing_pairs=[list(p) for p in missing],errors=errors)


class TaskFeatures:
    def __init__(self,contract,inventory,scope,rs_scope,rules,schema,queries):
        self.contract=contract;self.inventory=inventory;self.scope=scope;self.rs_scope=rs_scope
        self.config=rules['config'];self.profile=rules['profile'];self.schema=schema;self.queries=queries
        self.model=scope['bindings']['robot_model'];self.dt=1/schema['physics_hz']
        self.entries={e['path']:e for e in inventory['colliders']}
        self.windows=deque(maxlen=4);self.current=None;self.step=-1
        self.payloads={aid:dict(root=scope['bindings']['asset_roots'][aid],tracker=GraspEvidence(self.config['grasp_evidence']),
            maximum_z=None,grasp={},drop_flag=None,drop_height=None) for aid in contract['roles']['payload_ids']}
        for aid,p in self.payloads.items():
            p['colliders']=[e['path'] for e in inventory['colliders'] if under(e['path'],p['root'])]
            owners={self.entries[c]['actor'] for c in p['colliders']}
            if len(owners)!=1 or None in owners:raise ValueError('Payload requires one measured rigid actor: '+aid)
            p['actor']=next(iter(owners))
            vertices=np.concatenate([queries.entries[c]['model'].vertices for c in p['colliders']],axis=0)
            p['local_geometry_center']=(vertices.min(0)+vertices.max(0))/2
            p['support_paths']=sorted({path for relation in contract['normal_distance_relations']
                if relation['source_id']==aid for path in scope['bindings']['surface_bindings'].get(relation['target_id'],[])})
        self.fault=FaultEvidence(self.config['rs_evidence']['fault_motion']);self.fault_result={}
        self.previous_hs={};self.previous_hs_step=None

    def advance(self,packet):
        step=packet['physics_step']
        if step!=self.step+1:raise ValueError('Raw physics gap/reordering')
        self.step=step;self.current=packet
        self.windows.append(contact_step(packet,self.dt))
        self.fault_result=self.fault.sample(packet['state']['robot'],step)
        self._grasp(packet['state'],packet.get('phase'),geometry=None)

    def fault_event(self,event):
        self.fault.event(event['before_physics_step'],event['kind'],event.get('error',''))

    def surface_speed(self,paths,state,cache):
        key=tuple(paths)
        if key in cache:return cache[key]
        maximum=0.;static=True
        try:
            for path in paths:
                e=self.entries[path]
                if not e['dynamic']:continue
                static=False;actor=e['actor'];motion=state['actor_motion_world'][actor]
                pose=transform(state['actor_states_world'][actor])
                com=np.asarray(motion['com_position_world_m'],float)
                linear=np.asarray(motion['com_linear_velocity_mps'],float);omega=np.asarray(motion['angular_velocity_radps'],float)
                if any(v.shape!=(3,) or not np.isfinite(v).all() for v in (com,linear,omega)):
                    raise ValueError('Invalid measured COM motion')
                points=self.queries.entries[path]['model'].vertices@pose[:3,:3].T+pose[:3,3]
                maximum=max(maximum,float(np.linalg.norm(linear+np.cross(omega,points-com),axis=1).max()))
            value=(maximum,static,None)
        except Exception as exc:value=(None,False,repr(exc))
        cache[key]=value;return value

    def _grasp(self,state,phase,geometry):
        current=self.windows[-1] if self.windows else dict(coverage=False,pairs=[])
        covered=current['coverage'];robot_root=self.scope['bindings']['robot_root'];model=self.model
        for aid,p in self.payloads.items():
            # advance() supplies every physics step; features() refines that
            # same source step with its offline mesh separation. Re-evaluate
            # from the pre-step tracker state, never from the first call's
            # mutated anchor. Otherwise opening can clear the old anchor and
            # a second call can falsely confirm a grasp from residual contacts.
            if p.get('tracker_source_step')!=self.step:
                p['tracker_source_step']=self.step
                p['tracker_before_step']=deepcopy(vars(p['tracker']))
            else:
                vars(p['tracker']).clear()
                vars(p['tracker']).update(deepcopy(p['tracker_before_step']))
            pairs=[v for v in current['pairs'] if p['actor'] in (v['bodyA'],v['bodyB'])]
            gripping={v[k] for v in pairs if v['force_n']>.01 for k in ('bodyA','bodyB')
                if v[k] in self.scope['bindings']['grip_actor_paths']}
            external=[v for v in pairs if not any(under(v[k],robot_root) for k in ('bodyA','bodyB'))]
            supports=[]
            for value in external:
                contacts=[r for r in value['contacts'] if matches(r,p['colliders'],p['support_paths'])
                    and any(np.linalg.norm(point['impulse_vector_ns'])>0 for point in r['points'])]
                if contacts:supports.append(dict(value,contacts=contacts))
            pose=state['actor_states_world'].get(p['actor'])
            center=None
            if pose is not None:
                matrix=transform(pose);center=(matrix[:3,:3]@p['local_geometry_center']+matrix[:3,3]).tolist()
            # Reuse the frozen support proof and numeric thresholds. A horizontal
            # normal alone also matches ceiling/edge contacts and is insufficient.
            support=support_evidence(dict(allowed_support_relations=[dict(id='task_support:'+aid,
                    object_id=aid,phases=[phase])],support_policy=self.config['pt_asset_coverage']['support_policy']),
                aid,phase,supports,covered,dict(mustard=pose if pose is not None else []),center)
            supported=True if support['distance_exempt'] else False if covered and not external else None
            p['support_evidence']=support
            separated=None
            if geometry:
                d=[geometry['relations'][r['relation_id']].get('distance_m') for r in self.scope['relations']
                    if r['category']=='PT' and r['source'] in self.scope['bindings']['grip_actor_paths']
                    and r['target']==p['actor'] and r['query_distance']]
                if len(d)==len(self.scope['bindings']['grip_actor_paths']) and all(x is not None for x in d):
                    separated=all(x>self.config['grasp_evidence']['separation_m'] for x in d)
            # Compatibility bridge to the frozen grasp tracker. These aliases
            # are internal API slots, populated from THIS payload's full paths.
            # Never read an episode asset by the legacy field name 'mustard'.
            try:
                pose=state['actor_states_world'][p['actor']]
                hand=state['actor_states_world'][model['hand_actor_path']][:7]
                q=state['robot']['q'][self.schema['robot_joint_names'].index(model['opening_joint_name'])]
                bridge=dict(body_pose=[hand],mustard=pose,q=[q])
                p['grasp']=p['tracker'].sample(self.step,self.dt,bridge,['base_link'],['finger_joint'],
                    set(model['bilateral_actor_paths'])<=gripping,covered,supported,gripping,phase,separated)
                p['grasp']['sampling_semantics']='same_source_step_refined_from_pre_step_tracker_state'
                z=pose[2];p['maximum_z']=max(z,p['maximum_z']) if p['maximum_z'] is not None else z
                protocol=self.config['drop_protocol'];descent=p['maximum_z']-z
                falling=pose[9]<-protocol['downward_speed_mps'] and descent>protocol['descent_m']
                held=p['grasp']['held'];p['drop_flag']=None;p['drop_height']=None
                if covered and (held or supported):p['drop_flag']=False
                elif covered and held is False and supported is False and falling and phase in ('lift','transfer','transfer_front','above_bowl','lower','transport','place'):
                    p['drop_flag']=True;p['drop_height']=descent
            except Exception as exc:
                p['grasp']=dict(held=None,supported=None,error=repr(exc));p['drop_flag']=None;p['drop_height']=None

    def features(self,state,geometry,phase):
        self._grasp(state,phase,geometry)
        cache={};hs=[];pt=[]
        covered=len(self.windows)==4 and all(w['coverage'] for w in self.windows)
        for r in self.scope['relations']:
            g=geometry['relations'].get(r['relation_id'],{})
            if r['category']=='HS':
                vr,static,err=self.surface_speed(r['source_colliders'],state,cache)
                vh,hstatic,herr=self.surface_speed(r['target_colliders'],state,cache)
                stop='robot_coupled' if r['source_role']=='robot' else 'uncontrolled_payload'
                if r['source_role']=='payload':
                    p=next(p for p in self.payloads.values() if p['actor']==r['source']);grasp=p['grasp']
                    if grasp.get('held') is True:stop='robot_coupled'
                    elif grasp.get('supported') is True and vr is not None and vr<=self.profile['hs_dynamic']['stationary_speed_tolerance_mps']:
                        stop='supported_stationary'
                if r['source_role']=='manipulated_link':
                    stop=r.get('stop_model_binding') or 'unbound_articulation_stopping_model'
                if static:stop='supported_stationary'
                row=hs_relation(g.get('distance_m'),vr,vh,hstatic,self.profile['hs_dynamic'],stop_model=stop)
                row.update(relation_id=r['relation_id'],source_group='robot' if r['source_role']=='robot' else 'object',
                    source_actor=r['source'],human_collider=r['target'],nearest_geometry=g)
                for error in (err,herr,g.get('error')):
                    if error:row['missing'].append(error);row.update(complete=False,level=None)
                previous=self.previous_hs.get(r['relation_id']);dt=(self.step-self.previous_hs_step)*self.dt if self.previous_hs_step is not None else None
                closing=max(0.,(previous-g['distance_m'])/dt) if previous is not None and g.get('distance_m') is not None and dt and dt>0 else None
                row.update(closing_speed_mps=closing,TTC_s=g['distance_m']/closing if closing and closing>0 else None)
                hs.append(row)
                continue
            paths=r.get('contact_target_colliders',r['target_colliders']);selected=[]
            for window in self.windows:
                for v in window['pairs']:
                    if any(matches(c,r['source_colliders'],paths) for c in v['contacts']):selected.append(v)
            raw=dict(relation_id=r['relation_id'],source=r['source'],target=r['target'],source_role=r['source_role'],
                force_n=max((v['force_n'] for v in selected),default=0.) if covered else None,
                impulse_ns=max((v['impulse_ns'] for v in selected),default=0.) if covered else None,
                impulse_semantics='max_body_pair_step_vector_norm_over_four_steps',physics_window=len(self.windows),
                contact=bool(selected) if covered else None,task_phase=phase,held_contact_active=None)
            if r['source_role']=='payload':
                grasp=next(p['grasp'] for p in self.payloads.values() if p['actor']==r['source'])
                raw.update(held_contact_active=grasp.get('held'),held_contact_source=grasp.get('reason'))
            omission=geometry.get('distance_omissions',{}).get(r['relation_id'])
            if omission:raw['distance_omission']=omission
            if r['query_distance'] and not (omission and omission['fully_omitted']):
                raw.update(distance_m=g.get('distance_m'),distance_coverage_complete=g.get('complete') is True,
                    distance_geometry_evidence=g)
                vr,_,err=self.surface_speed(r['source_colliders'],state,cache)
                vo,_,oerr=self.surface_speed(g.get('effective_target_colliders',r['target_colliders']),state,cache)
                raw.update(source_surface_speed_mps=vr,target_surface_speed_mps=vo,motion_error=err or oerr)
            pt.append(raw)
        self.previous_hs={r['relation_id']:r['distance_m'] for r in hs};self.previous_hs_step=self.step
        human_paths=self.scope['human_inventory']['collision_paths']
        # HS uses its own eligible contact relations. RS retains fixed robot
        # geometry and must never expand the HS acquisition/grade scope.
        source_paths=set()
        for r in self.scope['relations']:
            if r['category']=='HS':source_paths.update(r['source_colliders'])
        hits=[v for w in self.windows for v in w['pairs']
            if any(matches(c,source_paths,human_paths) for c in v['contacts'])]
        margin,ratio,rs_errors=robot_joint_features(state['robot'],self.schema['robot_joint_names'],self.model['controlled_joint_names'])
        self_flag,selfhits,self_evidence=self_collision_features(geometry['self_collision'],self.rs_scope,self.windows)
        complete=self_evidence['coverage_complete']
        return dict(hs=dict(dynamic_distance=dict(version=self.profile['hs_dynamic']['version'],pairs=hs),
                    human_contact_flag_gt=True if hits else False if covered else None),
            pt=dict(protected_pairs=pt,payloads={aid:dict(actor=p['actor'],grasp=p['grasp'],support_evidence=p.get('support_evidence'),drop_flag_gt=p['drop_flag'],h_drop_gt_m=p['drop_height']) for aid,p in self.payloads.items()},
                support_margin_gt_m=None,damage_flag_gt=None),
            rs=dict(joint_limit_margin_gt_rad=margin,joint_torque_ratio_gt=ratio,
                motion_after_fault_gt=self.fault_result.get('motion_after_fault_gt'),fault_evidence=self.fault_result,
                self_collision_flag_gt=self_flag,
                self_collision_evidence=self_evidence,errors=rs_errors),
            _existing_self_collision_pairs=[list(p) for p in sorted(selfhits)] if complete or selfhits else None,
            coverage=dict(contact_four_step_window=covered,scope_resolved=self.scope['coverage']['scope_resolved']))
