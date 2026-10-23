"""Resolve task semantics against an actual frozen stage and joint graph."""
from collections import defaultdict
from kitchen_safety.task_scope import under


def bind_operation_contacts(contract,inventory,bindings,specification):
    """Validate explicitly selected operation surfaces, never a runtime name guess."""
    from kitchen_safety.task_contract import digest
    if specification is None:
        bindings['operation_contacts']=[];return
    if specification['task_contract_sha256']!=digest(contract):
        raise ValueError('Operation surface specification belongs to a different task revision')
    entries={e['path']:e for e in inventory['colliders']}
    operations={o['operation_id']:o for o in contract['operations']}
    bound=[];seen=set()
    for declaration in specification['operation_contacts']:
        oid=declaration['operation_id'];op=operations[oid];aid=declaration['asset_id']
        if oid in seen or op.get('articulation_id')!=aid or op['joint_name']!=declaration['joint_name']:
            raise ValueError('Operation surface does not match its commanded asset/joint')
        seen.add(oid);root=bindings['asset_roots'][aid]
        relative=declaration['relative_colliders']
        if not relative or any(p.startswith('/') or any(x in p.split('/') for x in ('','..','.')) for p in relative):
            raise ValueError('Operation surfaces require explicit asset-relative collider paths')
        paths=[root+'/'+p for p in relative]
        actors=bindings['manipulated_actor_paths'][aid]
        if any(p not in entries or entries[p]['actor'] not in actors for p in paths):
            raise ValueError('Operation surface is missing or not on the commanded moving link')
        if declaration['allowed_phases']!=['operate'] or declaration['source_parts']!='declared_grip_actors':
            raise ValueError('Operation contact requires explicit operate phase and bound grip parts')
        bound.append(dict(operation_id=oid,asset_id=aid,joint_name=op['joint_name'],
            target_colliders=sorted(paths),source_actor_paths=sorted(bindings['grip_actor_paths']),
            allowed_phases=['operate'],reason=declaration['reason'],origin=declaration['origin'],
            declaration_sha256=digest(declaration),force_and_impulse_retained=True))
    bindings['operation_contacts']=bound
    bindings['operation_contact_specification_sha256']=digest(specification)


def bind_human_surface_omissions(inventory,bindings,obstacle):
    """Bind explicit human surface omissions to this asset revision and root.

    There is no universal clothing-name filter and no scene-ID exclusion list.
    An authorized declaration must identify the exact source USD and paths.
    """
    import hashlib
    from pathlib import Path
    specification=obstacle.get('hs_surface_omissions') if obstacle else None
    bindings['hs_human_surface_omissions']=[]
    if specification is None:return
    if obstacle['category']!='HS' or not specification.get('authorization'):
        raise ValueError('Human surface omissions require explicit HS authorization')
    if hashlib.sha256(Path(obstacle['usd_path']).read_bytes()).hexdigest()!=specification['source_usd_sha256']:
        raise ValueError('Human asset revision differs from authorized surface omission')
    root=bindings['asset_roots'][obstacle['asset_id']]
    if root not in bindings['human_roots']:raise ValueError('Human omission root is not loaded')
    relative=specification['relative_colliders']
    if not relative or len(relative)!=len(set(relative)) or any(p.startswith('/') or
            any(x in p.split('/') for x in ('','..','.')) or any(x in p for x in '*?[]') for p in relative):
        raise ValueError('Human omissions require unique exact asset-relative collider paths')
    paths={root+'/'+p for p in relative}
    observed={e['path'] for e in inventory['colliders'] if under(e['path'],root)}
    if not paths<observed:raise ValueError('Omission is missing or removes all human surfaces')
    bindings['hs_human_surface_omissions']=[dict(collider_path=p,asset_id=obstacle['asset_id'],
        source_usd_sha256=specification['source_usd_sha256'],authorization=specification['authorization'],
        reason=specification['reason'],coverage_statement='explicit user scope exclusion; not proof of full anatomical coverage') for p in sorted(paths)]


def physical_joint_bindings(inventory):
    """Resolve a joint's authored descendant target to its physical owner.

    Some kitchen joints target a referenced model Xform below the enabled
    rigid actor. Exact full-path ancestry in this inventory resolves that
    relationship; no name matching or geometric proximity is used. Targets
    with no collider-owning ancestor are retained to preserve graph-only links.
    """
    actors={e['actor'] for e in inventory['colliders'] if e.get('actor')}
    joints=[];evidence=[]
    for source in inventory['joints']:
        joint=dict(source)
        for side in ('body0','body1'):
            resolved=[]
            for path in source[side]:
                ancestors=[a for a in actors if under(path,a)]
                owner=max(ancestors,key=len) if ancestors else path
                resolved.append(owner)
                if owner!=path:evidence.append(dict(joint=source['path'],relationship=side,
                    authored_target=path,physical_actor=owner,method='nearest full-path collider-owning rigid ancestor'))
            joint[side]=resolved
        joints.append(joint)
    return joints,evidence


def derive_bindings(contract, inventory, asset_roots, robot_root, *, human_roots=(),
                    expected_humans=False, grip_actor_paths=(), fixed_hs_authorization=False,
                    backend_fixed_actor_paths=()):
    """No leaf-name fallback for surfaces, robot bodies or articulation identity.

    Joint leaf names come from the task declaration but must resolve uniquely
    inside the declared articulated asset. Robot and human roots are supplied
    by the loaded native scene, never guessed from an old episode.
    """
    entries=inventory['colliders'];physical_joints,joint_owner_evidence=physical_joint_bindings(inventory)
    joints=[j for j in physical_joints if j['enabled']]
    colliders={e['path']:e for e in entries}
    robot_bodies={e['actor'] for e in entries if under(e['path'],robot_root) and e.get('actor')}
    fixed_edges=defaultdict(set);descendants=defaultdict(set);anchored=set()
    for joint in joints:
        a,b=joint['body0'],joint['body1']
        if len(a)>1 or len(b)>1:raise ValueError('Joint has ambiguous body relationships')
        left=a[0] if a else None;right=b[0] if b else None
        if left and right:descendants[left].add(right)
        if joint['type']=='PhysicsFixedJoint':
            if left and right:
                fixed_edges[left].add(right);fixed_edges[right].add(left)
            elif left or right:anchored.add(left or right)
    def closure(seeds, edges):
        found=set(seeds);pending=list(seeds)
        while pending:
            for child in edges.get(pending.pop(),()):
                if child not in found:found.add(child);pending.append(child)
        return found
    actual_actors={e['actor'] for e in entries if e.get('actor')}
    if not set(backend_fixed_actor_paths)<=actual_actors:
        raise ValueError('Fixed backend anchor is not a frozen physical actor')
    anchored.update(backend_fixed_actor_paths)
    all_fixed=closure(anchored,fixed_edges)
    fixed=all_fixed&robot_bodies
    manipulated={};unresolved=[]
    for op in contract['operations']:
        if 'articulation_id' not in op:continue
        aid=op['articulation_id'];root=asset_roots.get(aid)
        matches=[j for j in joints if root and under(j['path'],root)
                 and j['path'].rsplit('/',1)[-1]==op['joint_name']]
        if len(matches)!=1 or len(matches[0]['body1'])!=1:
            unresolved.append(dict(kind='commanded_joint_unbound',operation_id=op['operation_id'],
                asset_id=aid,joint_name=op['joint_name'],match_count=len(matches)))
            continue
        moving=closure(matches[0]['body1'],descendants)
        actual={e['actor'] for e in entries if e.get('actor') and under(e['path'],root)}
        moving &= actual
        if not moving:
            unresolved.append(dict(kind='commanded_joint_has_no_bound_moving_colliders',asset_id=aid))
        manipulated.setdefault(aid,[]).extend(sorted(moving))
    surfaces={}
    for declaration in contract['normal_distance_relations']:
        target=declaration['target_id'];root=asset_roots.get(target)
        if not root:continue
        # Only an explicitly declared reference surface or destination object
        # may be used. A background role cannot exempt the entire kitchen.
        asset=contract['asset_declarations'][target]
        if asset['kind']=='background':
            unresolved.append(dict(kind='support_is_whole_background',target_id=target));continue
        selected=[p for p in colliders if under(p,root)]
        if selected:surfaces[target]=sorted(selected)
    return dict(robot_root=robot_root,asset_roots=dict(asset_roots),human_roots=list(human_roots),
        expected_humans=bool(expected_humans),human_inventory_verified=inventory['complete'],
        grip_actor_paths=list(grip_actor_paths),fixed_robot_actor_paths=sorted(fixed),
        fixed_world_actor_paths=sorted(all_fixed),joint_actor_resolution=joint_owner_evidence,
        hs_fixed_omission_authorized=bool(fixed_hs_authorization),
        fixed_graph_evidence=dict(method='closure of enabled USD world-fixed joints and measured PhysX fixed-base root links',
            physx_fixed_base_roots=list(backend_fixed_actor_paths),
            world_anchors=sorted(anchored),robot_bodies=sorted(robot_bodies)),
        manipulated_actor_paths={k:sorted(set(v)) for k,v in manipulated.items()},
        surface_bindings=surfaces,stop_models={},unresolved=unresolved,
        fixed_root_limitation='Never infer fixed actors from body names or one stationary pose')


def robot_self_pairs(inventory, robot_root, *, explicit_structural_pairs=()):
    """Complete independent robot pairs minus fixed/direct-joint assemblies.

    Explicit pairs require provenance supplied by the caller; no planner-wide
    distance ignore table or observed overlap is converted to an exemption.
    """
    bodies={e['actor'] for e in inventory['colliders'] if e.get('actor') and under(e['path'],robot_root)}
    components={b:{b} for b in bodies};joints=[]
    physical_joints,joint_owner_evidence=physical_joint_bindings(inventory)
    for joint in physical_joints:
        if not joint['enabled'] or len(joint['body0'])!=1 or len(joint['body1'])!=1:continue
        a,b=joint['body0'][0],joint['body1'][0]
        if a not in bodies or b not in bodies:continue
        joints.append((a,b))
        if joint['type']=='PhysicsFixedJoint':
            group=components[a]|components[b]
            for actor in group:components[actor]=group
    allowed=set()
    import itertools
    for group in components.values():allowed.update(tuple(sorted(p)) for p in itertools.combinations(group,2))
    for a,b in joints:allowed.update(tuple(sorted((x,y))) for x in components[a] for y in components[b] if x!=y)
    for declaration in explicit_structural_pairs:
        pair=declaration['pair']
        if len(pair)!=2 or pair[0]==pair[1] or not set(pair)<=bodies or not declaration.get('source_sha256'):
            raise ValueError('Structural pair requires exact actors and source identity')
        allowed.add(tuple(sorted(pair)))
    return dict(evaluated_pairs=[list(p) for p in itertools.combinations(sorted(bodies),2) if p not in allowed],
        allowed_pairs=[list(p) for p in sorted(allowed)],joint_graph_complete=bool(joints),
        body_paths=sorted(bodies),joint_actor_resolution=joint_owner_evidence)


def bind_droid_model(body_paths,joint_names,config,root):
    """Explicit current embodiment adapter, separate from task semantics.

    Different embodiments must provide their own mapping; no cross-robot leaf
    fallback. Task payloads never choose this mapping by their asset names.
    """
    from pathlib import Path
    import hashlib,yaml
    def body(name):
        paths=[str(p) for p in body_paths if str(p).rsplit('/',1)[-1]==name]
        if len(paths)!=1:raise ValueError('DROID model body not unique: '+name)
        return paths[0]
    controlled=['panda_joint'+str(i) for i in range(1,8)]
    if any(list(joint_names).count(n)!=1 for n in controlled+['finger_joint']):
        raise ValueError('DROID controlled/finger joint mapping differs')
    source=Path(root)/config['rs_evidence']['structural_pairs_source'];data=source.read_bytes()
    ignores=yaml.safe_load(data)['robot_cfg']['kinematics']['self_collision_ignore']
    structural=[]
    for a,b in config['rs_evidence']['structural_pairs']:
        if b not in ignores.get(a,[]) and a not in ignores.get(b,[]):
            raise ValueError('Explicit structural contact is not in frozen robot model')
        structural.append(dict(pair=[body(a),body(b)],source_sha256=hashlib.sha256(data).hexdigest()))
    return dict(registry_name='droid_abs_joint_pos',controlled_joint_names=controlled,
        hand_actor_path=body('base_link'),bilateral_actor_paths=[body('left_inner_finger'),body('right_inner_finger')],
        opening_joint_name='finger_joint',opening_joint_positive_means='closing',
        structural_pairs=structural,robot_model_path=str(source),robot_model_sha256=hashlib.sha256(data).hexdigest())
