"""Compile exact runtime collider relations without querying any distance.

Inputs must come from the current loaded stage, not a previous task's inventory.
The result is frozen once. No policy here changes a numeric risk threshold.
"""
from collections import defaultdict
from copy import deepcopy
from kitchen_safety.task_contract import digest


def under(path, root):
    return path == root or path.startswith(root.rstrip('/') + '/')


def bind_scope(contract, inventory, bindings):
    """Bindings identify actual paths and proven fixed/moving bodies.

    inventory: complete, colliders[{path, actor, dynamic, fingerprint}], joints.
    bindings: robot_root, asset_roots, human_roots, human_inventory_verified,
      expected_humans, grip_actor_paths, fixed_robot_actor_paths,
      hs_fixed_omission_authorized, manipulated_actor_paths, surface_bindings.
    surface_bindings maps semantic IDs to exact collider lists; role roots alone
    cannot exempt a support surface. Missing bindings remain explicit gaps.
    """
    if inventory.get('complete') is not True:
        raise ValueError('Cannot freeze scope from an incomplete collider inventory')
    robot = bindings['robot_root']
    roots = bindings['asset_roots']
    humans = bindings.get('human_roots', [])
    if bindings.get('human_inventory_verified') is not True:
        raise ValueError('Human absence/presence must be verified against the loaded stage')
    if bool(humans) != bool(bindings['expected_humans']):
        raise ValueError('Loaded human inventory disagrees with requested variant')
    entries = inventory['colliders']
    by_path = {e['path']: e for e in entries}
    if len(by_path) != len(entries):
        raise ValueError('Collider paths are not unique')
    for e in entries:
        if not e['path'].startswith('/') or not e.get('fingerprint'):
            raise ValueError('Scope requires absolute collider paths and frozen mesh hashes')
        if not isinstance(e.get('dynamic'), bool):
            raise ValueError('Collider motion status must be explicit')
        if e['dynamic'] and not e.get('actor'):
            raise ValueError('Dynamic collider has no measured actor identity')
    unresolved, omitted, relations = deepcopy(bindings.get('unresolved',[])), [], []
    for aid in contract['asset_declarations']:
        if aid not in roots or not any(under(p, roots[aid]) for p in by_path):
            unresolved.append(dict(kind='asset_without_colliders', asset_id=aid))
    robot_entries = [e for e in entries if under(e['path'], robot)]
    human_entries = [e for e in entries if any(under(e['path'], r) for r in humans)]
    observed_human_entries=list(human_entries)
    surface_omissions=bindings.get('hs_human_surface_omissions',[])
    removed_human={x['collider_path'] for x in surface_omissions}
    if not removed_human<={e['path'] for e in human_entries}:
        raise ValueError('HS surface omission is outside current human inventory')
    human_entries=[e for e in human_entries if e['path'] not in removed_human]
    for declaration in surface_omissions:
        omitted.append(dict(category='HS',target=declaration['collider_path'],query_distance=False,
            persist_distance=False,acquisition_omission=True,reason=declaration['reason'],
            authorization=declaration['authorization'],source_usd_sha256=declaration['source_usd_sha256']))
    if not robot_entries or (humans and not human_entries):
        raise ValueError('Missing required robot/human collision surfaces')
    if any(under(h, robot) or under(robot, h) for h in humans):
        raise ValueError('Robot and human roots overlap')
    sources = defaultdict(list)
    roles = {}
    for e in robot_entries:
        if not e.get('actor'):
            unresolved.append(dict(kind='robot_actor_missing', collider=e['path']))
            continue
        sources[e['actor']].append(e['path']); roles[e['actor']] = 'robot'
    payload_owners = {}
    for aid in contract['roles']['payload_ids']:
        root = roots.get(aid)
        owners = set()
        for e in entries:
            if root and under(e['path'], root):
                actor = e.get('actor') or e['path']
                sources[actor].append(e['path']); roles[actor] = 'payload'
                owners.add(actor)
        payload_owners[aid] = owners
    for aid in contract['roles']['articulated_target_ids']:
        actors = bindings.get('manipulated_actor_paths', {}).get(aid)
        if not actors:
            unresolved.append(dict(kind='manipulated_links_unbound', asset_id=aid))
            continue
        for actor in actors:
            colliders = [e['path'] for e in entries if e.get('actor') == actor]
            if not colliders or not under(actor, roots.get(aid, '/__unbound__')):
                raise ValueError('Manipulated link is missing or outside its task asset: '+actor)
            sources[actor].extend(colliders); roles[actor] = 'manipulated_link'
    fixed = set(bindings.get('fixed_robot_actor_paths', []))
    actual_robot_actors = {e.get('actor') for e in robot_entries}
    if not fixed <= actual_robot_actors:
        raise ValueError('Fixed robot binding is not in the current actor inventory')
    if fixed and not bindings.get('fixed_graph_evidence'):
        raise ValueError('Fixed robot paths require actual joint-graph evidence')
    grips = set(bindings.get('grip_actor_paths', []))
    if not grips <= actual_robot_actors:
        raise ValueError('Grip binding is not in current robot inventory')
    for source, colliders in sorted(sources.items()):
        for human in human_entries:
            rid = 'HS:'+digest([source, human['path']])[:24]
            if source in fixed and bindings.get('hs_fixed_omission_authorized') is True:
                omitted.append(dict(relation_id=rid, category='HS', source=source,
                    target=human['path'], query_distance=False, persist_distance=False,
                    acquisition_omission=True, reason='authorized_actual_fixed_robot_structure'))
                continue
            relations.append(dict(relation_id=rid, category='HS', source=source,
                source_role=roles[source], source_colliders=sorted(set(colliders)),
                target=human['path'], target_colliders=[human['path']],
                query_distance=True, persist_distance=True, grade_distance=True,
                grade_contact=True, stop_model_binding=bindings.get('stop_models', {}).get(source),
                evidence_only=False))
    normal = {}
    for declaration in contract['normal_distance_relations']:
        target = declaration['target_id']
        surfaces = bindings.get('surface_bindings', {}).get(target)
        if not surfaces:
            unresolved.append(dict(kind='normal_contact_surface_unbound', source_id=declaration['source_id'],
                target_id=target, operation_id=declaration['operation_id']))
            continue
        if not set(surfaces) <= by_path.keys() or any(
                not under(p, roots.get(target, '/__unbound__')) for p in surfaces):
            raise ValueError('Normal contact binding escapes its declared task asset')
        for source in payload_owners[declaration['source_id']]:
            for target_path in surfaces:
                normal[(source, target_path)] = declaration
    targets = defaultdict(list)
    background_roots=[roots[aid] for aid,d in contract['asset_declarations'].items()
                      if d['kind']=='background' and aid in roots]
    for e in entries:
        if under(e['path'],robot) or any(under(e['path'],r) for r in humans):continue
        # Preserve body-pair force/impulse aggregation across all its colliders.
        # Static kitchen geometry has no actor; use its authored asset subtree,
        # splitting distance surfaces below without splitting contact evidence.
        key=e.get('actor')
        if key is None:
            background=next((r for r in background_roots if under(e['path'],r)),None)
            if background:
                key=background+'/'+e['path'][len(background)+1:].split('/')[0]
            else:
                matches=[r for r in roots.values() if under(e['path'],r)]
                key=max(matches,key=len) if matches else e['path']
        targets[key].append(e)
    # Verify derived evidence against this inventory, never accept an authored
    # allowlist claiming arbitrary bodies are a rigid assembly.
    assembly_pairs={}
    if 'fixed_assembly_distance_groups' in bindings:
        from kitchen_safety.task_binding import fixed_assembly_distance_groups
        groups=fixed_assembly_distance_groups(contract,inventory,bindings)
        if groups!=bindings['fixed_assembly_distance_groups']:
            raise ValueError('Fixed assembly omission differs from frozen joint/collider evidence')
        for group in groups:
            for left in group['actor_paths']:
                for right in group['actor_paths']:
                    if left!=right:
                        if (left,right) in assembly_pairs:raise ValueError('Ambiguous fixed assembly binding')
                        assembly_pairs[(left,right)]=group
    all_payload_colliders = {p for source in sources if roles[source] == 'payload' for p in sources[source]}
    for source, colliders in sorted(sources.items()):
        for target_id, target_entries in sorted(targets.items()):
            target_paths=[e['path'] for e in target_entries if e['path'] not in colliders and e.get('actor')!=source]
            if not target_paths:continue
            exempt=[p for p in target_paths if (source,p) in normal]
            distance_paths=sorted(set(target_paths)-set(exempt))
            rid = 'PT:'+digest([source, target_id])[:24]
            relation = dict(relation_id=rid, category='PT', source=source,
                source_role=roles[source], source_colliders=sorted(set(colliders)),
                target=target_id, target_colliders=distance_paths,
                contact_target_colliders=sorted(target_paths),
                query_distance=True, persist_distance=True, grade_distance=True,
                grade_force=True, grade_impulse=True, evidence_only=False,
                applicability='held_evidence_required' if roles[source] == 'payload'
                    else 'manipulation_control_evidence_required' if roles[source] == 'manipulated_link' else 'active')
            operation_omissions=[]
            for declaration in bindings.get('operation_contacts',[]):
                selected=sorted(set(declaration['target_colliders'])&set(distance_paths))
                if source in declaration['source_actor_paths'] and selected:
                    operation_omissions.append(dict(operation_id=declaration['operation_id'],
                        allowed_phases=declaration['allowed_phases'],target_colliders=selected,
                        declaration_sha256=declaration['declaration_sha256'],reason=declaration['reason']))
            if operation_omissions:relation['operation_distance_omissions']=operation_omissions
            if exempt:
                omitted.append(dict(relation_id=rid+':normal_surfaces',category='PT',source=source,
                    target=target_id,target_colliders=sorted(exempt),query_distance=False,persist_distance=False,
                    reason='normal task destination/support surfaces only',force_and_impulse_retained=True))
            if not distance_paths:
                relation.update(query_distance=False,persist_distance=False,grade_distance=False,
                    reason='all distance surfaces are normal task destination/support')
            fixed_world=set(bindings.get('fixed_world_actor_paths',[]))
            if source in fixed_world and all(not e['dynamic'] or e['actor'] in fixed_world for e in target_entries):
                omitted.append(dict(relation_id=rid,category='PT',source=source,target=target_id,
                    query_distance=False,persist_distance=False,reason='both endpoints physically world-fixed in this frozen episode',
                    force_and_impulse_retained=True,evidence=bindings.get('fixed_graph_evidence')))
                relation.update(query_distance=False,persist_distance=False,grade_distance=False,
                    reason='both endpoints are physically world-fixed; no relative approach')
            assembly=assembly_pairs.get((source,target_id))
            if assembly is not None:
                if (roles[source]!='manipulated_link' or any(e.get('actor')!=target_id for e in target_entries)
                        or set(colliders)!=set(assembly['collider_paths_by_actor'][source])
                        or set(target_paths)!=set(assembly['collider_paths_by_actor'][target_id])):
                    raise ValueError('Fixed assembly relation does not exactly match its physical bodies')
                omitted.append(dict(relation_id=rid,category='PT',source=source,target=target_id,
                    source_colliders=sorted(colliders),target_colliders=sorted(target_paths),
                    query_distance=False,persist_distance=False,force_and_impulse_retained=True,
                    reason='internal unbreakable fixed assembly',assembly_group_sha256=assembly['group_sha256']))
                relation.update(query_distance=False,persist_distance=False,grade_distance=False,
                    reason='internal unbreakable fixed assembly',assembly_group_sha256=assembly['group_sha256'])
            # Evidence still supports grasp state. Exempt only the declared grip
            # parts in the normal phase; other robot links retain PT distances.
            if source in grips and set(target_paths)<=all_payload_colliders:
                relation.update(evidence_only=True, grade_distance=False,
                    reason='grip_geometry_evidence; normal contact requires phase binding',
                    grade_distance_outside_normal_phase=True)
            relations.append(relation)
    result = dict(schema_version='robolaw-runtime-scope-1', task_id=contract['task_id'],
        task_sha256=digest(contract), inventory_sha256=digest(inventory), bindings=deepcopy(bindings),
        relations=relations, omitted=omitted, unresolved=unresolved,
        hs_acquisition_omissions=dict(version='arena-hs-data-omission-1',
            scope='hs_geometry_features_and_contacts',
            authorization='user-authorized fixed robot structure HS acquisition omission; current physical joint binding',
            human_collider_paths=sorted(removed_human),robot_actor_paths=sorted(fixed) if bindings.get('hs_fixed_omission_authorized') else [],
            human_roots=list(humans),omit_fixed_human=bool(fixed and humans and bindings.get('hs_fixed_omission_authorized')),
            semantics='Filter exact identities before reading PhysX contact points; RS geometry and shared reconstruction states are independent'),
        human_inventory=dict(complete=True, verified_absent=not humans,
            collision_paths=[e['path'] for e in human_entries],observed_collision_paths=[e['path'] for e in observed_human_entries],
            intentionally_omitted_collision_paths=sorted(removed_human)),
        coverage=dict(scope_resolved=not unresolved, scene_verified=False,
            manipulation_surface_bindings_complete={o['operation_id'] for o in contract['operations'] if 'articulation_id' in o}
                <= {o['operation_id'] for o in bindings.get('operation_contacts',[])},
            acceptance_gaps=[dict(asset_id=aid,
                reason='structural mating surface and stopping model pending; operation surfaces reported separately')
                for aid in contract['roles']['articulated_target_ids']],
            candidate_policy='conservative all relations until a recorded motion/approach-envelope proof exists',
            inherited_coastal_omissions=False),
        rs=dict(source='independent robot joint graph and full collider inventory',
            collider_paths=[e['path'] for e in robot_entries],
            robot_root=robot, joints=deepcopy(inventory.get('joints', []))))
    result['scope_sha256'] = digest(result)
    return result
