"""Validate explicit closed-world simulation absence evidence, not camera non-detection.

This is project applicability metadata, not a new frozen feature or ISO clause.
The runtime producer must attest a complete, live entity inventory each frame.
"""
def verified_absence(snapshot):
    metadata = snapshot.get('metadata', {})
    evidence = metadata.get('human_inventory')
    frame = snapshot.get('frame_index')
    if not isinstance(evidence, dict) or type(frame) is not int or frame < 0:
        return False
    required = {
        'schema_version': 'simulation-human-inventory-1',
        'source': 'PhysXDataCollector.live_scene_inventory',
        'scope': 'closed_world_simulation',
        'complete': True,
        'roles_explicit': True,
        'stage_validated': True,
        'frame_index': frame,
        'declared_human_names': [],
        'observed_human_paths': [],
        'unresolved_entity_names': [],
    }
    for key, value in required.items():
        actual = evidence.get(key)
        if type(actual) is not type(value) or actual != value:
            return False
    if metadata.get('operation_mode') != 'simulation':
        return False
    entities = evidence.get('loaded_entity_paths')
    if (not isinstance(entities, list) or not entities
            or not all(isinstance(x, str) and x.startswith('/') for x in entities)
            or len(set(entities)) != len(entities)):
        return False
    # Positive human contact is contradictory; missing contact is still unknown.
    hs = snapshot.get('features', {}).get('hs', {})
    if any(hs.get(key) is not None for key in (
            'd_robot_h_min_gt_m', 'd_ee_h_min_gt_m', 'd_obj_h_min_gt_m')):
        return False
    return hs.get('human_contact_flag_gt') is False


def scene_inventory(task, stage, declared_humans, roles_explicit, frame_index,
                    closed_world=False):
    """Snapshot registered scene entities; closed-world scope is explicit opt-in.

    The scene producer must guarantee that the task registries include every
    possible human entity. A valid prim alone never establishes that guarantee.
    """
    evidence = {
        'schema_version': 'simulation-human-inventory-1',
        'source': 'PhysXDataCollector.live_scene_inventory',
        'scope': 'closed_world_simulation' if closed_world is True else 'unverified_scope',
        'complete': False, 'roles_explicit': roles_explicit is True,
        'stage_validated': False, 'frame_index': frame_index,
        'declared_human_names': list(declared_humans),
        'observed_human_paths': [], 'unresolved_entity_names': [],
        'loaded_entity_paths': [],
    }
    if stage is None:
        return evidence
    entities = {}
    for registry in ('robots', 'objects', 'fixtures'):
        values = getattr(task, registry, {})
        if not isinstance(values, dict):
            evidence['unresolved_entity_names'].append(registry)
            continue
        for name, entity in values.items():
            path = getattr(entity, 'prim_path', None) or getattr(entity, '_prim_path', None)
            if not path or not stage.GetPrimAtPath(str(path)).IsValid():
                evidence['unresolved_entity_names'].append(str(name))
                continue
            entities[str(name)] = str(path)
    for name in declared_humans:
        if name not in entities:
            evidence['unresolved_entity_names'].append(str(name))
        else:
            evidence['observed_human_paths'].append(entities[name])
    evidence['loaded_entity_paths'] = sorted(set(entities.values()))
    evidence['stage_validated'] = bool(entities) and not evidence['unresolved_entity_names']
    evidence['complete'] = (closed_world is True and roles_explicit is True
                            and evidence['stage_validated'])
    return evidence
