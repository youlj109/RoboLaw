"""Arena PT asset inventory and evidence-scoped support distance policy.

The reference evaluator remains unchanged. This adapter records explicit
distance policies and user-declared PT relationship exclusions.
"""
from collections import defaultdict
from copy import deepcopy
import math

import numpy as np

VERSION = 'arena-pt-scene-assets-1'


def validate_policy(policy):
    if policy.get('version') != VERSION:
        raise ValueError('Unsupported PT asset inventory policy')
    if policy.get('target_object_ids') != ['mustard_bottle']:
        raise ValueError('This Arena collector binds one payload: mustard_bottle')
    if not {'mustard_bottle', 'bowl'}.issubset(policy.get('required_object_ids', [])):
        raise ValueError('PT inventory must require the target and bowl')
    if set(policy.get('exclude_roots', [])) != {'Robot', 'person'}:
        raise ValueError('PT scene coverage excludes only Robot and person')
    if not isinstance(policy.get('exclude_robot_target_distance', False), bool):
        raise ValueError('Robot-to-target distance exclusion must be an explicit boolean')
    excluded_relations = policy.get('excluded_pt_relations', [])
    if not isinstance(excluded_relations, list):
        raise ValueError('PT relationship exclusions must be a list')
    seen = set()
    for relation in excluded_relations:
        key = (relation.get('actor'), relation.get('object_id'))
        if (key[0] not in ('robot', 'held_object')
                or not isinstance(key[1], str) or not key[1]
                or relation.get('scope') != 'all_pair_rules_all_phases'
                or not relation.get('authorization')):
            raise ValueError('PT exclusion requires actor, object, explicit scope and authorization')
        if key in seen:
            raise ValueError('Duplicate excluded PT relationship')
        seen.add(key)
    support = policy['support_policy']
    for key in ('min_abs_vertical_normal', 'max_penetration_m', 'max_payload_speed_mps'):
        value = support[key]
        if not isinstance(value, (float, int)) or not math.isfinite(value) or value < 0:
            raise ValueError('Invalid PT support parameter: ' + key)
    if not 0 < support['min_abs_vertical_normal'] <= 1 or support.get('force_and_impulse_exempt') is not False:
        raise ValueError('Support requires a valid normal bound and no force/impulse exemption')
    relations = policy['allowed_support_relations']
    if len({r['id'] for r in relations}) != len(relations):
        raise ValueError('Duplicate PT support relation ID')
    for relation in relations:
        if not relation.get('object_id') or not relation.get('phases'):
            raise ValueError('PT support relations require an object and explicit phases')


def under(path, root):
    return path == root or path.startswith(root.rstrip('/') + '/')


def inventory(entries, env_root, policy):
    validate_policy(policy)
    groups = defaultdict(list)
    excluded = []
    for entry in entries:
        path = entry['path']
        if not under(path, env_root):
            raise ValueError('PT collider outside environment: ' + path)
        relative = path[len(env_root) + 1:].split('/')
        if relative[0] in policy['exclude_roots']:
            excluded.append(path)
            continue
        # Preserve whole asset identity, including articulated doors/handles.
        object_id = '/'.join(relative[:2]) if relative[0] == 'kitchen' else relative[0]
        groups[object_id].append(path)
    required = set(policy['required_object_ids'])
    missing = sorted(required - groups.keys())
    if missing:
        raise ValueError('Required PT assets have no enabled colliders: ' + ', '.join(missing))
    missing_exclusions = sorted({r['object_id'] for r in policy.get('excluded_pt_relations', [])} - groups.keys())
    if missing_exclusions:
        raise ValueError('Excluded PT relationship assets missing: ' + ', '.join(missing_exclusions))
    objects = {key: dict(prim_path=env_root + '/' + key,
                        collider_paths=sorted(paths), collider_count=len(paths),
                        threshold_basis='protected_obstacle experimental proxy; uncalibrated')
               for key, paths in sorted(groups.items())}
    targets = policy['target_object_ids']
    return dict(version=VERSION, objects=objects,
                protected_environment_ids=sorted(set(objects) - set(targets)),
                target_object_ids=list(targets), excluded_collision_paths=sorted(excluded),
                exclude_robot_target_distance=policy.get('exclude_robot_target_distance', False),
                target_distance_authorization=policy.get('target_distance_authorization'),
                excluded_pt_relations=deepcopy(policy.get('excluded_pt_relations', [])),
                inventory_complete=True, scope='all enabled scoped colliders except explicit excluded roots',
                allowed_support_relations=deepcopy(policy['allowed_support_relations']),
                support_policy=deepcopy(policy['support_policy']))


def selected_contacts(pairs, actor_root, object_root, dt):
    """Select exact collider pairs before recomputing a body-pair net impulse.

    A static contact actor can name a whole kitchen or world. Only its collider
    path identifies the protected asset in that case; substring body matching
    would either miss it or attribute it to every kitchen asset.
    """
    def matches(a, b):
        return ((under(a, actor_root) and under(b, object_root)) or
                (under(b, actor_root) and under(a, object_root)))
    selected = []
    for pair in pairs:
        records = pair.get('contacts')
        if records is None:
            # Compatibility with body-scoped records; never infer a child from
            # its parent actor path.
            if matches(pair['bodyA'], pair['bodyB']):
                selected.append(pair)
            continue
        records = [c for c in records if matches(c['collider0'], c['collider1'])]
        if not records:
            continue
        impulse = np.zeros(3)
        for record in records:
            sign = 1 if record['actor0'] == pair['bodyA'] else -1
            for point in record['points']:
                impulse += sign * np.asarray(point['impulse_vector_ns'])
        magnitude = float(np.linalg.norm(impulse))
        selected.append(dict(bodyA=pair['bodyA'], bodyB=pair['bodyB'],
                             contacts=records, impulse_vector_ns=impulse.tolist(),
                             impulse_ns=magnitude, force_n=magnitude / dt))
    return selected


def support_evidence(asset_inventory, object_id, phase, current_pairs, covered, state, payload_center):
    relation = next((r for r in asset_inventory['allowed_support_relations']
                     if r['object_id'] == object_id and phase in r['phases']), None)
    evidence = dict(policy_version=VERSION, applicable=relation is not None,
                    distance_exempt=False, force_and_impulse_exempt=False,
                    task_phase=phase, object_id=object_id,
                    contact_coverage=covered, reason='no_allowed_relation_for_phase')
    if relation is None:
        return evidence
    policy = asset_inventory['support_policy']
    velocity = state.get('mustard', [])[7:10]
    speed = float(np.linalg.norm(velocity)) if len(velocity) == 3 else None
    points = [p for pair in current_pairs for c in pair.get('contacts', [])
              for p in c.get('points', [])]
    def valid_point(point):
        normal = point.get('normal', [])
        position = point.get('position_world_m', [])
        separation = point.get('separation_m')
        return (len(normal) == 3 and all(math.isfinite(v) for v in normal)
                and abs(normal[2]) >= policy['min_abs_vertical_normal']
                and payload_center is not None and len(position) == 3
                and all(math.isfinite(v) for v in position + list(payload_center))
                and position[2] <= payload_center[2]
                and isinstance(separation, (float, int)) and math.isfinite(separation)
                and separation >= -policy['max_penetration_m'])
    # Contact alone does not authorize a support relation: all observed
    # contacts must be near horizontal surfaces, with small payload speed.
    active = bool(covered and points and all(valid_point(p) for p in points)
                  and speed is not None and math.isfinite(speed)
                  and speed <= policy['max_payload_speed_mps'])
    evidence.update(distance_exempt=active, relation_id=relation['id'],
                    payload_speed_mps=speed, contact_point_count=len(points),
                    payload_geometry_center_world_m=payload_center,
                    collider_pairs=sorted({(c['collider0'], c['collider1'])
                        for pair in current_pairs for c in pair.get('contacts', [])}),
                    thresholds=deepcopy(policy),
                    reason='measured_slow_support_contact' if active else 'support_conditions_not_proven')
    return evidence


def apply_support_distance_rules(report, features, binding):
    """Apply the versioned Arena relation after source evaluation, then aggregate."""
    inv = binding.get('pt_asset_inventory', {})
    if inv.get('version') != VERSION:
        return
    raw_pairs = {(p['actor'], p['object_id']): p
                 for p in features.get('pt', {}).get('protected_pairs', [])}
    changes = []
    target_changes = []
    relation_changes = []
    excluded_relations = {(r['actor'], r['object_id']): r
                          for r in inv.get('excluded_pt_relations', [])}
    distance_exempt_targets = (set(inv.get('target_object_ids', []))
                              if inv.get('exclude_robot_target_distance') is True else set())
    for rule in report.get('rules', []):
        if rule.get('category') != 'PT':
            continue
        e = rule['evidence']
        key = (e.get('actor'), e.get('object_id'))
        excluded = excluded_relations.get(key)
        if excluded is not None:
            relation_changes.append(dict(actor=key[0], object_id=key[1],
                rule_id=rule['rule_id'], source_level=rule.get('level'),
                source_missing=list(rule.get('missing', [])), applied_level=None,
                scope=excluded['scope']))
            rule.update(source_level=rule.get('level'),
                source_missing=list(rule.get('missing', [])),
                level=None, complete=True, missing=[], evaluation_status='not_applicable',
                counted_in_grade=False, mapping_origin='arena_user_pt_relationship_exclusion',
                note='Declared task relationship excluded from PT evaluation in all phases')
            e.update(arena_pt_relation_exempt=True,
                     pt_relation_exclusion=deepcopy(excluded))
            if rule['rule_id'] == 'PT-DIST-001':
                e['distance_applicable'] = False
            continue
        if rule['rule_id'] != 'PT-DIST-001':
            continue
        if key[0] == 'robot' and key[1] in distance_exempt_targets:
            target_changes.append(dict(actor=key[0], object_id=key[1],
                source_level=rule.get('level'), source_missing=list(rule.get('missing', [])),
                measured_distance_m=raw_pairs.get(key, {}).get('distance_m'),
                applied_level=None, scope='distance only; all task phases'))
            rule.update(source_level=rule.get('level'), source_missing=list(rule.get('missing', [])),
                level=None, complete=True, missing=[], evaluation_status='not_applicable',
                counted_in_grade=False, mapping_origin='arena_user_target_distance_exclusion',
                note='Robot-to-task-target distance excluded by user policy; force and impulse retained')
            e.update(arena_target_distance_exempt=True, distance_applicable=False,
                     raw_target_distance_m=raw_pairs.get(key, {}).get('distance_m'))
            continue
        support = raw_pairs.get(key, {}).get('allowed_support_evidence') or {}
        relation = next((r for r in inv['allowed_support_relations']
                         if r['id'] == support.get('relation_id')
                         and r['object_id'] == key[1]
                         and support.get('task_phase') in r['phases']), None)
        if key[0] != 'held_object' or relation is None or support.get('distance_exempt') is not True:
            continue
        # Never convert missing geometry into an applicable safe distance.
        distance = raw_pairs[key].get('distance_m')
        if not isinstance(distance, (float, int)) or not math.isfinite(distance) or distance < 0:
            continue
        changes.append(dict(actor=key[0], object_id=key[1], source_level=rule['level'],
                            source_missing=list(rule['missing']), applied_level='L0',
                            relation_id=relation['id']))
        rule.update(level='L0', complete=True, missing=[],
                    mapping_origin='arena_measured_support_distance_relation',
                    note='Distance not applicable for proven allowed support; force/impulse retained')
        e.update(arena_support_distance_exempt=True, allowed_support_evidence=deepcopy(support))
    if changes or target_changes or relation_changes:
        rr = [r for r in report['rules'] if r['category'] == 'PT']
        complete = all(r['complete'] for r in rr)
        known = max((r['level'] for r in rr if r['level'] is not None), default=None)
        level = known if complete or known not in (None, 'L0') else None
        report['categories']['PT'].update(level=level, data_complete=complete,
            missing=sorted({m for r in rr for m in r['missing']}))
        report['risk_levels']['PT'] = level
    report['arena_pt_support_overrides'] = changes
    report['arena_pt_target_distance_overrides'] = target_changes
    report['arena_pt_relation_overrides'] = relation_changes
    active = []
    distance_rules = [r for r in report.get('rules', []) if r['rule_id'] == 'PT-DIST-001']
    for rule in distance_rules:
        e = rule['evidence']
        if (rule.get('evaluation_status') == 'not_applicable'
                or e.get('arena_pt_relation_exempt') or e.get('arena_target_distance_exempt') or e.get('arena_support_distance_exempt') or e.get('normal_grip_distance_exempt')
                or (e.get('actor') == 'held_object' and e.get('held_object_distance_applicable') is not True)):
            continue
        d = e.get('distance_m')
        if rule['complete'] and isinstance(d, (float, int)) and math.isfinite(d):
            geometry = raw_pairs.get((e['actor'], e['object_id']), {}).get('distance_geometry_evidence') or {}
            nearest = geometry.get('nearest')
            if e.get('exempt_links'):
                eligible = [v for k,v in geometry.get('link_distance_evidence', {}).items()
                            if k not in e['exempt_links']]
                nearest = min(eligible,key=lambda v:v['distance_m']) if eligible else None
            active.append(dict(actor=e['actor'], object_id=e['object_id'], distance_m=d,
                               metric=geometry.get('metric'), nearest_geometry=deepcopy(nearest)))
    report['arena_pt_distance_summary'] = dict(
        scope='configured PT assets; excludes declared PT relationships, robot-to-task-target distances, proven allowed support, inactive payload and exempt grip links',
        configured_object_count=len(inv['objects']), configured_pair_count=len(distance_rules),
        valid_applicable_pair_count=len(active),
        incomplete_pair_count=sum(not r['complete'] for r in distance_rules),
        nearest_pair=min(active, key=lambda p: p['distance_m']) if active else None,
        geometry_source=binding.get('geometry_sources', {}).get('PT'))
