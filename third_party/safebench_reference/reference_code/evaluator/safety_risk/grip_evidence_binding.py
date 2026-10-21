"""Explicit, arm-specific collision-body bindings for single-pick evidence.

This is contact evidence, not a force limit or proof against object slip.
Unknown/malformed bindings never imply a confirmed failed grasp.
"""
from copy import deepcopy

SCHEMA = 'single-pick-grip-evidence-v1'


def build_grip_evidence_binding(robot_cfg, *, robot_id, arm, target_object_id,
                                source_config=None):
    """robot_id is the task.robots instance key, not the USD asset class."""
    if arm not in ('left', 'right'):
        raise ValueError('arm must be left or right')
    key = 'fl_filter_paths' if arm == 'left' else 'fr_filter_paths'
    paths = robot_cfg.get(key)
    # The four current robot configs explicitly list exactly two gripping links.
    if (not isinstance(paths, list) or len(paths) != 2 or
            not all(isinstance(p, str) and p.strip('/') for p in paths) or
            len(set(paths)) != 2):
        raise ValueError('two distinct gripping link paths required: ' + key)
    if not isinstance(robot_id, str) or not robot_id or '/' in robot_id:
        raise ValueError('robot_id must be the task robot instance key')
    if not isinstance(target_object_id, str) or not target_object_id or '/' in target_object_id:
        raise ValueError('target_object_id must be the task object instance key')
    return {'schema_version': SCHEMA, 'robot_id': robot_id, 'arm': arm,
            'target_object_id': target_object_id,
            'finger_groups': {'left': ['robot/' + robot_id + '/' + paths[0].strip('/')],
                              'right': ['robot/' + robot_id + '/' + paths[1].strip('/')]},
            'group_labels': 'opposing contact groups in configured order, not world left/right',
            'source_config': source_config, 'source_field': key,
            'configured_filter_paths': list(paths), 'proxy': True}


def resolve_grip_evidence_binding(raw):
    meta = raw.get('episode_meta', {})
    binding = meta.get('grip_evidence_binding')
    if binding is not None:
        groups = binding.get('finger_groups', {}) if isinstance(binding, dict) else {}
        if (isinstance(binding, dict) and binding.get('schema_version') == SCHEMA and
                binding.get('target_object_id') == meta.get('target_object_id') and
                set(groups) == {'left', 'right'} and
                all(isinstance(groups[s], list) and groups[s] and
                    all(isinstance(p, str) and p.startswith('robot/') for p in groups[s])
                    for s in groups) and not set(groups['left']) & set(groups['right'])):
            return dict(deepcopy(binding), status='bound', match_mode='exact_body')
        return {'status': 'unknown', 'reason': 'invalid_grip_evidence_binding',
                'target_object_id': meta.get('target_object_id', 'pick_object_left')}
    robot_id = str(meta.get('robot_id') or meta.get('robot_name') or '').lower()
    # Preserve known Franka episodes without new metadata, including older files
    # whose only robot identity is the recorded Panda collision-body name.
    legacy_panda = robot_id in ('franka', 'fr3', 'panda')
    if not robot_id:
        for pairs in raw.get('collision_gt', {}).get('collision_pair_gt', []):
            if isinstance(pairs, list) and any(isinstance(p, dict) and any(
                    str(p.get(k, '')).startswith('robot/') and
                    str(p.get(k, '')).endswith(('/panda_leftfinger', '/panda_rightfinger'))
                    for k in ('bodyA', 'bodyB')) for p in pairs):
                legacy_panda = True
                break
    if legacy_panda:
        return {'status': 'legacy_franka', 'match_mode': 'legacy_panda_leaf',
                'target_object_id': meta.get('target_object_id', 'pick_object_left'),
                'finger_groups': {'left': ['panda_leftfinger'], 'right': ['panda_rightfinger']}}
    return {'status': 'unknown', 'reason': 'missing_grip_evidence_binding',
            'target_object_id': meta.get('target_object_id', 'pick_object_left')}


def target_finger_contacts(pairs, binding, *, target_object_id=None):
    """Return (bilateral bool|None, matching group names), without cross-arm mixing."""
    if binding.get('status') not in ('bound', 'legacy_franka') or not isinstance(pairs, list):
        return None, []
    target = 'object/' + str(target_object_id or binding.get('target_object_id'))
    found = set()
    for pair in pairs:
        if not isinstance(pair, dict):
            return None, sorted(found)
        a, b = pair.get('bodyA', ''), pair.get('bodyB', '')
        other = b if a == target else a if b == target else ''
        if not isinstance(other, str) or not other.startswith('robot/'):
            continue
        for side, bodies in binding['finger_groups'].items():
            if (other in bodies if binding['match_mode'] == 'exact_body'
                    else any(other.endswith('/' + body) for body in bodies)):
                found.add(side)
    return found == {'left', 'right'}, sorted(found)
