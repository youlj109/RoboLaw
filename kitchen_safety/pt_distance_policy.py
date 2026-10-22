"""Explicit scene-scoped PT distance omissions; never inferred safe at runtime."""
from copy import deepcopy
import hashlib
import json

VERSION = 'arena-pt-distance-omission-1'
DISTANCE_FIELDS = ('distance_m', 'distance_source', 'distance_metric',
                   'link_distances_m', 'distance_coverage_complete', 'distance_geometry_evidence',
                   'allowed_support_evidence')


def omitted_keys(inventory):
    policy = inventory.get('distance_omissions') or {}
    return {(r['actor'], r['object_id']) for r in policy.get('relations', [])}


def inventory_signature(inventory, root):
    rows = sorted((name, sorted(path[len(root):] for path in asset['collider_paths']))
                  for name, asset in inventory['objects'].items())
    return hashlib.sha256(json.dumps(rows, separators=(',', ':')).encode()).hexdigest()


def validate_policies(policies):
    if not isinstance(policies, list):
        raise ValueError('PT distance omission policies must be a list')
    seen = set()
    for p in policies:
        if (p.get('version') != VERSION or not p.get('id') or not p.get('scenario_id')
                or p.get('scope') != 'distance_only_all_phases' or not p.get('authorization')
                or not p.get('selection_report_sha256')
                or not isinstance(p.get('inventory_signature'), str)
                or len(p['inventory_signature']) != 64
                or not isinstance(p.get('relations'), list) or not p['relations']):
            raise ValueError('PT distance omissions require explicit scope, scene, provenance and relations')
        if p['scenario_id'] in seen:
            raise ValueError('Duplicate scene distance omission policy')
        seen.add(p['scenario_id'])
        guard = p.get('layout_guard', {})
        if set(guard) != {'task', 'robot_position', 'robot_yaw_deg', 'stand_height', 'obstacle_usd_path'}:
            raise ValueError('PT omission policy requires explicit task/robot/obstacle guards')
        pairs = set()
        for r in p['relations']:
            key = (r.get('actor'), r.get('object_id'))
            if key[0] not in ('robot', 'held_object') or not isinstance(key[1], str) or not key[1] or key in pairs:
                raise ValueError('Invalid or duplicate PT distance omission relationship')
            pairs.add(key)


def resolve(policy, spec, inventory, root):
    """Resolve once at capture; offline replay uses this immutable binding only."""
    policies = policy.get('distance_omission_policies', [])
    validate_policies(policies)
    selected = next((p for p in policies if p['scenario_id'] == spec['obstacle_id']), None)
    if selected is None:
        return None
    layout = spec['layout']
    observed = {k: layout[k] for k in ('task', 'robot_position', 'robot_yaw_deg', 'stand_height')}
    observed['obstacle_usd_path'] = layout['obstacle']['usd_path']
    if observed != selected['layout_guard'] or inventory_signature(inventory, root) != selected['inventory_signature']:
        return dict(version=VERSION, policy_id=selected['id'], relations=[],
                    status='inactive_scene_guard_mismatch')
    for r in selected['relations']:
        if r['object_id'] not in inventory['objects']:
            raise ValueError('Omitted PT asset missing from scene inventory')
        if r['object_id'] in inventory['target_object_ids']:
            raise ValueError('Task-target geometry must remain available for grasp evidence')
    return dict(version=VERSION, policy_id=selected['id'], scenario_id=selected['scenario_id'],
                scope=selected['scope'], authorization=selected['authorization'],
                selection_report_sha256=selected['selection_report_sha256'],
                inventory_signature=selected['inventory_signature'],
                relations=deepcopy(selected['relations']), status='omitted_by_user_policy',
                semantics='No distance query, bounds, distance feature or distance rule for these relations; not a safe-distance certificate. Contact/force/impulse and required raw reconstruction evidence retained.')
