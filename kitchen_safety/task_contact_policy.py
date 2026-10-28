"""Resolve authorized normal-support PT exclusions before any risk queries.

Only declared task support surfaces are eligible. Shared raw contacts, grasp
and support evidence, HS and the independent RS geometry stay unchanged.
"""
import hashlib
import json
from pathlib import Path
from kitchen_safety.task_contract import digest

VERSION = 'robolaw-normal-support-exclusion-1'

def load_policy(root, contract):
    task_id = contract['task_id']
    if Path(task_id).name != task_id or task_id in ('.', '..'):
        raise ValueError('Invalid task identity for contact policy')
    path = Path(root) / 'configs/risk_scope_policies' / (task_id + '.json')
    if not path.exists():
        return None
    content = path.read_bytes()
    return dict(policy=json.loads(content), source_file_sha256=hashlib.sha256(content).hexdigest(),
                source_relative_path=str(path.relative_to(root)))

def resolve_exclusions(contract, inventory, bindings, sources, roles):
    envelope = bindings.get('pt_normal_support_exclusions')
    if envelope is None:
        return {}, None
    policy = envelope['policy']
    if (policy.get('version') != VERSION or policy.get('task_id') != contract['task_id']
            or policy.get('task_sha256') != digest(contract) or not policy.get('authorization')
            or policy.get('scope') != 'all_PT_features_and_rules'
            or policy.get('active_phases') != ['all']):
        raise ValueError('Normal-support exclusion must bind the exact task and explicit full PT scope')
    declarations = policy.get('relations')
    if not isinstance(declarations, list) or not declarations:
        raise ValueError('Normal-support policy needs explicit relationships')
    operations = {op['operation_id']:op for op in contract['operations']}
    entries = {e['path']:e for e in inventory['colliders']}
    fixed = set(bindings.get('fixed_robot_actor_paths', []))
    selected = {}
    for declaration in declarations:
        if set(declaration) != {'operation_id', 'source_role', 'target_id'}:
            raise ValueError('Invalid normal-support relationship fields')
        op = operations.get(declaration['operation_id'])
        target = declaration['target_id']
        if (not op or op['kind'] != 'PickAndPlaceTask'
                or target not in op['expected_support_ids']
                or contract['asset_declarations'][target]['kind'] != 'reference'):
            raise ValueError('Only an explicitly declared support reference can be excluded')
        root = bindings['asset_roots'].get(target)
        surfaces = bindings.get('surface_bindings', {}).get(target)
        if (not root or not surfaces or len(set(surfaces)) != len(surfaces)
                or any(p not in entries or not (p == root or p.startswith(root + '/')) for p in surfaces)):
            raise ValueError('Normal support requires exact current collision surface bindings')
        role = declaration['source_role']
        if role == 'moving_robot':
            actors = [s for s in sources if roles[s] == 'robot' and s not in fixed]
        elif role == 'payload':
            payload_root = bindings['asset_roots'][op['payload_id']]
            actors = [s for s in sources if roles[s] == 'payload'
                      and any(p == payload_root or p.startswith(payload_root + '/') for p in sources[s])]
        else:
            raise ValueError('Unsupported normal-support source role')
        if not actors:
            raise ValueError('Normal-support source has no resolved collision bodies')
        for actor in actors:
            selected.setdefault(actor, set()).update(surfaces)
    return selected, dict(version=VERSION, policy_sha256=digest(policy),
                         source_file_sha256=envelope['source_file_sha256'],
                         authorization=policy['authorization'])
