"""Intersect migrated scene relations with the user's reviewed exact surfaces.

This is an evaluation-coverage ceiling, never a claim that omitted assets are
safe. No geometry queries occur here; the complete scene stays reconstructible.
"""
from copy import deepcopy
import json
from pathlib import Path
from kitchen_safety.task_contract import digest

VERSION = 'robolaw-reviewed-replicator-scope-lock-1'


def load_lock(root, contract):
    path = Path(root)/'configs/risk_integration/replicator_scope_lock.json'
    document = json.loads(path.read_text())
    if document['version'] != VERSION or not document.get('authorization'):
        raise ValueError('Invalid reviewed asset scope lock')
    row = deepcopy(document['tasks'][contract['task_id']])
    if row['task_sha256'] != digest(contract):
        raise ValueError('Reviewed scope lock belongs to a different task contract')
    return dict(version=VERSION, task_id=contract['task_id'],
                authorization=document['authorization'], **row)


def apply_lock(scope, inventory, lock):
    if (lock['version'] != VERSION or lock['task_id'] != scope['task_id']
            or lock['task_sha256'] != scope['task_sha256']):
        raise ValueError('Reviewed asset lock identity mismatch')
    env = inventory['env_root'].rstrip('/')+'/'
    def absolute(path):
        if not path or path.startswith('/') or any(x in ('', '.', '..') for x in path.split('/')):
            raise ValueError('Invalid reviewed environment-relative identity')
        return env+path
    allowed = {absolute(target): {absolute(p) for p in paths}
               for target, paths in lock['pt_targets'].items()}
    loaded={e['path'] for e in inventory['colliders']}
    missing=set().union(*allowed.values())-loaded
    if missing:
        raise ValueError('Reviewed collider identities do not bind to the ylj scene: '+repr(sorted(missing)))
    # A newly introduced person cannot silently inherit verified-absent/L0.
    humans = {absolute(p) for p in lock['human_collision_paths']}
    observed = set(scope['human_inventory']['collision_paths'])
    if not observed <= humans:
        raise ValueError('ylj introduces unreviewed human surfaces; existing HS asset ceiling would expand')
    result = deepcopy(scope); retained = []; removed = []
    for relation in result['relations']:
        if relation['category'] != 'PT':
            retained.append(relation); continue
        before = deepcopy(relation)
        paths = allowed.get(relation['target'], set())
        contact = sorted(set(relation['contact_target_colliders']) & paths)
        distance = sorted(set(relation['target_colliders']) & paths)
        outside = sorted((set(relation['contact_target_colliders']) | set(relation['target_colliders']))-paths)
        if outside:
            removed.append(dict(relation_id=relation['relation_id']+':reviewed_ceiling', category='PT',
                source=relation['source'], source_colliders=deepcopy(relation['source_colliders']),
                target=relation['target'], target_colliders=outside,
                query_distance=False, persist_distance=False, extract_risk_features=False,
                grade_distance=False, grade_force=False, grade_impulse=False,
                reason='authorized_PT_target_subset',
                policy_origin='user_reviewed_asset_scope_ceiling', policy_sha256=digest(lock),
                shared_raw_grasp_support_evidence_retained=True))
        if not contact: continue
        relation['contact_target_colliders'] = contact
        relation['target_colliders'] = distance
        if not distance:
            relation.update(query_distance=False, persist_distance=False, grade_distance=False)
        if relation.get('operation_distance_omissions'):
            for declaration in relation['operation_distance_omissions']:
                declaration['target_colliders'] = sorted(set(declaration['target_colliders']) & paths)
        # All numeric thresholds, normal-contact flags and source surfaces stay.
        assert relation['source_colliders'] == before['source_colliders']
        retained.append(relation)
    result['relations'] = retained; result['omitted'].extend(removed)
    actual = {r['target'] for r in retained if r['category'] == 'PT'}
    assert actual <= allowed.keys()
    report = dict(version=VERSION, lock_sha256=digest(lock),
        authorization=lock['authorization'], approved_PT_assets=len(allowed),
        actual_PT_assets=len(actual), approved_target_paths=sorted(allowed),
        actual_target_paths=sorted(actual), omitted_surface_relations=len(removed),
        added_evaluated_assets=[], source_scope_sha256=lock['source_scope_sha256'],
        coverage='Only user-reviewed exact surfaces; scene extras are outside risk coverage, not safe')
    result['reviewed_scope_ceiling'] = report
    result['bindings']['reviewed_scope_lock'] = deepcopy(lock)
    result.pop('scope_sha256', None); result['scope_sha256'] = digest(result)
    return result
