"""Explicit HS acquisition omissions, frozen per episode; no geometry fallback."""
from copy import deepcopy

VERSION = 'arena-hs-data-omission-1'


def validate(policy):
    if not policy:
        return
    if (policy.get('version') != VERSION or not policy.get('authorization')
            or policy.get('scope') != 'hs_geometry_features_and_contacts'):
        raise ValueError('HS data omissions require explicit version, scope and authorization')
    for key in ('human_collider_paths', 'robot_body_names'):
        values = policy.get(key)
        if not isinstance(values, list) or len(values) != len(set(values)):
            raise ValueError('HS omission paths/names must be unique lists')
        for path in values:
            if (not isinstance(path, str) or not path or path.startswith('/')
                    or any(p in ('', '.', '..') for p in path.split('/'))
                    or any(c in path for c in '*?[]')):
                raise ValueError('HS omissions require exact relative paths or body names')
            if key == 'robot_body_names' and '/' in path:
                raise ValueError('HS robot omission requires a rigid body name')


def resolve(policy, root, human_ids, fixed_body_names):
    validate(policy)
    if not policy:
        return {}
    if not set(policy['robot_body_names']) <= set(fixed_body_names):
        raise ValueError('HS robot data omission is restricted to declared fixed bodies')
    if any(p.split('/')[0] not in human_ids for p in policy['human_collider_paths']):
        raise ValueError('HS omitted human collider must be under a declared human root')
    return dict(version=VERSION, scope=policy['scope'], authorization=policy['authorization'],
                human_collider_paths=[root + '/' + p for p in policy['human_collider_paths']],
                robot_body_names=list(policy['robot_body_names']), robot_actor_paths=[],
                human_roots=[root + '/' + p for p in human_ids],
                omit_fixed_human=bool(policy['robot_body_names']),
                semantics='No HS mesh query, nearest point, surface speed, TTC, contact-point extraction or derived audit for omitted assets. Shared simulation state, RS/PT geometry and reconstruction evidence retain their own scopes.')


def under(path, root):
    return isinstance(path, str) and (path == root or path.startswith(root + '/'))


def omitted_contact(record, policy):
    """Decide using identities before extracting PhysX points; never omit unknown IDs."""
    if not policy:
        return False
    colliders = (record.get('collider0'), record.get('collider1'))
    if any(c in policy['human_collider_paths'] for c in colliders):
        return True
    actors = (record.get('actor0'), record.get('actor1'))
    for fixed_side, human_side in ((0, 1), (1, 0)):
        fixed = any(under(actors[fixed_side], p) or under(colliders[fixed_side], p)
                    for p in policy['robot_actor_paths'])
        human = any(under(actors[human_side], p) or under(colliders[human_side], p)
                    for p in policy['human_roots'])
        if fixed and human:
            return True
    return False


def frozen(policy):
    return deepcopy(policy)
