"""Explicit task-local PT target selection, before query/feature creation.

This is an authorized evaluation subset, not proof that excluded assets cannot
collide. Shared raw reconstruction, grasp/support evidence and HS/RS stay intact.
"""
from kitchen_safety.task_contract import digest

VERSION = 'robolaw-authorized-pt-target-subset-1'
OMISSION_REASON = 'authorized_PT_target_subset'


def resolve_target_scope(contract, inventory, bindings, targets):
    envelope = bindings.get('pt_normal_support_exclusions')
    policy = (envelope or {}).get('policy', {}).get('target_scope')
    workspace=bindings.get('workspace_policy')
    if workspace:
        from kitchen_safety.task_workspace import target_selection
        policy=target_selection(workspace,targets)
        envelope=dict(source_file_sha256=workspace['source_file_sha256'])
    if policy is None:
        if (envelope or {}).get('policy',{}).get('target_scope_request') is not None:
            raise ValueError('PT target selection must first bind this episode measured geometry')
        return None, None
    if (policy.get('version') != VERSION or policy.get('task_id') != contract['task_id']
            or policy.get('task_sha256') != digest(contract)
            or policy.get('scope') != 'all_PT_features_and_rules'
            or policy.get('active_phases') != ['all'] or not policy.get('authorization')):
        raise ValueError('PT target subset requires exact task identity and full authorized scope')
    env_root = inventory['env_root'].rstrip('/')
    if bindings.get('variant_risk_admission'):
        from kitchen_safety.variant_risk_admission import add_targets
        policy=add_targets(policy,bindings,targets,env_root)
    declarations = policy.get('included_targets')
    if not isinstance(declarations, list) or not declarations:
        raise ValueError('PT target subset requires explicit retained targets')

    def absolute(relative):
        if (not isinstance(relative, str) or not relative or relative.startswith('/')
                or any(p in ('', '.', '..') for p in relative.split('/'))):
            raise ValueError('PT subset paths must be exact environment-relative paths')
        return env_root + '/' + relative

    expected_targets = {};risk_paths={}
    for row in declarations:
        target = absolute(row['target'])
        paths = row.get('colliders')
        if (target in expected_targets or not isinstance(paths, list) or not paths
                or len(set(paths)) != len(paths)):
            raise ValueError('Duplicated or empty PT subset binding')
        expected_targets[target] = {absolute(path) for path in paths}
        selected=row.get('risk_colliders',paths)
        if not selected or len(set(selected))!=len(selected):raise ValueError('Invalid workspace collision subset')
        risk_paths[target]={absolute(path) for path in selected}
        if not risk_paths[target]<=expected_targets[target]:raise ValueError('Workspace collider subset escapes target')
    for target, expected in expected_targets.items():
        actual = {e['path'] for e in targets.get(target, [])}
        if actual != expected:
            raise ValueError('Retained PT body/collider topology changed: ' + target)
    # Excluded task payloads keep shared raw grasp/support evidence and geometry,
    # but have no PT distance queries. Mesh separation then remains unmeasured.
    excluded_payloads = []
    for aid in contract['roles']['payload_ids']:
        root = bindings['asset_roots'][aid]
        payload_paths = {e['path'] for e in inventory['colliders']
                         if e['path'] == root or e['path'].startswith(root + '/')}
        selected = set().union(*risk_paths.values())
        if not payload_paths:
            raise ValueError('Task payload collision geometry is missing')
        if not payload_paths <= selected:
            excluded_payloads.append(aid)
    excluded = sorted(set(targets) - set(expected_targets))
    report = dict(version=VERSION, policy_sha256=digest(policy),
        source_file_sha256=envelope['source_file_sha256'],
        authorization=policy['authorization'], scope=policy['scope'],
        included_target_paths=sorted(expected_targets), excluded_target_paths=excluded,
        included_collider_paths_by_target={t:sorted(p) for t,p in risk_paths.items()},
        excluded_payload_ids=excluded_payloads,
        excluded_payload_grasp_separation='unknown; no out-of-scope distance queries',
        selection_basis=policy['selection_basis'],
        semantics='User-selected PT scope; excluded targets produce no risk queries, distances, force/impulse features or rules; shared raw evidence retained')
    return set(expected_targets), report


def select_targets_by_bounds(contract, inventory, bindings, request, world_bounds):
    """Resolve a user-selected spatial subset from THIS run's initial geometry.

    Bounds are only an asset-membership filter, never a risk distance or grade.
    The exact retained bodies/colliders are frozen before any GPU query plan.
    """
    from collections import defaultdict
    if (request.get('version') != 'robolaw-initial-support-target-selection-1'
            or request.get('task_id') != contract['task_id']
            or request.get('task_sha256') != digest(contract)
            or request.get('scope') != 'all_PT_features_and_rules'
            or request.get('active_phases') != ['all']
            or not request.get('authorization')):
        raise ValueError('Invalid task-bound support selection request')
    env_root = inventory['env_root'].rstrip('/')
    robot = bindings['robot_root']
    humans = bindings.get('human_roots', [])
    def under(path, root):return path == root or path.startswith(root.rstrip('/')+'/')
    def relative_path(path):
        if not isinstance(path,str) or not path or path.startswith('/') or any(p in ('','.','..') for p in path.split('/')):
            raise ValueError('Invalid support selection path')
        return env_root+'/'+path
    support_root = relative_path(request['support_root'])
    excluded_roots = [relative_path(x) for x in request['excluded_asset_roots']]
    support = [e['path'] for e in inventory['colliders'] if under(e['path'], support_root)]
    if not support or set(world_bounds) != {e['path'] for e in inventory['colliders']}:
        raise ValueError('Support selection requires complete current collider bounds')
    def union(paths):
        return [[min(world_bounds[p][0][i] for p in paths) for i in range(3)],
                [max(world_bounds[p][1][i] for p in paths) for i in range(3)]]
    top = union(support)
    background_roots = [bindings['asset_roots'][aid] for aid,a in contract['asset_declarations'].items()
                        if a['kind']=='background' and aid in bindings['asset_roots']]
    targets = defaultdict(list)
    for e in inventory['colliders']:
        path = e['path']
        if under(path, robot) or any(under(path,h) for h in humans):continue
        key = e.get('actor')
        if key is None:
            background = next((r for r in background_roots if under(path,r)),None)
            if background:key = background+'/'+path[len(background)+1:].split('/')[0]
            else:
                matches = [r for r in bindings['asset_roots'].values() if under(path,r)]
                key = max(matches,key=len) if matches else path
        targets[key].append(path)
    tolerance = request['location_tolerance_m']
    if type(tolerance) not in (int,float) or not 0 <= tolerance <= .01:
        raise ValueError('Invalid spatial membership tolerance')
    included = []
    for target,paths in sorted(targets.items()):
        if any(under(target,r) or any(under(p,r) for p in paths) for r in excluded_roots):continue
        lo,hi = union(paths)
        if (all(top[0][i]-tolerance <= lo[i] <= hi[i] <= top[1][i]+tolerance for i in (0,1))
                and lo[2] >= top[1][2]-tolerance and hi[2] > top[1][2]+tolerance):
            included.append(dict(target=target[len(env_root)+1:],
                colliders=sorted(p[len(env_root)+1:] for p in paths), initial_bounds_world_m=[lo,hi]))
    if not included:raise ValueError('No island assets resolved; review the current scene')
    return dict(version=VERSION,task_id=contract['task_id'],task_sha256=digest(contract),
        scope=request['scope'],active_phases=request['active_phases'],authorization=request['authorization'],
        included_targets=included,selection_basis=dict(kind='per_episode_measured_initial_support_bounds',
            request_sha256=digest(request),inventory_sha256=digest(inventory),
            support_root=support_root,support_bounds_world_m=top,
            excluded_asset_roots=excluded_roots,location_tolerance_m=tolerance,
            semantics='Explicit user spatial evaluation scope; not proof of collision impossibility'))


def materialize_target_selection(contract, inventory, bindings, initial_state, mesh_directory, envelope):
    """CPU-only once per capture; freezes actual geometry, never risk distances."""
    from copy import deepcopy
    from pathlib import Path
    import numpy as np
    from kitchen_safety.hs_geometry import transform
    request = envelope['policy'].get('target_scope_request')
    if request is None:return envelope
    bounds = {}
    for e in inventory['colliders']:
        with np.load(Path(mesh_directory)/(e['fingerprint']+'.npz'),allow_pickle=False) as mesh:
            vertices=mesh['vertices']
            faces=mesh['faces']
        import hashlib
        if hashlib.sha256(np.ascontiguousarray(vertices,dtype=np.float64).tobytes()+np.ascontiguousarray(faces,dtype=np.int32).tobytes()).hexdigest()!=e['fingerprint']:
            raise ValueError('Selection geometry hash changed: '+e['path'])
        if e['dynamic']:
            pose=transform(initial_state['actor_states_world'][e['actor']])
            vertices=vertices@pose[:3,:3].T+pose[:3,3]
        if vertices.ndim!=2 or vertices.shape[1]!=3 or not len(vertices) or not np.isfinite(vertices).all():
            raise ValueError('Invalid current mesh membership geometry')
        bounds[e['path']]=[vertices.min(0).tolist(),vertices.max(0).tolist()]
    result=deepcopy(envelope)
    result['policy']['target_scope']=select_targets_by_bounds(contract,inventory,bindings,request,bounds)
    return result
