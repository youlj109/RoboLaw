"""CPU reconstruction of schema-2 full-physics neutral evidence.

The returned compatibility view labels physics samples and camera frames
separately. It never fabricates an action-rate or missing startup record.
"""
import copy
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from scripts.extensions.replicator.neutral_task_probe import audit, environment_route, initial_pose_audit


def require(value, message):
    if not value:
        raise ValueError(message)


def source_topology(info, data):
    from pxr import Usd, UsdGeom, UsdPhysics
    from kitchen_safety.extensions.replicator.startup_recorder import sha
    from scripts.extensions.replicator.render_comparison import resolve_recorded_path
    path = resolve_recorded_path(info['source_kitchen']['path'])
    require(sha(path) == info['source_kitchen']['sha256'] == info['background_physics']['source_sha256'],
            'Neutral original kitchen source hash differs')
    stage = Usd.Stage.Open(str(path))
    root = str(stage.GetDefaultPrim().GetPath())
    live = '/World/envs/env_0/kitchen'
    def runtime(path):
        require(str(path).startswith(root+'/'), 'Source endpoint is outside original kitchen')
        return live+str(path)[len(root):]
    endpoints, anchors = set(), set()
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.Joint):
            continue
        joint = UsdPhysics.Joint(prim)
        for role, rel in [('body0', joint.GetBody0Rel()), ('body1', joint.GetBody1Rel())]:
            for target in rel.GetTargets():
                body = stage.GetPrimAtPath(target)
                require(bool(body), 'Missing original joint endpoint')
                while body and not body.IsPseudoRoot() and not body.HasAPI(UsdPhysics.RigidBodyAPI):
                    body = body.GetParent()
                if body and not body.IsPseudoRoot():
                    endpoints.add((runtime(prim.GetPath()), role, runtime(body.GetPath())))
                else:
                    anchors.add(runtime(target))
    profile = info['background_physics']
    endpoints.add((runtime(profile['mount_joint']), 'body1', runtime(profile['mounted_body'])))
    topology = info['background_topology']
    actual = {(row['joint'], row['role'], row['rigid_body']) for row in topology['joint_endpoints']}
    require(actual == endpoints and set(topology['authored_joint_connected_paths']) == {row[2] for row in endpoints},
            'Neutral source joint endpoints omitted or changed')
    require(set(info['static_anchor_paths']) == set(topology['static_anchor_paths']) == anchors and
            not topology['animated_static_anchors'] and not topology['unobserved_joint_connected_bodies'],
            'Neutral source static/dynamic topology incomplete')
    aliases = topology['articulation_root_aliases']
    paths = info['background_paths']
    cache = UsdGeom.XformCache()
    for key, raw_key in [('static_anchors', 'static_anchor_transforms'), ('task_references', 'task_reference_transforms')]:
        names = info['static_anchor_paths'] if key == 'static_anchors' else info['task_reference_names']
        for i, name in enumerate(names):
            row = info[key][name]
            live_path = name if key == 'static_anchors' else row['path']
            require(live_path.startswith(live+'/') and row['source_path'] == root+live_path[len(live):],
                    'Neutral static reference source mapping differs')
            prim = stage.GetPrimAtPath(row['source_path'])
            require(bool(prim), 'Missing original static reference')
            matrix = np.asarray(cache.GetLocalToWorldTransform(prim)).T
            require(np.array_equal(matrix, np.asarray(row['source_world_transform'])) and
                    np.max(np.abs(data[raw_key][0, i]-matrix)) <= 1e-6,
                    'Neutral static reference differs from actual original USD')
            while prim and not prim.IsPseudoRoot():
                transform = UsdGeom.Xformable(prim)
                require(not prim.HasAPI(UsdPhysics.RigidBodyAPI) and
                        not (transform and transform.TransformMightBeTimeVarying()),
                        'Neutral claimed static reference has a dynamic or animated original ancestor')
                prim = prim.GetParent()
    for authored, alias in aliases.items():
        native = alias['native_path']
        require(native in paths and alias['unique_joint_graph_root'] == [authored], 'Invalid neutral native alias identity')
        edges = [row for row in endpoints if row[0].startswith(native+'/')]
        graph_roots = {row[2] for row in edges if row[1] == 'body0'} - {row[2] for row in edges if row[1] == 'body1'}
        require(graph_roots == {authored}, 'Native alias is not the unique authored joint graph root')
        body = stage.GetPrimAtPath(root+authored[len(live):])
        mass = UsdPhysics.MassAPI(body).GetMassAttr().Get()
        require(mass is not None and abs(alias['source_mass_kg']-mass) <= 1e-6 and
                abs(alias['native_mass_kg']-mass) <= 1e-4*max(1., mass), 'Neutral source/native alias mass differs')
        matrix = np.asarray(cache.GetLocalToWorldTransform(body)).T
        rotation = matrix[:3, :3]/np.linalg.norm(matrix[:3, :3], axis=0)
        measured = data['background'][0, paths.index(native)]
        error = float(np.linalg.norm(matrix[:3, 3]-measured[:3]))
        angle = float(np.rad2deg((Rotation.from_matrix(rotation)*Rotation.from_quat(measured[3:7]).inv()).magnitude()))
        require(error <= .002 and angle <= 1., 'Neutral initial native alias pose differs from source')
    native = {aliases.get(row[2], {}).get('native_path', row[2]) for row in endpoints}
    require(native == set(topology['joint_connected_native_paths']) and native <= set(paths),
            'Neutral native endpoint map incomplete')
    return dict(joint_connected_bodies=sorted({row[2] for row in endpoints}),
        articulation_root_aliases=aliases,
        joint_targets=[dict(joint=joint, role=role, rigid_body=body, kind='dynamic_rigid_body',
            native_rigid_body=aliases.get(body, {}).get('native_path', body)) for joint, role, body in sorted(endpoints)])


def validate_schema2(info, data):
    """Recompute numeric gates; source/topology validation follows separately."""
    require(info['schema_version'] == 2, 'Unsupported full-physics neutral schema')
    require(info['environment_factory'] == environment_route(info['task_id']), 'Wrong source environment factory')
    require(info['passed'] is True, 'Neutral probe declared a failure; retain it')
    require(bool(info['checks']) and all(value is True for value in info['checks'].values()),
            'Neutral claimed checks contain a failed or malformed gate')
    actual_initial = initial_pose_audit(data['robot_body_pose_w'][0], info['robot_body_names'],
        data['task_object_states'][0], info['task_object_names'], info['layout'])
    require(actual_initial == info['recorded_initial_source_poses'] and actual_initial['passed'],
            'Neutral recorded initial source pose audit differs from raw evidence')
    for key, report_key in [('static_anchor_transforms', 'static_anchors'), ('task_reference_transforms', 'task_references')]:
        names = info['static_anchor_paths'] if report_key == 'static_anchors' else info['task_reference_names']
        for i, name in enumerate(names):
            row = info[report_key][name]
            require(np.max(np.abs(data[key][0, i]-np.asarray(row['source_world_transform']))) <= 1e-6,
                    'Neutral raw initial static/source reference differs')
    recomputed = audit(data, info)
    require(recomputed['passed'], 'Recomputed neutral physics gates failed')
    for key, value in recomputed.items():
        if key == 'checks':
            require(all(info['checks'].get(name) is passed for name, passed in value.items()), 'Neutral claimed checks differ')
        else:
            require(info.get(key) == value, 'Neutral claimed raw statistics differ: ' + key)
    return recomputed


def load_neutral(directory, require_full_physics=False):
    """Return (normalized metadata, raw data), preserving legacy review support."""
    from kitchen_safety.extensions.replicator.startup_recorder import sha
    directory = Path(directory)
    info = json.loads((directory/'neutral_probe.json').read_text())
    with np.load(directory/'neutral_states.npz', allow_pickle=False) as stored:
        data = {name: stored[name] for name in stored.files}
    if info.get('schema_version') != 2:
        require(not require_full_physics, 'Final delivery requires new schema-2 complete post-reset neutral physics')
        result = copy.deepcopy(info)
        result['duration_seconds'] = len(data['q'])*float(info['dt'])
        result['raw_schema'] = 'legacy_action_rate'
        return result, data
    require(sha(directory/'neutral_states.npz') == info['input_sha256']['neutral_states.npz'], 'Neutral raw NPZ hash changed')
    from scripts.extensions.replicator.audit_task_source_relation import check_layout, source
    relation = check_layout(info['layout'])
    require(relation == info['source_relation_audit'] and relation['initial_source_relations_passed'],
            'Neutral full original source relations differ')
    _, original = source(info['task_id'])
    require(info['expected_source_relations'] == len(original['relations']) and
            set(info['task_object_names']) == {row['id'] for row in original['objects']}, 'Neutral source identities differ')
    references = {row['id']: '/World/envs/env_0/kitchen/'+row['prim_path'] for row in original['object_references']}
    require(dict(zip(info['task_reference_names'], info['task_reference_paths'])) == references,
            'Neutral source static references differ')
    recomputed = validate_schema2(info, data)
    mapped = source_topology(info, data)
    result = copy.deepcopy(info)
    result.update(mapped)
    # These keys are compatibility metadata only: raw physics samples remain
    # untouched; video_frames/dt retain original action-rate camera semantics.
    result.update(raw_schema='full_post_reset_physics', video_frames=info['frames'], video_dt=info['dt'],
        frames=len(data['physics_step']), dt=info['physics_dt'], duration_seconds=recomputed['observed_duration_seconds'],
        background_tolerances=dict(translation_m=.002, rotation_deg=1.), independently_recomputed_physics=True)
    return result, data
