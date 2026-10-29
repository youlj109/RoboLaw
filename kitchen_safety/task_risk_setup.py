"""CPU-only preparation shared by native and ylj scene capture entry points."""
import json
from pathlib import Path
from kitchen_safety.risk_observer import write_json

ROOT = Path(__file__).resolve().parents[1]


def prepare(env, contract, metadata, output, obstacle=None, reviewed_lock=None, variant_admission=None):
    from kitchen_safety.task_environment import resolve_loaded_roots
    from kitchen_safety.task_mesh import freeze_stage
    from kitchen_safety.articulation_capture import SceneCapture
    from kitchen_safety.task_binding import (derive_bindings, robot_self_pairs, bind_droid_model,
        bind_operation_contacts, bind_human_surface_omissions)
    from kitchen_safety.task_scope import bind_scope
    from kitchen_safety.task_contact_policy import load_policy
    from kitchen_safety.task_workspace import load_policy as load_workspace, materialize
    from kitchen_safety.task_target_policy import materialize_target_selection
    output = Path(output); stage = env.unwrapped.sim.stage; scene = env.unwrapped.scene
    roots = resolve_loaded_roots(stage, metadata, '/World/envs/env_0')
    inventory = freeze_stage(stage, '/World/envs/env_0', output/'meshes')
    write_json(output/'mesh_inventory.json', inventory)
    telemetry = SceneCapture(scene['robot']._physics_sim_view, inventory)
    measured = telemetry.capture()
    missing = sorted({e['actor'] for e in inventory['colliders'] if e['dynamic']}
                     - set(measured['actor_states_world']))
    measured.update(missing_actor_paths=missing, complete=measured['complete'] and not missing,
                    source_physics_step=-1, source_time_s=0., scope='initial state only; not a trajectory')
    write_json(output/'initial_measured_state.json', measured)
    config = json.loads((ROOT/'configs/safety_observer_arena.json').read_text())
    robot = scene['robot']; body_paths = [str(p) for p in robot.root_view.link_paths[0]]
    grip_names = set(config['pt_v4_geometry']['allowed_grip_body_names'])
    grips = [p for p in body_paths if p.rsplit('/', 1)[-1] in grip_names]
    if {p.rsplit('/', 1)[-1] for p in grips} != grip_names:
        raise ValueError('Current DROID grip profile does not bind all declared bodies')
    human_roots = [roots[obstacle['asset_id']]] if obstacle and obstacle['category'] == 'HS' else []
    robot_root = str(robot.root_view.prim_paths[0])
    bindings = derive_bindings(contract, inventory, roots, robot_root, human_roots=human_roots,
        expected_humans=bool(human_roots), fixed_hs_authorization=True, grip_actor_paths=grips,
        backend_fixed_actor_paths=[p for m in telemetry.articulation.metadata for p in m['fixed_actor_paths']])
    bind_human_surface_omissions(inventory, bindings, obstacle)
    admitted=None
    if variant_admission:
        if reviewed_lock is None:raise ValueError("Variant admission requires the native reviewed ceiling")
        from kitchen_safety.variant_risk_admission import bind
        admitted=bind(inventory,bindings,variant_admission,reviewed_lock)
    bindings['robot_model'] = bind_droid_model(body_paths, robot.joint_names, config, ROOT)
    catalog = json.loads((ROOT/'configs/risk_task_semantics.json').read_text())
    if catalog['schema_version'] != 'robolaw-task-contact-semantics-1':
        raise ValueError('Unsupported task contact semantics catalog')
    bind_operation_contacts(contract, inventory, bindings, catalog['tasks'].get(contract['task_id']))
    support = load_policy(ROOT, contract); workspace = load_workspace(ROOT, contract)
    if support is not None:
        bindings['pt_normal_support_exclusions'] = support if workspace else materialize_target_selection(
            contract, inventory, bindings, measured, output/'meshes', support)
    if workspace:
        bindings['workspace_policy'] = materialize(contract, inventory, bindings, measured,
            output/'meshes', stage, workspace, json.loads((ROOT/config['profile_file']).read_text()))
    if admitted and workspace:
        from kitchen_safety.variant_risk_admission import workspace as admit_workspace
        bindings["workspace_policy"]=admit_workspace(bindings["workspace_policy"],admitted)
    scope = bind_scope(contract, inventory, bindings)
    if reviewed_lock:
        from kitchen_safety.reviewed_scope_lock import apply_lock
        if admitted:
            from kitchen_safety.variant_risk_admission import extend_lock
            reviewed_lock=extend_lock(reviewed_lock,admitted,inventory["env_root"])
        scope = apply_lock(scope, inventory, reviewed_lock)
    if admitted:
        from kitchen_safety.variant_risk_admission import validate_scope
        validate_scope(scope,admitted)
    rs = robot_self_pairs(inventory, robot_root,
                         explicit_structural_pairs=bindings['robot_model']['structural_pairs'])
    write_json(output/'relation_scope.json', scope); write_json(output/'rs_pair_scope.json', rs)
    return dict(inventory=inventory, measured=measured, missing=missing, bindings=bindings,
                roots=roots, scope=scope, rs=rs, config=config)
