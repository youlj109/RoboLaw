"""Task roles and source provenance, independent of simulator or asset names.

Compilation does not resolve collider paths. Only a measured stage inventory
can bind roles, joint links, manipulation surfaces and the final query scope.
"""
from copy import deepcopy
import hashlib
import json


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, ensure_ascii=False,
        separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def compile_task(document, provenance):
    d = deepcopy(document)
    assets = {x['id']: dict(x, kind='spawned_object') for x in d.get('objects', [])}
    for x in d.get('object_references', []):
        if x['id'] in assets:
            raise ValueError('Duplicate task asset ID: '+x['id'])
        assets[x['id']] = dict(x, kind='reference')
    background = d['background']
    assets[background['id']] = dict(background, kind='background')
    operations, payloads, goals, actuated, exemptions = [], set(), set(), set(), []
    support_edges = {}
    for rel in d.get('relations', []):
        if rel['kind'] == 'on':
            support_edges.setdefault(rel['subject'], set()).add(rel['reference'])

    def supports(identifier):
        seen, stack = set(), list(support_edges.get(identifier, []))
        while stack:
            item = stack.pop()
            if item == identifier:
                raise ValueError('Cyclic support graph')
            if item in seen:
                continue
            seen.add(item)
            stack.extend(support_edges.get(item, []))
        return seen

    for index, sub in enumerate(d['task']['subtasks']):
        kind, params = sub['kind'], sub['params']
        operation = dict(operation_id='op_%02d' % index, kind=kind, parameters=deepcopy(params))
        if kind == 'PickAndPlaceTask':
            source, goal = params['pick_up_object'], params['destination_location']
            payloads.add(source); goals.add(goal)
            if source not in assets or goal not in assets:
                raise ValueError('Task references an undeclared payload or destination')
            allowed = supports(source) | supports(goal) | {goal}
            operation.update(payload_id=source, destination_id=goal,
                expected_support_ids=sorted(allowed - {goal}))
            for target in sorted(allowed):
                exemptions.append(dict(source_id=source, target_id=target,
                    operation_id=operation['operation_id'], category='PT',
                    query_distance=False, persist_distance=False, grade_distance=False,
                    grade_force=True, grade_impulse=True, active_phases=['all'],
                    reason='normal_task_destination' if target == goal else 'declared_task_support',
                    binding_requirement='exact destination/support collision surfaces; no entire cabinet exemption'))
        else:
            keys = {'OpenDoorTask': ('openable_object','openable_joint_name'),
                'PressButtonTask': ('pressable_object','pressable_joint_name'),
                'TurnKnobTask': ('turnable_object','turnable_joint_name')}
            if kind not in keys:
                raise ValueError('Unsupported task kind requires explicit adaptation: '+kind)
            key, joint_key = keys[kind]
            target = params[key]
            if target not in assets:
                raise ValueError('Undeclared articulated asset: '+target)
            joint = assets[target].get('params', {}).get(joint_key)
            if not joint:
                raise ValueError('Missing commanded articulation joint')
            actuated.add(target)
            operation.update(articulation_id=target, joint_name=joint,
                moving_links='resolve from actual joint graph, including fixed descendants',
                manipulation_surfaces='runtime binding required; never exempt entire articulation',
                stop_model='unbound; no inherited robot stopping guarantee')
        operations.append(operation)
    families = {o['kind'] == 'PickAndPlaceTask' for o in operations}
    family = 'pick_and_place' if families == {True} else 'articulated_object' if families == {False} else 'mixed'
    return dict(schema_version='robolaw-task-contract-1',
        task_id=d['env_name'][len('kitchen_bench_'):] if d['env_name'].startswith('kitchen_bench_') else d['env_name'],
        environment_id=background['registry_name'], family=family,
        description=d['task']['description'], composition=d['task']['composition'],
        provenance=deepcopy(provenance), arena_environment_spec=d,
        asset_declarations=assets, operations=operations,
        roles=dict(payload_ids=sorted(payloads), destination_ids=sorted(goals),
            articulated_target_ids=sorted(actuated), robot=d['embodiment'],
            human_ids='resolved from each loaded variant; closed-world inventory required'),
        normal_distance_relations=exemptions,
        hs=dict(sources=['moving_robot_links','all_payloads','manipulated_articulation_links'],
            fixed_base_policy='derive from fixed-world joint graph; scope authorization required',
            clothing_policy='no inherited omission; protect all enabled human surfaces unless explicitly bound',
            stopping_model='per source; missing model produces unknown',
            dynamic_rule_version='arena-hs-dynamic-ssm-1'),
        pt=dict(candidate_scope='all potential collision/approach pairs across full task and randomization envelope',
            pruning='offline conservative proof including protective/warning bound; otherwise retain',
            robot_to_payload='gripper part/phase only; evidence-only when required; other arm parts remain',
            robot_to_destination='retain unless separately authorized for this task',
            distance_rule='pt-hs-inspired-v4', inherited_distance_omissions=[]),
        rs=dict(robot_joint_scope='robot only; no external hinge/slider conflation',
            self_collision='independent full robot including fixed base; structural graph exclusions',
            external_obstacles='contact evidence preserved; torque/limit rules unchanged',
            fault_coverage='explicit; no exception is not proof of healthy state'),
        raw_requirements=dict(physics_hz=120, action_hz=30, label_hz=30,
            every_physics_step=['robot_actual_q_dq_torque_limits', 'all_dynamic_actor_pose_COM_motion',
                'external_articulation_link_pose_COM_motion_joint_state', 'contact_reports_and_coverage',
                'actual_command_submissions', 'fault_events_and_authority'],
            frozen=['full_collision_meshes', 'joint_graph_and_units', 'role_and_surface_bindings',
                'rules_and_hashes', 'gdist_build_and_tuning', 'randomization_samples', 'render_reconstruction']),
        gates=dict(specification=True, loaded_stage=False, raw_trajectory=False,
            offline_HS_PT_RS=False, aligned_video=False),
        acceptance='pending; task success and evidence completeness are separate')
