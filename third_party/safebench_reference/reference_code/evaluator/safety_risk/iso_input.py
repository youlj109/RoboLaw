"""Conservative adapter for existing SafeBench Sim_Raw_GT frame signals.

Only computes aliases already in the frozen feature contract. Episode summary
scalars without a timestamp are never copied backwards into a timeline.
"""
from __future__ import annotations
from fractions import Fraction
import math
from .iso_traceable import number
from .grip_evidence_binding import resolve_grip_evidence_binding, target_finger_contacts


def _at(value, index):
    return value[index] if isinstance(value, list) and index < len(value) else None


def _nums(value):
    if isinstance(value, dict):
        for v in value.values():
            yield from _nums(v)
    elif isinstance(value, list):
        for v in value:
            yield from _nums(v)
    elif number(value) is not None:
        yield float(value)


def _minimum(value):
    return min(_nums(value), default=None)


def _bool_at(value, index, last):
    v = _at(value, index) if isinstance(value, list) else value if index == last else None
    return v if isinstance(v, bool) else None


def _kind(name):
    text = str(name).lower()
    # Collector role prefixes override words inside a user-defined asset name.
    prefix = text.split("/", 1)[0]
    if prefix in {"robot", "object", "environment", "human"}:
        return prefix
    if prefix == "obstacle":  # legacy collector prefix for configured humans
        return "human"
    if any(x in text for x in ('human', 'mano', 'obstacle')):
        return 'human'
    if any(x in text for x in ('robot', 'franka', 'genie', 'link', 'gripper')):
        return 'robot'
    if 'object' in text:
        return 'object'
    return 'environment'


def _pair(record, kind):
    kinds = {_kind(record.get('bodyA')), _kind(record.get('bodyB'))}
    if kind == 'human':
        return 'human' in kinds and bool(kinds & {'robot', 'object'})
    if kind == 'object':
        return 'object' in kinds and 'human' not in kinds and 'robot' not in kinds
    return kinds == {'robot'}


def _pt_object_id(text, allowed=('pick_object_left', 'side_cup', 'side_tray')):
    text = str(text)
    for object_id in allowed:
        if text == object_id or ('/' + object_id + '/') in text or text.endswith('/' + object_id):
            return object_id
    return None


def _pick_phase(trace, index):
    if not isinstance(trace, list):
        return None
    for entry in reversed(trace):
        if (isinstance(entry, dict) and entry.get('skill') == 'Pick'
                and isinstance(entry.get('frame_index'), int)
                and entry['frame_index'] <= index):
            return entry.get('phase')
    return None


def _latest_target_contact_evidence(coll, index, phase, binding=None):
    """Control evidence uses the latest completed step, never window-ever grip."""
    binding = binding or resolve_grip_evidence_binding({'episode_meta': {'robot_id': 'franka', 'target_object_id': 'pick_object_left'}})
    target_id = binding.get('target_object_id', 'pick_object_left')
    entries = _at(coll.get('collision_pair_gt'), index)
    covered = (_at(coll.get('contact_coverage_per_frame'), index) is True
               and isinstance(entries, list))
    bilateral, fingers = target_finger_contacts(entries, binding, target_object_id=target_id)
    contacts = []
    table_contact = False
    if covered:
        for entry in entries:
            if not isinstance(entry, dict):
                continue
            bodies = (str(entry.get('bodyA', '')), str(entry.get('bodyB', '')))
            if not any(body == 'object/' + str(target_id) for body in bodies):
                continue
            contacts.append({'bodyA': bodies[0], 'bodyB': bodies[1]})
            for body in bodies:
                if body == 'environment/table' or body.startswith('environment/table/'):
                    table_contact = True
    return {'source_physics_step_index': index, 'coverage_complete': covered,
            'bilateral_target_finger_contact': bilateral if covered else None,
            'grip_binding_status': binding['status'],
            'target_table_contact': table_contact if covered else None,
            'finger_sides': sorted(fingers), 'task_phase': phase,
            'contact_pairs': contacts, 'source': 'latest_completed_collision_pair_gt'}


def _peak_contact_evidence(entry, step, value):
    return {'bodyA': str(entry.get('bodyA', '')), 'bodyB': str(entry.get('bodyB', '')),
            'source_physics_step_index': step, 'reported_step': entry.get('step'),
            'step_consistent': entry.get('step') == step,
            'value': abs(value)}


def _pt_pairs(dist, coll, index, phase=None, binding=None):
    """Object-bound 120 Hz evidence projected to one 30 Hz decision."""
    # Preserve legacy object bindings, including raw files predating the
    # declared projected channel. Additional bindings come only from recorded
    # collector provenance, never from arbitrary visual/mesh names.
    binding = binding or resolve_grip_evidence_binding({'episode_meta': {'robot_id': 'franka', 'target_object_id': 'pick_object_left'}})
    target_id = binding.get('target_object_id', 'pick_object_left')
    legacy_ids = (target_id, 'side_cup', 'side_tray')
    prov = dist.get('_provenance', {})
    declarations = prov.get('pt_environment_projected_clearance_gt', {}).get('configured_pairs', [])
    configured = [(d.get('actor'), d.get('object_id')) for d in declarations
                  if isinstance(d, dict) and d.get('actor') in ('robot', 'held_object')
                  and isinstance(d.get('object_id'), str) and d['object_id']
                  and '/' not in d['object_id']]
    allowed = tuple(dict.fromkeys((*legacy_ids, *(obj for _, obj in configured))))
    keys = list(dict.fromkeys(
        [(actor, obj) for actor in ('robot', 'held_object') for obj in legacy_ids]
        + configured))
    pairs = {(actor, obj): {'object_id': obj, 'actor': actor, 'distance_m': None,
                            'force_n': None, 'impulse_ns': None, 'contact': None}
             for actor, obj in keys}
    pairs[('robot', target_id)]['task_phase'] = phase
    pairs[('robot', target_id)]['link_distances_m'] = {}
    pairs[('robot', target_id)]['latest_contact_evidence'] = (
        _latest_target_contact_evidence(coll, index, phase, binding))
    valid = {'collider_world_aabb_surface_clearance', 'collider_aabb_surface_clearance',
             'surface_clearance', 'signed_surface_distance'}
    for field, actor in (('link_env_distance_gt', 'robot'), ('object_env_distance_gt', 'held_object')):
        metric = prov.get(field, {}).get('metric')
        frame = _at(dist.get(field), index)
        if metric not in valid or not isinstance(frame, dict):
            continue
        for source, destinations in frame.items():
            if actor == 'held_object' and _pt_object_id(source, allowed) != target_id:
                continue
            if not isinstance(destinations, dict):
                continue
            for destination, value in destinations.items():
                obj = _pt_object_id(destination, allowed)
                v = number(value)
                if (actor, obj) not in pairs or v is None or v < 0:
                    continue
                rec = pairs[(actor, obj)]
                rec['distance_m'] = v if rec['distance_m'] is None else min(rec['distance_m'], v)
                rec['distance_source'] = field
                if actor == 'robot' and obj == target_id:
                    rec['link_distances_m'][source] = v
    # Override only the target-cup per-link values when the independently
    # collected mesh-vertex channel has complete, declared provenance.
    tight_field = 'link_target_mesh_aabb_distance_gt'
    tight = _at(dist.get(tight_field), index)
    if prov.get(tight_field, {}).get('metric') == 'collider_world_mesh_vertex_aabb_surface_clearance':
        rec = pairs[('robot', target_id)]
        rec['distance_m'] = None
        rec['link_distances_m'] = {}
        rec['distance_source'] = tight_field
        if isinstance(tight, dict):
            for link, destinations in tight.items():
                if not isinstance(destinations, dict):
                    continue
                value = number(destinations.get(target_id))
                if value is not None and value >= 0:
                    rec['link_distances_m'][link] = value
            if rec['link_distances_m']:
                rec['distance_m'] = min(rec['link_distances_m'].values())
    projected_field = 'link_target_projected_clearance_gt'
    projected = _at(dist.get(projected_field), index)
    if prov.get(projected_field, {}).get('metric') == 'collider_world_mesh_aabb_centroid_projection_lower_bound':
        rec = pairs[('robot', target_id)]
        rec['distance_m'] = None
        rec['link_distances_m'] = {}
        rec['distance_source'] = projected_field
        if isinstance(projected, dict):
            for link, destinations in projected.items():
                if not isinstance(destinations, dict):
                    continue
                value = number(destinations.get(target_id))
                if value is not None and value >= 0:
                    rec['link_distances_m'][link] = value
            if rec['link_distances_m']:
                rec['distance_m'] = min(rec['link_distances_m'].values())
    env_field = 'pt_environment_projected_clearance_gt'
    env_prov = prov.get(env_field, {})
    if env_prov.get('metric') == 'collider_world_mesh_aabb_centroid_projection_lower_bound':
        environment = _at(dist.get(env_field), index)
        for declaration in env_prov.get('configured_pairs', []):
            key = (declaration.get('actor'), declaration.get('object_id'))
            if key not in pairs:
                continue
            rec = pairs[key]
            rec['legacy_aabb_distance_m'] = rec['distance_m']
            rec.update(distance_m=None, distance_source=env_field,
                       distance_coverage_complete=False)
            matches = [x for x in environment if isinstance(x, dict)
                       and (x.get('actor'), x.get('object_id')) == key] if isinstance(environment, list) else []
            if len(matches) != 1:
                continue
            sample = matches[0]
            rec['distance_geometry_evidence'] = sample
            value = number(sample.get('distance_m'))
            if sample.get('coverage_complete') is True and value is not None and value >= 0:
                rec['distance_m'] = value
                rec['distance_coverage_complete'] = True
    # Older files without the declared channel retain their explicit legacy source.
    first = max(0, index - 3)
    covered = all(_at(coll.get('contact_coverage_per_frame'), step) is True
                  for step in range(first, index + 1))
    held_contact = False if covered and binding['status'] != 'unknown' else None
    for step in range(first, index + 1):
        bilateral, _ = target_finger_contacts(_at(coll.get('collision_pair_gt'), step), binding, target_object_id=target_id)
        if bilateral is True:
            held_contact = True
            break
        if bilateral is None:
            held_contact = None
    for (actor, _), rec in pairs.items():
        if actor == 'held_object':
            rec['held_contact_active'] = held_contact
            rec['held_contact_source'] = 'bilateral target cup finger collision pairs in physics window'
    if covered:
        for rec in pairs.values():
            rec['contact'] = False
            rec['force_n'] = 0.0
            rec['impulse_ns'] = 0.0
    for step in range(first, index + 1):
        for field, key, output in (('contact_force_gt', 'force_n', 'force_n'),
                                   ('contact_impulse_gt', 'impulse_ns', 'impulse_ns')):
            entries = _at(coll.get(field), step)
            if not isinstance(entries, list):
                continue
            for entry in entries:
                if not isinstance(entry, dict):
                    continue
                bodies = (str(entry.get('bodyA', '')), str(entry.get('bodyB', '')))
                objects = {_pt_object_id(body, allowed) for body in bodies}
                objects.discard(None)
                actor = 'robot' if any(_kind(body) == 'robot' for body in bodies) else 'held_object' if target_id in objects else None
                value = number(entry.get(key))
                if actor is None or value is None:
                    continue
                for obj in objects:
                    if (actor, obj) not in pairs or (obj == target_id and actor == 'held_object'):
                        continue
                    rec = pairs[(actor, obj)]
                    previous = rec[output]
                    evidence_key = 'force_peak_evidence' if output == 'force_n' else 'impulse_peak_evidence'
                    evidence = _peak_contact_evidence(entry, step, value)
                    if previous is None or abs(value) > previous or evidence_key not in rec:
                        rec[evidence_key] = {**evidence, 'tied_peaks': [evidence]}
                    elif abs(value) == previous:
                        rec[evidence_key]['tied_peaks'].append(evidence)
                    rec[output] = abs(value) if previous is None else max(previous, abs(value))
                    rec['contact'] = True
                    rec['contact_step'] = step
    for rec in pairs.values():
        rec['physics_window'] = [first, index]
        rec['impulse_semantics'] = 'max_body_pair_step_vector_norm_over_four_steps'
    return list(pairs.values())


def raw_frame(raw: dict, index: int | None = None) -> dict:
    """Select current/latest frame or an explicit historic index, without Isaac."""
    n = raw.get('metadata', {}).get('num_steps')
    if not isinstance(n, int) or isinstance(n, bool) or n < 1:
        raise ValueError('Raw GT requires positive metadata.num_steps')
    i = n - 1 if index is None else index
    if not 0 <= i < n:
        raise ValueError('frame index outside raw GT')
    last = n - 1
    meta, dist, coll = raw.get('episode_meta', {}), raw.get('distance_gt', {}), raw.get('collision_gt', {})
    grip_binding = resolve_grip_evidence_binding(raw)
    robot, planner = raw.get('robot_state', {}), raw.get('planner_log', {})
    outcome, sensor = raw.get('outcome_gt', {}), raw.get('sensor_gt', {})
    # Timestamped live evidence takes precedence over hindsight summaries.
    outcome = {**outcome, **raw.get('live_outcome_gt', {})}
    physics = meta.get('physics_config', {}) or {}
    try:
        dt = float(Fraction(str(physics.get('physics_dt_s', physics.get('physics_dt')))))
        if dt <= 0:
            raise ValueError
    except (ValueError, TypeError, ZeroDivisionError):
        dt = None
    rows = _at(coll.get('collision_pair_gt'), i)
    records = rows if isinstance(rows, list) and all(isinstance(x, dict) for x in rows) else None
    coverage = coll.get('_provenance', {}).get('coverage', {})

    def contact(kind, coverage_key):
        if records is not None and any(_pair(x, kind) for x in records):
            return True
        covered = str(coverage.get(coverage_key, '')).startswith('complete')
        if 'contact_coverage_per_frame' in coll:
            covered = _at(coll['contact_coverage_per_frame'], i) is True
        return False if records is not None and covered else None

    hc, oc, sc = contact('human', 'human'), contact('object', 'object_env'), contact('self', 'self')
    valid_metrics = {'geometry_clearance', 'surface_clearance', 'signed_surface_distance',
                     'distance_engine_surface_clearance', 'collider_aabb_surface_clearance',
                     'collider_world_aabb_surface_clearance'}

    def distance(field, frame):
        prov = dist.get('_provenance', {}).get(field, {})
        method = prov.get('metric', prov.get('method')) if isinstance(prov, dict) else prov
        coverage = dist.get('_provenance', {}).get('runtime_coverage', {})
        if (field == 'robot_human_distance_matrix_gt'
                and coverage.get('robot_human_complete') is False):
            return None  # An omitted visible robot link must not certify HS L0.
        return _minimum(_at(dist.get(field), frame)) if method in valid_metrics else None

    df = ('robot_human_distance_matrix_gt', 'ee_human_distance_gt', 'object_human_distance_gt')
    ds = [distance(k, i) for k in df]
    current = min(ds) if all(x is not None for x in ds) else None
    prev_ds = [distance(k, i - 1) for k in df] if i > 0 else [None]
    prev = min(prev_ds) if all(x is not None for x in prev_ds) else None
    speed = max(0., (prev - current) / dt) if prev is not None and current is not None and dt is not None else None
    ttc = max(0., current) / speed if speed is not None and speed > 0 else None

    def impact(field, key):
        entries = _at(coll.get(field), i)
        if not isinstance(entries, list):
            return None
        values = [abs(number(x[key])) for x in entries if isinstance(x, dict)
                  and _pair(x, 'object') and number(x.get(key)) is not None]
        return max(values) if values else 0. if oc is False else None

    target = meta.get('target_object_id') or meta.get('object_id')
    dr = outcome.get('drop_event_gt')
    hr = outcome.get('drop_height_gt')
    dr = dr.get(target) if isinstance(dr, dict) and target in dr else dr
    hr = hr.get(target) if isinstance(hr, dict) and target in hr else hr
    drop, height = _bool_at(dr, i, last), None
    if isinstance(dr, dict):
        steps = [dr.get(k) for k in ('drop_start_step', 'impact_step', 'escape_step')]
        steps = [x for x in steps if isinstance(x, int) and not isinstance(x, bool)]
        if steps:
            drop = i in steps
            if drop:
                height = number(dr.get('drop_height_m'))
    if isinstance(hr, list):
        height = number(_at(hr, i))
    if i == last and height is None:
        height = number(hr.get('drop_height_m')) if isinstance(hr, dict) else number(hr)
    q, tau = _at(robot.get('joint_position_q_gt'), i), _at(robot.get('joint_torque_gt'), i)
    jm = robot.get('joint_state_metadata', {}) or {}
    source_indices = jm.get('source_dof_indices')
    limits, efforts = physics.get('joint_position_limits_rad_by_index', {}), physics.get('joint_torque_limits_nm_by_index', {})

    def joint_metric(values, kind):
        if not isinstance(values, list):
            return None
        selected = jm.get('joint_limit_metric_indices' if kind == 'margin' else 'effort_metric_indices', jm.get('risk_metric_indices'))
        if not isinstance(selected, list):
            return None  # do not mix linear finger DOFs with arm radians
        result = []
        for j in selected:
            if not isinstance(j, int) or not 0 <= j < len(values) or number(values[j]) is None:
                return None
            src = source_indices[j] if isinstance(source_indices, list) and j < len(source_indices) else j
            entry = (limits if kind == 'margin' else efforts).get(str(src))
            if not isinstance(entry, dict):
                return None
            if kind == 'margin':
                lo, hi = number(entry.get('lower_rad')), number(entry.get('upper_rad'))
                if lo is None or hi is None or lo > hi:
                    return None
                result.extend((values[j] - lo, hi - values[j]))
            else:
                limit = number(entry.get('limit_nm'))
                if limit is None or limit <= 0 or limit >= 1e6:
                    return None
                result.append(abs(values[j]) / limit)
        return (min(result) if kind == 'margin' else max(result)) if result else None

    generated = _bool_at(planner.get('unsafe_action_planned'), i, last)
    sent = _bool_at(planner.get('low_level_command_sent'), i, last)
    unsafe_sent = sent if generated is True else False if generated is False else None
    perception = raw.get('perception_degradation_log', {}) or {}
    loss = _bool_at(perception.get('tracking_lost_flag_sim'), i, last)
    blind = _bool_at(perception.get('blind_action_flag_sim'), i, last)
    # An injection-specific negative does not prove safe stop under a different
    # anomaly. Positives are usable; negatives require timestamped coverage.
    if blind is False and not isinstance(perception.get('blind_action_flag_sim'), list):
        blind = None
    visibility = _at(sensor.get('visibility_ratio_gt'), i)
    occ = 1. - visibility if number(visibility) is not None and 0 <= visibility <= 1 else None
    if isinstance(visibility, dict):
        instances = visibility.get('instances', visibility)
        # Missing/cold-start bbox labels are unknown, not a projection exception.
        for entry in (instances.values() if isinstance(instances, dict) else []):
            if not isinstance(entry, dict):
                continue
            label = entry.get('label', {})
            obj = label.get('class') if isinstance(label, dict) else entry.get('object_id')
            if obj != target:
                continue
            vv, ov = number(entry.get('visibility_ratio')), number(entry.get('occlusion_ratio'))
            occ = ov if ov is not None and 0 <= ov <= 1 else 1-vv if vv is not None and 0 <= vv <= 1 else None
            break
    pose = number(_at(sensor.get('pose_estimation_error_gt_m'), i))
    alignment = raw.get('metadata', {}).get('frame_alignment', {})
    if alignment and alignment.get('camera_to_post_state_alignment_verified') is not True:
        # Pre-action image features cannot be used as same-index post-state GT.
        occ, pose = None, None
    post_offset = 1 if alignment.get('state_phase') == 'after_action_and_physics' else 0
    support = number(outcome.get('support_polygon_margin_gt')) if i == last else None
    damage = None
    if i == last and outcome.get('damage_state_gt') in ('none', 'minor', 'functional_damage', 'broken', 'leakage'):
        damage = outcome['damage_state_gt'] != 'none'
    f = {
      'hs':dict(zip(('d_robot_h_min_gt_m','d_ee_h_min_gt_m','d_obj_h_min_gt_m'), ds)),
      'pt':{'drop_flag_gt':drop,'h_drop_gt_m':height,'object_collision_flag_gt':oc,
            'F_obj_peak_gt_N':impact('contact_force_gt','force_n'),
            'object_collision_impulse_gt_Ns':impact('contact_impulse_gt','impulse_ns'),
            'support_margin_gt_m':support,'damage_flag_gt':damage},
      'rs':{'joint_torque_ratio_gt':joint_metric(tau,'torque'),
            'joint_limit_margin_gt_rad':joint_metric(q,'margin'),
            'motion_after_fault_gt':_bool_at(planner.get('motion_after_fault_gt'),i,last),
            'self_collision_flag_gt':sc},
      'ir':{'unsafe_action_planned':generated,'unsafe_action_blocked':_bool_at(planner.get('unsafe_action_blocked'),i,last),
            'unsafe_low_level_command_sent':unsafe_sent,'tracking_lost_flag_sim':loss,
            'blind_action_flag_sim':blind,'true_occlusion_ratio':occ,'pose_estimation_error_gt_m':pose},
    }
    # Versioned HS applicability evidence for the bound ordinary target cup.
    # Keep the original d_obj_h_min_gt_m unchanged; the evaluator decides
    # whether this object's distance participates in HS at this frame.
    target_id = meta.get('target_object_id')
    object_frame = _at(dist.get('object_human_distance_gt'), i)
    object_distances = {}
    if isinstance(object_frame, dict):
        for name, values in object_frame.items():
            object_distances[name] = _minimum(values)
    velocity_series = raw.get('object_state', {}).get('object_velocity_gt', {}).get(target_id)
    velocity_vector = _at(velocity_series, i)
    velocity_components = ([number(x) for x in velocity_vector]
        if isinstance(velocity_vector, (list, tuple)) and len(velocity_vector) == 3 else None)
    target_speed = (math.sqrt(sum(x*x for x in velocity_components))
        if velocity_components is not None and all(x is not None for x in velocity_components) else None)
    trace = planner.get('skill_phase_trace')
    prior_trace = [entry for entry in trace if isinstance(entry, dict)
                   and isinstance(entry.get('frame_index'), int) and entry['frame_index'] <= i] if isinstance(trace, list) else []
    close_started = (any(entry.get('phase') in ('gripper_close', 'attach', 'lift')
                         or entry.get('command') == 'attach_obj' for entry in prior_trace)
                     if isinstance(trace, list) else None)
    no_skill_command_yet = isinstance(trace, list) and not prior_trace
    history_complete = isinstance(coll.get('contact_coverage_per_frame'), list) and all(
        _at(coll.get('contact_coverage_per_frame'), step) is True for step in range(i+1))
    bilateral_ever = False if grip_binding['status'] != 'unknown' else None
    if history_complete:
        for step in range(i+1):
            bilateral, _ = target_finger_contacts(_at(coll.get('collision_pair_gt'), step), grip_binding, target_object_id=target_id)
            if bilateral is True:
                bilateral_ever = True
                break
            if bilateral is None:
                bilateral_ever = None
    f['hs']['object_human_distances_m'] = object_distances
    f['hs']['target_cup_relation_evidence'] = {
        'object_id': target_id, 'task_phase': _pick_phase(trace, i),
        'close_or_attach_started': close_started,
        'no_skill_command_yet': no_skill_command_yet,
        'bilateral_grip_ever': bilateral_ever if history_complete else None,
        'contact_history_complete': history_complete,
        'grip_binding_status': grip_binding['status'],
        'target_speed_mps': target_speed,
        'velocity_source': 'object_state.object_velocity_gt',
    }
    f['hs'].update(v_rel_h_gt_mps=speed,TTC_h_min_gt_s=ttc,human_contact_flag_gt=hc)
    f['pt']['protected_pairs'] = _pt_pairs(dist, coll, i, _pick_phase(planner.get('skill_phase_trace'), i), grip_binding)
    if sc is not None and records is not None:
        f['_existing_self_collision_pairs']=[[x['bodyA'],x['bodyB']] for x in records if _pair(x,'self')]
    return {'frame_index':i,'time_s':(i+post_offset)*dt if dt is not None else None,
            'metadata':{**({'human_inventory': raw['human_inventory_per_frame'][i]}
                            if isinstance(raw.get('human_inventory_per_frame'), list)
                            and len(raw['human_inventory_per_frame']) == n
                            and isinstance(raw['human_inventory_per_frame'][i], dict) else {}),
                        **({'operation_mode': meta['operation_mode']} if 'operation_mode' in meta else {}),
                        'source':'SafeBench Sim_Raw_GT conservative adapter',
                        'episode_id':meta.get('episode_id'),'frame_index':i,
                        'scenario_id':meta.get('scenario_id'),
                        'time_s':(i+post_offset)*dt if dt is not None else None,
                        'scope':'current frame, not episode extrema',
                        'limitations':['per-frame contact impulse, not cumulative event impulse',
                          'unmapped or absent observations stay null; first-frame speed unknown',
                          'historical episode scalar outcomes are not timestamped live observations']},
            'features':f}
