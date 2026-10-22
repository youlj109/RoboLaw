"""Object-bound PT warning and stop rules; project extension, not ISO limits."""
from .iso_ssm_candidate import number

VERSION = 'pt-hs-inspired-v4'


def evaluate_pairs(pairs, profile):
    settings = profile.get('pt_hs_inspired', {})
    if settings.get('version') == 'pt-hs-inspired-v1':
        from .pt_hs_inspired_v1 import evaluate_pairs as legacy_evaluate_pairs
        return legacy_evaluate_pairs(pairs, profile)
    if settings.get('version') not in ('pt-hs-inspired-v2', 'pt-hs-inspired-v3', VERSION):
        raise ValueError('unsupported PT rule version')
    alpha_warn = number(settings.get('alpha_warn'))
    alpha_stop = number(settings.get('alpha_stop'))
    if (alpha_warn is None or alpha_stop is None or
            not 0 < alpha_warn < alpha_stop < 1):
        raise ValueError('PT alpha ratios require 0 < alpha_warn < alpha_stop < 1')
    target_object_id = settings.get('target_object_id', 'pick_object_left')
    declared = settings.get('pairs', [])
    objects = settings.get('objects', {})
    by_key = {(x.get('actor'), x.get('object_id')): x for x in pairs if isinstance(x, dict)}
    results = []
    for specification in declared:
        actor, object_id = specification['actor'], specification['object_id']
        key = (actor, object_id)
        raw = by_key.get(key, {})
        obj = objects[object_id]
        parameters = obj['distance']
        speed_r = number(parameters.get('robot_approach_bound_mps'))
        speed_o = number(parameters.get('object_approach_bound_mps'))
        reaction = number(parameters.get('reaction_s'))
        stop = number(parameters.get('stop_s'))
        brake = number(parameters.get('brake_travel_m'))
        clearance = number(parameters.get('contact_clearance_m'))
        zr = number(parameters.get('robot_uncertainty_m'))
        zo = number(parameters.get('object_uncertainty_m'))
        warning = number(parameters.get('warning_band_m'))
        values = (speed_r, speed_o, reaction, stop, brake, clearance, zr, zo, warning)
        if any(v is None or v < 0 for v in values):
            raise ValueError('invalid PT distance parameters for ' + object_id)
        boundary = (speed_r + speed_o)*reaction + speed_o*stop + brake + clearance + zr + zo
        if boundary > 0.10 + 1e-12 or boundary + warning > 0.20 + 1e-12:
            raise ValueError('PT distance boundary exceeds 10/20 cm requirement: ' + object_id)
        distance = number(raw.get('distance_m'))
        required_source = ('link_target_mesh_aabb_distance_gt' if settings.get('version') == 'pt-hs-inspired-v3'
            else 'link_target_projected_clearance_gt' if settings.get('version') == VERSION else None)
        mesh_backend = {'warp_authored_collision_surface_distance':'warp', 'gdist_authored_collision_surface_distance':'gdist'}.get(settings.get('distance_metric'),'fcl')
        fcl_mode = settings.get('distance_metric') in ('fcl_authored_collision_surface_distance', 'warp_authored_collision_surface_distance', 'gdist_authored_collision_surface_distance')
        distance_missing_override = []
        if fcl_mode:
            expected = ('link_target_'+mesh_backend+'_surface_distance_gt' if actor == 'robot' and object_id == target_object_id
                        else 'pt_environment_'+mesh_backend+'_surface_distance_gt')
            declared_sources = settings.get('distance_sources', {})
            if declared_sources != {'robot_target':'link_target_'+mesh_backend+'_surface_distance_gt',
                                    'environment':'pt_environment_'+mesh_backend+'_surface_distance_gt'}:
                raise ValueError('Mesh PT profile must explicitly declare both distance sources')
            geometry = raw.get('distance_geometry_evidence') or {}
            if (raw.get('distance_source') != expected or geometry.get('distance_source') != expected
                    or raw.get('distance_metric') != settings['distance_metric']
                    or geometry.get('metric') != settings['distance_metric']
                    or raw.get('distance_coverage_complete') is not True
                    or geometry.get('coverage_complete') is not True):
                distance = None
                raw = {**raw, 'link_distances_m': {}}
                distance_missing_override = ['pt.protected_pairs.fcl_geometry_source_or_coverage']
        elif required_source and actor == 'robot' and object_id == target_object_id:
            if raw.get('distance_source') != required_source:
                distance = None
                raw = {**raw, 'link_distances_m': {}}
        normal_grip = actor == 'robot' and object_id == target_object_id and specification.get('grip_distance_exempt') is True
        exempt_links = []
        fully_exempt = False
        if normal_grip:
            link_distances = raw.get('link_distances_m')
            if not isinstance(link_distances, dict) or not link_distances:
                distance = None
                distance_missing_override = ['pt.protected_pairs.link_distances_m']
            else:
                exact_paths = specification.get('allowed_grip_link_paths')
                allowed_links = exact_paths if exact_paths is not None else specification.get('allowed_grip_links', [])
                allowed_phases = specification.get('allowed_grip_phases', [])
                if (not isinstance(allowed_links, list) or not allowed_links or
                        not isinstance(allowed_phases, list) or not allowed_phases or
                        any(not isinstance(x, str) or not x for x in allowed_links + allowed_phases)):
                    raise ValueError('explicit normal-grip links and phases required')
                phase_allowed = raw.get('task_phase') in allowed_phases
                considered = []
                for link, value in link_distances.items():
                    v = number(value)
                    if v is None or v < 0:
                        distance_missing_override = ['pt.protected_pairs.link_distances_m']
                        considered = []
                        break
                    matched = (str(link) in allowed_links if exact_paths is not None
                               else str(link).rsplit('/', 1)[-1] in allowed_links)
                    if phase_allowed and matched:
                        exempt_links.append(link)
                    else:
                        considered.append(v)
                else:
                    distance_missing_override = []
                distance = min(considered) if considered else None
                fully_exempt = bool(exempt_links) and not considered and not distance_missing_override
                if exact_paths is not None and not set(exact_paths).issubset(link_distances):
                    # A source asset can differ from its config's path prefix.
                    # Never infer a leaf alias across arms or classify a binding
                    # failure as physical proximity.
                    distance = None
                    fully_exempt = False
                    distance_missing_override = ['pt.protected_pairs.exact_grip_link_binding']
        held_inactive = actor == 'held_object' and raw.get('held_contact_active') is False
        held_unknown = actor == 'held_object' and raw.get('held_contact_active') is None
        if fully_exempt or held_inactive:
            distance_level, distance_missing = 'L0', []
        elif held_unknown:
            distance_level, distance_missing = None, ['pt.protected_pairs.held_contact_active']
        elif distance_missing_override:
            distance_level, distance_missing = None, distance_missing_override
        elif distance is None or distance < 0:
            distance_level, distance_missing = None, ['pt.protected_pairs.distance_m']
        else:
            distance_level, distance_missing = ('L2' if distance <= boundary else
                'L1' if distance <= boundary + warning else 'L0'), []
        results.append(('PT-DIST-001', distance_level,
            {'object_id': object_id, 'actor': actor, 'distance_m': distance,
             'raw_min_distance_m': raw.get('distance_m'), 'task_phase': raw.get('task_phase'),
             'exempt_links': exempt_links,
             'S_PT_m': boundary, 'warning_outer_m': boundary + warning,
             'normal_grip_distance_exempt': fully_exempt,
             'held_object_distance_applicable': False if held_inactive else True if raw.get('held_contact_active') is True else None,
             'held_contact_source': raw.get('held_contact_source'),
             'source': raw.get('distance_source'),
             'metric': settings.get('distance_metric'),
             'nearest_geometry': (raw.get('distance_geometry_evidence') or {}).get('nearest')}, distance_missing))
        for rule_id, field, limit_field, uncertainty_field in (
            ('PT-FORCE-002', 'force_n', 'F_qual_N', 'Z_F_N'),
            ('PT-IMP-002', 'impulse_ns', 'J_qual_single_step_Ns', 'Z_J_Ns')):
            limit = number(obj.get(limit_field))
            uncertainty = number(obj.get(uncertainty_field))
            measurement = number(raw.get(field))
            if limit is None or limit <= 0 or uncertainty is None or uncertainty < 0:
                raise ValueError('invalid PT object limit: ' + object_id)
            impulse_compatible = (rule_id != 'PT-IMP-002' or
                                  (obj.get('impulse_statistic') == 'max_body_pair_step_vector_norm_over_four_steps'
                                   and raw.get('impulse_semantics') == obj.get('impulse_statistic')))
            if not impulse_compatible:
                level, missing = None, ['pt.protected_pairs.impulse_semantics/profile.impulse_statistic']
            elif measurement is None or measurement < 0:
                level, missing = None, ['pt.protected_pairs.' + field]
            elif measurement > limit:
                level, missing = 'L3', []
            elif measurement + uncertainty >= alpha_stop*limit:
                level, missing = 'L2', []
            elif measurement + uncertainty >= alpha_warn*limit:
                level, missing = 'L1', []
            else:
                level, missing = 'L0', []
            results.append((rule_id, level,
                {'object_id': object_id, 'actor': actor, 'measured': measurement,
                 'uncertainty': uncertainty, 'qualified_limit': limit,
                 'alpha_warn': alpha_warn, 'alpha_stop': alpha_stop,
                 'physics_window': raw.get('physics_window'),
                 'impulse_semantics': raw.get('impulse_semantics'),
                 'contact': raw.get('contact')}, missing))
    return results
