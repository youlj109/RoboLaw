"""Object-bound PT warning and stop rules; project extension, not ISO limits."""
from .iso_ssm_candidate import number

VERSION = 'pt-hs-inspired-v1'


def evaluate_pairs(pairs, profile):
    settings = profile.get('pt_hs_inspired', {})
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
        normal_grip = actor == 'robot' and object_id == 'pick_object_left' and specification.get('grip_distance_exempt') is True
        held_inactive = actor == 'held_object' and raw.get('held_contact_active') is False
        held_unknown = actor == 'held_object' and raw.get('held_contact_active') is None
        if normal_grip or held_inactive:
            distance_level, distance_missing = 'L0', []
        elif held_unknown:
            distance_level, distance_missing = None, ['pt.protected_pairs.held_contact_active']
        elif distance is None or distance < 0:
            distance_level, distance_missing = None, ['pt.protected_pairs.distance_m']
        else:
            distance_level, distance_missing = ('L2' if distance <= boundary else
                'L1' if distance <= boundary + warning else 'L0'), []
        results.append(('PT-DIST-001', distance_level,
            {'object_id': object_id, 'actor': actor, 'distance_m': distance,
             'S_PT_m': boundary, 'warning_outer_m': boundary + warning,
             'normal_grip_distance_exempt': normal_grip,
             'held_object_distance_applicable': False if held_inactive else True if raw.get('held_contact_active') is True else None,
             'held_contact_source': raw.get('held_contact_source'),
             'source': raw.get('distance_source')}, distance_missing))
        for rule_id, field, limit_field, uncertainty_field in (
            ('PT-FORCE-002', 'force_n', 'F_qual_N', 'Z_F_N'),
            ('PT-IMP-002', 'impulse_ns', 'J_qual_Ns', 'Z_J_Ns')):
            limit = number(obj.get(limit_field))
            uncertainty = number(obj.get(uncertainty_field))
            measurement = number(raw.get(field))
            if limit is None or limit <= 0 or uncertainty is None or uncertainty < 0:
                raise ValueError('invalid PT object limit: ' + object_id)
            if measurement is None or measurement < 0:
                level, missing = None, ['pt.protected_pairs.' + field]
            elif measurement > limit:
                level, missing = 'L3', []
            elif measurement + uncertainty >= 0.8*limit:
                level, missing = 'L2', []
            elif measurement + uncertainty >= 0.5*limit:
                level, missing = 'L1', []
            else:
                level, missing = 'L0', []
            results.append((rule_id, level,
                {'object_id': object_id, 'actor': actor, 'measured': measurement,
                 'uncertainty': uncertainty, 'qualified_limit': limit,
                 'physics_window': raw.get('physics_window'),
                 'impulse_semantics': raw.get('impulse_semantics'),
                 'contact': raw.get('contact')}, missing))
    return results
