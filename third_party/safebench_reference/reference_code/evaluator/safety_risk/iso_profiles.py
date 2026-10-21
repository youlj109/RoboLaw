"""Resolve predeclared task groups to a fixed profile, without fitting outcomes."""
from __future__ import annotations
import copy
from .iso_traceable import Evaluator, digest


def resolve_profile(library, scenario_id, *, stopping_model='runtime_l2', sensitivity='nominal'):
    if library.get('schema_version') != 'iso-task-library-1':
        raise ValueError('Expected iso-task-library-1')
    if scenario_id not in library.get('bindings', {}):
        raise ValueError('Unknown scenario_id; no guessed fallback: ' + str(scenario_id))
    if stopping_model not in library['stopping_models'] or sensitivity not in library['sensitivity']:
        raise ValueError('Unknown stopping model or sensitivity profile')
    binding = library['bindings'][scenario_id]
    groups = binding['asset_groups']
    if not groups or any(g not in library['asset_groups'] for g in groups):
        raise ValueError('Task has invalid asset group bindings')
    factor = library['sensitivity'][sensitivity]
    p = copy.deepcopy(library['base_profile'])
    p['profile_id'] = library['library_id'] + '/' + scenario_id + '/' + stopping_model + '/' + sensitivity
    p['binding'] = {
        'scenario_id': scenario_id,
        'robot_id': '+'.join(sorted({r['name'] for r in binding['robots']})) or 'unknown_robot',
        'operation_mode': 'simulation',
        'payload_profile': 'task_declared_assets_mass_unverified',
        'object_class': '+'.join(groups),
    }
    p['references']['GROUP_PROXY_DESIGN'] = {
        'title': 'SafeBench grouped experimental proxy design 2026-09-23',
        'locator': 'docs/ISO_GROUPED_PROXY_PROFILES_CN.md',
        'role': 'Author-selected temporary budgets and class-relative limits; not industry consensus or calibration',
    }
    p['references']['RUNTIME_CONFIG'] = {
        'title': 'SafeBench runtime L2 stopping schedule at f27801c',
        'locator': 'configs/simbox/safety_scenarios.yaml + safety_risk/intervention_policy.py',
        'role': '90 maximum braking frames at 30Hz; actual joint rest must be verified; no measured Cartesian speed bound',
    }
    p['references']['TASK_BINDING'] = {
        'title': scenario_id, 'locator': binding['task_yaml'], 'sha256': binding['task_sha256'],
        'role': 'Task robot/object identity evidence only, not qualification limits',
    }

    def set_value(key, value, basis, sources):
        rec = p['parameters'][key]
        rec.update(value=value, evidence_grade='P', source_ids=sources,
                   value_basis=basis, calibrated=False,
                   applicability='Unvalidated experimental binding: ' + scenario_id)

    model = library['stopping_models'][stopping_model]
    for key in ('T_R', 'T_S', 'B'):
        set_value(key, model[key], model['basis'], model['source_ids'])
    for key, val in library['common_parameter_overrides'].items():
        set_value(key, val, library['uncertainty_basis'], ['GROUP_PROXY_DESIGN'])
    for key in ('h_qual', 'F_qual', 'J_qual'):
        val = min(library['asset_groups'][g][key] for g in groups) * factor
        basis = ('Componentwise minimum across declared candidate groups ' + ','.join(groups)
                 + f'; fixed pre-episode {sensitivity} factor={factor}. Group values are P-grade expert assumptions, not tests.')
        set_value(key, val, basis, ['GROUP_PROXY_DESIGN', 'SAFEBENCH_BASELINE', 'TASK_BINDING'])
    for key, lim in (('Z_F', 'F_qual'), ('Z_J', 'J_qual')):
        fraction = library['uncertainty_policy'][key + '_fraction_of_' + lim]
        set_value(key, fraction*p['parameters'][lim]['value'],
                  f'Fixed uncertainty budget={fraction}*{lim}; not a measured confidence interval', ['GROUP_PROXY_DESIGN'])
    p['task_profile_resolution'] = {
        'library_id': library['library_id'], 'library_sha256': digest(library),
        'scenario_id': scenario_id, 'stopping_model': stopping_model,
        'sensitivity': sensitivity, 'asset_groups': groups,
        'robot_configurations': binding['robots'], 'objects': binding['objects'],
        'semantic_review_required': binding['review_required'],
        'assumed_robot_speed_bound_mps': model['assumed_robot_speed_bound_mps'],
        'speed_bound_enforced': False,
        'notes': ['Task binding is fixed before evaluation, not selected from a resulting risk grade.',
                  'Same numeric stop proxy is shared across robot families because no comparative stopping evidence exists.',
                  'A runtime_l2 SSM envelope can exceed tabletop size; do not shrink it to force L0/L1.',
                  'Proxy limits do not model spills, heat, sharp edges or food hygiene beyond available frozen features.'],
    }
    Evaluator(p)  # reuse all parameter/unit/provenance validation
    return p


def scenario_from_input(data):
    candidates = [data.get('metadata', {}).get('scenario_id'), data.get('episode_meta', {}).get('scenario_id')]
    for sample in data.get('samples', []) if isinstance(data.get('samples'), list) else []:
        candidates.append(sample.get('metadata', {}).get('scenario_id') if isinstance(sample, dict) else None)
    values = {v for v in candidates if isinstance(v, str) and v}
    if len(values) > 1:
        raise ValueError('Input mixes different scenario IDs; one episode must use one fixed task profile')
    return next(iter(values), None)
