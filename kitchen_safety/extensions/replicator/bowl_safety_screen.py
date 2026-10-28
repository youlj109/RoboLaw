"""Bind a complete offline hazard screen to its exact source and baseline.

This is an execution precondition, never task or multi-seed acceptance. JSON
and file hashing only; safe before SimulationApp without loading USD/CUDA.
"""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
REQUIRED = (
    'original_task_layout_unchanged', 'hazard_supported', 'hazard_source_kitchen_clear',
    'hazard_fixed_base_clear', 'initial_configuration_safe', 'fixed_stage_goals_safe',
    'baseline_full_source_risk', 'baseline_controls_match',
)


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def local(value):
    text = str(value)
    path = ROOT / text[len('/workspace/'):] if text.startswith('/workspace/') else Path(text)
    path = (ROOT / path).resolve() if not path.is_absolute() else path.resolve()
    if not path.is_relative_to(ROOT):
        raise ValueError('Screen input is outside the verified repository: ' + text)
    return path


def load_screen(path, layout_path, variant, policy, control_profile):
    if variant == 'base':
        if path is not None:
            raise ValueError('A no-hazard control must not use a hazard risk screen')
        return dict(required=False, passed=True, physical_success=False)
    if path is None:
        raise ValueError('A hazard requires a complete exact-layout source-geometry screen')
    path = local(path)
    data = json.loads(path.read_text())
    if (data.get('schema_version') != 1 or data.get('task_id') != policy['task_id'] or
            data.get('variant') != variant or data.get('passed') is not True or
            not all(data.get('checks', {}).get(name) is True for name in REQUIRED)):
        raise ValueError('Incomplete or failed original hazard geometry screen')
    if data['layout_sha256'] != sha(layout_path):
        raise ValueError('Geometry screen describes a different layout')
    risk = data['source_risk']
    required = policy['execution_clearances_m'][policy['hazard_name']]
    minimum = risk['minimum']
    if not (risk['required_m'] == required and risk['full_physics_coverage'] is True and
            risk['samples'] == risk['expected_samples'] > 1 and risk['violates_safety_margin'] is True and
            minimum['required_m'] == required and minimum['distance_m'] < required and
            data['code_unchanged'] is True):
        raise ValueError('Screen lacks complete source-geometry risk under the original safety margin')
    baseline = local(data['baseline']['episode'])
    expected = data['baseline']['input_sha256']
    if set(expected) != {'physics_states.npz', 'physics_metadata.json', 'trajectory.npz'}:
        raise ValueError('Screen must bind complete actual baseline states, schema and actions')
    inputs = {str((baseline/name).relative_to(ROOT)): digest for name, digest in expected.items()}
    inputs.update(data['source_reports_sha256'])
    source = data['source_hazard_geometry']
    inputs[source['asset_path']] = source['asset_sha256']
    inputs.update(source['source_layers_sha256'])
    for row in source['catalog_sources']:
        inputs[row['path']] = row['sha256']
    inputs[source['asset_reference_config']['path']] = source['asset_reference_config']['sha256']
    inputs[risk['report_path']] = risk['report_sha256']
    mismatches = [name for name, digest in inputs.items() if sha(local(name)) != digest]
    if mismatches:
        raise ValueError('Geometry screen source bytes changed: ' + repr(mismatches))
    original = json.loads((baseline/'result.json').read_text())
    metadata = json.loads((baseline/'physics_metadata.json').read_text())
    inputs[str((baseline/'result.json').relative_to(ROOT))] = sha(baseline/'result.json')
    padding = policy['transport_object_padding_m_by_variant'].get(control_profile, 0.)
    original_padding = original.get('transport_object_padding_m', 0.)
    if not (original['task_id'] == policy['task_id'] and original['variant'] == 'base' and
            original['physical_trial_passed'] is True and original_padding == padding and
            original['controller_profile_sha256'] == sha(ROOT/'configs/extensions/replicator/g_shape_bowl_sink_controller.json') and
            original['target_support_policy_sha256'] == sha(ROOT/'configs/extensions/replicator/g_bowl_target_support.json') and
            metadata['physics_samples'] == risk['samples']):
        raise ValueError('Screen baseline is incomplete or uses a different payload control padding')
    return dict(required=True, passed=True, path=str(path.relative_to(ROOT)), sha256=sha(path),
        verified_input_sha256={str(local(k).relative_to(ROOT)): v for k, v in inputs.items()},
        baseline=str(baseline.relative_to(ROOT)), source_risk_minimum=minimum,
        source_required_m=required, physical_success=False,
        scope='Exact offline screen only; live full planning/execution, hazard stability, neutral, '
              'source clearance and paired-seed checks remain required.')
