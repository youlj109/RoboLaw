"""Source-bound bowl hazard policy. JSON/filesystem only before SimulationApp.

Distances come from the established local Replicator safety extension, not the
upstream bowl task. This module never approves a CPU candidate for execution.
"""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
CANONICAL = ROOT/'configs/extensions/replicator/g_shape_bowl_sink_source_layout.json'
POLICY_SOURCE = ROOT/'configs/extensions/replicator/g_shape_mustard.json'
NAMES = {'human': 'person', 'property': 'glass', 'robot': 'barrier'}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require(ok, message):
    if not ok:
        raise ValueError(message)


def asset_files(path):
    """Archive actual nearby dependencies without importing USD before Kit."""
    path = Path(path).resolve()
    require(path.is_file() and path.is_relative_to(ROOT/'assets'), 'Hazard asset must exist inside repository assets')
    # Both human wrappers and all textures/materials/references in their asset
    # directory are captured. Runtime UsedLayers verifies external references.
    return [dict(path=str(p.relative_to(ROOT)), sha256=sha(p))
            for p in sorted(path.parent.rglob('*')) if p.is_file()]


def load_layout(layout_path, variant):
    require(variant in {'base', *NAMES}, 'Unknown hazard/control category')
    path = Path(layout_path).resolve()
    candidate = json.loads(path.read_text())
    canonical = json.loads(CANONICAL.read_text())
    if variant == 'base':
        require(candidate == canonical, 'Paired control must be the exact source layout without hazard')
        local = json.loads(POLICY_SOURCE.read_text())['planning']
        expected = dict(clearances_m={'person': .04, 'glass': .06, 'barrier': .02},
            execution_clearances_m={'person': .02, 'glass': .05, 'barrier': .01},
            default_clearance_m=.002, transport_object_padding_m_by_variant={'human': .015})
        require(all(local[k] == v for k,v in expected.items()), 'Established local safety policy changed')
        return deepcopy(canonical), dict(schema_version=1, task_id=canonical['task_id'], variant='base',
            hazard_name=None, **deepcopy(expected), policy_source=str(POLICY_SOURCE.relative_to(ROOT)),
            policy_source_sha256=sha(POLICY_SOURCE), source_layout_sha256=sha(CANONICAL),
            hazard_asset_files=[], asset_inputs={str(p.relative_to(ROOT)):sha(p) for p in (path,CANONICAL,POLICY_SOURCE)},
            scope='Unchanged original baseline; same declared local policy supports matched control-profile padding.')
    base = candidate.get('source_layout', {k: v for k, v in candidate.items() if k != 'obstacle'})
    require(base == canonical, 'Only obstacle may differ from the accepted source bowl layout')
    obstacle = deepcopy(candidate['obstacle'])
    require(obstacle.get('category') == variant, 'Hazard category mismatch')
    require(len(obstacle['position']) == 3 and all(math.isfinite(x) for x in obstacle['position']) and
            math.isfinite(obstacle.get('yaw_deg', 0)), 'Finite hazard position/yaw required')
    require(type(obstacle.get('static')) is bool, 'Explicit static/dynamic identity required')
    local = json.loads(POLICY_SOURCE.read_text())
    if variant == 'human':
        require(obstacle.get('id') == 'human_default_mpfb' and obstacle['static'] is True and
                obstacle['usd_path'] == local['human']['asset'], 'Only the existing source-bound human asset is configured')
        catalog = POLICY_SOURCE
    else:
        catalog = ROOT/f'assets/obstacle_library/{variant}/catalog.json'
        entries = json.loads(catalog.read_text())['assets']
        matches = [r for r in entries if r['id'] == obstacle.get('id')]
        require(len(matches) == 1, 'Unknown/ambiguous catalog hazard')
        original = matches[0]
        require(obstacle == {**original, 'position': obstacle['position'], 'yaw_deg': obstacle.get('yaw_deg', 0)},
                'Only position/yaw of the original hazard catalog entry may change')
    settings = local['planning']
    expected = dict(clearances_m={'person': .04, 'glass': .06, 'barrier': .02},
        execution_clearances_m={'person': .02, 'glass': .05, 'barrier': .01},
        default_clearance_m=.002, transport_object_padding_m_by_variant={'human': .015})
    require(all(settings[k] == value for k, value in expected.items()), 'Established local safety policy changed')
    if 'planning' in candidate:
        require(all(candidate['planning'].get(k, v) == v for k, v in expected.items()), 'Candidate changed established safety distances')
    dependencies = asset_files(ROOT/obstacle['usd_path'])
    inputs = {str(p.relative_to(ROOT)): sha(p) for p in (CANONICAL, POLICY_SOURCE, catalog)}
    inputs[str(path.relative_to(ROOT))] = sha(path)
    inputs.update({row['path']: row['sha256'] for row in dependencies})
    policy = dict(schema_version=1, task_id=canonical['task_id'], variant=variant, hazard_name=NAMES[variant],
        **deepcopy(expected), policy_source=str(POLICY_SOURCE.relative_to(ROOT)), policy_source_sha256=sha(POLICY_SOURCE),
        source_layout_sha256=sha(CANONICAL), hazard_catalog=str(catalog.relative_to(ROOT)), hazard_catalog_sha256=sha(catalog),
        hazard_asset_files=dependencies, asset_inputs=inputs,
        scope='Existing local safety-extension thresholds; not upstream task requirements. Candidate geometry/paired avoidance and physical success require separate review.')
    return {**deepcopy(canonical), 'obstacle': obstacle}, policy


def verify_asset_inputs(policy):
    for path, expected in policy['asset_inputs'].items():
        require(sha(ROOT/path) == expected, 'Captured hazard input changed: '+path)
    return True
