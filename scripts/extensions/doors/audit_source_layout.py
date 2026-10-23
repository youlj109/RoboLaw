"""Current source/layout provenance; historical missing files are never treated as verified."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import numpy as np
from kitchen_safety.extensions.doors.environment import ROOT, TASK


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('episodes', nargs='+', type=Path)
    p.add_argument('--logs', type=Path, required=True)
    a = p.parse_args()
    assignment = json.loads((ROOT/'student_skills/robolaw-doors/references/assignment.json').read_text())
    # The manifest stores the pinned hash; search its structure without assuming a key layout.
    def objects(value):
        if isinstance(value, dict):
            yield value
            for child in value.values():
                yield from objects(child)
        elif isinstance(value, list):
            for child in value:
                yield from objects(child)
    task_entries = [x for x in objects(assignment) if TASK.stem in x.values()]
    if not task_entries:
        raise ValueError('Task absent from group assignment')
    expected_hash = '679575de18414f7ccb49a5d43bcb2e1b952eb4c82c680ab79c55734aa638d3ee'
    if expected_hash not in json.dumps(task_entries):
        raise ValueError('C01 assignment hash changed; review rather than assume')
    config = ROOT/'configs/extensions/doors/fixed_source_layout.json'
    fixed = json.loads(config.read_text())
    cached = Path.home()/'.cache/isaaclab_arena/usd/robot_on_stand/doors_droid_on_stand_0.8_1.08x0.91032.usd'
    environment = ROOT/'kitchen_safety/extensions/doors/environment.py'
    text = environment.read_text()
    original_relations_attached = '_attach_spatial_relations_to_assets(graph.relations,assets)' in text
    collector = ROOT/'scripts/extensions/doors/collect.py'
    tree = ast.parse(collector.read_text())
    forbidden = []
    # This is a static check of our execution entrypoint, not a claim to audit native reset internals.
    blocked = ('write_joint', 'write_root', 'set_joint', 'set_world_pose', 'set_local_pose', 'CreateFixedJoint')
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            if node.func.attr.startswith(blocked):
                forbidden.append({'line': node.lineno, 'call': ast.unparse(node.func)})
    overall = True
    for ep in a.episodes:
        output = ep/'source_layout_audit.json'
        if output.exists():
            raise FileExistsError(output)
        m = json.loads((ep/'metadata.json').read_text())
        log = a.logs/(ep.name+'.log')
        lines = log.read_text(errors='replace').splitlines()
        placements = [s for s in lines if '[placement] Validated' in s]
        errors = []
        if m['source_yaml_sha256'] != expected_hash or sha(TASK) != expected_hash:
            errors.append('Source task hash differs')
        if not np.allclose(m['robot_base'], fixed['expected_robot_base'], atol=2e-6, rtol=0) or abs(m['robot_yaw_deg']-fixed['expected_robot_yaw_deg']) > 1e-6:
            errors.append('Current measured source placement differs')
        if sha(cached) != fixed['source_robot_usd_sha256']:
            errors.append('Composed source robot/stand hash differs')
        if not original_relations_attached or not placements or any('Falling back to best-loss layouts' in s for s in lines):
            errors.append('Missing source relation validation or fallback detected')
        if forbidden:
            errors.append('Execution entrypoint contains forbidden direct state writes')
        origin_metadata = ROOT/fixed['baseline_episode']/'metadata.json'
        report = {'passed': not errors, 'errors': errors, 'task_source_sha256': sha(TASK),
                  'source_placement_measurement': {'position': m['robot_base'], 'yaw_deg': m['robot_yaw_deg']},
                  'frozen_layout_sha256': sha(config), 'composed_robot_stand_sha256': sha(cached),
                  'original_relation_attachment_present': original_relations_attached,
                  'current_solver_validation_lines': placements, 'log_sha256': sha(log),
                  'execution_entrypoint_direct_state_write_calls': forbidden,
                  'source_implementation_sha256': {'environment': sha(environment), 'collect': sha(collector)},
                  'historical_reference_metadata_available': origin_metadata.exists(),
                  'historical_reference_metadata_hash_matches': sha(origin_metadata)==fixed['metadata_sha256'] if origin_metadata.exists() else None,
                  'scope': 'Current pinned YAML, same measured frozen source placement, composed robot/stand hash, retained source relations and current actual solver log. Static direct-write scan covers the owned collector only; native source reset is outside this scan. Historical baseline bytes, when absent, remain unverified; current successful physical baseline is independently audited.'}
        output.write_text(json.dumps(report, indent=2))
        overall &= report['passed']
        print(ep, report['passed'], errors, flush=True)
    return 0 if overall else 1


if __name__ == '__main__':
    raise SystemExit(main())
