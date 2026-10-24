"""Classify unchanged CPU IK goal envelopes; never execute a trajectory."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from kitchen_safety.extensions.doors.invariant_envelopes_lwx import invariant_envelopes
from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from scripts.extensions.doors.physical_hazard_geometry import physical_hazards
from scripts.extensions.doors.search_avoidance_layouts import clearance


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--diagnostic', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Evidence exists')
    diagnosis = json.loads(args.diagnostic.read_text())
    baseline, config_path = Path(diagnosis['baseline']), Path(diagnosis['config'])
    if sha(config_path) != diagnosis['config_sha256'] or sha(baseline / 'trajectory.json') != diagnosis['baseline_sha256']:
        raise ValueError('Diagnostic source changed')
    metadata = json.loads((baseline / 'metadata.json').read_text())
    config = json.loads(config_path.read_text())
    paired = json.loads(Path(diagnosis['paired']).read_text())
    if sha(diagnosis['paired']) != diagnosis['paired_sha256']:
        raise ValueError('Paired evidence changed')
    variant = paired['variant']
    stage, kin, world, _ = reconstruct(metadata, config, config[variant])
    hazard = physical_hazards(stage, world, config[variant]['name'])
    margin = world.clearances[config[variant]['name']]
    fingers = {name: metadata['initial_q'][metadata['robot_joint_names'].index(name)]
               for name in world.GRIPPER_JOINT_NAMES}
    samples, bounds = [], []
    for trial in diagnosis['trials']:
        if 'arm_q' not in trial:
            continue
        shapes = world.shapes(np.asarray(trial['arm_q']), fingers)
        arms = [shape for shape in shapes if shape[0] in ('arm', 'arm_mount_envelope')]
        distal = invariant_envelopes(shapes)
        bounds.append(np.asarray([[lo, hi] for _, _, lo, hi in distal]))
        samples.append(dict(index=trial['index'],
                            arm_clearances=[clearance([(f'arm_segment_{i}', *shape[1:])], hazard, margin)
                                            for i, shape in enumerate(arms)],
                            invariant_envelope_clearances=[clearance([shape], hazard, margin)
                                                           for shape in distal]))
    if not samples:
        raise ValueError('No converged IK solutions')
    extrema = np.asarray([[check['distance_m'] for check in row['invariant_envelope_clearances']]
                          for row in samples])
    report = dict(diagnostic=str(args.diagnostic), diagnostic_sha256=sha(args.diagnostic),
                  implementation_sha256=sha(__file__),
                  helper_sha256=sha('kitchen_safety/extensions/doors/invariant_envelopes_lwx.py'),
                  scope='Bounded CPU unchanged-target IK classification only. Conservative envelopes are not source intersection evidence, and do not prove global infeasibility.',
                  physics_replay=False, unsafe_path_executed=False,
                  arm_envelope_order=[[0, 1, .075], [1, 3, .065], [3, 4, .065], [4, 5, .065], [5, 7, .065], [7, 8, .055]],
                  invariant_names=[shape[0] for shape in distal], required_m=margin,
                  ik_solution_count=len(samples),
                  maximum_invariant_aabb_spread_m=float(np.ptp(np.asarray(bounds), axis=0).max()),
                  minimum_distances_m=extrema.min(axis=0).tolist(),
                  maximum_distances_m=extrema.max(axis=0).tolist(), samples=samples)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print('CLASSIFIED', len(samples), 'IK goals; invariant distances min/max', report['minimum_distances_m'], report['maximum_distances_m'],
          'maximum AABB spread', report['maximum_invariant_aabb_spread_m'], flush=True)


if __name__ == '__main__':
    main()
