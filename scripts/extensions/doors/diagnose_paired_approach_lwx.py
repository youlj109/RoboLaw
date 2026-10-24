"""Offline contact-approach discovery from paired endpoints; never step physics.

The collector seeds each IK command with measured joints. This diagnostic uses
perfect command tracking, so passing it is not physical acceptance. Replaying a
recorded rejection separately preserves that distinction.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from scripts.extensions.doors.physical_hazard_geometry import physical_hazards, physical_robot_shapes
from scripts.extensions.doors.search_avoidance_layouts import clearance


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline', type=Path, required=True)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--variant', choices=['human', 'property', 'robot'], required=True)
    p.add_argument('--paired', type=Path, required=True)
    p.add_argument('--actual-reference', action='append', type=Path, default=[])
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        p.error('Refusing to overwrite evidence')
    m, config, paired = read(a.baseline / 'metadata.json'), read(a.config), read(a.paired)
    if paired['baseline_sha256'] != sha(a.baseline / 'trajectory.json') or paired['config_sha256'] != sha(a.config):
        raise ValueError('Paired baseline/config provenance mismatch')
    if paired['controller_parameters'] != m['controller_parameters']:
        raise ValueError('Controller parameters differ')
    stage, kin, world, geometry = reconstruct(m, config, config[a.variant])
    cp = m['controller_parameters']
    margin = cp.get('ik_limit_margin_rad', 0.)
    kin.limits[:, 0] += margin
    kin.limits[:, 1] -= margin
    world.allow_handle_contact = True
    names = m['robot_joint_names']
    fingers = {n: m['initial_q'][names.index(n)] for n in world.GRIPPER_JOINT_NAMES}
    arm_ids = [names.index('panda_joint' + str(i)) for i in range(1, 8)]
    key = config[a.variant]['name']
    hazard = physical_hazards(stage, world, key)
    required = world.execution_clearances[key]

    def measure(q, fq):
        shapes = world.shapes(q, fq)
        return dict(source=clearance(physical_robot_shapes(shapes), hazard, required),
                    conservative_hazard=world.clearance(q, mode='execution', gripper_q=fq, obstacle_filter=[key]),
                    conservative_world=world.clearance(q, mode='execution', gripper_q=fq))

    def minimum(items, field):
        return min(items, key=lambda v: v['measurement'][field]['slack_m']) if items else None

    report = dict(scope='Offline perfect-tracking IK approach discovery only; no physical or GPU feasibility claim.',
                  physics_replay=False, unsafe_path_executed=False,
                  baseline=str(a.baseline), baseline_sha256=sha(a.baseline / 'trajectory.json'),
                  paired=str(a.paired), paired_sha256=sha(a.paired),
                  config=str(a.config), config_sha256=sha(a.config),
                  implementation_sha256=sha(__file__), controller_parameters=cp,
                  original_execution_margin_m=required, extra_discovery_slack_m=.003,
                  joint_subdivision_resolution_rad=.005, pairs=[], recorded_references=[])
    direction = Rotation.from_euler('z', m['robot_yaw_deg'], degrees=True).apply([1., 0., 0.])
    for pair in paired['pairs']:
        plan = pair['on']
        item = dict(seed=pair['seed'], available=bool(plan.get('success')), samples=[])
        report['pairs'].append(item)
        if not item['available']:
            continue
        q = np.asarray(plan['positions'][-1])
        target = np.asarray(plan['target_pose']).copy()
        handle = target[:3, 3] + direction * .13
        item['samples'].append(dict(fraction=0., arm_q=q.tolist(), measurement=measure(q, fingers)))
        try:
            for fraction in np.linspace(0., 1., 50)[1:]:
                target[:3, 3] = handle - direction * .13 * (1. - fraction)
                command = kin.ik(target, q, attempts=1)
                count = max(1, int(np.ceil(np.max(np.abs(command - q)) / .005)))
                for t in np.linspace(0., 1., count + 1)[1:]:
                    sample_q = q + t * (command - q)
                    item['samples'].append(dict(fraction=float(fraction), arm_q=sample_q.tolist(), measurement=measure(sample_q, fingers)))
                q = command
        except RuntimeError as exc:
            item['error'] = str(exc)
        item['minimum_source'] = minimum(item['samples'], 'source')
        item['minimum_conservative_hazard'] = minimum(item['samples'], 'conservative_hazard')
        item['minimum_conservative_world'] = minimum(item['samples'], 'conservative_world')
        item['discovery_eligible'] = bool(not item.get('error') and
            item['minimum_conservative_hazard']['measurement']['conservative_hazard']['slack_m'] >= .003 and
            all(v['measurement']['conservative_world']['valid'] for v in item['samples']))
        print('APPROACH', item['seed'], 'eligible', item['discovery_eligible'],
              'hazard_slack_m', item['minimum_conservative_hazard']['measurement']['conservative_hazard']['slack_m'], flush=True)
    for root in a.actual_reference:
        rm, result = read(root / 'metadata.json'), read(root / 'result.json')
        if rm['controller_parameters'] != cp or rm['robot_base'] != m['robot_base'] or rm['variant_spec'] != config[a.variant]:
            raise ValueError('Recorded reference policy, layout or hazard differs')
        failure = result.get('failure') or ''
        if 'EXECUTION_COLLISION ' not in failure:
            raise ValueError('Reference has no recorded execution rejection')
        failure = json.loads(failure.split('EXECUTION_COLLISION ', 1)[1])
        full = np.asarray(failure['q'])
        fq = {n: float(full[names.index(n)]) for n in world.GRIPPER_JOINT_NAMES}
        report['recorded_references'].append(dict(episode=str(root), result_sha256=sha(root / 'result.json'),
            phase=failure['phase'], action_index=failure['action_index'], original_failure=failure['failure'],
            reconstructed_measurement=measure(full[arm_ids], fq)))
    report['all_predicted_approaches_discovery_eligible'] = bool(report['pairs'] and
        all(v.get('discovery_eligible', False) for v in report['pairs']))
    a.output.write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
