"""Full CPU pull proposals after proxy rejection; diagnostic only, never step physics."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from kitchen_safety.extensions.doors.task_model import door_target_pose, joint_position_from_openness, opening_command
from kitchen_safety.extensions.doors.task_spheres import initial_spheres
from scripts.extensions.doors.physical_hazard_geometry import physical_hazards


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--variant', choices=['property', 'robot'], required=True)
    parser.add_argument('--paired', type=Path, required=True)
    parser.add_argument('--approach', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Refusing to replace diagnostic evidence')
    metadata = read(args.baseline / 'metadata.json')
    paired, approach = read(args.paired), read(args.approach)
    baseline_hash, config_hash = sha(args.baseline / 'trajectory.json'), sha(args.config)
    for document in [paired, approach]:
        if document['baseline_sha256'] != baseline_hash or document['config_sha256'] != config_hash:
            raise ValueError('Baseline/config provenance mismatch')
        if document['controller_parameters'] != metadata['controller_parameters']:
            raise ValueError('Controller provenance mismatch')
    if approach['paired_sha256'] != sha(args.paired):
        raise ValueError('Approach uses another paired result')
    config = read(args.config)
    stage, kin, world, geometry = reconstruct(metadata, config, config[args.variant])
    cp = metadata['controller_parameters']
    margin = cp.get('ik_limit_margin_rad', 0.)
    kin.limits[:, 0] += margin
    kin.limits[:, 1] -= margin
    hazards = physical_hazards(stage, world, config[args.variant]['name'])
    if not hazards:
        raise ValueError('No original hazard geometry')
    padding = world.clearances[config[args.variant]['name']] + cp.get('planner_hazard_extra_padding_m', 0.)
    rows = read(args.baseline / 'trajectory.json')
    pulls = [(index, row) for index, row in enumerate(rows) if row['phase'] == 'pull_door']
    grasp = [row for row in rows if row['phase'] == 'grasp_handle']
    if not pulls or not grasp:
        raise ValueError('Missing retained baseline grasp/pull motion')
    full = np.asarray(grasp[-1]['next_obs']['q']).copy()
    lower, upper = geometry['limits_rad']
    threshold = joint_position_from_openness(geometry['openness_threshold'], lower, upper)
    limit = max(lower + .01, threshold - .10) if lower < 0. else min(upper - .01, threshold + .10)
    direction = Rotation.from_euler('z', metadata['robot_yaw_deg'], degrees=True).apply([1., 0., 0.])
    report = dict(scope='CPU conservative hazard-proxy proposals continue after overlap for diagnostic coverage only. Candidate arm tracks commands ideally while door/fingers follow recorded baseline timing. No changed dynamics, exact rejected command, GPU self-collision, safe execution or acceptance claim.', physics_replay=False, unsafe_path_executed=False,
                  baseline=str(args.baseline), baseline_sha256=baseline_hash, config=str(args.config), config_sha256=config_hash,
                  paired=str(args.paired), paired_sha256=sha(args.paired), approach=str(args.approach), approach_sha256=sha(args.approach),
                  implementation_sha256=sha(__file__), controller_parameters=cp, planning_padding_m=padding,
                  sphere_model='Original cuRobo spheres including existing 5mm buffer versus axis-expanded original per-collider hazard AABBs.', pairs=[])
    approach_pairs = {item['seed']: item for item in approach['pairs']}
    for pair in paired['pairs']:
        prior = approach_pairs[pair['seed']]
        item = dict(seed=pair['seed'], available=bool(pair['on'].get('success') and prior.get('discovery_eligible')), samples=[])
        report['pairs'].append(item)
        if not item['available']:
            continue
        q = np.asarray(prior['samples'][-1]['arm_q'])
        pose = np.asarray(pair['on']['target_pose'])
        handle, rotation = pose[:3, 3] + direction * .13, pose[:3, :3]
        try:
            for index, row in pulls:
                angle = opening_command(row['obs']['door_q'], cp['pull_lead_rad'], limit, lower)
                target = door_target_pose(np.asarray(geometry['hinge']), np.asarray(geometry['axis']), handle, rotation, angle, metadata['door_motion_reference_joint_rad'])
                command = kin.ik(target, q, attempts=1)
                delta = command - q
                command = q + delta / max(1., float(np.abs(delta).max()) / .025)
                full[:] = row['obs']['q']
                full[:7] = command
                spheres = initial_spheres(dict(metadata, initial_q=full.tolist()), kin)
                centers, radii = np.asarray([s[2] for s in spheres]), np.asarray([s[3] for s in spheres])
                minimum = None
                for name, _, low, high in hazards:
                    slack = np.linalg.norm(np.maximum(np.maximum(low - padding - centers, centers - high - padding), 0.), axis=1) - radii
                    sphere = int(np.argmin(slack))
                    value = dict(slack_m=float(slack[sphere]), obstacle=name, link=spheres[sphere][0], sphere=sphere)
                    if minimum is None or value['slack_m'] < minimum['slack_m']:
                        minimum = value
                item['samples'].append(dict(baseline_action_index=index, recorded_baseline_door_q=row['obs']['door_q'], arm_q=command.tolist(), minimum_proxy=minimum))
                q = command
                if minimum['slack_m'] < 0.:
                    item['offline_proxy_overlap_seen'] = True
        except (RuntimeError, ValueError) as exc:
            item['error'] = str(exc)
        item['discovery_eligible'] = bool(not item.get('error') and len(item['samples']) == len(pulls) and all(s['minimum_proxy']['slack_m'] >= 0. for s in item['samples']))
        item['minimum_slack_m'] = min((s['minimum_proxy']['slack_m'] for s in item['samples']), default=None)
        print('PULL_PROXY', item['seed'], item['discovery_eligible'], item['minimum_slack_m'], len(item['samples']), flush=True)
        args.output.write_text(json.dumps(report, indent=2))
    report['all_projected_pulls_discovery_eligible'] = bool(report['pairs'] and all(item.get('discovery_eligible', False) for item in report['pairs']))
    report['complete'] = True
    args.output.write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
