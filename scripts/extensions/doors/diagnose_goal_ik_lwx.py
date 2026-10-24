"""Bounded CPU endpoint IK diagnosis; no planning path or physical execution."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from kitchen_safety.extensions.doors.task_spheres import initial_spheres
from scripts.extensions.doors.physical_hazard_geometry import physical_hazards
from scripts.extensions.doors.search_avoidance_layouts import clearance


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline', type=Path, required=True)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--paired', type=Path, required=True)
    p.add_argument('--variant', choices=['human', 'property', 'robot'], required=True)
    p.add_argument('--trials', type=int, default=48)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists() or not 1 <= a.trials <= 128:
        p.error('Evidence exists or trial count outside [1,128]')
    m, config, paired = read(a.baseline / 'metadata.json'), read(a.config), read(a.paired)
    if paired['baseline_sha256'] != sha(a.baseline / 'trajectory.json') or paired['config_sha256'] != sha(a.config):
        raise ValueError('Paired provenance mismatch')
    if paired['controller_parameters'] != m['controller_parameters'] or paired['variant'] != a.variant:
        raise ValueError('Paired policy/variant mismatch')
    pair = next(v for v in paired['pairs'] if v['seed'] == 42)
    if not pair['off']['success']:
        raise ValueError('A successful unchanged-target off endpoint is required')
    target = np.asarray(pair['off']['target_pose'])
    stage, kin, world, geometry = reconstruct(m, config, config[a.variant])
    margin = m['controller_parameters'].get('ik_limit_margin_rad', 0.)
    kin.limits[:, 0] += margin
    kin.limits[:, 1] -= margin
    fingers = {n: m['initial_q'][m['robot_joint_names'].index(n)] for n in world.GRIPPER_JOINT_NAMES}
    ids = [m['robot_joint_names'].index('panda_joint' + str(i)) for i in range(1, 8)]
    hazard = physical_hazards(stage, world, config[a.variant]['name'])
    required = world.clearances[config[a.variant]['name']]
    rng = np.random.default_rng(17)
    seeds = [np.asarray(pair['off']['positions'][-1]), np.asarray(m['initial_q'])[ids]]
    seeds += [rng.uniform(kin.limits[:, 0] + .02, kin.limits[:, 1] - .02) for _ in range(a.trials - 1)]
    report = dict(scope='Bounded CPU endpoint feasibility diagnosis only. No path, GPU self-collision, dynamics or acceptance claim.',
                  physics_replay=False, unsafe_path_executed=False,
                  baseline=str(a.baseline), baseline_sha256=sha(a.baseline / 'trajectory.json'),
                  config=str(a.config), config_sha256=sha(a.config),
                  paired=str(a.paired), paired_sha256=sha(a.paired), implementation_sha256=sha(__file__),
                  controller_parameters=m['controller_parameters'], target=target.tolist(),
                  random_seed=17, trials=[])
    for i, seed in enumerate(seeds[:a.trials]):
        item = dict(index=i, seed_q=seed.tolist())
        try:
            q = kin.ik(target, seed, attempts=1)
            pose = kin.fk(q)
            item.update(arm_q=q.tolist(), position_error_m=float(np.linalg.norm(pose[:3, 3] - target[:3, 3])),
                        rotation_error_rad=float(Rotation.from_matrix(pose[:3, :3] @ target[:3, :3].T).magnitude()))
            item['conservative_world'] = world.clearance(q, mode='planning', gripper_q=fingers)
            meta = dict(m)
            full = np.asarray(m['initial_q']).copy()
            full[ids] = q
            meta['initial_q'] = full.tolist()
            spheres = initial_spheres(meta, kin)
            import fcl
            objects = [(f'{link}/{idx}', fcl.CollisionObject(fcl.Sphere(float(r)), fcl.Transform(c)), c-r, c+r)
                       for link, idx, c, r in spheres]
            item['source_hazard_sphere_minimum'] = clearance(objects, hazard, required)
            item['cpu_external_feasible'] = bool(item['position_error_m'] <= .002 and item['rotation_error_rad'] <= .02
                                                and item['conservative_world']['valid'])
        except RuntimeError as exc:
            item['error'] = str(exc)
        report['trials'].append(item)
        a.output.write_text(json.dumps(report, indent=2) + '\n')
        if (i + 1) % 8 == 0:
            print('ENDPOINT_DIAGNOSTIC', i + 1, 'CPU_external_feasible', sum(v.get('cpu_external_feasible', False) for v in report['trials']), flush=True)
    report['complete'] = True
    report['cpu_external_feasible_count'] = sum(v.get('cpu_external_feasible', False) for v in report['trials'])
    a.output.write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
