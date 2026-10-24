"""Bounded offline shortcut diagnosis. Never controls or steps simulation."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


from kitchen_safety.extensions.doors.pregrasp_shortcuts_lwx import metrics, shortcut


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline', type=Path, required=True)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--paired', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        p.error('Evidence exists')
    sha = lambda path: hashlib.sha256(Path(path).read_bytes()).hexdigest()
    m = json.loads((a.baseline/'metadata.json').read_text())
    c = json.loads(a.config.read_text())
    paired = json.loads(a.paired.read_text())
    if paired['baseline_sha256'] != sha(a.baseline/'trajectory.json') or paired['config_sha256'] != sha(a.config):
        raise ValueError('Input provenance mismatch')
    if paired['controller_parameters'] != m['controller_parameters']:
        raise ValueError('Input controller mismatch')
    from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
    from kitchen_safety.extensions.doors.task_planner import TaskPlanner
    report = dict(scope='Offline path diagnosis only; no physical success or new controller acceptance claim.',
                  physics_replay=False, unsafe_path_executed=False, complete=False,
                  implementation_sha256=sha(__file__), baseline=str(a.baseline),
                  baseline_sha256=sha(a.baseline/'trajectory.json'), config=str(a.config),
                  config_sha256=sha(a.config), paired=str(a.paired), paired_sha256=sha(a.paired), pairs=[])
    for pair in paired['pairs']:
        out = dict(seed=pair['seed'])
        for mode in ['off', 'on']:
            source = pair[mode]
            if not source['success']:
                raise ValueError('Successful recorded paths required')
            stage, kin, world, _ = reconstruct(m, c, c[paired['variant']] if mode == 'on' else None)
            cp = m['controller_parameters']
            margin = cp.get('ik_limit_margin_rad', 0.)
            kin.limits[:, 0] += margin
            kin.limits[:, 1] -= margin
            planner = TaskPlanner(world, kin, dt=m['dt'], seed=pair['seed'],
                                  max_joint_speed=cp['max_joint_speed'],
                                  hazard_extra_padding_m=cp.get('planner_hazard_extra_padding_m', 0.))
            validate = lambda path: planner.validate_path(path, phase='door_pregrasp', grip_binary=0., resolution=.005)
            original = np.asarray(source['positions'])
            path, trials = shortcut(original, kin, validate)
            out[mode] = dict(before=metrics(original, kin), after=metrics(path, kin),
                             final_validation=validate(path), trials=trials, positions=path.tolist(),
                             peak_joint_speed_rad_s=float(np.abs(np.diff(path, axis=0)).max()/m['dt']),
                             unchanged_endpoints=bool(np.array_equal(original[[0,-1]], path[[0,-1]])))
            print('SHORTCUT', pair['seed'], mode, out[mode]['before'], out[mode]['after'], flush=True)
            del planner, world, stage
            import gc, torch
            gc.collect()
            torch.cuda.empty_cache()
        report['pairs'].append(out)
        a.output.write_text(json.dumps(report, indent=2)+'\n')
    report['complete'] = True
    a.output.write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
