"""Bounded offline shortcut diagnosis. Never controls or steps simulation."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def metrics(path, kin):
    tcp = np.asarray([kin.fk(q)[:3, 3] for q in path])
    return dict(joint_l2_length_rad=float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()),
                tcp_length_m=float(np.linalg.norm(np.diff(tcp, axis=0), axis=1).sum()))


def shortcut(path, kin, validate, budget=48):
    """Keep endpoints and action times; accept only safe, shorter TCP/joint chords."""
    path = np.asarray(path, dtype=float).copy()
    if path.ndim != 2 or path.shape[1] != 7 or len(path) < 2 or not np.isfinite(path).all():
        raise ValueError('Expected finite N by 7 path')
    original = path.copy()
    trials = []
    proposals = []
    n = len(path) - 1
    for span in sorted(set([n, max(2, 3*n//4), max(2, n//2), max(2, n//4)]), reverse=True):
        starts = sorted(set([*range(0, max(1, n-span+1), max(1, n//8)), max(0, n-span)]))
        proposals.extend((i, min(n, i+span)) for i in starts)
    for i, j in proposals[:budget]:
        old = metrics(path[i:j+1], kin)
        chord = np.linspace(path[i], path[j], j-i+1)
        new = metrics(chord, kin)
        item = dict(start=i, end=j, before=old, proposed=new, accepted=False)
        if new['tcp_length_m'] >= old['tcp_length_m'] - 1e-6 or new['joint_l2_length_rad'] > old['joint_l2_length_rad'] + 1e-9:
            item['reason'] = 'No TCP improvement without extra joint distance'
        else:
            validation = validate(chord)
            item['validation'] = validation
            if (validation.get('valid') and validation.get('validation_complete')
                    and validation.get('fcl_samples_unchecked') == 0):
                path[i:j+1] = chord
                item['accepted'] = True
        trials.append(item)
    if not np.array_equal(original[[0, -1]], path[[0, -1]]):
        raise AssertionError('Endpoint changed')
    return path, trials


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
