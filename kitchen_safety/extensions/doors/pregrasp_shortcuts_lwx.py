"""Deterministic bounded shortcuts; caller must retain full path safety gates."""
import numpy as np

POLICY = dict(version='bounded_chords_v1', max_proposals=48, resolution_rad=.005,
              objective='Reduce TCP length without increasing joint length; same endpoints and action times',
              acceptance='Complete cuRobo and independent FCL checks at original planning margins')

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

