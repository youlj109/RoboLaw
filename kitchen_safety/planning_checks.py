"""Independent planning checks with unchanged dense samples and predicates."""
from collections import deque

import numpy as np


def dense_path_samples(path, resolution=.02):
    """Exactly the samples from calling CollisionWorld.edge on every edge.

    Preserve duplicate endpoints, input dtypes and a+(b-a)*u arithmetic; do
    not substitute path vertices or deduplicate nearly equal coordinates.
    Empty/single-waypoint paths have no edges, as with the original all().
    """
    samples = []
    for a, b in zip(path[:-1], path[1:]):
        steps = max(2, int(np.ceil(np.max(np.abs(b-a))/resolution))+1)
        samples.extend(a+(b-a)*u for u in np.linspace(0, 1, steps))
    return samples


def hierarchical_indices(count):
    if count <= 0:
        return
    yield 0
    if count == 1:
        return
    yield count-1
    pending = deque([(1, count-2)])
    while pending:
        low, high = pending.popleft()
        if low > high:
            continue
        middle = (low+high)//2
        yield middle
        pending.append((middle+1, high))
        pending.append((low, middle-1))


def check_planned_path(world, path, ignore=(), resolution=.02):
    """Reject on any real failing sample; acceptance checks every old sample.

    The world's current attachment/gripper/margins remain untouched. This is
    called after MotionGen returns, so no solver attempts or RNG draws change.
    Failed paths leave the actual failing sample's collision diagnostics.
    """
    samples = dense_path_samples(path, resolution)
    return all(world.valid(samples[i], ignore) for i in hierarchical_indices(len(samples)))
