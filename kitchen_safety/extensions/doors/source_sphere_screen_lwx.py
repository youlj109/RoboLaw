"""Exact endpoint sphere screening after a conservative source AABB broadphase."""
import numpy as np
import fcl


def source_spheres_safe(hazards, centers, radii, required_m):
    """Return whether all actual source shapes clear all endpoint spheres.

    AABB distance is only a lower bound and cannot establish a violation.
    Close pairs therefore use the source CollisionObject and an exact FCL sphere.
    """
    centers = np.asarray(centers, dtype=float)
    radii = np.asarray(radii, dtype=float)
    for _, source, low, high in hazards:
        lower = np.linalg.norm(
            np.maximum(np.maximum(np.asarray(low) - centers, centers - np.asarray(high)), 0),
            axis=1,
        ) - radii
        for idx in np.flatnonzero(lower <= required_m + 1e-9):
            sphere = fcl.CollisionObject(
                fcl.Sphere(float(radii[idx])), fcl.Transform(centers[idx])
            )
            if fcl.collide(sphere, source, fcl.CollisionRequest(), fcl.CollisionResult()):
                return False
            if fcl.distance(sphere, source, fcl.DistanceRequest(), fcl.DistanceResult()) < required_m:
                return False
    return True
