"""Task-local pre-execution gate for the unchanged ordered handle candidates."""
from copy import deepcopy

POLICY = {"version": "dense_pregrasp_v4", "resolution_rad": 0.005,
          "checks": "cuRobo constraints and independent FCL planning clearances at all dense samples"}


def check_pregrasp(planner, trajectory):
    """Reject before any action; sampling certifies sampled geometry, not physics."""
    report = planner.validate_path(trajectory['positions'], phase='door_pregrasp',
                                   grip_binary=0., resolution=POLICY['resolution_rad'])
    report = deepcopy(report)
    report['policy'] = deepcopy(POLICY)
    report['accepted'] = bool(report['valid'] and report['validation_complete']
                              and report['fcl_samples_unchecked'] == 0)
    return report
