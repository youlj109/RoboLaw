"""Seed offline layout discovery around measured source robot hulls."""
import argparse
import hashlib
import itertools
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from scripts.extensions.doors.physical_hazard_geometry import physical_hazards, physical_robot_shapes


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline', type=Path, required=True)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--asset', type=Path, required=True)
    p.add_argument('--variant', choices=['property', 'robot'], default='property')
    p.add_argument('--stride', type=int, default=4)
    p.add_argument('--hazard-part-centers', action='store_true', help='Also propose contact near each original hazard collider, including narrow bottle necks')
    p.add_argument('--yaws', default='0,90,180,270')
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    evidence = a.output.with_suffix('.provenance.json')
    if a.output.exists() or evidence.exists() or a.stride < 1:
        p.error('Output exists or invalid stride')
    metadata = json.loads((a.baseline / 'metadata.json').read_text())
    rows = json.loads((a.baseline / 'trajectory.json').read_text())
    config = json.loads(a.config.read_text())
    # Same variant template as search_task_hazards: several task configs contain
    # only scene adaptations and deliberately have no property/robot entries.
    template = Path('configs/extensions/doors/front_variants_lwx_final.json')
    variants = json.loads(template.read_text())
    spec = dict(variants[a.variant], asset=str(a.asset), position=[0., 0., 0.], yaw_deg=0.)
    spec.pop('support', None)
    spec.pop('candidate_evidence', None)
    stage, kin, world, _ = reconstruct(metadata, config, spec)
    hazard = physical_hazards(stage, world, spec['name'])
    # The grid aligns bounding-box centers only. Exact source intersection,
    # support, scene legality, immutable endpoints and safety follow in search.
    low = np.min([lo for _, _, lo, _ in hazard], axis=0)
    high = np.max([hi for _, _, _, hi in hazard], axis=0)
    local_center = (low + high) / 2
    local_centers = [(lo+hi)/2 for _, _, lo, hi in hazard] if a.hazard_part_centers else [local_center]
    pre = [i for i, row in enumerate(rows) if row['phase'] == 'pregrasp'][5:-5:a.stride]
    if not pre:
        raise ValueError('No measured interior pregrasp samples')
    values, seen, parts = [], set(), set()
    offsets = sorted(itertools.product([-.08, -.04, 0., .04, .08],
                                      [-.08, -.04, 0., .04, .08], [-.04, 0., .04]),
                     key=lambda x: sum(v * v for v in x))
    for action in pre:
        q = np.asarray(rows[action]['next_obs']['q'])
        fingers = {name: q[metadata['robot_joint_names'].index(name)] for name in world.GRIPPER_JOINT_NAMES}
        for index, (name, _, lo, hi) in enumerate(physical_robot_shapes(world.shapes(q[:7], fingers))):
            parts.add((index, name))
            center = (lo + hi) / 2
            for yaw in map(float, a.yaws.split(',')):
                for local_center in local_centers:
                    origin = center - Rotation.from_euler('z', yaw, degrees=True).apply(local_center)
                    for delta in offsets:
                        value = tuple(np.round(origin + delta, 6)) + (yaw,)
                        if value not in seen:
                            seen.add(value)
                            values.append(value)
    a.output.write_text(json.dumps(values, indent=2) + '\n')
    report = dict(scope='Offline discovery proposals centered near measured physical source hull AABBs. No collision, safety, support, feasibility or acceptance claim.',
                  baseline=str(a.baseline), baseline_sha256=sha(a.baseline / 'trajectory.json'),
                  config=str(a.config), config_sha256=sha(a.config), asset=str(a.asset), asset_sha256=sha(a.asset),
                  implementation_sha256=sha(__file__), template=str(template), template_sha256=sha(template),
                  output=str(a.output), output_sha256=sha(a.output),
                  sampled_actions=pre, source_parts=sorted(parts), candidates=len(values),
                  parameters={k: str(v) if isinstance(v, Path) else v for k, v in vars(a).items()},
                  physics_replay=False, unsafe_path_executed=False)
    evidence.write_text(json.dumps(report, indent=2) + '\n')
    print('GRID', len(values), 'proposals from', len(pre), 'measured actions and', len(parts), 'source parts', flush=True)


if __name__ == '__main__':
    main()
