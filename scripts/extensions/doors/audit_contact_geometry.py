"""Check measured loaded contact points against moving source handle meshes."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import fcl
from kitchen_safety.extensions.doors.reconstruction import reconstruct, DOOR
from kitchen_safety.planning import transform


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('episode', type=Path)
    a = p.parse_args(); root = a.episode
    output = root / 'contact_geometry_audit.json'
    if output.exists():p.error('Refusing to replace contact evidence')
    m = json.loads((root / 'metadata.json').read_text())
    samples = json.loads((root / 'physics.json').read_text())['samples']
    stage, kin, world = reconstruct(m, m['variant_spec'])
    reference = transform(m['initial_door_pose'][:3], m['initial_door_pose'][3:])
    inverse_reference = np.linalg.inv(reference)
    selected = [o for o in world.objects if '/fridge_door_handle' in o[0]]
    if not selected:raise RuntimeError('No source handle collision shapes')
    sphere = fcl.Sphere(1e-6)
    checked = 0; maximum = 0.; violations = []; worst = None
    for index, row in enumerate(samples):
        contact = row['contacts']
        if not contact['count']:continue
        pose = row['state']['door_body_pose']
        world.move_body(DOOR, transform(pose[:3], pose[3:]) @ inverse_reference)
        for point, force in zip(contact['points_world_m'], contact['normal_forces_n']):
            if abs(force) <= m['review_gates']['contact_force_noise_n']:continue
            probe = fcl.CollisionObject(sphere, fcl.Transform(np.asarray(point)))
            distances = [(max(0., float(fcl.distance(probe, obj, fcl.DistanceRequest(), fcl.DistanceResult()))), name)
                         for name, obj, *_ in selected]
            distance, name = min(distances)
            checked += 1
            value = {'physics_sample': index, 'phase': row['phase'], 'distance_m': distance,
                     'source_handle_shape': name, 'point_world_m': point, 'normal_force_n': force}
            if distance >= maximum:maximum = distance;worst = value
            if distance > m['review_gates']['handle_contact_bounds_padding_m']:
                violations.append(value)
    report = {'passed': checked > 0 and not violations, 'loaded_points_checked': checked,
              'maximum_source_surface_distance_m': maximum, 'worst': worst,
              'tolerance_m': m['review_gates']['handle_contact_bounds_padding_m'],
              'violations': violations, 'source_handle_shapes': [n for n, *_ in selected],
              'physics_sha256': hashlib.sha256((root / 'physics.json').read_bytes()).hexdigest(),
              'scope': 'Measured normal-load contacts at every physics substep, independently localized to source handle collision meshes transformed by actual door poses. This is source-geometry proximity, not identification of a PhysX cooked face or measurement of tangential friction.'}
    output.write_text(json.dumps(report, indent=2));print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
