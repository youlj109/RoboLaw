"""Check actual robot hulls and the measured moving door against the source stand."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from kitchen_safety.extensions.doors.reconstruction import reconstruct, DOOR
from kitchen_safety.planning import transform
from scripts.extensions.doors.physical_hazard_geometry import physical_robot_shapes
from scripts.extensions.doors.search_avoidance_layouts import clearance


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('episode', type=Path)
    a = p.parse_args()
    output = a.episode / 'stand_audit.json'
    if output.exists():
        p.error('Refusing to overwrite evidence')
    m = json.loads((a.episode / 'metadata.json').read_text())
    records = json.loads((a.episode / 'physics.json').read_text())['samples']
    stage, kin, world = reconstruct(m)
    stand = [o for o in world.objects if '/stand_instanceable/' in o[0]]
    reference = transform(m['initial_door_pose'][:3], m['initial_door_pose'][3:])
    minima = {}; violations = []
    for i, row in enumerate(records):
        state = row['state']
        q = np.asarray(state['q'])
        joints = dict(zip(m['robot_joint_names'], q))
        fingers = {n: joints[n] for n in world.GRIPPER_JOINT_NAMES}
        robot = physical_robot_shapes(world.shapes(q[:7], fingers))
        pose = state['door_body_pose']
        world.move_body(DOOR, transform(pose[:3], pose[3:]) @ np.linalg.inv(reference))
        door = [o for o in world.objects if o[0].startswith(DOOR + '/')]
        if not robot or not door or not stand:
            raise RuntimeError('Incomplete source geometry')
        for name, shapes in [('robot', robot), ('door', door)]:
            value = clearance(shapes, stand, .002, True)
            measured = {'physics_sample': i, 'phase': row['phase'], **value}
            if name not in minima or value['slack_m'] < minima[name]['slack_m']:
                minima[name] = measured
            if not value['valid']:
                violations.append({'body': name, **measured})
    report = {'passed': not violations, 'samples': len(records),
              'minimum': minima, 'violations': violations,
              'margin_m': .002, 'stand_colliders': [o[0] for o in stand],
              'physics_sha256': hashlib.sha256((a.episode / 'physics.json').read_bytes()).hexdigest(),
              'scope': 'All recorded 120Hz physics states from post-reset through released terminal; actual source robot hulls and articulated gripper meshes plus measured moving door versus source fixed stand. The nonphysical proximal arm capsule is excluded only from its own mounting stand; real meshes retain the original 2mm clearance. Triangle/hull models are not asserted identical to PhysX cooked shapes.'}
    output.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
