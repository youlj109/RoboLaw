"""Conservative rigid distal envelopes for offline placement discovery only."""


def invariant_envelopes(shapes):
    """Keep the source (7,8) arm capsule and the rigid gripper/finger proxies.

    CollisionWorld.shapes builds six arm envelopes in its documented segment
    order; DoorCollisionWorld renames the proximal one. The final (7,8)
    segment is downstream of the last moving Panda joint. Its transform is
    fixed by a Cartesian endpoint, independently of redundant elbow posture.
    These are conservative planning proxies, never source collision evidence.
    """
    arms = [shape for shape in shapes if shape[0] in ('arm', 'arm_mount_envelope')]
    if len(arms) != 6 or arms[-1][0] != 'arm':
        raise RuntimeError('Source arm envelope topology changed')
    rigid = [shape for shape in shapes if shape[0] in ('gripper', 'finger')]
    if [shape[0] for shape in rigid] != ['gripper', 'finger', 'finger']:
        raise RuntimeError('Source rigid gripper envelope topology changed')
    return [('arm_distal_envelope', *arms[-1][1:]), *rigid]
