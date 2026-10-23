from kitchen_safety.extensions.controls.mechanism_monitor import MechanismMonitor


def monitor():
    result = MechanismMonitor.__new__(MechanismMonitor)
    result.initial = [0., 0.]
    result.metadata = [{'path': '/kitchen/knob_joint', 'rotational': True},
                       {'path': '/kitchen/button_joint', 'rotational': False}]
    result.samples = [{'q': [0., .002]}]
    result.missing = []
    return result


def test_only_declared_task_joint_may_move():
    value = monitor()
    assert not value.report()['passed']
    assert value.report(('button_joint',))['passed']
    value.samples[0]['q'][0] = .1
    assert not value.report(('button_joint',))['passed']


def test_missing_runtime_root_is_never_accepted():
    value = monitor()
    value.missing = [{'root': '/kitchen/door'}]
    assert not value.report(('button_joint',))['passed']
