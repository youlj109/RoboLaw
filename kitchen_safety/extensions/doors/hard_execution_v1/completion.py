"""Additional measured-opening hold gate; native task predicate stays separate."""


class OpeningCompletion:
    def __init__(self, spec):
        self.required_rad = float(spec.get('hard_completion_door_rad', 0.))
        self.hold_actions = int(spec.get('hard_completion_hold_actions', 5))
        self.speed_limit = float(spec.get('hard_completion_door_speed_rad_s', spec['settled_door_dq_rad_s']))
        self.stable_actions = 0
        if self.required_rad < 0 or self.hold_actions < 1 or self.speed_limit <= 0:
            raise ValueError('Invalid measured opening completion requirement')

    def reached(self, door_q, native_open, events_complete):
        return bool(native_open and events_complete and door_q >= self.required_rad)

    def observe(self, *, door_q, door_dq, native_open, events_complete):
        stable = self.reached(door_q, native_open, events_complete) and abs(door_dq) <= self.speed_limit
        self.stable_actions = self.stable_actions + 1 if stable else 0
        return self.complete

    @property
    def complete(self):
        return self.stable_actions >= self.hold_actions

    def snapshot(self):
        return {'required_door_rad': self.required_rad, 'required_stable_actions': self.hold_actions,
                'door_speed_limit_rad_s': self.speed_limit, 'consecutive_stable_actions': self.stable_actions,
                'measured_opening_and_hold_completed': self.complete}
