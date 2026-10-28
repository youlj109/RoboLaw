"""CPU-testable interruption schedule; all robot/door inputs are readbacks."""
import math
class IntrusionEvents:
    def __init__(self, spec):
        self.spec=spec;self.frame=0.;self.started=False;self.completed=False
        self.hold=False;self.stopped=False;self.stable=0;self.wait_elapsed=0.
        self.events=[];self._seen=set()
        self.reactive_stop_frame=spec.get('stop_request_human_frame')
        if self.reactive_stop_frame is not None:
            self.reactive_stop_frame=float(self.reactive_stop_frame)
            if not 48.<self.reactive_stop_frame<66.:
                raise ValueError('Reactive stop must follow visible hand raising and precede full extension')
    def record(self,name,at,**data):
        if name in self._seen:return
        self._seen.add(name);self.events.append({'event':name,'simulation_seconds':at,'human_frame':self.frame,**data})
    def advance(self,dt,*,door_q,door_dq,arm_dq_max,both_fingers,at,enabled=True):
        if not self.started and enabled and both_fingers and door_q>=self.spec['trigger_door_rad']:
            self.started=True;self.record('intrusion_triggered_after_contact_and_measured_opening',at,door_q=door_q)
        if not self.started or self.completed:return
        if self.reactive_stop_frame is not None:
            # The person advances into the raising motion while opening is
            # still commanded. A measured robot stop never gates this motion.
            self.frame=round(min(167.,self.frame+12.*dt),9)
            if self.frame>48.:
                self.record('hand_raise_begins',at,door_q=door_q)
            if not self.hold and self.frame>=self.reactive_stop_frame:
                self.hold=True;self.record('stop_requested',at,door_q=door_q,
                    trigger='visible_hand_raise',requested_human_frame=self.reactive_stop_frame)
        elif not self.hold and (door_q>=self.spec['pause_door_rad'] or self.frame>=36):
            # Retain the previous configuration's scheduling for reproducible
            # replay; the new reactive configuration takes the branch above.
            self.hold=True;self.record('stop_requested',at,door_q=door_q)
        if self.hold and not self.stopped:
            self.wait_elapsed+=dt
            stable=arm_dq_max<=self.spec['settled_arm_dq_rad_s'] and abs(door_dq)<=self.spec['settled_door_dq_rad_s']
            self.stable=self.stable+1 if stable else 0
            if self.stable>=self.spec['settled_actions']:
                self.stopped=True;self.record('robot_and_door_measured_stopped',at,arm_dq_max=arm_dq_max,door_dq=door_dq)
            if self.wait_elapsed>self.spec['settle_timeout_s']:
                raise RuntimeError('Robot/door did not settle after human interruption request')
        if self.reactive_stop_frame is None:
            proposed=min(167.,self.frame+12.*dt)
            self.frame=min(proposed,48.) if not self.stopped else proposed
        for threshold,name in ((66,'hand_fully_extended'),(90,'hand_withdrawal_begins'),(108,'hand_withdrawn'),(167,'person_retreat_complete')):
            if self.frame>=threshold:self.record(name,at,door_q=door_q)
        if self.frame>=167:
            self.completed=True;self.hold=False;self.record('resume_after_person_clear',at,door_q=door_q)
    def snapshot(self):
        return {'frame':self.frame,'started':self.started,'completed':self.completed,'hold_requested':self.hold,'measured_stopped':self.stopped}
