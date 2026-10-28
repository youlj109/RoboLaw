"""Unintegrated CPU candidate: explicit position targets for a smooth opening.

No production collector imports this. It changes commands, never measured state,
source limits, drive gains, solver settings, or task acceptance. Actual dynamics
must still pass the original joint-bound/source/self gates in a new real run.
"""
from dataclasses import dataclass
import math

INTERFACE = 'droid_arm7_and_gripper1_joint_position_radians_v1'
SOURCE_OPEN_RAD = 0.0
SOURCE_CLOSE_RAD = math.pi / 4
SOURCE_HARD_LIMITS_RAD = (0.0, 0.7853982)
ACTION_DT_S = 1 / 15
PHYSICS_DT_S = 1 / 120
RELEASE_SECONDS = 2.0
RAMP_SECONDS = 1.0


def original_binary_target(value):
    """Equivalent endpoints only; intermediate floats are not binary actions."""
    if not math.isfinite(value) or value not in (0., 1.):
        raise ValueError('Only original binary endpoint actions may be translated')
    return SOURCE_CLOSE_RAD if value == 1 else SOURCE_OPEN_RAD


@dataclass(frozen=True)
class ReleaseSchedule:
    initial_actual_q_rad: float
    initial_actual_dq_rad_s: float
    action_dt_s: float = ACTION_DT_S
    physics_dt_s: float = PHYSICS_DT_S
    release_seconds: float = RELEASE_SECONDS
    ramp_seconds: float = RAMP_SECONDS
    source_open_rad: float = SOURCE_OPEN_RAD
    source_closed_rad: float = SOURCE_CLOSE_RAD

    def __post_init__(self):
        if not (math.isfinite(self.initial_actual_q_rad) and math.isfinite(self.initial_actual_dq_rad_s)):
            raise ValueError('Need actual finite release-boundary q/dq')
        if not SOURCE_HARD_LIMITS_RAD[0] <= self.initial_actual_q_rad <= SOURCE_HARD_LIMITS_RAD[1]:
            raise ValueError('Initial actual q violates original bounds; never clamp it')
        expected=(ACTION_DT_S,PHYSICS_DT_S,RELEASE_SECONDS,RAMP_SECONDS,SOURCE_OPEN_RAD,SOURCE_CLOSE_RAD)
        actual=(self.action_dt_s,self.physics_dt_s,self.release_seconds,self.ramp_seconds,self.source_open_rad,self.source_closed_rad)
        if actual!=expected:
            raise ValueError('Unreviewed schedule/interface/source endpoint change')

    def target_at_action(self, index):
        if type(index) is not int or not 0 <= index < 30:
            raise ValueError('Exactly 30 actions over the unchanged 2 seconds')
        if index >= 14:
            return SOURCE_OPEN_RAD
        s=(index+1) / 15
        # Equivalent to 1-(10s^3-15s^4+6s^5), factored to avoid cancellation
        # close to zero. These are 15 Hz held targets, not a 120 Hz trajectory.
        return self.initial_actual_q_rad * (1-s)**3 * (1+3*s+6*s*s)

    def target_for_physics_ordinal(self, ordinal):
        """Target applied during release substeps 1..240, held for each action."""
        if type(ordinal) is not int or not 1 <= ordinal <= 240:
            raise ValueError('Need an actual release physics ordinal in 1..240')
        return self.target_at_action((ordinal-1)//8)

    def describe(self):
        targets=[self.target_at_action(i) for i in range(30)]
        return dict(candidate_only=True,production_integrated=False,physical_verified=False,accepted=False,
            interface=INTERFACE,initial_actual_q_rad=self.initial_actual_q_rad,
            initial_actual_dq_rad_s=self.initial_actual_dq_rad_s,
            action_dt_s=self.action_dt_s,physics_dt_s=self.physics_dt_s,
            release_seconds=2.,ramp_seconds=1.,actions=30,physics_steps=240,
            open_first_action=14,open_first_applied_physics_ordinal=113,
            targets_rad=targets,maximum_target_delta_rad=max(abs(b-a) for a,b in zip(targets,targets[1:])),
            interpolating_curve_maximum_speed_rad_s=1.875*self.initial_actual_q_rad,
            interpolating_curve_maximum_acceleration_rad_s2=10/math.sqrt(3)*self.initial_actual_q_rad,
            limits='Analytic bounds describe the target curve only. Actual actuator motion, overshoot and source contacts are not predicted. Initial measured velocity is retained, not replaced with zero.',
            integration='Use an explicit local JointPositionActionCfg with scale1 and use_default_offset=False. Record last action dimension in radians and actual applied targets for every physics substep. A 0..1 ramp through the old binary action is forbidden.')
