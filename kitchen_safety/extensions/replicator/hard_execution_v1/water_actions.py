"""Local physical actuator-target limiter for the passive-valve sequence.

The original binary 0/pi4 request is preserved. Only its commanded target is
slewed, once per physics step; no joint or object state is written. A verified
grasp freezes the last target until the original binary open command arrives.
The bowl continuation disables this local controller entirely.
"""
import math
import torch
from isaaclab_arena.embodiments.droid.actions import BinaryJointPositionZeroToOneAction


class WaterRateLimitedBinaryAction(BinaryJointPositionZeroToOneAction):
    def __init__(self,cfg,env):
        super().__init__(cfg,env)
        self.valve_control_enabled=False
        self.target_rate_rad_s=.10
        self.release_rate_rad_s=None
        self.grasp_target_held=False
        self._binary_requested_target=self._processed_actions.clone()
        self._previous_applied_target=self._processed_actions.clone()
        self._held_target=None

    def configure_valve(self,enabled,*,rate_rad_s=.10):
        if not math.isfinite(rate_rad_s) or not 0<rate_rad_s<=.10:
            raise ValueError('Water valve target rate must be in (0,.10] rad/s')
        self.valve_control_enabled=bool(enabled)
        self.target_rate_rad_s=float(rate_rad_s)
        self.release_rate_rad_s=None
        self.grasp_target_held=False;self._held_target=None
        self._previous_applied_target=self._processed_actions.clone()

    def configure_release(self,*,rate_rad_s=.05):
        if not self.valve_control_enabled or not math.isfinite(rate_rad_s) or not 0<rate_rad_s<=.10:
            raise ValueError('Slow valve release requires active valve mode and rate in (0,.10] rad/s')
        self.release_rate_rad_s=float(rate_rad_s)

    def hold_verified_grasp(self):
        if not self.valve_control_enabled:
            raise RuntimeError('Cannot hold valve target outside valve control mode')
        self._held_target=self._processed_actions.clone()
        self.grasp_target_held=True

    def process_actions(self,actions):
        # Reuse the exact upstream >.5 binary convention and clipping.
        super().process_actions(actions)
        self._binary_requested_target=self._processed_actions.clone()
        if self.valve_control_enabled:
            self._processed_actions=self._previous_applied_target.clone()

    def apply_actions(self):
        if self.valve_control_enabled:
            closing=self._raw_actions>.5
            if not bool(closing.any()):
                self._held_target=None;self.grasp_target_held=False
            desired=self._binary_requested_target
            if self._held_target is not None:
                desired=torch.where(closing,self._held_target,desired)
            delta=float(self._env.physics_dt)*self.target_rate_rad_s
            limited=self._previous_applied_target+torch.clamp(
                desired-self._previous_applied_target,min=-delta,max=delta)
            opening=desired
            if self.release_rate_rad_s is not None:
                opening_delta=float(self._env.physics_dt)*self.release_rate_rad_s
                opening=self._previous_applied_target+torch.clamp(
                    desired-self._previous_applied_target,min=-opening_delta,max=opening_delta)
            self._processed_actions=torch.where(closing,limited,opening)
        else:
            self._processed_actions=self._binary_requested_target.clone()
        self._previous_applied_target=self._processed_actions.clone()
        super().apply_actions()

    def reset(self,env_ids=None):
        super().reset(env_ids)
        if hasattr(self,'_previous_applied_target'):
            self._previous_applied_target[env_ids]=self._open_command
            self._processed_actions[env_ids]=self._open_command
            self._binary_requested_target[env_ids]=self._open_command
            self._held_target=None;self.grasp_target_held=False
