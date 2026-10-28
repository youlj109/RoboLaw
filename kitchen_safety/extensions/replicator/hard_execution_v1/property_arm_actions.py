"""Original position control with a per-substep intended-contact target stop.

Only actuator targets are changed. No state, pose, constraint or source asset
is written. The current action finishes its normal recorded physics substeps.
"""
import torch
from isaaclab.envs.mdp.actions.joint_actions import JointPositionAction


class PropertyContactStopJointAction(JointPositionAction):
    def __init__(self,cfg,env):
        super().__init__(cfg,env)
        self.counter_contact_hold_active=False
        self._counter_hold_target=None
        self._requested_position_target=self._processed_actions.clone()

    def hold_observed_position(self,position):
        target=torch.as_tensor(position,device=self._processed_actions.device,dtype=self._processed_actions.dtype)
        target=target.reshape_as(self._processed_actions)
        if not bool(torch.isfinite(target).all()):raise ValueError('Nonfinite observed arm target')
        self._counter_hold_target=target.clone()
        self.counter_contact_hold_active=True
        self._processed_actions=self._counter_hold_target.clone()

    def release_contact_hold(self):
        self.counter_contact_hold_active=False
        self._counter_hold_target=None

    def process_actions(self,actions):
        super().process_actions(actions)
        self._requested_position_target=self._processed_actions.clone()
        if self.counter_contact_hold_active:self._processed_actions=self._counter_hold_target.clone()

    def apply_actions(self):
        if self.counter_contact_hold_active:self._processed_actions=self._counter_hold_target.clone()
        super().apply_actions()

    def reset(self,env_ids=None):
        super().reset(env_ids)
        if hasattr(self,'counter_contact_hold_active'):self.release_contact_hold()
