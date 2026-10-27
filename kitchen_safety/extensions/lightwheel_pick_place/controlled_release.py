"""Continuous release commands through original joint-position drives, no state writes."""
import types
import torch

def enable_continuous_release(term):
    """Keep native open/close targets and drives; interpolate commanded targets only."""
    def process_actions(self, actions):
        self._raw_actions[:]=actions
        fraction=actions.clamp(0.,1.)
        self._processed_actions=self._open_command+fraction*(self._close_command-self._open_command)
        if self.cfg.clip is not None:
            self._processed_actions=torch.clamp(self._processed_actions,min=self._clip[:,:,0],max=self._clip[:,:,1])
    term.process_actions=types.MethodType(process_actions,term)
    return {'mode':'continuous_joint_position_target_during_release_only','open_command':term._open_command.detach().cpu().tolist(),'close_command':term._close_command.detach().cpu().tolist(),'scope':'Original arm and gripper drives, masses, collisions, friction and native predicates unchanged. Original prefix uses original binary action processor. No object/joint state setters during execution.'}
