"""Explicit version routing shared by online inference, CLI and replay."""
from .iso_traceable import Evaluator as AttachmentEvaluator

def create_evaluator(profile, **kwargs):
    version=profile.get('evaluator_family','attachment_literal')
    if version=='attachment_literal':
        return AttachmentEvaluator(profile,**kwargs)
    if version=='ssm_priority_candidate_v1':
        from .iso_ssm_candidate import Evaluator
        return Evaluator(profile,**kwargs)
    raise ValueError('Unknown evaluator_family: '+str(version))
