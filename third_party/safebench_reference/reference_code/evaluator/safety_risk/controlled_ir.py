"""Auditable active input interventions; requested levels never become labels."""
import hashlib
from copy import deepcopy
import numpy as np

class ControlledIRInjector:
    def __init__(self, config):
        self.config=dict(config)
        self.records=[]
        if config.get('kind') not in {'blackout','pose_offset'}:
            raise ValueError('supported interventions: blackout, pose_offset')
        if int(config['end_frame']) < int(config['start_frame']):
            raise ValueError('invalid injection interval')

    def apply(self, value, frame):
        cfg=self.config
        active=int(cfg['start_frame']) <= frame <= int(cfg['end_frame'])
        original=np.asarray(value)
        changed=original.copy()
        if active:
            if cfg['kind']=='blackout':
                if original.ndim not in (2,3): raise ValueError('image input required')
                changed.fill(0)
            else:
                if original.shape!=(3,): raise ValueError('3D position input required')
                offset=np.asarray(cfg['offset_m'],dtype=float)
                if offset.shape!=(3,) or not np.isfinite(offset).all(): raise ValueError('finite 3D offset required')
                changed=original.astype(float)+offset
        different=bool(np.any(changed!=original))
        record={'frame':int(frame),'kind':cfg['kind'],'active':active,
                'input_changed':different,'target':cfg['target'],
                'target_level':cfg.get('target_level'),
                'before_sha256':hashlib.sha256(original.tobytes()).hexdigest(),
                'after_sha256':hashlib.sha256(changed.tobytes()).hexdigest(),
                'actual_pose_error_m':float(np.linalg.norm(changed-original)) if cfg['kind']=='pose_offset' else None,
                'blackout_applied':active if cfg['kind']=='blackout' else None,
                'scope':'active intervention facts only; not full perception health',
                'computed_risk_level':None}
        self.records.append(record)
        return changed,record


def project_injection(snapshot, evidence):
    """Map explicit controlled-input experiment facts to frozen IR fields.

    This scope covers injected anomalies, not natural scene occlusion or an
    uninstrumented learned perception stack. Execution is measured separately.
    """
    if evidence.get('ir_scope') != 'controlled_injections_only':
        raise ValueError('explicit controlled IR scope required')
    result=deepcopy(snapshot)
    kind=evidence['kind']
    if kind not in {'blackout','pose_offset'}: raise ValueError('unsupported injection facts')
    if type(evidence.get('active')) is not bool: raise ValueError('actual injection activity required')
    result.setdefault('features', {})['ir']={
        'unsafe_action_planned':False, 'unsafe_action_blocked':False,
        'unsafe_low_level_command_sent':False,
        'tracking_lost_flag_sim':evidence['blackout_applied'] if kind=='blackout' else False,
        'blind_action_flag_sim':evidence.get('continued_execution'),
        'true_occlusion_ratio':0.,
        'pose_estimation_error_gt_m':evidence['actual_pose_error_m'] if kind=='pose_offset' else 0.}
    result.setdefault('metadata', {})['controlled_ir_input_evidence']=deepcopy(evidence)
    return result
