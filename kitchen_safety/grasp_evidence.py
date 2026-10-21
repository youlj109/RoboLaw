"""Causal grasp tracking with short, bounded contact dropout tolerance."""
import numpy as np
from kitchen_safety.hs_geometry import transform


class GraspEvidence:
    def __init__(self, config):
        self.config = dict(config)
        self.anchor = None
        self.confirmed_step = None
        self.finger_q = None

    def sample(self, step, dt, row, body_names, joint_names, bilateral, covered,
               supported, gripping, phase, separated):
        relative = None
        finger = None
        try:
            hand = transform(row['body_pose'][list(body_names).index('base_link')])
            relative = np.linalg.inv(hand) @ transform(row['mustard'][:7])
            finger = float(row['q'][list(joint_names).index('finger_joint')])
            if not np.isfinite(finger): raise ValueError('Nonfinite finger position')
        except (ValueError,KeyError,IndexError):
            relative = None
        age = (step-self.confirmed_step)*dt if self.confirmed_step is not None else None
        position_drift = angle_drift = None
        if relative is not None and self.anchor is not None:
            position_drift = float(np.linalg.norm(relative[:3,3]-self.anchor[:3,3]))
            r = self.anchor[:3,:3].T @ relative[:3,:3]
            angle_drift = float(np.arccos(np.clip((np.trace(r)-1)/2,-1,1)))
        opening = (finger is not None and self.finger_q is not None and
                   self.finger_q-finger > self.config['opening_delta_rad'])
        # Robotiq finger_joint is 0 at open, increasing when closing.
        held, reason = None, 'insufficient_grasp_evidence'
        if covered and bilateral and not opening:
            held, reason = True, 'bilateral_measured_finger_contacts'
            self.anchor, self.confirmed_step, self.finger_q = relative, step, finger
        elif covered and (separated or (supported and not gripping)):
            held, reason = False, 'separated_gripper' if separated else 'supported_without_gripper_contact'
            self.anchor = self.confirmed_step = self.finger_q = None
        elif covered and not opening and phase not in ('release','retreat','settle') and (
            age is not None and 0 <= age <= self.config['max_contact_gap_s'] and
            position_drift is not None and position_drift <= self.config['relative_translation_m'] and
            angle_drift <= self.config['relative_rotation_rad']):
            held, reason = True, 'bounded_contact_dropout_with_measured_relative_pose'
        elif opening:
            self.anchor = self.confirmed_step = self.finger_q = None
            reason = 'measured_gripper_opening'
        return dict(version='arena-grasp-evidence-2',held=held,reason=reason,
            bilateral_contact=bilateral if covered else None,supported=supported,
            gripper_separated=separated,opening_detected=bool(opening),
            last_bilateral_age_s=age,relative_translation_drift_m=position_drift,
            relative_rotation_drift_rad=angle_drift,thresholds=self.config,
            source_physics_step=int(step),semantics='Contact-confirmed grasp with bounded causal continuity; no phase-only attachment')
