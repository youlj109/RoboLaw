"""Fixed JS goals cannot substitute a CPU path or a claimed endpoint gate."""
import copy
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from scripts.extensions.replicator import audit_physical_retry_round13 as a


class FK:
    source={}
    limits=np.array([[-2.,2.]]*7)
    def fk(self,q):
        t=np.eye(4);t[:3,3]=q[:3];return t


class EndpointTests(unittest.TestCase):
    def test_all_three_actual_endpoint_constraints(self):
        goal=np.zeros(7);target=np.eye(4)
        self.assertTrue(a.independent_joint_endpoint(goal,target,goal,target)['joint_goal_endpoint_gate_passed'])
        for kind in ('joint','position','rotation'):
            q=goal.copy();actual=target.copy()
            if kind=='joint':q[6]=.0011
            elif kind=='position':actual[0,3]=.0021
            else:
                theta=.021;actual[:3,:3]=[[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]]
            with self.subTest(kind=kind):self.assertFalse(a.independent_joint_endpoint(q,actual,goal,target)['joint_goal_endpoint_gate_passed'])

    def test_saved_gate_cannot_forge_returned_endpoint(self):
        computed=a.independent_joint_endpoint(np.array([.002]+[0]*6),np.eye(4),np.zeros(7),np.eye(4))
        with self.assertRaisesRegex(ValueError,'acceptance differs'):a.verify_joint_endpoint(dict(computed,joint_goal_endpoint_gate_passed=True),computed)
        with self.assertRaisesRegex(ValueError,'metric differs'):a.verify_joint_endpoint(dict(computed,joint_space_endpoint_error_rad=0),computed)

    def test_unattempted_frozen_joint_goal_must_match_source_fk(self):
        pose=np.eye(4).tolist();recipe={'joint_space_target_q_by_phase':{'lower':[[0]*7]},'planning_options_by_phase':{'lower':{'enable_graph':True}},'hold_orientation_phases':[]}
        candidate={'phases':[],'targets':{'lower':pose}}
        checked=a.audit_joint_space_goals(candidate,recipe,{},FK());self.assertFalse(checked[0]['solver_call_attempted'])
        recipe['joint_space_target_q_by_phase']['lower'][0][0]=.01
        with self.assertRaisesRegex(ValueError,'misses declared'):a.audit_joint_space_goals(candidate,recipe,{},FK())

    def test_returned_segment_rejection_and_no_executable_prefix(self):
        with tempfile.TemporaryDirectory() as folder,patch.object(a,'ROOT',Path(folder)):
            root=Path(folder);(root/'scene').mkdir();kin=FK();goal=np.zeros(7);goal[2]=.1;target=kin.fk(goal)
            joints=np.array([np.zeros(7),goal/2,goal]);joints[-1,0]=.002
            poses=np.array([kin.fk(q) for q in joints]);file=root/'scene/lower.npz';np.savez(file,q=joints,tcp_world=poses)
            check=a.independent_joint_endpoint(joints[-1],poses[-1],goal,target)
            segment=dict(segment_index=0,attempts=1,max_attempts=1,target_world=target.tolist(),start_world=np.eye(4).tolist(),start_q=[0]*7,
                contact_masks=[],nominal_payload_attached=True,vertical_motion_cost_requested=False,samples=3,phase_sample_start=0,phase_sample_stop=3,
                solver_success=True,solver_status='JOINT_GOAL_ENDPOINT_LIMIT',solver_status_before_gate='SUCCESS',status='failed',
                trajectory={'path':'scene/lower.npz','sha256':a.digest(file)},deviation_limit_m=None,deviation_gate_passed=True,
                max_tcp_segment_deviation_m=a.segment_deviation(poses[:,:3,3],np.zeros(3),target[:3,3]),**check)
            parent=dict(segment,phase='lower',segments=[segment],failed_segment_index=0,samples=0,segment_count_declared=1,segment_count_attempted=1,per_segment_max_attempts=1)
            candidate=dict(start_q=[0]*7,status='failed',failed_phase='lower',phases=[parent],targets={'lower':target},artifacts={})
            recipe={'joint_space_target_q_by_phase':{'lower':[goal.tolist()]}}
            row={'candidate_planning_result':{'path':'scene/result.json'}}
            checked=a.audit_segment_paths(candidate,recipe,row,kin)
            self.assertFalse(checked['segments'][0]['joint_goal_endpoint_audit']['joint_goal_endpoint_gate_passed'])
            segment['joint_goal_endpoint_gate_passed']=True
            with self.assertRaisesRegex(ValueError,'acceptance differs'):a.audit_segment_paths(candidate,recipe,row,kin)


class PolicyTests(unittest.TestCase):
    def previous(self):
        def pose(z):
            t=np.eye(4);t[2,3]=z;return t.tolist()
        recipe=dict(scene_id='u_shape_bowl_sink__human_default_mpfb',retry_round=12,retained_scene_revision=True,scene_settings_changed_this_round=False,payload_name='bowl',
            target_poses_world={p:pose(0) for p in a.PHASES},nominal_attachment=pose(0),nominal_gripper_joint_rad=.7,planning_gripper_joint_rad=.2,
            phase_waypoints_world={'lower':[pose(.05),pose(.025)],'retreat':[pose(.1)]},
            joint_space_target_q_by_phase={'lower':[[0,0,.05,0,0,0,0],[0,0,.025,0,0,0,0],[0]*7],'retreat':[[0,0,.1,0,0,0,0],[0]*7]},hold_orientation_phases=[],linear_motion_phases=[],
            planning_options_by_phase={'lower':{'enable_graph':True},'retreat':{'enable_graph':True}},
            planning_path_policy=dict(kind='collision_checked_graph_joint_goals_with_carry_waypoints',previous_straight_line_gate_removed=True,reason_zh='携带路标'))
        segments=[dict(status='generated',solver_success=True,joint_goal_endpoint_gate_passed=True) for _ in range(2)]+[dict(status='failed',solver_success=False)]
        return dict(recipe=recipe,status='failed',failed_phase='lower',phases=[dict(phase='lower',failed_segment_index=2,segments=segments)])

    def revised(self,old):
        r=copy.deepcopy(old['recipe']);r['retry_round']=13;r['planning_path_policy']['kind']='collision_checked_graph_joint_goals_raised_release'
        r['target_poses_world']['lower']=r['phase_waypoints_world']['lower'].pop();r['joint_space_target_q_by_phase']['lower'].pop();return r

    def check(self,old,new):
        prior={'scene_id':'s','candidate_planning_result':{'path':'old','sha256':'old'}}
        with patch.object(a,'bound',return_value=Path('plan')),patch.object(a,'read',return_value={'source_previous_retry_manifest':{}}),patch.object(a,'bound_json',side_effect=[{'entries':[prior]},old]):
            return a.audit_retained_targets({'scene_id':'s','retry_plan_path':'x','retry_plan_sha256':'y'},new)

    def test_only_previous_last_accepted_waypoint_can_be_the_new_release(self):
        old=self.previous();new=self.revised(old);a.audit_free_path_policy({'phases':[]},new);self.check(old,new)
        for what in ('lower','transfer','grip','waypoint','q','retreat','holds'):
            bad=copy.deepcopy(new)
            if what in ('lower','transfer'):bad['target_poses_world'][what][0][3]=.01
            elif what=='grip':bad['nominal_gripper_joint_rad']=.6
            elif what=='waypoint':bad['phase_waypoints_world']['lower'][0][0][3]=.01
            elif what=='q':bad['joint_space_target_q_by_phase']['lower'][-1][0]=.01
            elif what=='retreat':bad['joint_space_target_q_by_phase']['retreat'][0][0]=.01
            else:bad['execution_overrides']={'close_seconds':2}
            with self.subTest(what=what),self.assertRaises(ValueError):self.check(old,bad)

    def test_previous_endpoint_must_actually_have_been_accepted(self):
        old=self.previous();new=self.revised(old)
        for what in ('solver','endpoint','wrong_failure','not_last'):
            bad=copy.deepcopy(old)
            if what=='solver':bad['phases'][0]['segments'][-2]['solver_success']=False
            elif what=='endpoint':bad['phases'][0]['segments'][-2]['joint_goal_endpoint_gate_passed']=False
            elif what=='wrong_failure':bad['failed_phase']='retreat'
            else:bad['phases'][0]['failed_segment_index']=0
            with self.subTest(what=what),self.assertRaisesRegex(ValueError,'final accepted'):self.check(bad,new)

    def test_25mm_vertical_raise_not_arbitrary_target_shift(self):
        old=self.previous();old['recipe']['phase_waypoints_world']['lower'][-1][2][3]=.03
        new=self.revised(old)
        with self.assertRaisesRegex(ValueError,'exactly 25mm'):self.check(old,new)

    def test_no_new_pose_cost_or_path_gate(self):
        recipe=self.revised(self.previous())
        for change in ({'hold_orientation_phases':['lower']},{'linear_motion_phases':['lower']},
                       {'planning_options_by_phase':{'lower':{'enable_graph':False},'retreat':{'enable_graph':True}}},
                       {'segment_deviation_limits_m':{'lower':.005}}):
            with self.subTest(change=change),self.assertRaises(ValueError):a.audit_free_path_policy({'phases':[]},dict(recipe,**change))


if __name__=='__main__':unittest.main()
