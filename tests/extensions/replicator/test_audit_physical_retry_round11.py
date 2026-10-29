"""Fixed JS goals cannot substitute a CPU path or a claimed endpoint gate."""
import copy
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from scripts.extensions.replicator import audit_physical_retry_round11 as a


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
    def recipe(self):
        return dict(retry_round=11,retained_scene_revision=True,scene_settings_changed_this_round=False,payload_name='bowl',
            joint_space_target_q_by_phase={'lower':[[0]*7],'retreat':[[0]*7]},hold_orientation_phases=[],linear_motion_phases=[],
            planning_options_by_phase={'lower':{'enable_graph':True},'retreat':{'enable_graph':True}},
            planning_path_policy=dict(kind='collision_checked_graph_joint_goals',previous_straight_line_gate_removed=True,reason_zh='固定目标分支'))

    def test_no_implicit_cartesian_soft_cost_or_graph(self):
        recipe=self.recipe();a.audit_free_path_policy({'phases':[]},recipe)
        for change in ({'hold_orientation_phases':['lower']},{'linear_motion_phases':['lower']},
                       {'planning_options_by_phase':{'lower':{'enable_graph':False},'retreat':{'enable_graph':True}}},
                       {'segment_deviation_limits_m':{'lower':.005}}):
            with self.subTest(change=change),self.assertRaises(ValueError):a.audit_free_path_policy({'phases':[]},dict(recipe,**change))

    def test_mustard_cannot_relax_existing_lift_corridor(self):
        recipe=self.recipe();recipe.update(payload_name='mustard_bottle',joint_space_target_q_by_phase={},hold_orientation_phases=['lift'],linear_motion_phases=['lift'],planning_options_by_phase={'lift':{'constraint_weight':1.,'enable_graph':True}})
        recipe['planning_path_policy']['kind']='collision_checked_lift_corridor'
        start=np.eye(4);end=start.copy();end[2,3]=.03
        recipe['target_poses_world']={'approach':start,'lift':end}
        recipe['lift_path_validation']=dict(reference_phase='approach',max_xy_deviation_m=.008,max_orientation_deviation_deg=4.,min_z_offset_m=-.002)
        a.audit_free_path_policy({'phases':[]},recipe)
        recipe['lift_path_validation']['max_xy_deviation_m']=.02
        with self.assertRaisesRegex(ValueError,'prior lift corridor'):a.audit_free_path_policy({'phases':[]},recipe)

    def test_geometry_may_not_change_with_joint_goal_repair(self):
        old=dict(target_poses_world={'lower':np.eye(4).tolist()},nominal_attachment=np.eye(4).tolist(),phase_waypoints_world={},
                 nominal_gripper_joint_rad=.7,planning_gripper_joint_rad=.2,payload_name='mustard_bottle')
        prior={'scene_id':'s','candidate_planning_result':{'path':'old','sha256':'old'}}
        with patch.object(a,'bound',return_value=Path('plan')),patch.object(a,'read',return_value={'source_previous_retry_manifest':{}}),patch.object(a,'bound_json',side_effect=[{'entries':[prior]},{'recipe':old}]):
            self.assertEqual(a.audit_retained_targets({'scene_id':'s','retry_plan_path':'x','retry_plan_sha256':'y'},copy.deepcopy(old)),prior['candidate_planning_result'])
        bad=copy.deepcopy(old);bad['nominal_gripper_joint_rad']=.6
        with patch.object(a,'bound',return_value=Path('plan')),patch.object(a,'read',return_value={'source_previous_retry_manifest':{}}),patch.object(a,'bound_json',side_effect=[{'entries':[prior]},{'recipe':old}]):
            with self.assertRaisesRegex(ValueError,'geometry/grip'):a.audit_retained_targets({'scene_id':'s','retry_plan_path':'x','retry_plan_sha256':'y'},bad)


class StartupCrashTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name)
        change=patch.object(a,'ROOT',self.root);change.start();self.addCleanup(change.stop)
        self.driver=self.root/'driver.py';self.driver.write_text("print('PHYSICAL_RETRY_START', row['scene_id'])\nresult,path=execution.execute_one(row,args.output,args)\n")
        self.executor=self.root/'executor.py';self.executor.write_text('def execute_one():\n    out.mkdir()\n    write_json(out, result)\n    build_environment()\n')
        self.log=self.root/'crash.log';self.log.write_text("appState = 'startup'\nPHYSICAL_RETRY_START s\nexit code 139\n")
        self.plan={'entries':[{'scene_id':'s','planning_status':'generated'}]};self.planpath=self.write('plan.json',self.plan)
        self.archive={'retry_round':11,'retry_plan_path':'plan.json','retry_plan_sha256':a.digest(self.planpath),
            'entries':[dict(self.plan['entries'][0],execution_status='running',attempts=1,executed=False)],
            'runtime_revisions':[{'source_sha256':{'scripts/extensions/replicator/execute_failure_retry.py':a.digest(self.driver),
                'kitchen_safety/extensions/replicator/retry_physical_execution.py':a.digest(self.executor)}}]}
        self.archivepath=self.write('archive.json',self.archive)
        self.review=dict(schema_version=2,exit_code=139,physical_attempts_consumed=0,scene_execution_started=False,
            pre_resume_manifest_bytes_preserved=True,scene_native_evidence_paths=[],live_driver_pids=[],output_execution_lock_available=True,
            source_plan=a.file_binding(self.planpath),manifest_before_resume=a.file_binding(self.archivepath),
            source_log=a.file_binding(self.log),driver_source=a.file_binding(self.driver),executor_source=a.file_binding(self.executor),
            execution_marker_seen=True,execution_complete_marker_seen=False,
            selected=[dict(self.archive['entries'][0],episode_paths=[])])
        self.reviewpath=self.write('review.json',self.review)

    def write(self,name,value):
        path=self.root/name;path.write_text(json.dumps(value));return path

    def check(self):return a.audit_pre_environment_crash(self.review,self.plan,self.reviewpath)

    def test_running_bookkeeping_and_start_marker_do_not_fake_native_episode(self):
        checked=self.check();self.assertEqual(checked['physical_attempts_consumed'],0)
        self.assertTrue(checked['execution_marker_seen']);self.assertTrue(checked['pre_resume_manifest_digest_revalidated'])

    def test_full_log_cannot_claim_marker_absent(self):
        self.review['execution_marker_seen']=False
        with self.assertRaisesRegex(ValueError,'marker observation'):self.check()

    def test_archived_initial_result_means_restore_not_authorized(self):
        self.archive['entries'][0]['execution_result_path']='started/result.json';self.write('archive.json',self.archive)
        self.review['manifest_before_resume']=a.file_binding(self.archivepath)
        with self.assertRaisesRegex(ValueError,'started scene episode'):self.check()

    def test_source_order_and_runtime_sha_must_support_prebuild_boundary(self):
        self.executor.write_text('def execute_one():\n    build_environment()\n    out.mkdir()\n    write_json(out,result)\n')
        self.review['executor_source']=a.file_binding(self.executor)
        with self.assertRaisesRegex(ValueError,'pre-build evidence'):self.check()


if __name__=='__main__':unittest.main()
