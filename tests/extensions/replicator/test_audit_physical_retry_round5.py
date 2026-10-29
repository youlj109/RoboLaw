"""Bound-history and target regressions for the CPU-only round5 auditor."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from scripts.extensions.replicator import audit_physical_retry_round5 as audit
from scripts.extensions.replicator import audit_physical_retry_round2 as old
from tests.extensions.replicator.test_audit_physical_retry_round2 import HistoryTests as PriorHistoryTests


class Round5HistoryTests(PriorHistoryTests):
    def setUp(self):
        super().setUp()
        change=patch.object(audit,'ROOT',self.root);change.start();self.addCleanup(change.stop)
        # The fixture's prior retry is the real round1 failure.
        self.round1=copy.deepcopy(self.previous)
        rp=self.root/self.round1['execution_result_path'];result=json.loads(rp.read_text())
        result.update(runtime_state_setters_used=False,physical_weld_used=False,payload_attachment_used=False,
                      planning_rerun=False,rendering_enabled=False,render_generation_delta=0,
                      source_inputs_unchanged=True,execution_sources_unchanged=True)
        self.round1.update(self.write_result('round1/result.json',result))
        self.history=[audit.prior_binding(self.original),audit.prior_binding(self.round1)]
        candidate={'status':'failed','trajectory_complete':False,'executed':False,'attempts':1}
        sha=self.write_text('round2-candidate.json',json.dumps(candidate))
        self.previous=dict(self.round1,execution_status='planning_failed',planning_status='failed',attempts=0,
            executed=False,physical_attempt_number=3,prior_physical_attempts=self.history,
            candidate_planning_result={'path':'round2-candidate.json','sha256':sha},
            execution_result_path=None,execution_result_sha256=None)
        self.row=dict(self.previous,physical_attempt_number=3,prior_physical_attempts=self.history,
            previous_retry_result={'path':self.round1['execution_result_path'],'sha256':self.round1['execution_result_sha256']})
        self.context=dict(original={'scene':self.original},retakes={},round1={'scene':self.round1},
            previous={'scene':self.previous},round1_manifest={'retry_plan_path':'round1-plan.json','retry_plan_sha256':'round1-plan'},
            previous_manifest={'retry_plan_path':'round4-plan.json','retry_plan_sha256':'round4-plan'})
        self.context['prior_rounds']=[(self.context['round1_manifest'],{'scene':self.round1}),
            ({'retry_plan_path':'round2-plan.json','retry_plan_sha256':'round2-plan'},{'scene':copy.deepcopy(self.previous)}),
            ({'retry_plan_path':'round3-plan.json','retry_plan_sha256':'round3-plan'},{'scene':copy.deepcopy(self.previous)}),
            (self.context['previous_manifest'],self.context['previous'])]

    def test_prior_retry_can_have_its_own_retimed_trajectory(self):
        results=audit.audit_history(self.row,self.row,self.context)
        self.assertEqual(len(results),2)
        self.assertNotEqual(results[0]['trajectory_sha256'],results[1]['trajectory_sha256'])

    def test_missing_round1_history_or_tampered_history_rejected(self):
        bad=copy.deepcopy(self.row);bad['prior_physical_attempts']=bad['prior_physical_attempts'][:1]
        with self.assertRaisesRegex(ValueError,'history differs'):audit.audit_history(bad,self.row,self.context)
        (self.root/'round1/result.json').write_text('{}')
        with self.assertRaisesRegex(ValueError,'SHA mismatch'):audit.audit_history(self.row,self.row,self.context)

    def test_previous_planning_failure_does_not_increment_actual_number(self):
        self.assertEqual(len(audit.audit_history(self.row,self.row,self.context)),2)
        bad=dict(self.row,physical_attempt_number=4)
        with self.assertRaisesRegex(ValueError,'counts planning failure'):audit.audit_history(bad,bad,self.context)

    def test_previous_failed_plan_with_episode_is_rejected(self):
        self.previous['execution_result_path']='invented.json'
        with self.assertRaisesRegex(ValueError,'invented'):audit.audit_history(self.row,self.row,self.context)

    def test_actual_round4_after_three_failed_plans_uses_new_candidate(self):
        candidate={'status':'generated','trajectory_complete':True,'executed':False,'attempts':1}
        sha=self.write_text('round4-candidate.json',json.dumps(candidate))
        previous=dict(self.round1,retry_schema_version=2,physical_attempt_number=3,prior_physical_attempts=self.history,
            retry_plan_path='round4-plan.json',retry_plan_sha256='round4-plan',
            candidate_planning_result={'path':'round4-candidate.json','sha256':sha},
            trajectory_path='round4.npz',trajectory_sha256=self.write_text('round4.npz','newgeometry'))
        result=json.loads((self.root/self.round1['execution_result_path']).read_text())
        result.update(previous);result.update(planning_result_path='round4-candidate.json',planning_result_sha256=sha,
            original_planning_result={'path':'old-plan.json','sha256':'old-plan'})
        previous.update(self.write_result('round4/result.json',result));self.context['previous']['scene']=previous
        row=dict(previous,physical_attempt_number=4,prior_physical_attempts=self.history+[audit.prior_binding(previous)],
            previous_retry_result={'path':previous['execution_result_path'],'sha256':previous['execution_result_sha256']})
        checked=audit.audit_history(row,row,self.context);self.assertEqual(len(checked),3)
        self.assertEqual(checked[-1]['planning_result_path'],'round4-candidate.json')
        result['planning_result_path']='old-plan.json'
        previous.update(self.write_result('round4/result.json',result))
        row.update(prior_physical_attempts=self.history+[audit.prior_binding(previous)],
            previous_retry_result={'path':previous['execution_result_path'],'sha256':previous['execution_result_sha256']})
        with self.assertRaisesRegex(ValueError,'actual planning source'):audit.audit_history(row,row,self.context)


class CandidateTests(unittest.TestCase):
    def fixture(self):
        poses={p:np.eye(4).tolist() for p in audit.PHASES}
        original={'targets':poses,'nominal_attachment':np.eye(4).tolist(),'payload_name':'banana'}
        declared={'target_poses_world':copy.deepcopy(poses),'nominal_attachment':np.eye(4).tolist(),
                  'payload_name':'banana','linear_motion_phases':['lower']}
        candidate={'targets':copy.deepcopy(poses),'nominal_attachment':np.eye(4).tolist(),'payload_name':'banana'}
        return candidate,declared,original

    def test_frozen_targets_and_nominal_attachment(self):
        args=self.fixture();audit.audit_candidate_targets(*args)
        args[0]['targets']['lower'][0][3]=.001
        with self.assertRaisesRegex(ValueError,'changed frozen target'):audit.audit_candidate_targets(*args)
        args=self.fixture();args[0]['nominal_attachment'][0][3]=.001;args[1]['nominal_attachment'][0][3]=.001
        with self.assertRaisesRegex(ValueError,'preserve original'):audit.audit_candidate_targets(*args)

    def test_release_bound_and_rigid_transform(self):
        args=self.fixture();args[0]['targets']['lower'][0][3]=.061;args[1]['target_poses_world']['lower'][0][3]=.061
        with self.assertRaisesRegex(ValueError,'60mm'):audit.audit_candidate_targets(*args)
        args=self.fixture();args[1]['target_poses_world']['lower'][0][0]=2
        with self.assertRaisesRegex(ValueError,'Invalid frozen'):audit.audit_candidate_targets(*args)

    def test_vertical_record_is_checked_but_not_a_hard_path_acceptance(self):
        candidate,_,_=self.fixture();poses=np.repeat(np.eye(4)[None],3,axis=0)
        poses[1,0,3]=.02
        candidate['phases']=[dict(phase='lower',vertical_motion_cost_requested=True,samples=3,
            max_tcp_xy_goal_deviation_m=.02,vertical_motion_scope='Soft cost, not a physical path guarantee')]
        raw={'tcp_world':poses,'phase':np.array(['lower']*3)}
        audit.audit_vertical_cost_record(candidate,raw)
        candidate['phases'][0]['max_tcp_xy_goal_deviation_m']=0
        with self.assertRaisesRegex(ValueError,'deviation differs'):audit.audit_vertical_cost_record(candidate,raw)
        poses[0,0,3]=.005
        with self.assertRaisesRegex(ValueError,'aligned'):audit.audit_vertical_cost_record(candidate,raw)

    def test_numerical_checker_is_unchanged_prior_module(self):
        for name in ('audit_retiming','expected_command_stream','predicate_components','audit_terminal_order'):
            self.assertIs(getattr(audit,name),getattr(old,name))

    def test_current_failed_plan_never_creates_episode(self):
        row=dict(scene_id='scene',task_id='task',execution_status='planning_failed',planning_status='failed',
                 attempts=0,executed=False,task_success=False,physical_attempt_number=4,
                 candidate_planning_result={},previous_retry_result={})
        with tempfile.TemporaryDirectory() as folder,patch.object(audit,'audit_history'),patch.object(audit,'audit_candidate',return_value={'failure_reason':'IK_FAIL'}):
            result=audit.audit_planning_failure(row,{},Path(folder),{})
            self.assertEqual(result['attempts'],0);self.assertIsNone(result['physical_attempt_number'])
            with self.assertRaisesRegex(ValueError,'consumed'):
                audit.audit_planning_failure(dict(row,attempts=1),{},Path(folder),{})

    def test_episode_inventory_rejects_orphaned_extra_trial(self):
        with tempfile.TemporaryDirectory() as folder,patch.object(audit,'ROOT',Path(folder)):
            root=Path(folder);episode=root/'demos/executions/scene';episode.mkdir(parents=True)
            for name in ('result.json','physics_states.npz','physics_metadata.json','actions.json'):
                (episode/name).write_text('fixture')
            manifest={'entries':[{'execution_result_path':'demos/executions/scene/result.json'}]}
            audit.audit_episode_inventory(manifest,root/'demos')
            extra=root/'demos/executions/scene/unrecorded';extra.mkdir();(extra/'actions.json').write_text('another trial')
            with self.assertRaisesRegex(ValueError,'orphaned'):
                audit.audit_episode_inventory(manifest,root/'demos')

    def test_startup_crash_observation_is_not_a_fabricated_snapshot(self):
        with tempfile.TemporaryDirectory() as folder,patch.object(audit,'ROOT',Path(folder)):
            root=Path(folder);reports=root/'reports/replicator_physical_retry_round5_20261029';reports.mkdir(parents=True)
            log=root/'crash.log';log.write_text("appState = 'startup'\nexit code 139\n")
            driver=root/'driver.py';driver.write_text('source')
            review=dict(kind='pre_episode_process_startup_failure_review',exit_code=139,physical_attempts_consumed=0,
                runtime_modified=False,source_log=audit.file_binding(log),driver_source=audit.file_binding(driver),
                selected=[dict(scene_id='scene',execution_status='pending',attempts=0,executed=False,episode_paths=[])],
                manifest_before_resume={'path':'live-manifest.json','sha256':'historically-observed-not-archived'})
            path=reports/'pre_episode_startup_failure_review.json';path.write_text(json.dumps(review))
            plan={'entries':[dict(scene_id='scene',planning_status='generated')]}
            checked=audit.audit_startup_failure_reviews(plan)
            self.assertFalse(checked[0]['pre_resume_manifest_digest_revalidated'])
            self.assertFalse(checked[0]['pre_resume_manifest_bytes_preserved'])
            log.write_text(log.read_text()+'PHYSICAL_RETRY_START scene\n')
            review['source_log']=audit.file_binding(log);path.write_text(json.dumps(review))
            with self.assertRaisesRegex(ValueError,'before any scene'):
                audit.audit_startup_failure_reviews(plan)




class OptionsTests(unittest.TestCase):
    def fixture(self):
        declared=dict(planning_options_by_phase={'lift':dict(constraint_weight=20,enable_graph=False)})
        detail=dict(phase='lift',planning_options=copy.deepcopy(declared['planning_options_by_phase']['lift']),
            planner_seed_reset='all_including_finetune',graph_enabled=False,graph_fallback_enabled=False,
            orientation_hold_requested=True,vertical_motion_cost_requested=False,
            pose_metric_hold_weights=[20.,20.,20.,0.,0.,0.],start_world=np.eye(4).tolist(),target_world=np.eye(4).tolist())
        return dict(phases=[dict(detail,segments=[copy.deepcopy(detail)])]),declared

    def test_frozen_weight_graph_and_all_seed_metadata(self):
        c,d=self.fixture();self.assertEqual(audit.audit_planning_options(c,d),2)
        for key,value in [('planning_options',{'constraint_weight':1,'enable_graph':False}),
                          ('planner_seed_reset','main_only'),('graph_enabled',True),('graph_fallback_enabled',True),
                          ('pose_metric_hold_weights',[1,1,1,0,0,0]),('orientation_hold_requested',False)]:
            c,d=self.fixture();c['phases'][0]['segments'][0][key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):audit.audit_planning_options(c,d)

    def test_invalid_or_undeclared_options_rejected(self):
        for option in [{'constraint_weight':True},{'constraint_weight':.9},{'constraint_weight':51},
                       {'constraint_weight':float('nan')},{'enable_graph':1},{'max_attempts':2}]:
            c,d=self.fixture();d['planning_options_by_phase']['lift']=option
            with self.subTest(option=option),self.assertRaises(ValueError):audit.audit_planning_options(c,d)
        c,d=self.fixture();d['planning_options_by_phase']={}
        with self.assertRaises(ValueError):audit.audit_planning_options(c,d)

    def test_vertical_weights_include_base_xy_constraints(self):
        c,d=self.fixture();d['linear_motion_phases']=['lift']
        for detail in [c['phases'][0],*c['phases'][0]['segments']]:
            detail.update(vertical_motion_cost_requested=True,pose_metric_hold_weights=[20,20,20,20,20,0])
        audit.audit_planning_options(c,d)
        c['phases'][0]['segments'][0]['pose_metric_hold_weights']=[20,20,20,0,0,0]
        with self.assertRaisesRegex(ValueError,'weights'):audit.audit_planning_options(c,d)

    def test_stale_compensation_never_authorizes_generated_lower_or_executed(self):
        candidate=dict(scene_id='u_shape_bowl_sink__human_default_mpfb',status='failed',failed_phase='lift',
            executed=False,trajectory_complete=False,phases=[{'phase':'lift'}])
        row=dict(execution_status='planning_failed',planning_status='failed',executed=False,attempts=0,trajectory_path=None)
        cases=[(dict(candidate,status='generated',trajectory_complete=True),row),
               (dict(candidate,failed_phase='lower',phases=[{'phase':'lower'}]),row),
               (candidate,dict(row,executed=True,attempts=1)),(candidate,dict(row,execution_result_path='actual.json'))]
        with patch.object(audit.numeric,'audit_destination_compensation',side_effect=ValueError('Predicted actual-offset bowl pose differs')):
            for c,r in cases:
                with self.subTest(c=c,r=r),self.assertRaisesRegex(ValueError,'cannot authorize'):
                    audit.audit_destination_compensation(c,{},np.eye(4),r)

if __name__=='__main__':unittest.main()
