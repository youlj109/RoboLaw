"""Independent round9 geometry gates do not invent failed paths or episodes."""
import copy
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from scripts.extensions.replicator import audit_physical_retry_round9 as a


class LiftGateTests(unittest.TestCase):
    def setUp(self):
        self.limits=dict(reference_phase='approach',max_xy_deviation_m=.008,max_orientation_deviation_deg=4.,min_z_offset_m=-.002)
        self.reference=np.eye(4);self.reference[:3,3]=[1.,2.,3.]
        self.poses=np.repeat(self.reference[None],3,axis=0);self.poses[:,2,3]+=[0.,.01,.03]

    def test_gate_uses_actual_approach_frame_and_world_xy(self):
        gate=a.independent_lift_gate(self.poses,self.reference,self.limits)
        self.assertTrue(gate['passed']);self.assertEqual(gate['source_samples'],3)
        self.assertAlmostEqual(gate['final_z_offset_m'],.03)
        a.verify_lift_gate_record(gate,gate)
        forged=copy.deepcopy(gate);forged['reference_world'][0][3]+=.03
        with self.assertRaisesRegex(ValueError,'actual approach'):a.verify_lift_gate_record(forged,gate)

    def test_each_corridor_limit_independently_rejects(self):
        for field in ('xy','angle','z'):
            poses=self.poses.copy()
            if field=='xy':poses[1,0,3]+=.009
            elif field=='z':poses[1,2,3]=self.reference[2,3]-.003
            else:
                angle=np.deg2rad(5);poses[1,:3,:3]=[[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]]
            with self.subTest(field=field):self.assertFalse(a.independent_lift_gate(poses,self.reference,self.limits)['passed'])

    def test_saved_pass_cannot_override_independent_rejection(self):
        self.poses[1,0,3]+=.009;gate=a.independent_lift_gate(self.poses,self.reference,self.limits)
        forged=dict(gate,passed=True)
        with self.assertRaisesRegex(ValueError,'outcome differ'):a.verify_lift_gate_record(forged,gate)
        forged=copy.deepcopy(gate);forged['thresholds']['max_xy_deviation_m']=.02
        with self.assertRaisesRegex(ValueError,'limits/count'):a.verify_lift_gate_record(forged,gate)

    def test_fk_recomputed_for_interpolated_retiming(self):
        class FK:
            def fk(self,q):
                t=np.eye(4);t[:3,3]=q[:3];return t
        q=np.zeros((3,7));q[:,2]=[0,.03,.06];tcp=np.array([FK().fk(v) for v in q])
        data=dict(q=q,tcp_world=tcp,phase=np.array(['approach','lift','lift']))
        recipe={'lift_path_validation':self.limits}
        self.assertTrue(a.audit_array_lift_gate(data,recipe,FK())['passed'])
        data['tcp_world'][1,0,3]=.1
        with self.assertRaisesRegex(ValueError,'source FK'):a.audit_array_lift_gate(data,recipe,FK())

    def test_zero_samples_are_not_a_passing_gate(self):
        with self.assertRaisesRegex(ValueError,'Invalid lift'):a.independent_lift_gate(np.empty((0,4,4)),self.reference,self.limits)


class PolicyTests(unittest.TestCase):
    def recipe(self):
        return dict(retry_round=9,retained_scene_revision=True,scene_settings_changed_this_round=False,
            planning_path_policy=dict(kind='collision_checked_free_path',previous_straight_line_gate_removed=True,reason_zh='继续自由绕行'),
            hold_orientation_phases=[],linear_motion_phases=[])

    def test_free_path_retains_revision_and_rejects_old_global_gate(self):
        recipe=self.recipe();a.audit_free_path_policy({'phases':[]},recipe)
        for change in ({'retained_scene_revision':False},{'segment_deviation_limits_m':{'lower':.005}},{'joint_space_target_q_by_phase':{'lower':[[0]*7]}}):
            with self.subTest(change=change),self.assertRaises(ValueError):a.audit_free_path_policy({'phases':[]},dict(recipe,**change))

    def test_lift_only_gate_accepts_declared_soft_options(self):
        recipe=self.recipe();approach=np.eye(4);lift=approach.copy();lift[2,3]=.03
        recipe.update(lift_path_validation=dict(reference_phase='approach',max_xy_deviation_m=.008,max_orientation_deviation_deg=4.,min_z_offset_m=-.002),
            hold_orientation_phases=['lift'],linear_motion_phases=['lift'],target_poses_world={'approach':approach.tolist(),'lift':lift.tolist()},
            planning_options_by_phase={'lift':{'constraint_weight':20.,'enable_graph':False}})
        recipe['planning_path_policy']['kind']='collision_checked_lift_corridor';a.audit_free_path_policy({'phases':[]},recipe)
        recipe['target_poses_world']['lift'][0][3]=.02
        with self.assertRaisesRegex(ValueError,'laterally'):a.audit_free_path_policy({'phases':[]},recipe)

    def test_failed_plan_may_never_consume_episode(self):
        row=dict(scene_id='s',planning_status='failed',execution_status='planning_failed',attempts=1,executed=True,task_success=False)
        with self.assertRaisesRegex(ValueError,'consumed'):a.audit_planning_failure(row,{},Path('/tmp'),{})


class HistoryTests(unittest.TestCase):
    """History arithmetic stays independent from the separately tested revision gate."""
    def setUp(self):
        from tests.extensions.replicator.test_audit_physical_retry_round6 import Round6HistoryTests
        self.f=Round6HistoryTests('test_prior_retry_can_have_its_own_retimed_trajectory');self.f.setUp();self.addCleanup(self.f.doCleanups)
        for target,name,value in [(a,'ROOT',self.f.root),(a,'validate_retained_scene_revision',{})]:
            change=patch.object(target,name,return_value=value) if name.startswith('validate') else patch.object(target,name,value)
            change.start();self.addCleanup(change.stop)
        self.f.context['prior_rounds'].append(({'retry_round':6},{'scene':copy.deepcopy(self.f.previous)}))
        def revised(prior,number):
            return dict(prior,config_path=f'r{number}.json',config_sha256=f'r{number}config',
                source_scene_path=f'r{number}.usda',source_scene_sha256=f'r{number}source',layout_changed=True,
                scene_revision={'path':f'r{number}-revision.json','sha256':f'r{number}revision'})
        self.prior7=revised(self.f.previous,7)
        self.f.context['prior_rounds'].append(({'retry_round':7},{'scene':self.prior7}))
        candidate={'status':'generated','trajectory_complete':True,'executed':False,'attempts':1}
        candidate_ref={'path':'r8-candidate.json','sha256':self.f.write_text('r8-candidate.json',json.dumps(candidate))}
        self.prior8=revised(dict(self.f.round1,retry_schema_version=2,planning_status='generated',
            physical_attempt_number=3,prior_physical_attempts=self.f.history,retry_plan_path='r8-plan.json',retry_plan_sha256='r8plan',
            candidate_planning_result=candidate_ref,trajectory_path='r8.npz',trajectory_sha256=self.f.write_text('r8.npz','r8-new')) ,8)
        self.result=json.loads((self.f.root/self.f.round1['execution_result_path']).read_text());self.result.update(self.prior8)
        self.result.update(planning_result_path=candidate_ref['path'],planning_result_sha256=candidate_ref['sha256'],
            original_planning_result={'path':self.f.original['result_path'],'sha256':self.f.original['result_sha256']})
        self.prior8.update(self.f.write_result('r8/result.json',self.result))
        self.f.context['prior_rounds'].append(({'retry_round':8,'retry_plan_path':'r8-plan.json','retry_plan_sha256':'r8plan'},{'scene':self.prior8}))
        self.f.context['previous']={'scene':self.prior8};self.f.context['previous_manifest_binding']={'path':'r8manifest','sha256':'r8manifest'}
        for name,number in [('validate_revision7',7),('validate_scene_revision',8)]:
            value={'document':{'new_config':{'path':f'r{number}.json','sha256':f'r{number}config'},'new_source_scene':{'path':f'r{number}.usda','sha256':f'r{number}source'}}}
            change=patch.object(a,name,return_value=value);change.start();self.addCleanup(change.stop)
        self.refresh()

    def refresh(self):
        self.row=dict(self.prior8,physical_attempt_number=4,prior_physical_attempts=self.f.history+[a.prior_binding(self.prior8)],
            previous_retry_result={'path':self.prior8['execution_result_path'],'sha256':self.prior8['execution_result_sha256']},
            retained_scene_revision=True,scene_settings_changed_this_round=False,scene_settings_changed_once=True)

    def test_actual_r8_revised_episode_added_exactly_once(self):
        values=a.audit_history(self.row,self.row,self.f.context)
        self.assertEqual(len(values),3);self.assertEqual(values[-1]['config_path'],'r8.json')
        self.assertEqual(values[-1]['planning_result_path'],'r8-candidate.json')
        self.assertEqual(values[0]['config_path'],self.f.original['config_path'])

    def test_skipping_actual_r8_or_counting_failed_plans_rejected(self):
        for number in (3,5):
            bad=dict(self.row,physical_attempt_number=number)
            with self.subTest(number=number),self.assertRaisesRegex(ValueError,'counts planning failure'):a.audit_history(bad,bad,self.f.context)

    def test_old_config_cannot_replace_actual_revised_episode(self):
        self.result['config_path']=self.f.original['config_path']
        self.prior8.update(self.f.write_result('r8/result.json',self.result));self.refresh()
        with self.assertRaisesRegex(ValueError,'own manifest'):a.audit_history(self.row,self.row,self.f.context)


if __name__=='__main__':unittest.main()
