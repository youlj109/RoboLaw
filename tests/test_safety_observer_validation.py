import json
from pathlib import Path
import tempfile
import unittest
import h5py
import numpy as np

from kitchen_safety.risk_profile_loader import load_config
from kitchen_safety.risk_observer import RiskObserver
from kitchen_safety.risk_reporting import validate_observer, write_indexes
from kitchen_safety.risk_export import risk_arrays
from kitchen_safety.risk_camera import append_camera_provenance


def fixture(path):
    binding=dict(robot_id='droid',operation_mode='simulation',payload_profile='bottle',object_class='bottle',
        scenario_id='human_default_mpfb',protected_environment_ids=[],gripper_contact_groups=['left_inner_finger','right_inner_finger'],
        geometry_source='test',human_inventory={'verified_absent':False},camera_names=['a','b','c'],
        command_index_schema='physx-submitted-environment-indices-1',command_backend_row=0)
    identity=dict(run_id='fixture',attempt_id='attempt_0',env_id=0,episode_generation=0)
    observer=RiskObserver(path,load_config('configs/safety_observer_arena.json'),binding,identity,1/120,1/15)
    action=[0.]*8
    calls=[dict(method=method,nominal=[0.]*13,submitted=[0.]*13,unchanged=True,
        indices=[0,1,2,3],submitted_environment_indices=[0,1,2,3],environment_row=0) for method in
           ('set_dof_actuation_forces','set_dof_position_targets','set_dof_velocity_targets')]
    for j in range(16):
        import time
        observer.evaluate_before_action({},'settle',j//8)
        for call in calls:call['submitted_monotonic_ns']=time.perf_counter_ns()
        observer.record_commands('settle',j//8,action,action,{'writer_calls':calls})
        observer.after_physics({'mustard':[0,0,1,0,0,0,1,0,0,0,0,0,0],'bowl':[0,0,1]},'settle',j//8,
            {'coverage':True,'pairs':[]},{})
    observer.finish({'success':False})
    with h5py.File(path/'trajectory.h5','w') as h:
        h.attrs['dt']=1/15;h.create_dataset('action',data=np.zeros((2,8),dtype=np.float32))
    return observer


class ValidationTests(unittest.TestCase):
    def test_replay_commands_and_action_causal_export(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);fixture(root)
            report=validate_observer(root)
            self.assertTrue(report['passed'],report)
            labels=risk_arrays(root,2,7)
            self.assertEqual(labels['risk.observation_id'].tolist(),[0,2])
            self.assertEqual(labels['risk.source_physics_step'].tolist(),[-1,7])
            self.assertFalse(labels['risk.valid'].any())
            self.assertTrue((labels['risk.level']==-1).all())

    def test_command_changes_missing_sends_cross_environment_and_future_inputs_are_rejected(self):
        for kind in ('changed_command','missing_send','cross_env','future','label','pre_submit','mask','row'):
            with self.subTest(kind=kind),tempfile.TemporaryDirectory() as tmp:
                root=Path(tmp);fixture(root)
                name='safety_commands.jsonl' if kind in ('changed_command','missing_send','pre_submit','mask','row') else 'safety_physics.jsonl' if kind=='cross_env' else 'safety_decisions.jsonl'
                rows=[json.loads(x) for x in (root/name).read_text().splitlines()]
                if kind=='changed_command':rows[0]['targets']['writer_calls'][0]['submitted'][0]=1.
                if kind=='pre_submit':rows[0]['targets']['writer_calls'][0]['submitted_monotonic_ns']=0
                if kind=='mask':
                    rows[0]['targets']['writer_calls'][0]['indices']=[1,2,3]
                    rows[0]['targets']['writer_calls'][0]['submitted_environment_indices']=[1,2,3]
                if kind=='row':rows[0]['targets']['writer_calls'][0]['environment_row']=1
                if kind=='missing_send':rows[0]['targets']['writer_calls'].pop()
                if kind=='cross_env':rows[0]['env_id']=1
                if kind=='future':rows[0]['source_physics_step']=1
                if kind=='label':rows[0]['report']['risk_levels']['HS']='L0'
                (root/name).write_text(''.join(json.dumps(r)+'\n' for r in rows))
                self.assertFalse(validate_observer(root)['passed'])

    def test_camera_indices_cannot_shift_duplicate_or_cross_generations(self):
        for corruption in ('none','shift','duplicate','sequence','time','reset','stale','missing'):
            with self.subTest(corruption=corruption),tempfile.TemporaryDirectory() as tmp:
                root=Path(tmp);fixture(root)
                binding=json.loads((root/'safety_observer_binding.json').read_text())
                for action in range(2):
                    append_camera_provenance(root,binding,action,100+action,200+action,
                                             mode='offline_measured_state_render')
                path=root/'safety_camera_provenance.jsonl'
                rows=[json.loads(x) for x in path.read_text().splitlines()]
                if corruption=='shift':
                    for r in rows:
                        r['source_action_index']+=10;r['acquisition_sequence']+=10
                        r['source_physics_step']=r['source_action_index']*8-1
                        r['source_sim_time_s']=r['source_action_index']/15
                if corruption=='duplicate':rows[3]=rows[0].copy()
                if corruption=='sequence':rows[0]['acquisition_sequence']=10
                if corruption=='time':rows[0]['source_sim_time_s']=10
                if corruption=='reset':rows[0]['episode_generation']=1
                if corruption=='stale':rows[0]['stale']=True
                if corruption=='missing':rows.pop()
                path.write_text(''.join(json.dumps(r)+'\n' for r in rows))
                self.assertEqual(validate_observer(root)['passed'],corruption=='none')

    def test_static_negative_does_not_require_invented_motion_but_moving_or_successful_does(self):
        from scripts.validate_collection import require_visible_motion
        q=np.zeros(13);body=np.zeros((18,7))
        self.assertFalse(require_visible_motion({'success':False},True,q,q,body,body))
        self.assertTrue(require_visible_motion({'success':True},True,q,q,body,body))
        self.assertTrue(require_visible_motion({'success':False},False,q,q,body,body))
        self.assertTrue(require_visible_motion({'success':False},True,q,q+.1,body,body))

    def test_failed_initialization_is_not_dropped_from_all_attempts(self):
        with tempfile.TemporaryDirectory() as tmp:
            r=write_indexes(tmp,[{'path':'no_episode','obstacle_id':'human_default_mpfb',
                'failure_kind':'planning_failed','error':'no plan'}])
            self.assertEqual(r,{'all_attempts':1,'successful_tasks':0,'positive_demonstrations':0,'obstruction_confirmed':0,'negative_samples':1,'risk_trajectories':0})

    def test_positive_demonstration_still_requires_original_independent_audit(self):
        from unittest.mock import patch
        from kitchen_safety.risk_reporting import result_axes
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);fixture(root)
            for name,value in {
                'result.json':{'success':True},
                'safety_task_evidence.json':{'extended_success':True,'mismatch':False},
                'safety_risk_summary.json':{'risk':{c:dict(known_max='L0',complete_observations=4,observations=4)
                    for c in ('HS','PT','RS','IR')}}}.items():
                (root/name).write_text(json.dumps(value))
            validation=dict(passed=True,physics_contact_coverage=1.,camera={'complete':True},action_unchanged=True)
            with patch('kitchen_safety.risk_reporting.validate_observer',return_value=validation):
                v=dict(data_success=True,audit_pass=False,augmentation_eligible=True)
                self.assertFalse(result_axes(root,v)['positive_demonstration'])
                v['audit_pass']=True
                self.assertTrue(result_axes(root,v)['positive_demonstration'])

    def test_extension_failure_retains_legacy_data_result_and_exports_combined_failure(self):
        from unittest.mock import patch
        from scripts.validate_collection import attach_safety_validation
        from kitchen_safety.risk_reporting import result_axes
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);fixture(root)
            item={'legacy_passed':True,'passed':False}
            validation=dict(passed=False,errors=['camera source shifted'],camera={'complete':False})
            with patch('kitchen_safety.risk_reporting.validate_observer',return_value=validation):
                self.assertFalse(attach_safety_validation(item,root,root,True))
                self.assertTrue(item['legacy_passed']);self.assertFalse(item['passed'])
                (root/'validation.json').write_text(json.dumps([item]))
                axes=result_axes(root,{'data_success':False,'audit_pass':False})
            self.assertTrue(axes['data_integrity']['legacy_passed'])
            self.assertFalse(axes['data_integrity']['combined_passed'])
            self.assertFalse(axes['data_integrity']['safety_extension_passed'])
            self.assertTrue((risk_arrays(root,2,0)['result.axes'][:,1]==0).all())

    def test_extension_exception_is_reported_without_discarding_completed_legacy_checks(self):
        from unittest.mock import patch
        from scripts.validate_collection import attach_safety_validation
        item={'legacy_passed':True}
        with patch('kitchen_safety.risk_reporting.validate_observer',side_effect=FileNotFoundError('missing evidence')):
            self.assertFalse(attach_safety_validation(item,Path('/tmp'),Path('/tmp'),True))
        self.assertTrue(item['legacy_passed'])
        self.assertFalse(item['passed'])
        self.assertFalse(item['safety_extension']['passed'])
        self.assertIn('missing evidence',item['error'])

if __name__=='__main__':unittest.main()
