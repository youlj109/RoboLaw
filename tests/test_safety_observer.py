import copy
import json
from pathlib import Path
import random
import tempfile
import unittest

from kitchen_safety.risk_observer import RiskObserver, unknown_report
from kitchen_safety.risk_profile_loader import load_config
from kitchen_safety.risk_camera import append_camera_provenance


def binding(human=True):
    return dict(robot_id='arena_droid_franka_robotiq_2f85',operation_mode='simulation',
        payload_profile='mustard_bottle_authored_mass',object_class='hope_mustard_bottle',scenario_id='human_default_mpfb',
        protected_environment_ids=[],gripper_contact_groups=['left_inner_finger','right_inner_finger'],self_collision_reporting_enabled=True,
        geometry_source='test_geometry',human_inventory={'verified_absent':not human},
        camera_names=['external_camera','external_camera_2','wrist_camera'])


class Sink:
    def __init__(self):self.calls=[]
    def submit(self,command):self.calls.append(copy.deepcopy(command));return command


class ObserverTests(unittest.TestCase):
    def test_all_levels_unknown_and_evaluator_exception_cannot_modify_submission(self):
        settings=load_config('configs/safety_observer_arena.json')
        for level in ['L0','L1','L2','L3',None,'error']:
            with self.subTest(level=level),tempfile.TemporaryDirectory() as tmp:
                class Evaluator:
                    def evaluate(self,value):
                        # Even an ill-behaved evaluator can only mutate its private copy.
                        value['features']['mutable'].append('changed')
                        if level=='error':raise RuntimeError('injected evaluator error')
                        r=unknown_report('test');r['risk_levels'].update({c:level for c in ('HS','PT','RS','IR')})
                        for c in r['categories'].values():c.update(level=level,data_complete=level is not None)
                        return r
                identity=dict(run_id='test',attempt_id='one',env_id=0,episode_generation=4)
                observer=RiskObserver(tmp,settings,binding(),identity,1/120,1/15,evaluator=Evaluator())
                source={'mutable':[]};action=[.1]*7+[1.];sink=Sink();rng=random.getstate()
                for j in range(9):
                    submitted=sink.submit(action)
                    observer.before_physics(source,'lift',j//8,action,submitted,{'writer_calls':[]})
                    observer.after_physics({'mustard':[0,0,1,0,0,0,1,0,0,0,0,0,0],'bowl':[0,0,1]},'lift',j//8,{'coverage':False,'pairs':[]},{})
                observer.finish({'success':False})
                self.assertEqual(sink.calls,[action]*9)
                self.assertEqual(source,{'mutable':[]});self.assertEqual(rng,random.getstate())
                self.assertEqual([r['source_physics_step'] for r in observer.records],[-1,3,7])
                rows=[json.loads(s) for s in (Path(tmp)/'safety_physics.jsonl').read_text().splitlines()]
                self.assertEqual([r['observation_id'] for r in rows],[0]*4+[1]*4+[2])
                if level=='error':self.assertTrue(all(r['error'] and r['report']['risk_levels']['HS'] is None for r in observer.records))

    def test_nonfinite_sensor_feature_remains_null_without_crashing(self):
        with tempfile.TemporaryDirectory() as tmp:
            o=RiskObserver(tmp,load_config('configs/safety_observer_arena.json'),binding(),
                dict(run_id='x',attempt_id='bad_sensor',env_id=0,episode_generation=0),1/120,1/15)
            o.evaluate_before_action({'hs':{'d_robot_h_min_gt_m':float('nan')}},'settle',0)
            self.assertIsNone(o.records[0]['input']['features']['hs']['d_robot_h_min_gt_m'])
            o.finish({'success':False})

    def test_identity_and_initial_contact_unknown(self):
        settings=load_config('configs/safety_observer_arena.json')
        with tempfile.TemporaryDirectory() as tmp:
            identity=dict(run_id='x',attempt_id='a',env_id=3,episode_generation=9)
            o=RiskObserver(tmp,settings,binding(),identity,1/120,1/15)
            o.before_physics({},'settle',0,[0]*8,[0]*8,{'writer_calls':[]})
            r=o.records[0]
            self.assertIsNone(r['report']['categories']['HS']['level'])
            self.assertEqual(r['source_physics_step'],-1)
            self.assertEqual(r['env_id'],3);self.assertEqual(r['episode_generation'],9)
            o.finish({'success':False})


    def test_feature_projection_agrees_with_source_raw_frame(self):
        import numpy as np
        from types import SimpleNamespace
        from kitchen_safety.risk_binding import ArenaRiskRuntime
        from kitchen_safety.risk_projection import reference_projection
        from kitchen_safety.risk_profile_loader import bind_profile
        state=dict(q=[0.]*7,dq=[0.]*7,applied_torque=[1.]*7,
                   joint_pos_limits=[[-2.,2.]]*7,joint_effort_limits=[10.]*7,
                   mustard=[0,0,1,0,0,0,1,0,0,0,0,0,0])
        dist=dict(robot_to={'person':.4},object_to={'person':.3},links_to={'person':{'base_link':.5,'panda_link1':.4}})
        # HS must use narrow-phase values even when AABB channels disagree.
        dist['hs_geometry']={'complete':True,'source':'analytical regression fixture',
            'features':{'d_robot_h_min_gt_m':.6,'d_ee_h_min_gt_m':.7,'d_obj_h_min_gt_m':.8}}
        contacts={'coverage':True,'pairs':[]}
        b=binding();b['human_inventory']['observed_human_paths']=['/person/body']
        b['controlled_joint_names']=[f'panda_joint{i}' for i in range(1,8)]
        runtime=object.__new__(ArenaRiskRuntime)
        runtime.settings=load_config('configs/safety_observer_arena.json')
        runtime.robot=SimpleNamespace(joint_names=b['controlled_joint_names'])
        runtime.latest=[state];runtime.distances=[dist];runtime.windows=[[contacts]*4]
        runtime.observers=[SimpleNamespace(binding=b,profile=bind_profile(load_config('configs/safety_observer_arena.json'),b))]
        actual=runtime.features(0,'lift')
        expected=reference_projection(state,dist,contacts,list(range(7)),1/120,True)
        self.assertEqual(actual['hs']['d_robot_h_min_gt_m'],.6)
        self.assertEqual(actual['hs']['d_ee_h_min_gt_m'],.7)
        self.assertEqual(actual['hs']['d_obj_h_min_gt_m'],.8)
        for cat,keys in {'hs':['d_robot_h_min_gt_m','d_ee_h_min_gt_m','d_obj_h_min_gt_m','human_contact_flag_gt'],
                         'rs':['joint_limit_margin_gt_rad','joint_torque_ratio_gt','self_collision_flag_gt'],
                         'ir':list(expected['ir'])}.items():
            for key in keys:self.assertEqual(actual[cat].get(key),expected[cat].get(key),(cat,key))

    def test_reject_nonintegral_or_mislabelled_physics_window(self):
        settings=load_config('configs/safety_observer_arena.json')
        for dt in [1/100,1/240]:
            with tempfile.TemporaryDirectory() as tmp,self.assertRaises(ValueError):
                RiskObserver(tmp,settings,binding(),{},dt,1/15)

    def test_three_camera_frozen_source_identity_with_late_delivery(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=dict(identity=dict(run_id='run',attempt_id='old',env_id=1,episode_generation=0),
                physics_dt_s=1/120,action_dt_s=1/15,binding=binding())
            frozen=copy.deepcopy(b)
            b['identity']['episode_generation']=1
            append_camera_provenance(tmp,frozen,2,100,900,mode='offline_measured_state_render')
            rows=[json.loads(x) for x in (Path(tmp)/'safety_camera_provenance.jsonl').read_text().splitlines()]
            self.assertEqual(len(rows),3)
            self.assertTrue(all(r['episode_generation']==0 and r['source_physics_step']==15 for r in rows))

if __name__=='__main__':unittest.main()
