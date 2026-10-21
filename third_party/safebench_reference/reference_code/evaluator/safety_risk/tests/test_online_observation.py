import unittest
from copy import deepcopy
from pathlib import Path
from safety_risk.online_iso import ObservationISOEvaluator, DISTANCE_FIELDS, COLLISION_FIELDS
from scripts.verify_online_iso import verify


class ObservationOnlineTests(unittest.TestCase):
    def make(self):
        profile = Path(__file__).resolve().parents[1]/'configs/iso_profile.experimental.json'
        evaluator = ObservationISOEvaluator({'profile_file':str(profile), 'mode':'shadow',
                                             'physics_dt_s':1/120, 'decision_hz':30})
        data={'step_ids':[], 'runtime_physics':{'physics_dt_s':1/120}}
        for key in (*DISTANCE_FIELDS,*COLLISION_FIELDS,'joint_position_q_gt',
                    'joint_torque_gt','low_level_command_sent','target_object_linear_velocity_gt'):
            data[key]=[]
        return evaluator,data

    def append(self,data,step):
        data['step_ids'].append(step)
        for key in DISTANCE_FIELDS:data[key].append({})
        for key in COLLISION_FIELDS:data[key].append(True if key=='contact_coverage_per_frame' else [])
        data['joint_position_q_gt'].append([0.])
        data['joint_torque_gt'].append([0.])
        data['low_level_command_sent'].append(True)
        data['target_object_linear_velocity_gt'].append(None)

    def run_stream(self,n=9):
        online,data=self.make()
        for step in range(n):
            views={'head':{'status':'new' if step<8 else 'stale','acquisition_sequence':step//4}}
            online.evaluate_before_action(data,step,views,step/120)
            self.append(data,step)
            online.record_execution(step)
        return online,data

    def test_observation_rate_pre_action_sources_and_tail(self):
        online,data=self.run_stream()
        self.assertEqual([r['input']['frame_index'] for r in online.records],[None,3,7])
        self.assertTrue(all(x is None for x in online.records[0]['report']['risk_levels'].values()))
        self.assertFalse(online.records[-1]['observation']['visual_inference_required'])
        self.assertEqual([r['observation_frame_id'] for r in online.execution_references],[0]*4+[1]*4+[2])
        raw={'metadata':{'num_steps':9},'online_iso':online.export()}
        self.assertTrue(verify(raw)['verified'])
        before=deepcopy(online.export())
        data['collision_pair_gt'][7].append({'bodyA':'robot','bodyB':'human'})
        self.assertEqual(before,online.export())

    def test_future_state_and_duplicate_decision_rejected(self):
        online,data=self.make()
        self.append(data,0)
        with self.assertRaisesRegex(ValueError,'previously completed'):
            online.evaluate_before_action(data,0,{'head':{'status':'new'}},0.)
        online,data=self.make()
        online.evaluate_before_action(data,0,{'head':{'status':'missing_or_invalid'}},0.)
        with self.assertRaisesRegex(ValueError,'duplicate'):
            online.evaluate_before_action(data,0,{'head':{'status':'new'}},0.)

    def test_execution_without_decision_rejected(self):
        online,_=self.make()
        with self.assertRaisesRegex(ValueError,'preceding observation'):
            online.record_execution(0)

    def test_partial_recording_clock_gap_returns_failure(self):
        online,_=self.run_stream()
        raw={'metadata':{'num_steps':9,'physics_frame_clocks':[
            {'before_sim_time_s':i/120,'after_sim_time_s':(i+1)/120} for i in range(8)]},
            'online_iso':online.export()}
        result=verify(raw)
        self.assertFalse(result['verified'])
        self.assertIn('physical_clock_coverage_mismatch',result['errors'])

    def test_replay_detects_source_reference_and_clock_tampering(self):
        online,_=self.run_stream()
        original={'metadata':{'num_steps':9,'physics_frame_clocks':[
            {'before_sim_time_s':i/120,'after_sim_time_s':(i+1)/120} for i in range(9)]},
            'online_iso':online.export()}
        self.assertTrue(verify(original)['verified'])
        for mutation in ('source','reference','clock','count'):
            raw=deepcopy(original)
            if mutation=='source':raw['online_iso']['records'][1]['input']['frame_index']=4
            elif mutation=='reference':raw['online_iso']['execution_references'][3]['observation_frame_id']=1
            elif mutation=='clock':raw['online_iso']['records'][1]['observation']['decision_sim_time_s']=0.
            else:raw['online_iso']['records'].pop()
            self.assertFalse(verify(raw)['verified'],mutation)


if __name__=='__main__':unittest.main()
