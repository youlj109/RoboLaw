"""Physics evidence semantics independent of a running Kit process."""
from collections import defaultdict, deque
from types import SimpleNamespace
import unittest

from kitchen_safety.risk_binding import ContactMonitor, ArenaRiskRuntime
from kitchen_safety.risk_profile_loader import load_config, bind_profile


def runtime():
    r=object.__new__(ArenaRiskRuntime)
    r.settings=load_config('configs/safety_observer_arena.json')
    names=[f'panda_joint{i}' for i in range(1,8)]
    b=dict(robot_id='arena_droid_franka_robotiq_2f85',operation_mode='simulation',
        payload_profile='bottle',object_class='bottle',scenario_id='human_default_mpfb',
        human_inventory=dict(observed_human_paths=['/World/envs/env_0/person/body']),
        controlled_joint_names=names,gripper_contact_groups=['left_inner_finger','right_inner_finger'],
        self_collision_reporting_enabled=True,protected_environment_ids=[])
    r.robot=SimpleNamespace(joint_names=names)
    r.observers=[SimpleNamespace(binding=b,profile=bind_profile(r.settings,b))]
    r.latest=[dict(q=[0.]*7,joint_pos_limits=[[-2.,2.]]*7,applied_torque=[0.]*7,
        joint_effort_limits=[10.]*7,mustard=[0,0,1,0,0,0,1,0,0,0,0,0,0])]
    r.distances=[dict(source='test AABB',robot_to={'person':1.,'mustard_bottle':.1},
        object_to={'person':1.},links_to={'person':{'left_inner_finger':1.}})]
    r.windows=[deque(maxlen=4)]
    return r


class ContactSemanticsTests(unittest.TestCase):
    def test_body_pair_vector_norm_uses_actual_dt_and_collider_orientation(self):
        m=object.__new__(ContactMonitor)
        m.status={'subscribed':True,'errors':[]};m.errors=[];m.dt=1/120
        a='/World/envs/env_0/Robot/left_inner_finger';b='/World/envs/env_0/mustard_bottle'
        m.pending=defaultdict(list)
        m.pending[0]=[
            {'actor0':a,'actor1':b,'points':[{'impulse_vector_ns':[3.,0.,0.]}]},
            {'actor0':b,'actor1':a,'points':[{'impulse_vector_ns':[0.,-4.,0.]}]}]
        value=m.drain(0)
        self.assertTrue(value['coverage']);self.assertEqual(len(value['pairs']),1)
        self.assertEqual(value['pairs'][0]['impulse_ns'],5.)
        self.assertEqual(value['pairs'][0]['force_n'],600.)
        self.assertEqual(m.drain(0)['pairs'],[])
        m.errors=['callback failed']
        self.assertFalse(m.drain(0)['coverage'])

    def test_one_substep_contact_survives_window_without_becoming_initial_evidence(self):
        r=runtime()
        self.assertIsNone(r.features(0,'lift')['hs']['human_contact_flag_gt'])
        empty={'coverage':True,'pairs':[]}
        pair={'bodyA':'/World/envs/env_0/Robot/left_inner_finger',
            'bodyB':'/World/envs/env_0/person/body','force_n':12.,'impulse_ns':.1,'contacts':[]}
        for row in [empty,{'coverage':True,'pairs':[pair]},empty,empty]:r.windows[0].append(row)
        self.assertTrue(r.features(0,'lift')['hs']['human_contact_flag_gt'])
        for _ in range(4):r.windows[0].append(empty)
        self.assertFalse(r.features(0,'lift')['hs']['human_contact_flag_gt'])
        r.windows[0].append({'coverage':False,'pairs':[]})
        self.assertIsNone(r.features(0,'lift')['hs']['human_contact_flag_gt'])

if __name__=='__main__':unittest.main()
