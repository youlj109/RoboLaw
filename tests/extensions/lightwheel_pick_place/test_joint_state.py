"""Read-only joint-coordinate regressions, including COM velocity semantics."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.lightwheel_pick_place.joint_state import KitchenJointRecorder
class View:
    def __init__(self,poses,vel):self.poses=poses;self.vel=vel
    def get_transforms(self):return self.poses
    def get_velocities(self):return self.vel
class JointStateTests(unittest.TestCase):
    def recorder(self,kind,axis,pose,vel,com=None):
        r=KitchenJointRecorder.__new__(KitchenJointRecorder);r.body_view=View(np.array([pose]),np.array([vel]));r.native_views=[];r.body_com=np.array([com or [0,0,0]],float);r.body_paths=['/body'];r.inventory={}
        endpoint=lambda i:dict(index=i,static_matrix=np.eye(4),local_R=np.eye(3),local_p=np.zeros(3))
        r.joints=[dict(path='/joint',kind=kind,axis=np.array(axis,float),native=None,a=endpoint(None),b=endpoint(0),unit='rad' if kind=='revolute' else 'm')];return r
    def test_revolute_position_and_speed_use_live_pose_and_angular_velocity(self):
        quat=Rotation.from_euler('z',.3).as_quat();r=self.recorder('revolute',[0,0,1],[0,0,0,*quat],[0,0,0,0,0,.4]);s=r.read();self.assertAlmostEqual(s['kitchen_joint_q'][0],.3);self.assertAlmostEqual(s['kitchen_joint_dq'][0],.4)
    def test_prismatic_units_are_metres_and_metres_per_second(self):
        r=self.recorder('prismatic',[1,0,0],[.03,0,0,0,0,0,1],[.02,0,0,0,0,0]);s=r.read();self.assertAlmostEqual(s['kitchen_joint_q'][0],.03);self.assertAlmostEqual(s['kitchen_joint_dq'][0],.02)
    def test_com_velocity_is_converted_to_joint_anchor_velocity(self):
        r=self.recorder('prismatic',[0,1,0],[0,0,0,0,0,0,1],[0,.2,0,0,0,2],com=[.1,0,0]);self.assertAlmostEqual(r.read()['kitchen_joint_dq'][0],0.)
    def test_d6_rotation_coordinate_and_rate_are_observed(self):
        quat=Rotation.from_euler('z',.3).as_quat();r=self.recorder('revolute',[0,0,1],[0,0,0,*quat],[0,0,0,0,0,.4]);r.joints[0]['component']='rotZ';s=r.read();self.assertAlmostEqual(s['kitchen_joint_q'][0],.3);self.assertAlmostEqual(s['kitchen_joint_dq'][0],.4)
    def test_d6_translation_coordinate_and_rate_are_observed(self):
        r=self.recorder('prismatic',[1,0,0],[.03,0,0,0,0,0,1],[.02,0,0,0,0,0]);r.joints[0]['component']='transX';s=r.read();self.assertAlmostEqual(s['kitchen_joint_q'][0],.03);self.assertAlmostEqual(s['kitchen_joint_dq'][0],.02)
    def test_body_motion_is_rejected_even_if_joint_array_is_constant(self):
        r=self.recorder('prismatic',[1,0,0],[0,0,0,0,0,0,1],[0,0,0,0,0,0]);poses=np.array([[[0,0,0,0,0,0,1]],[[.01,0,0,0,0,0,1]]]);rows=dict(kitchen_joint_q=np.zeros((2,1)),kitchen_joint_dq=np.zeros((2,1)),kitchen_body_pose=poses,kitchen_body_velocity=np.zeros((2,1,6)));self.assertFalse(r.summarize(rows,0)['passed'])
if __name__=='__main__':unittest.main()
