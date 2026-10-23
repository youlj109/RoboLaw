"""Regression checks for task thresholds, connected source bodies and scaled shapes."""
import unittest
import numpy as np
from kitchen_safety.extensions.doors.task_evidence import predicate_progress,openness
from kitchen_safety.extensions.doors.primitive_distance import ellipsoid_solid_distance,source_distance

class TaskEvidenceTests(unittest.TestCase):
 def test_freezer_keeps_stricter_source_threshold(self):
  self.assertIsNone(predicate_progress([0.,.1,.3*np.pi/2],0.,np.pi/2,threshold=.4))
  self.assertIsNone(predicate_progress([0.,.1,.4*np.pi/2],0.,np.pi/2,threshold=.4))
  self.assertEqual(predicate_progress([0.,.1,.400001*np.pi/2],0.,np.pi/2,threshold=.4),2)
 def test_prior_away_sample_required(self):
  self.assertIsNone(predicate_progress([.5],0.,1.,threshold=.4))
  self.assertEqual(predicate_progress([0.,.1,.5],0.,1.,threshold=.4),2)
 def test_negative_limit_source_convention(self):
  self.assertEqual(openness(0.,-1.,0.),0.)
  self.assertEqual(openness(-1.,-1.,0.),1.)
  self.assertIsNone(predicate_progress([0.,-.1,-.4],-1.,0.,threshold=.4))
  self.assertEqual(predicate_progress([0.,-.1,-.41],-1.,0.,threshold=.4),2)
 def test_nonuniform_ellipse_against_dense_surface(self):
  radii=np.asarray([.012,.021]);theta=np.linspace(0.,2*np.pi,200000);surface=np.column_stack([radii[0]*np.cos(theta),radii[1]*np.sin(theta)])
  for point in [[.03,.02],[-.02,.03],[.001,.04]]:
   expected=np.linalg.norm(surface-point,axis=1).min()
   self.assertAlmostEqual(ellipsoid_solid_distance(point,radii),expected,places=8)
  self.assertEqual(ellipsoid_solid_distance([0.,0.],radii),0.)
 def test_scaled_rotated_cylinder_cap_and_side(self):
  from pxr import Usd,UsdGeom
  from scipy.spatial.transform import Rotation
  s=Usd.Stage.CreateInMemory();c=UsdGeom.Cylinder.Define(s,'/Handle');c.CreateAxisAttr('Z');c.CreateRadiusAttr(1.);c.CreateHeightAttr(2.)
  R=Rotation.from_euler('xyz',[.3,-.5,.2]).as_matrix();T=np.eye(4);T[:3,:3]=R@np.diag([.01,.02,.1]);T[:3,3]=[3.,-2.,1.]
  distance=source_distance(c.GetPrim(),T)
  for local,expected in [([.03,0.,0.],.02),([0.,.04,0.],.02),([0.,0.,.13],.03),([.03,0.,.13],np.hypot(.02,.03))]:self.assertAlmostEqual(distance(R@local+T[:3,3]),expected,places=10)
 def test_source_joint_geometry_child_retains_whole_rigid_cluster(self):
  from pxr import Usd,UsdGeom,UsdPhysics,Sdf
  from kitchen_safety.extensions.doors.task_model import resolve
  s=Usd.Stage.CreateInMemory();root='/World/envs/env_0/kitchen/cab_main_main_group'
  for part in ['corpus','left_door','left_door_handle','right_door_handle']:
   p=UsdGeom.Xform.Define(s,root+'/'+part);UsdPhysics.RigidBodyAPI.Apply(p.GetPrim())
  panel=UsdGeom.Cube.Define(s,root+'/left_door/CabinetDoorPanel013');h=UsdGeom.Cube.Define(s,root+'/left_door_handle/Collisions/Handle');UsdPhysics.CollisionAPI.Apply(h.GetPrim())
  j=UsdPhysics.RevoluteJoint.Define(s,root+'/left_door_joint');j.CreateBody0Rel().SetTargets([Sdf.Path(root+'/corpus')]);j.CreateBody1Rel().SetTargets([panel.GetPath()]);j.CreateAxisAttr('Z');j.CreateLowerLimitAttr(0.);j.CreateUpperLimitAttr(90.)
  f=UsdPhysics.FixedJoint.Define(s,root+'/handle_fixed');f.CreateBody0Rel().SetTargets([Sdf.Path(root+'/left_door')]);f.CreateBody1Rel().SetTargets([Sdf.Path(root+'/left_door_handle')])
  result=resolve(s,'C02');self.assertEqual(set(result['moving_bodies']),{root+'/left_door',root+'/left_door_handle'});self.assertEqual(result['handle_colliders'],[str(h.GetPath())]);np.testing.assert_allclose(result['axis'],[0,0,1])

if __name__=='__main__':unittest.main()


class SourceReferenceMotionTests(unittest.TestCase):
 def test_negative_lower_limit_reset_is_identity(self):
  from kitchen_safety.extensions.doors.task_model import door_target_pose
  hinge=np.array([5.,-3.,1.6]);handle=hinge+np.array([.05,.3,0.]);R=np.eye(3)
  target=door_target_pose(hinge,[0,0,1],handle,R,-np.pi/2,-np.pi/2)
  np.testing.assert_allclose(target[:3,3],handle,atol=1e-12)
  np.testing.assert_allclose(target[:3,:3],R,atol=1e-12)
 def test_negative_lower_limit_uses_relative_quarter_turn(self):
  from kitchen_safety.extensions.doors.task_model import door_target_pose
  target=door_target_pose([1,2,3],[0,0,2],[2,2,3],np.eye(3),0.,-np.pi/2)
  np.testing.assert_allclose(target[:3,3],[1,3,3],atol=1e-12)
 def test_downward_hinge_and_negative_world_axis(self):
  from kitchen_safety.extensions.doors.task_model import door_target_pose
  target=door_target_pose([0,0,1],[0,-1,0],[0,0,1.5],np.eye(3),np.pi/2,0.)
  np.testing.assert_allclose(target[:3,3],[-.5,0,1],atol=1e-12)


class OpeningCommandTests(unittest.TestCase):
 def test_inverse_openness_in_both_source_directions(self):
  from kitchen_safety.extensions.doors.task_model import joint_position_from_openness
  for lower,upper in [(0.,1.57),(-1.57,0.),(-.7,.2)]:
   for value in [0.,.2,.4,1.]:
    self.assertAlmostEqual(openness(joint_position_from_openness(value,lower,upper),lower,upper),value)
 def test_negative_lower_opens_toward_lower_limit(self):
  from kitchen_safety.extensions.doors.task_model import opening_command
  self.assertAlmostEqual(opening_command(0.,.02,-.4,-1.57),-.02)
  self.assertAlmostEqual(opening_command(-.39,.02,-.4,-1.57),-.4)
  self.assertAlmostEqual(opening_command(.39,.02,.4,0.),.4)
 def test_pitch_preserves_default_and_requested_tool_tilt(self):
  from kitchen_safety.extensions.doors.task_model import grasp_rotation
  from scipy.spatial.transform import Rotation
  a=np.array([1.,0.,0.]);R=grasp_rotation(a,45.)
  np.testing.assert_allclose(R,np.diag([1.,-1.,-1.])@Rotation.from_euler('x',45,degrees=True).as_matrix(),atol=1e-12)
  tilted=grasp_rotation(a,45.,15.)
  self.assertAlmostEqual(Rotation.from_matrix(R.T@tilted).magnitude(),np.pi/12)
