"""Round8 permits only versioned hazard poses; old layout outcomes stay historical."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from scripts.extensions.replicator import audit_physical_retry_round8 as audit
from tests.extensions.replicator import test_audit_physical_retry_round6 as fixtures


class HistoryTests(unittest.TestCase):
    def setUp(self):
        self.f=fixtures.Round6HistoryTests('test_prior_retry_can_have_its_own_retimed_trajectory');self.f.setUp()
        self.addCleanup(self.f.doCleanups)
        self.row=dict(self.f.row,config_path='new.json',config_sha256='new',source_scene_path='new.usda',source_scene_sha256='newscene',
                      layout_changed=True,scene_revision={'path':'revision.json','sha256':'revision'})
        self.f.context['previous_manifest_binding']={'path':'round6manifest.json','sha256':'prior'}
        self.f.context['prior_rounds'].append(({'retry_plan_path':'round6-plan.json','retry_plan_sha256':'round6-plan'},self.f.context['previous']))
        self.prior7=dict(self.f.previous,config_path='r7.json',config_sha256='r7',source_scene_path='r7.usda',source_scene_sha256='r7scene',scene_revision={'path':'r7revision','sha256':'r7ref'},layout_changed=True)
        self.f.context['prior_rounds'].append(({'retry_round':7},{'scene':self.prior7}))
        self.f.context['previous']={'scene':self.prior7}
        validator=patch.object(audit,'validate_revision7',return_value={'document':{'new_config':{'path':'r7.json','sha256':'r7'},'new_source_scene':{'path':'r7.usda','sha256':'r7scene'}}})
        validator.start();self.addCleanup(validator.stop)
        change=patch.object(audit,'ROOT',self.f.root);change.start();self.addCleanup(change.stop)
        change=patch.object(audit,'validate_scene_revision',return_value={'document':{'prior_manifest':self.f.context['previous_manifest_binding']}})
        change.start();self.addCleanup(change.stop)

    def test_new_config_keeps_original_actual_lineage(self):
        values=audit.audit_history(self.row,self.row,self.f.context)
        self.assertEqual(len(values),2);self.assertEqual(self.row['physical_attempt_number'],3)
        self.assertNotEqual(values[-1]['config_path'],self.row['config_path'])

    def test_planning_skips_cannot_increment_actual_counter(self):
        bad=dict(self.row,physical_attempt_number=4)
        with self.assertRaisesRegex(ValueError,'counts planning failure'):audit.audit_history(bad,bad,self.f.context)

    def test_original_history_cannot_be_rebound_to_new_layout(self):
        self.f.context['prior_rounds'][0][1]['scene']['config_path']='new.json'
        with self.assertRaisesRegex(ValueError,'Historical original/revised source'):audit.audit_history(self.row,self.row,self.f.context)


class PolicyTests(unittest.TestCase):
    def fixture(self):
        return {'phases':[{'phase':'approach'}]}, {'planning_path_policy':{
            'kind':'collision_checked_free_path','previous_straight_line_gate_removed':True,'reason_zh':'允许碰撞检查的绕行'},
            'segment_deviation_limits_m':{},'hold_orientation_phases':[]}

    def test_free_path_does_not_claim_five_mm_pass(self):
        candidate,recipe=self.fixture();audit.audit_free_path_policy(candidate,recipe)
        recipe['segment_deviation_limits_m']={'lift':.005}
        with self.assertRaisesRegex(ValueError,'incompatible'):audit.audit_free_path_policy(candidate,recipe)

    def test_old_joint_goal_or_graph_override_cannot_silently_return(self):
        for patch_value in ({'joint_space_target_q_by_phase':{'lift':[[0]*7]}},
                            {'planning_options_by_phase':{'lift':{'enable_graph':False}}}):
            candidate,recipe=self.fixture();recipe.update(patch_value)
            with self.assertRaises(ValueError):audit.audit_free_path_policy(candidate,recipe)

    def test_failed_solver_has_no_physical_trial(self):
        row=dict(scene_id='s',task_id='t',execution_status='planning_failed',planning_status='failed',attempts=1,
                 executed=True,task_success=False)
        with self.assertRaisesRegex(ValueError,'consumed'):audit.audit_planning_failure(row,{},Path('/tmp'),{})

    def test_saved_free_path_tcp_is_independently_checked(self):
        with tempfile.TemporaryDirectory() as directory,patch.object(audit,'ROOT',Path(directory)):
            path=Path(directory)/'path.npz';q=np.zeros((3,7));tcp=np.repeat(np.eye(4)[None],3,axis=0)
            np.savez(path,q=q,tcp_world=tcp,phase=np.array(['pregrasp']*3))
            kin=type('FK',(),{'source':{},'limits':np.array([[-1,1]]*7),'fk':lambda self,q:np.eye(4)})()
            candidate=dict(status='generated',start_q=[0]*7,targets={'pregrasp':np.eye(4)},
                phases=[dict(phase='pregrasp',status='generated',samples=3)],
                artifacts={'trajectory':{'path':'path.npz','sha256':audit.digest(path)}})
            checked=audit.audit_saved_free_path(candidate,{},kin);self.assertEqual(checked['independent_fk_samples'],3)
            tcp[1,0,3]=.02;np.savez(path,q=q,tcp_world=tcp,phase=np.array(['pregrasp']*3))
            candidate['artifacts']['trajectory']['sha256']=audit.digest(path)
            with self.assertRaisesRegex(ValueError,'independent source FK'):audit.audit_saved_free_path(candidate,{},kin)


class USDRevisionTests(unittest.TestCase):
    def setUp(self):
        try:from pxr import Usd,UsdGeom,UsdPhysics,Gf
        except ImportError:self.skipTest('Requires CPU USD, available through scripts/arena_python.sh')
        self.Usd,self.UsdGeom,self.UsdPhysics,self.Gf=Usd,UsdGeom,UsdPhysics,Gf
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name)
        self.old={'obstacle':{'position':[0.,0.,0.],'yaw_deg':0.}}
        self.new={'obstacle':{'position':[.1,.2,0.],'yaw_deg':10.}}
        for filename,layout in [('old.usda',self.old),('new.usda',self.new)]:
            stage=Usd.Stage.CreateNew(str(self.root/filename));UsdGeom.Xform.Define(stage,'/World')
            hazard=UsdGeom.Xform.Define(stage,'/World/hazard');hazard.AddTranslateOp().Set(Gf.Vec3d(*layout['obstacle']['position']))
            hazard.AddRotateZOp().Set(layout['obstacle']['yaw_deg']);cube=UsdGeom.Cube.Define(stage,'/World/hazard/Asset')
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
            UsdGeom.Xform.Define(stage,'/World/robot').AddTranslateOp().Set(Gf.Vec3d(1.,2.,3.))
            UsdGeom.Camera.Define(stage,'/World/overview_camera').AddTransformOp().Set(Gf.Matrix4d(1.))
            stage.GetRootLayer().customLayerData={'source_layout_json':json.dumps(layout),'layout_sha256':'bound elsewhere'}
            stage.GetRootLayer().Save()

    def check(self):return audit.audit_usd_pose_revision(self.root/'old.usda',self.root/'new.usda',self.old,self.new)
    def change(self,path,attribute,value):
        stage=self.Usd.Stage.Open(str(self.root/'new.usda'));stage.GetPrimAtPath(path).GetAttribute(attribute).Set(value);stage.GetRootLayer().Save()

    def test_only_selected_hazard_pose_and_camera_are_allowed(self):
        self.assertTrue(self.check()['original_robot_and_objects_unchanged'])
        self.change('/World/overview_camera','xformOp:transform',self.Gf.Matrix4d(2.))
        self.assertTrue(self.check()['physical_prim_geometry_properties_unchanged'])

    def test_original_robot_transform_is_rejected(self):
        self.change('/World/robot','xformOp:translate',self.Gf.Vec3d(9,2,3))
        with self.assertRaisesRegex(ValueError,'Composed USD changed'):self.check()

    def test_disabling_hazard_collision_is_rejected(self):
        self.change('/World/hazard/Asset','physics:collisionEnabled',False)
        with self.assertRaisesRegex(ValueError,'Composed USD changed'):self.check()

    def test_unbound_hazard_translation_is_rejected(self):
        self.change('/World/hazard','xformOp:translate',self.Gf.Vec3d(.9,.2,0))
        with self.assertRaisesRegex(ValueError,'differs from selected config'):self.check()


if __name__=='__main__':unittest.main()
