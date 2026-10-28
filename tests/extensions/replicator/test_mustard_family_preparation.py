"""CPU topology/authoring checks; no physics or preparation acceptance shortcut."""
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,Sdf
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.replicator.mustard_family_background import (
    HOUSING,MOUNT,KETTLE_BODIES,HOUSING_ONLY_RECIPE,inspect_background_profile,
    family_mounted_kitchen,validate_housing_only,
)
from kitchen_safety.extensions.replicator.mustard_family_preparation import ROOT,source_task_profile
from scripts.extensions.replicator.audit_task_source_relation import source


class MustardFamilyTests(unittest.TestCase):
    def test_all_trace_body_contact_gate_rejects_early_nozzle_contact(self):
        from copy import deepcopy
        from kitchen_safety.extensions.replicator.mustard_family_collision import audit_bottle_body_contacts,MustardContactPolicy
        geometry=dict(mesh_bounds_m=[[-.05,-.03,-.08],[.05,.03,.08]],safe_contact_z_range_m=[-.04,.03])
        entry=dict(count=1,points_object_local_m=[[.02,0,0]],normal_forces_n=[1.],separations_m=[0.])
        row=dict(object_name='mustard_bottle',phase='close',physics_step=1,
            fingers={name:deepcopy(entry) for name in ('left_inner_finger','right_inner_finger')})
        rows=[deepcopy(row) for _ in range(11)]
        for index,item in enumerate(rows):item['physics_step']=index
        self.assertTrue(audit_bottle_body_contacts(rows,geometry)['passed'])
        rows[0]['fingers']['left_inner_finger']['points_object_local_m'][0][2]=.07
        report=audit_bottle_body_contacts(rows,geometry)
        self.assertFalse(report['passed']);self.assertEqual(report['failures'][0]['physics_step'],0)
        policy=MustardContactPolicy(source_support='/World/envs/env_0/kitchen/CapSurface')
        self.assertFalse(policy.finger_pair('gripper/base_link','approach'))
        self.assertFalse(policy.finger_pair('gripper/left_inner_finger','pregrasp'))
        self.assertTrue(policy.finger_pair('gripper/left_inner_finger','close'))

    def fixture(self,path,kettle_paths=()):
        stage=Usd.Stage.CreateNew(str(path));root=UsdGeom.Xform.Define(stage,'/World');stage.SetDefaultPrim(root.GetPrim())
        for body_path in (HOUSING,*kettle_paths):
            body=UsdGeom.Xform.Define(stage,body_path)
            UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
            body.AddTranslateOp().Set((.25,.5,.75))
        stage.GetRootLayer().Save()
        return stage

    def test_absent_kettle_and_distinct_kettle_b_do_not_receive_a_solver_recipe(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'source.usda'
            kettle='/World/KettleB01/body'
            stage=self.fixture(path,[kettle])
            body=stage.GetPrimAtPath(kettle)
            body.CreateAttribute('physxRigidBody:solverPositionIterationCount',Sdf.ValueTypeNames.Int).Set(7)
            stage.GetRootLayer().Save()
            before=path.read_bytes();profile=inspect_background_profile(path)
            self.assertEqual(profile['recipe'],HOUSING_ONLY_RECIPE)
            self.assertEqual(profile['solver_edit_bodies'],[])
            overlay,info=family_mounted_kitchen(path,Path(directory)/'overlay')
            composed=Usd.Stage.Open(str(overlay))
            self.assertEqual(path.read_bytes(),before)
            self.assertEqual(composed.GetPrimAtPath(kettle).GetAttribute('physxRigidBody:solverPositionIterationCount').Get(),7)
            self.assertTrue(validate_housing_only(composed)['passed'])
            self.assertFalse(info['neutral_validated'])
            self.assertFalse(info['execution_ready'])
            self.assertEqual(UsdPhysics.Joint(composed.GetPrimAtPath(MOUNT)).GetBody1Rel().GetTargets(),[Sdf.Path(HOUSING)])
            self.assertEqual(family_mounted_kitchen(path,Path(directory)/'overlay')[1],info)

    def test_extra_physics_edit_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'source.usda';self.fixture(path)
            overlay,_=family_mounted_kitchen(path,Path(directory)/'overlay')
            stage=Usd.Stage.Open(str(overlay))
            stage.GetPrimAtPath(HOUSING).CreateAttribute('physics:mass',Sdf.ValueTypeNames.Float).Set(1.)
            with self.assertRaisesRegex(ValueError,'Unexpected local attribute'):validate_housing_only(stage)

    def test_partial_kettle_a_topology_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'source.usda';self.fixture(path,KETTLE_BODIES[:1])
            with self.assertRaisesRegex(ValueError,'Partial KettleA'):inspect_background_profile(path)

    def test_nominal_grasp_rotates_with_bottle_instead_of_g_world_yaw(self):
        import copy
        layout=json.loads((ROOT/'configs/extensions/replicator/l_island_mustard_bowl_source_layout.json').read_text())
        _,spec=source(layout['task_id'])
        old=source_task_profile(layout,spec)
        changed=copy.deepcopy(layout);changed['objects']['mustard_bottle']['yaw_deg']+=90
        new=source_task_profile(changed,spec)
        a=old['nominal_geometry_candidates']['ordered_grasp_candidates'][0]
        b=new['nominal_geometry_candidates']['ordered_grasp_candidates'][0]
        rotate=Rotation.from_euler('z',90,degrees=True).as_matrix()
        origin=np.asarray(layout['objects']['mustard_bottle']['position'])
        at,bt=np.asarray(a['pregrasp_world']),np.asarray(b['pregrasp_world'])
        np.testing.assert_allclose(bt[:3,:3],rotate@at[:3,:3],atol=1e-12)
        np.testing.assert_allclose(bt[:3,3]-origin,rotate@(at[:3,3]-origin),atol=1e-12)
        self.assertEqual(old['source_support_collider_paths']['mustard_bottle'],'/World/envs/env_0/kitchen/CapSurface_03')


if __name__=='__main__':unittest.main()
