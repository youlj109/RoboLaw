import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from kitchen_safety.extensions.replicator import banana_background_physics as leaf
from kitchen_safety.extensions.replicator import background_physics as base


class BananaBackgroundPhysicsTest(unittest.TestCase):
    def setUp(self):
        from pxr import Usd, UsdPhysics
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name); self.source = self.root/'source.usda'
        self.stage = Usd.Stage.CreateNew(str(self.source)); self.stage.DefinePrim('/World')
        self.stage.SetDefaultPrim(self.stage.GetPrimAtPath('/World'))
        for path in [base.HOUSING, *base.KETTLE_BODIES]:
            prim = self.stage.DefinePrim(path)
            UsdPhysics.RigidBodyAPI.Apply(prim)
            mass = UsdPhysics.MassAPI.Apply(prim)
            mass.CreateMassAttr(.123); mass.CreateCenterOfMassAttr((.01,.02,.03))
            mass.CreateDiagonalInertiaAttr((.04,.05,.06))
        joint = UsdPhysics.RevoluteJoint.Define(self.stage, '/World/KettleA01/hinge')
        joint.CreateBody1Rel().SetTargets([base.KETTLE_BODIES[2]])
        joint.CreateLowerLimitAttr(-12.); joint.CreateUpperLimitAttr(34.)
        self.stage.GetRootLayer().Save()
        self.base_path, self.base_info = base.mounted_kitchen(self.source, self.root/'base')

    def test_eight_local_solver_values_preserve_original_properties_and_base(self):
        from pxr import Usd, UsdPhysics
        hashes = [leaf.sha(self.source), leaf.sha(self.base_path)]
        out = self.root/'leaf.usda'
        report = leaf.create_leaf(self.base_path,out)
        self.assertTrue(report['passed']); self.assertEqual(len(report['solver_attributes']),8)
        stage = Usd.Stage.Open(str(out))
        for path in base.KETTLE_BODIES:
            prim = stage.GetPrimAtPath(path); mass = UsdPhysics.MassAPI(prim)
            self.assertAlmostEqual(mass.GetMassAttr().Get(),.123)
            self.assertEqual(tuple(mass.GetCenterOfMassAttr().Get()), tuple(UsdPhysics.MassAPI(self.stage.GetPrimAtPath(path)).GetCenterOfMassAttr().Get()))
            self.assertEqual(tuple(mass.GetDiagonalInertiaAttr().Get()),tuple(UsdPhysics.MassAPI(self.stage.GetPrimAtPath(path)).GetDiagonalInertiaAttr().Get()))
            self.assertEqual({k:prim.GetAttribute(k).Get() for k in leaf.SOLVER_ATTRIBUTES},leaf.SOLVER_ATTRIBUTES)
            self.assertTrue(prim.HasAPI(UsdPhysics.RigidBodyAPI))
        joint = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath('/World/KettleA01/hinge'))
        self.assertEqual(joint.GetLowerLimitAttr().Get(),-12.); self.assertEqual(joint.GetUpperLimitAttr().Get(),34.)
        self.assertEqual(UsdPhysics.FixedJoint(stage.GetPrimAtPath(base.MOUNT)).GetBody1Rel().GetTargets(),[base.HOUSING])
        self.assertEqual(hashes,[leaf.sha(self.source),leaf.sha(self.base_path)])

    def test_mass_pose_joint_and_time_sample_changes_are_rejected(self):
        from pxr import Sdf, Usd
        out = self.root/'leaf.usda'; leaf.create_leaf(self.base_path,out); stage=Usd.Stage.Open(str(out))
        prim=stage.GetPrimAtPath(base.KETTLE_BODIES[0])
        for name in ['physics:mass','xformOp:translate','physics:collisionEnabled']:
            prim.CreateAttribute(name,Sdf.ValueTypeNames.Float).Set(99.)
            with self.assertRaisesRegex(ValueError,'attribute'): leaf.validate_local_authoring(stage,self.base_path)
            prim.RemoveProperty(name)
        prim.CreateRelationship('physics:body0').SetTargets(['/World'])
        with self.assertRaisesRegex(ValueError,'relationships'): leaf.validate_local_authoring(stage,self.base_path)
        prim.RemoveProperty('physics:body0')
        prim.GetAttribute('physxRigidBody:solverPositionIterationCount').Set(255,Usd.TimeCode(1.))
        with self.assertRaisesRegex(ValueError,'attribute'): leaf.validate_local_authoring(stage,self.base_path)

    def test_explicit_api_replacement_and_sublayer_offset_rejected(self):
        from pxr import Sdf, Usd
        out=self.root/'leaf.usda';leaf.create_leaf(self.base_path,out);stage=Usd.Stage.Open(str(out))
        prim=stage.GetPrimAtPath(base.KETTLE_BODIES[0]);prim.SetMetadata('apiSchemas',Sdf.TokenListOp.CreateExplicit(['PhysxRigidBodyAPI']))
        with self.assertRaisesRegex(ValueError,'API schema'):leaf.validate_local_authoring(stage,self.base_path)
        stage.GetRootLayer().subLayerOffsets[0]=Sdf.LayerOffset(1.)
        with self.assertRaisesRegex(ValueError,'inherit'):leaf.validate_local_authoring(stage,self.base_path)

    def test_cache_provenance_binds_both_outcomes_and_rejects_tampering(self):
        layout={'task_id':leaf.TASK,'kitchen_usd':str(self.source)}
        layout_path=self.root/'layout.json';layout_path.write_text(json.dumps(layout))
        with patch.object(leaf,'LAYOUT_PATH',layout_path),patch.object(leaf,'LAYOUT_SHA256',leaf.sha(layout_path)),patch.object(leaf,'SOURCE_SHA256',leaf.sha(self.source)):
            out,info=leaf.mounted_banana_kitchen(self.source,self.root/'public',layout=layout)
            self.assertFalse(info['physical_profile_certified'])
            self.assertEqual([x['passed'] for x in info['kettle_solver']['diagnostics']],[True,False])
            self.assertEqual(info['recipe_sha256'],leaf.sha(info['recipe_source_path']))
            self.assertEqual(info['base_overlay_sha256'],leaf.sha(info['base_overlay_path']))
            self.assertEqual(leaf.mounted_banana_kitchen(self.source,self.root/'public',layout=layout)[1],info)
            saved=json.loads(out.with_suffix('.json').read_text());saved['physical_profile_certified']=True
            out.with_suffix('.json').write_text(json.dumps(saved))
            with self.assertRaisesRegex(ValueError,'Cached'):leaf.mounted_banana_kitchen(self.source,self.root/'public',layout=layout)
            with self.assertRaisesRegex(ValueError,'canonical'):leaf.mounted_banana_kitchen(self.source,self.root/'other',layout={**layout,'extra':1})

    def test_existing_output_or_missing_body_rejected(self):
        out=self.root/'leaf.usda';leaf.create_leaf(self.base_path,out)
        with self.assertRaises(FileExistsError):leaf.create_leaf(self.base_path,out)
        # Removing the source body cannot be repaired by inherited solver overs.
        self.stage.RemovePrim(base.KETTLE_BODIES[0]);self.stage.GetRootLayer().Save()
        with self.assertRaisesRegex(ValueError,'inventory'):leaf.create_leaf(self.base_path,self.root/'bad.usda')


if __name__=='__main__':
    unittest.main()
