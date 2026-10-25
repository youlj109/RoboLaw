import tempfile
import unittest
from pathlib import Path

from kitchen_safety.extensions.replicator.background_physics import (
    HOUSING, KETTLE_BODIES, MOUNT, RECIPE, SOLVER_ATTRIBUTES, mounted_kitchen, validate_local_authoring)


class BackgroundPhysicsTest(unittest.TestCase):
    def setUp(self):
        from pxr import Usd, UsdPhysics
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name); self.source = self.root/'source.usda'
        self.stage = Usd.Stage.CreateNew(str(self.source)); self.stage.DefinePrim('/World')
        self.stage.SetDefaultPrim(self.stage.GetPrimAtPath('/World'))
        for path in [HOUSING, *KETTLE_BODIES]:
            prim = self.stage.DefinePrim(path)
            UsdPhysics.RigidBodyAPI.Apply(prim)
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(.123)
        self.stage.GetRootLayer().Save()

    def test_v2_only_adds_named_solver_attributes_and_original_mount(self):
        from pxr import Usd, UsdPhysics
        source_bytes = self.source.read_bytes()
        path, info = mounted_kitchen(self.source, self.root/'overlays')
        self.assertEqual(info['recipe'], RECIPE)
        self.assertTrue(info['local_authoring_validation']['passed'])
        self.assertEqual(len(info['local_authoring_validation']['solver_attributes']), 8)
        self.assertEqual([x['passed'] for x in info['kettle_solver']['diagnostics']], [False, False, True])
        stage = Usd.Stage.Open(str(path))
        self.assertEqual(UsdPhysics.FixedJoint(stage.GetPrimAtPath(MOUNT)).GetBody1Rel().GetTargets(), [HOUSING])
        for body in KETTLE_BODIES:
            prim = stage.GetPrimAtPath(body)
            self.assertAlmostEqual(UsdPhysics.MassAPI(prim).GetMassAttr().Get(), .123)
            self.assertEqual({name: prim.GetAttribute(name).Get() for name in SOLVER_ATTRIBUTES}, SOLVER_ATTRIBUTES)
        self.assertEqual(self.source.read_bytes(), source_bytes)
        self.assertEqual(mounted_kitchen(self.source, self.root/'overlays')[1], info)

    def test_extra_mass_pose_or_joint_edit_is_rejected(self):
        from pxr import Sdf, Usd
        path, _ = mounted_kitchen(self.source, self.root/'overlays')
        stage = Usd.Stage.Open(str(path))
        stage.GetPrimAtPath(KETTLE_BODIES[0]).CreateAttribute('physics:mass', Sdf.ValueTypeNames.Float).Set(1.)
        with self.assertRaisesRegex(ValueError, 'Unexpected local authored attribute'):
            validate_local_authoring(stage)

    def test_missing_source_body_cannot_be_silently_omitted(self):
        self.stage.RemovePrim(KETTLE_BODIES[0]); self.stage.GetRootLayer().Save()
        with self.assertRaisesRegex(ValueError, 'Missing source Kettle'):
            mounted_kitchen(self.source, self.root/'overlays')


if __name__ == '__main__':
    unittest.main()
