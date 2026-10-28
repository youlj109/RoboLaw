"""CPU checks on source geometry and reversible water prerequisite states."""
import json
import unittest
from pathlib import Path

from pxr import Gf, Usd, UsdGeom, UsdPhysics

from kitchen_safety.extensions.replicator.hard_v1 import water_scene

ROOT = Path(__file__).resolve().parents[4]


class WaterSceneTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.layout = json.loads((ROOT / "configs/extensions/replicator/visual_scene_variants_final03/g_shape_bowl_sink/robot_projecting_crossbar.json").read_text())

    def make_scene(self):
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/World")
        kitchen = stage.DefinePrim("/World/kitchen/Asset")
        kitchen.GetReferences().AddReference(str(ROOT / self.layout["kitchen_usd"]))
        bowl = UsdGeom.Xform.Define(stage, water_scene.BOWL)
        bowl.AddTranslateOp().Set(Gf.Vec3d(*self.layout["objects"]["bowl"]["position"]))
        asset = stage.DefinePrim(water_scene.BOWL + "/Asset")
        asset.GetReferences().AddReference(str(ROOT / self.layout["objects"]["bowl"]["usd_path"]))
        obstacle = self.layout["obstacle"]
        hazard = UsdGeom.Xform.Define(stage, "/World/hazard")
        hazard.AddTranslateOp().Set(Gf.Vec3d(*obstacle["position"]))
        hazard.AddRotateZOp().Set(obstacle["yaw_deg"])
        stage.DefinePrim("/World/hazard/Asset").GetReferences().AddReference(str(ROOT / obstacle["usd_path"]))
        evidence = water_scene.build(stage, self.layout, "memory://water_scene")
        return stage, evidence

    def test_bowl_supported_within_real_basin_and_stream_has_no_collision(self):
        stage, result = self.make_scene()
        self.assertFalse(stage.GetCompositionErrors())
        self.assertAlmostEqual(result["measurements"]["bowl_support_gap_m"], .0015, places=6)
        self.assertGreater(result["measurements"]["minimum_basin_footprint_margin_m"], .012)
        self.assertGreater(result["measurements"]["visible_stream_height_m"], .05)
        self.assertEqual(result["destination_position"], self.layout["objects"]["bowl"]["position"])
        for prim in Usd.PrimRange(stage.GetPrimAtPath(water_scene.ROOT + "/FlowVisual")):
            self.assertFalse(prim.HasAPI(UsdPhysics.CollisionAPI))
            self.assertFalse(prim.HasAPI(UsdPhysics.RigidBodyAPI))

    def test_off_on_states_consistent_with_real_joint_and_no_animation(self):
        stage, _ = self.make_scene()
        root = stage.GetPrimAtPath(water_scene.ROOT)
        joint = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(water_scene.ROOT + "/ValveJoint"))
        target = joint.GetBody1Rel().GetTargets()[0]
        self.assertTrue(stage.GetPrimAtPath(target).HasAPI(UsdPhysics.RigidBodyAPI))
        for on, expected in [(False, 0.), (True, 55.)]:
            water_scene.apply_state(stage, on)
            self.assertEqual(root.GetAttribute("hard:flowing").Get(), on)
            self.assertEqual(UsdPhysics.DriveAPI(joint.GetPrim(), "angular").GetTargetPositionAttr().Get(), expected)
            visual = stage.GetPrimAtPath(water_scene.ROOT + "/FlowVisual")
            self.assertEqual(UsdGeom.Imageable(visual).ComputeVisibility(), "inherited" if on else "invisible")
        for prim in stage.Traverse():
            if str(prim.GetPath()).startswith(water_scene.ROOT):
                for attr in prim.GetAttributes():
                    self.assertEqual(attr.GetNumTimeSamples(), 0)

    def test_readback_angle_controls_water_without_commanding_valve(self):
        stage, _ = self.make_scene()
        root = stage.GetPrimAtPath(water_scene.ROOT)
        joint = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(water_scene.ROOT + "/ValveJoint"))
        target = UsdPhysics.DriveAPI(joint.GetPrim(), "angular").GetTargetPositionAttr()
        for angle, flowing, fraction in [(0., False, 0.), (2., False, 0.),
                                          (28.5, True, .5), (55., True, 1.)]:
            water_scene.sync_flow_from_joint_angle(stage, angle)
            self.assertEqual(root.GetAttribute("hard:flowing").Get(), flowing)
            self.assertAlmostEqual(root.GetAttribute("hard:valveOpenFraction").Get(), fraction)
            self.assertEqual(target.Get(), 55.)
        with self.assertRaises(ValueError):
            water_scene.sync_flow_from_joint_angle(stage, float("nan"))

    def test_visible_destination_clear_of_easy_obstacle_and_valve_clear_of_spout(self):
        stage, result = self.make_scene()
        marker = result["destination_marker"]
        self.assertEqual(result["task"], "请把水槽里的碗取出，放到台面的指定区域。")
        self.assertEqual(marker["center"][:2], result["destination_position"][:2])
        self.assertGreater(marker["minimum_obstacle_aabb_clearance_m"], .002)
        self.assertGreaterEqual(marker["radius_m"], .085)
        self.assertFalse(stage.GetPrimAtPath(marker["path"]).HasAPI(UsdPhysics.CollisionAPI))
        self.assertGreater(result["measurements"]["valve_lever_to_source_spout_aabb_clearance_m"], .002)
        self.assertGreater(result["measurements"]["source_spout_lowest_z_m"],
                           result["measurements"]["valve_lever_highest_z_m"])


if __name__ == "__main__":
    unittest.main()
