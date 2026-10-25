"""Reject proxy-only necessity, incomplete pairs and relaxed original margins."""
import copy
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from scripts.extensions.replicator.paired_variant_plans import densify, pair_gates, validate_seeds
from scripts.extensions.replicator.variant_geometry import (
    MeasuredHazardShapes, SourceRobotGeometry, expectation_passed, variant_settings,
)


def geometry(safe):
    return {mode: dict(passed=safe, violations=0 if safe else 1)
            for mode in ("source_mesh", "source_hull", "conservative_execution")}


class VariantPairGateTest(unittest.TestCase):
    def rows(self):
        return [dict(seed=seed, hazard_in_planning_world=on, success=True,
                     fixed_query_passed=True, conservative_planning_passed=on,
                     full_world_validation=dict(valid=True, validation_complete=True),
                     geometry=geometry(on)) for seed in (42, 43, 44) for on in (False, True)]

    def test_proxy_only_or_convex_only_risk_does_not_prove_source_necessity(self):
        rows = self.rows()
        self.assertTrue(all(p["passed"] for p in pair_gates(rows, [42, 43, 44])))
        rows[0]["geometry"]["source_mesh"] = dict(passed=True, violations=0)
        self.assertFalse(pair_gates(rows, [42, 43, 44])[0]["passed"])
        self.assertFalse(expectation_passed(rows[0]["geometry"], "risk"))

    def test_original_planning_margin_and_fixed_query_are_independent_gates(self):
        for key in ("conservative_planning_passed", "fixed_query_passed"):
            rows = self.rows()
            rows[1][key] = False
            self.assertFalse(pair_gates(rows, [42, 43, 44])[0]["passed"])
        rows = self.rows()
        rows[0]["full_world_validation"]["validation_complete"] = False
        self.assertFalse(pair_gates(rows, [42, 43, 44])[0]["passed"])

    def test_failure_missing_seed_duplicate_cannot_disappear_from_acceptance(self):
        rows = self.rows()
        rows[3]["success"] = False
        self.assertFalse(pair_gates(rows, [42, 43, 44])[1]["passed"])
        self.assertFalse(pair_gates(rows[:-1], [42, 43, 44])[-1]["passed"])
        self.assertFalse(pair_gates(rows + [copy.deepcopy(rows[0])], [42, 43, 44])[0]["passed"])

    def test_minimum_three_distinct_seeds(self):
        validate_seeds([42, 43, 44])
        for seeds in ([42], [42, 42, 44], [42, 43, -1]):
            with self.assertRaises(ValueError):
                validate_seeds(seeds)

    def test_variant_margins_and_padding_are_not_human_defaults(self):
        settings = dict(execution_clearances_m=dict(person=.02, glass=.05, barrier=.01),
                        clearances_m=dict(person=.04, glass=.06, barrier=.02),
                        transport_object_padding_m_by_variant=dict(human=.015))
        for variant, expected in [("human", ("person", .02, .04, .015)),
                                  ("property", ("glass", .05, .06, 0.)),
                                  ("robot", ("barrier", .01, .02, 0.))]:
            layout = dict(obstacle=dict(category=variant), planning=copy.deepcopy(settings))
            self.assertEqual(variant_settings(layout, variant), expected)
            layout["planning"]["execution_clearances_m"][expected[0]] *= .99
            with self.assertRaises(ValueError):
                variant_settings(layout, variant)
        for padding in (0., .01499, float("nan"), float("inf")):
            layout = dict(obstacle=dict(category="human"), planning=copy.deepcopy(settings))
            layout["planning"]["transport_object_padding_m_by_variant"]["human"] = padding
            with self.assertRaises(ValueError):
                variant_settings(layout, "human")

    def test_dense_samples_include_every_edge_and_endpoint(self):
        path = np.asarray([[0.] * 7, [.08] * 7, [.16] * 7])
        dense = densify(path)
        self.assertEqual(dense.shape, (17, 7))
        np.testing.assert_allclose(dense[8], path[1])
        np.testing.assert_allclose(dense[-1], path[-1])
        for invalid in (path[:1], np.zeros((2, 8)), path * np.nan):
            with self.assertRaises(ValueError):
                densify(invalid)

    def test_measured_source_primitives_move_without_accumulating_transforms(self):
        from pxr import Usd, UsdGeom, UsdPhysics
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/Hazard")
        cylinder = UsdGeom.Cylinder.Define(stage, "/Hazard/Cylinder")
        cylinder.CreateRadiusAttr(.1)
        cylinder.CreateHeightAttr(.4)
        UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())
        shapes = MeasuredHazardShapes(stage, "/Hazard", np.array([0, 0, 0, 0, 0, 0, 1]))
        np.testing.assert_allclose(shapes.at([1, 2, 3, 0, 0, 0, 1])[0][2], [.9, 1.9, 2.8])
        np.testing.assert_allclose(shapes.at([0, 0, 0, 0, 0, 0, 1])[0][2], [-.1, -.1, -.2])
        for _ in range(3):
            np.testing.assert_allclose(shapes.at([1, 2, 3, 0, 0, 0, 1])[0][2], [.9, 1.9, 2.8])

    def test_source_audit_error_writes_rejection_and_returns_nonzero(self):
        from scripts.extensions.replicator.audit_variant_source_geometry import main
        with tempfile.TemporaryDirectory() as tmp:
            report = Path(tmp) / "rejection.json"
            code = main(["--episode", tmp, "--layout", str(Path(tmp) / "missing.json"),
                         "--report", str(report), "--expect", "risk"])
            self.assertEqual(code, 1)
            self.assertFalse(json.loads(report.read_text())["passed"])
            with self.assertRaises(FileExistsError):
                main(["--episode", tmp, "--layout", "missing", "--report", str(report), "--expect", "safe"])

    def test_source_robot_includes_authored_collision_groups_without_convexifying(self):
        from pxr import Usd, UsdGeom, UsdPhysics
        from types import SimpleNamespace
        from unittest.mock import patch
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "robot.usda"
            stage = Usd.Stage.CreateNew(str(path))
            UsdGeom.Xform.Define(stage, "/panda")
            for name in ("link", "gripper"):
                body = UsdGeom.Xform.Define(stage, "/panda/" + name)
                UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
                group = UsdGeom.Xform.Define(stage, "/panda/" + name + "/colliders")
                UsdPhysics.CollisionAPI.Apply(group.GetPrim())
                mesh = UsdGeom.Mesh.Define(stage, str(group.GetPath()) + "/triangle")
                mesh.CreatePointsAttr([(0, 0, 0), (1, 0, 0), (0, 1, 0)])
                mesh.CreateFaceVertexCountsAttr([3])
                mesh.CreateFaceVertexIndicesAttr([0, 1, 2])
            stage.GetRootLayer().Save()

            def links(q):
                transform = np.eye(4)
                transform[0, 3] = q[0]
                return {"/panda/link": transform, "/panda/gripper": np.eye(4)}

            world = SimpleNamespace(kin=SimpleNamespace(links=links), gripper_root="/panda/gripper",
                                    _gripper_links=lambda grip: {"/panda/gripper": np.eye(4)})
            with patch("kitchen_safety.planning.ROBOT_USD", path):
                robot = SourceRobotGeometry(world)
            self.assertEqual(len(robot.parts), 2)
            self.assertTrue(all(p["triangles"] == 1 for p in robot.provenance["parts"]))
            self.assertTrue(all(p["collision_api_owner"].endswith("colliders") for p in robot.provenance["parts"]))
            initial = robot.shapes(np.zeros(7), {})
            moved = robot.shapes(np.asarray([.5] + [0.] * 6), {})
            self.assertTrue(all(s[0].startswith("source_robot") for s in moved))
            np.testing.assert_allclose(initial[0][2], [0, 0, 0])
            np.testing.assert_allclose(moved[0][2], [.5, 0, 0])


if __name__ == "__main__":
    unittest.main()
