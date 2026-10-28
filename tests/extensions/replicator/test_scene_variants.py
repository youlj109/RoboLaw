"""Integrity checks for the complete first-stage 14 × 11 scene delivery."""
import hashlib
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[3]


def read(path):
    return json.loads((ROOT/path).read_text())


class SceneVariantDeliveryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest = read('configs/extensions/replicator/scene_variants/manifest.json')
        cls.tasks = read('student_skills/robolaw-replicator/references/assignment.json')['tasks']
        cls.assets = {x['id']: x for category in ('property', 'robot')
            for x in read(f'assets/obstacle_library/{category}/catalog.json')['assets']}

    def test_exact_assigned_cross_product(self):
        tasks = {row['id'] for row in self.tasks}
        variants = {'human_default_mpfb', *read('configs/reproduction_presets.json')['nonhuman10']['ids']}
        rows = self.manifest['scenes']
        self.assertEqual(len(tasks), 14)
        self.assertEqual(len(variants), 11)
        self.assertEqual(len(rows), 154)
        self.assertEqual({(r['task_id'], r['variant_id']) for r in rows},
                         {(task, variant) for task in tasks for variant in variants})
        self.assertEqual(len({r['config_path'] for r in rows}), 154)

    def test_original_task_pose_values_unchanged(self):
        for row in self.manifest['scenes']:
            layout = read(row['config_path'])
            short = row['task_id'].removeprefix('kitchen_bench_replicator_')
            source = read('configs/extensions/replicator/' +
                ('g_shape_mustard.json' if short == 'g_shape_mustard_bowl' else short+'_source_layout.json'))
            for key in ('task_id', 'task', 'kitchen_usd', 'stand_height', 'robot_position', 'robot_yaw_deg'):
                self.assertEqual(layout[key], source[key], (row['scene_id'], key))
            if short == 'g_shape_mustard_bowl':
                self.assertEqual(layout['objects']['mustard_bottle']['position'], source['mustard_position'])
                self.assertEqual(layout['objects']['mustard_bottle']['yaw_deg'], source['mustard_yaw_deg'])
                self.assertEqual(layout['objects']['bowl']['position'], source['bowl_position'])
                self.assertEqual(layout['objects']['bowl']['yaw_deg'], source['bowl_yaw_deg'])
            else:
                self.assertEqual(layout['objects'], source['objects'])

    def test_original_obstacle_physics_and_assets_unchanged(self):
        for row in self.manifest['scenes']:
            hazard = read(row['config_path'])['obstacle']
            self.assertEqual(hazard['id'], row['variant_id'])
            self.assertEqual(hazard['category'], row['category'])
            self.assertTrue((ROOT/hazard['usd_path']).is_file())
            if hazard['id'] == 'human_default_mpfb':
                self.assertEqual(hazard['usd_path'], read('configs/layout.json')['human']['asset'])
                self.assertIs(hazard['static'], True)
            else:
                for key, value in self.assets[hazard['id']].items():
                    if key not in ('position', 'yaw_deg'):
                        self.assertEqual(hazard[key], value, (row['scene_id'], key))

    def test_every_configuration_has_bound_cpu_evidence(self):
        for row in self.manifest['scenes']:
            evidence = read(row['cpu_evidence_path'])
            self.assertEqual(hashlib.sha256((ROOT/row['config_path']).read_bytes()).hexdigest(), evidence['config_sha256'])
            self.assertIs(evidence['placement']['passed'], True)
            self.assertIs(evidence['original_task_poses_preserved'], True)
            self.assertIs(evidence['avoidance_verified'], False)
            self.assertIs(evidence['task_success_verified'], False)
            self.assertGreaterEqual(evidence['placement']['source_geometry_clearance_m'], .002)
            self.assertGreaterEqual(evidence['placement']['task_object_and_stand_clearance_m'], .015)
            for path, expected in evidence['hazard_asset_sha256'].items():
                self.assertEqual(hashlib.sha256((ROOT/path).read_bytes()).hexdigest(), expected)


if __name__ == '__main__':
    unittest.main()
