"""Fixed-scene conversion must preserve original task and matching controls."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from kitchen_safety.extensions.replicator import layout as legacy
from scripts.extensions.replicator.safety_scene_trial_support import (
    TASK, collector_arguments, convert_scene, review_jobs, select_baseline, sha,
)


IDS = ('human_default_mpfb', 'property_tall_glass', 'property_wine_glass', 'property_glass_bottle',
       'property_ceramic_mug', 'property_vase', 'property_tablet_stand', 'robot_steel_clamp',
       'robot_projecting_crossbar', 'robot_narrow_opening', 'robot_sharp_equipment_housing')


def canonical(reference):
    result = {key: copy.deepcopy(reference[key]) for key in
              ('task_id', 'kitchen_usd', 'stand_height', 'robot_position', 'robot_yaw_deg')}
    result['objects'] = {
        name: dict(usd_path=asset, position=reference[prefix + '_position'], yaw_deg=reference[prefix + '_yaw_deg'])
        for name, prefix, asset in [('mustard_bottle', 'mustard', 'assets/robolab/objects/hope/mustard_bottle.usd'),
                                    ('bowl', 'bowl', 'assets/robolab/objects/ycb/bowl.usd')]}
    result['obstacle'] = copy.deepcopy(reference['obstacle'])
    result['obstacle'].update(position=[.52, -.6, 1.01], yaw_deg=27.)
    return result


class SafetyTrialTests(unittest.TestCase):
    def setUp(self):
        self.reference = legacy.resolve_layout('property_tall_glass')
        self.scene = canonical(self.reference)

    def test_four_legacy_outputs_are_unchanged(self):
        base = json.loads((legacy.ROOT / 'configs/extensions/replicator/g_shape_mustard.json').read_text())
        self.assertEqual(legacy.resolve_layout(), base)
        for identifier, position, yaw in (
                ('human_default_mpfb', base['human']['position'], base['human']['yaw_deg']),
                ('property_vase', [.61, -.61, base['counter_height'] + .002], 0),
                ('robot_projecting_crossbar', [.61, -.84, base['counter_height'] + .002], 180)):
            expected = copy.deepcopy(base)
            expected['obstacle'] = (dict(id=identifier, category='human', static=True,
                usd_path=base['human']['asset']) if identifier == 'human_default_mpfb' else copy.deepcopy(legacy.get_asset(identifier)))
            expected['obstacle'].update(position=position, yaw_deg=yaw)
            self.assertEqual(legacy.resolve_layout(identifier), expected)

    def test_all_eleven_convert_only_hazard_pose(self):
        for identifier in IDS:
            reference = legacy.resolve_layout(identifier)
            output = convert_scene(canonical(reference), reference)
            expected = copy.deepcopy(reference)
            expected['obstacle'].update(position=[.52, -.6, 1.01], yaw_deg=27.)
            self.assertEqual(output, expected)
            self.assertNotEqual(output['obstacle']['position'], reference['obstacle']['position'])

    def test_catalog_path_escape_and_unknown_id_rejected(self):
        for asset in ('/etc/passwd', 'assets/people/mpfb_corridor/collidable.usda',
                      'assets/obstacle_library/property/missing.usda'):
            entry = {**self.reference['obstacle'], 'usd_path': asset}
            with patch.object(legacy, 'get_asset', return_value=entry), self.assertRaises(ValueError):
                legacy.resolve_layout('property_tall_glass')
        with patch.object(legacy, 'get_asset', return_value=self.reference['obstacle']), self.assertRaises(ValueError):
            legacy.resolve_layout('invented_id')

    def test_geometry_physics_and_unsupported_tasks_rejected(self):
        changes = [lambda x: x.update(task_id='kitchen_bench_replicator_u_shape_mustard_bowl'),
                   lambda x: x.update(robot_position=[0, 0, 0]),
                   lambda x: x['objects']['bowl'].update(position=[0, 0, 0]),
                   lambda x: x['objects']['mustard_bottle'].update(scale=[2, 2, 2]),
                   lambda x: x['obstacle'].update(static=True),
                   lambda x: x['obstacle'].update(usd_path='different.usda'),
                   lambda x: x['obstacle'].update(scale=[2, 2, 2]),
                   lambda x: x['obstacle'].update(position=[float('nan'), 0, 0]),
                   lambda x: x['obstacle'].update(yaw_deg=float('inf'))]
        for change in changes:
            scene = copy.deepcopy(self.scene)
            change(scene)
            with self.assertRaises(ValueError):
                convert_scene(scene, self.reference)

    def inventory(self, root):
        record = root / 'record.json'
        record.write_text('{"success":true}')
        controls = dict(initial_settling_seconds=5., initial_settling_actions=75, seed=42, planner_seed=42,
            profile='base', transport_object_padding_m=0., transfer_mode='goal',
            collector_options=dict(grasp_tilt_deg=35., grasp_yaw_deg=180., grasp_offset_z_m=0.,
                grasp_offset_y_m=-.01, transport_bottle_height_m=1.085, supported_release=True,
                transport_tilt_cap_deg=None, placement_strategy='pose', placement_tcp_height_m=None))
        config = dict(simulation_device='cpu', physics_threads=2, torch_threads=2, execution_checks='deferred',
                      camera_owner='writer', camera_device=None, episode_length_s=180., max_steps=0,
                      background_rgb=True, cpu_rgb_copy=True, reuse_planner=False)
        base = dict(profile='base', status='verified_baseline', checks=dict(independent=True), files={'record.json': sha(record)},
                    source_layout={k: v for k, v in self.reference.items() if k != 'obstacle'},
                    controls=controls, execution_configuration=config, episode_path='fixture/base')
        human = copy.deepcopy(base)
        human.update(profile='human', episode_path='fixture/humanbase')
        human['controls'].update(profile='human', transport_object_padding_m=.015)
        return dict(tasks=[dict(task_id=TASK, status='verified_baseline', baselines=[base, human])])

    def test_matching_baseline_and_every_control_are_explicit(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            inventory = self.inventory(root)
            base = select_baseline(root, inventory, self.reference, 'property')
            self.assertEqual(base['profile'], 'base')
            human = select_baseline(root, inventory, legacy.resolve_layout('human_default_mpfb'), 'human')
            self.assertEqual(human['profile'], 'human')
            command = collector_arguments('layout.json', 'episode', self.reference, base)
            self.assertIn('--layout-json', command)
            self.assertNotIn('--episode-spec', command)
            self.assertIn('--record-rgb', command)
            self.assertIn('--overview-camera', command)
            self.assertIn('--supported-release', command)
            self.assertEqual(command[command.index('--seed') + 1], '42')
            self.assertEqual(command[command.index('--grasp-yaw-deg') + 1], '180.0')

    def test_invalid_baseline_never_silently_falls_back(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            valid = self.inventory(root)
            changes = [lambda x: x['tasks'][0].update(status='blocked_baseline'),
                       lambda x: x['tasks'][0]['baselines'][0]['checks'].update(independent=False),
                       lambda x: x['tasks'][0]['baselines'][0]['controls'].update(transport_object_padding_m=.1),
                       lambda x: x['tasks'][0]['baselines'][0]['controls'].update(planner_seed=3),
                       lambda x: x['tasks'][0]['baselines'][0]['controls']['collector_options'].update(unknown=1)]
            for change in changes:
                inventory = copy.deepcopy(valid)
                change(inventory)
                with self.assertRaises(ValueError):
                    select_baseline(root, inventory, self.reference, 'property')
            (root / 'record.json').write_text('{"success":false}')
            with self.assertRaises(ValueError):
                select_baseline(root, valid, self.reference, 'property')

    def test_independent_reviews_include_no_aggregate_acceptance(self):
        jobs = review_jobs('episode', 'report', 'layout.json', 'base')
        names = {item['name'] for item in jobs}
        self.assertTrue({'source_relations', 'fixed_base', 'self_collision', 'native_recording_integrity',
                         'source_geometry_safe', 'baseline_source_geometry_risk', 'video_export'} <= names)
        self.assertNotIn('accepted', names)


if __name__ == '__main__':
    unittest.main()
