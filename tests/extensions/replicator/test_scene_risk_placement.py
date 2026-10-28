"""Evidence-scope and trajectory-selection contracts for the offline screener."""
import json
from pathlib import Path
import tempfile
import unittest

from kitchen_safety.extensions.replicator.scene_risk_placement import (
    initial_entries,initialize_reports,normalize_layout,sample_indices,ROOT,
)


class SceneRiskPlacementTests(unittest.TestCase):
    def test_complete_matrix_starts_unverified(self):
        tasks=json.loads((ROOT/'student_skills/robolaw-replicator/references/assignment.json').read_text())['tasks']
        entries=[e for task in tasks for e in initial_entries(task['id'])]
        self.assertEqual(len(entries),154)
        self.assertEqual(len({e['scene_id'] for e in entries}),154)
        for entry in entries:
            self.assertEqual(entry['status'],'waiting_baseline')
            self.assertIsNone(entry['config_path'])
            for key in ('geometry_passed','risk_verified','native_stability_verified','safe_execution_verified','video_saved'):
                self.assertIs(entry[key],False)

    def test_initialization_preserves_existing_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            initialize_reports(directory)
            paths=list(Path(directory).glob('tasks/*/result.json'))
            self.assertEqual(len(paths),14)
            path=paths[0];original=json.loads(path.read_text());original['sentinel']='existing result'
            path.write_text(json.dumps(original));before=path.read_bytes()
            initialize_reports(directory)
            self.assertEqual(path.read_bytes(),before)

    def test_sample_selection_covers_phase_boundaries_without_inventing_states(self):
        phases=['startup']*7+['approach']*3+['close']*4+['transfer']*20+['lower']*3+['release']*4+['retreat']*4
        sample,fixed=sample_indices(phases,count=5)
        self.assertEqual(sample,sorted(set(sample)))
        self.assertTrue(all(0<=i<len(phases) for i in sample))
        for i in (0,6,7,9,10,13,14,33,34,36,37,40,41,44):self.assertIn(i,sample)
        self.assertEqual(fixed,[0,9,13,36,40,44])
        self.assertNotIn(33,fixed)

    def test_schema_conversion_preserves_source_poses_and_copies(self):
        source=json.loads((ROOT/'configs/extensions/replicator/g_shape_mustard.json').read_text())
        result=normalize_layout(source)
        canonical=json.loads((ROOT/'configs/extensions/replicator/scene_variants/g_shape_mustard_bowl/property_vase.json').read_text())
        canonical.pop('obstacle')
        self.assertEqual(result,canonical)
        result['robot_position'][0]+=1
        self.assertNotEqual(result['robot_position'],source['robot_position'])


if __name__=='__main__':unittest.main()
