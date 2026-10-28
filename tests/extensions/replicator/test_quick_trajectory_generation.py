"""CPU checks for truthful one-shot trajectory artifacts and failure hints."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np


ROOT=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('quick_generation',ROOT/'scripts/extensions/replicator/quick_trajectory_generate.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class Kin:
    def fk(self,q):
        value=np.eye(4);value[:3,3]=q[:3];return value


class QuickArtifactsTests(unittest.TestCase):
    def test_partial_prefix_never_gets_complete_trajectory_key(self):
        with tempfile.TemporaryDirectory() as directory:
            result=dict(status='failed',scene_id='example',config_sha256='abc',artifacts={})
            paths=[np.zeros((3,7)),np.ones((2,7))]
            module.save_path(Path(directory),result,paths,['pregrasp','approach'],[0.,0.],Kin(),1/30)
            self.assertFalse(result['trajectory_complete'])
            self.assertTrue(result['partial_trajectory'])
            self.assertNotIn('trajectory',result['artifacts'])
            archive=np.load(Path(directory)/'partial_trajectory.npz',allow_pickle=False)
            self.assertEqual(archive['q'].shape,(5,7))
            self.assertEqual(archive['tcp_world'].shape,(5,4,4))
            self.assertEqual(archive['phase'].tolist(),['pregrasp']*3+['approach']*2)
            metadata=json.loads((Path(directory)/'partial_trajectory.json').read_text())
            self.assertFalse(metadata['complete'])
            self.assertEqual(metadata['phase_ranges'][1]['start'],3)

    def test_generated_archive_has_numeric_finite_q_and_exact_phase_ranges(self):
        with tempfile.TemporaryDirectory() as directory:
            result=dict(status='generated',scene_id='example',config_sha256='abc',artifacts={})
            phases=['pregrasp','approach','lift','transfer','lower','retreat']
            module.save_path(Path(directory),result,[np.full((2,7),i) for i in range(6)],phases,[0,0,.5,.5,.5,0],Kin(),1/30)
            archive=np.load(Path(directory)/'trajectory.npz',allow_pickle=False)
            self.assertTrue(np.isfinite(archive['q']).all())
            self.assertTrue(result['trajectory_complete'])
            self.assertEqual(len(archive['q']),12)
            self.assertEqual(archive['gripper'].tolist(),[0,0,0,0,.5,.5,.5,.5,.5,.5,0,0])
            self.assertIn('trajectory',result['artifacts'])

    def test_no_prefix_does_not_invent_a_trajectory(self):
        with tempfile.TemporaryDirectory() as directory:
            result=dict(status='failed',artifacts={})
            module.save_path(Path(directory),result,[],[],[],Kin(),1/30)
            self.assertEqual(list(Path(directory).iterdir()),[])
            self.assertEqual(result['artifacts'],{})

    def test_lift_start_collision_hint_names_phase_start(self):
        hint=module.suggest('MotionGenStatus.INVALID_START_STATE_WORLD_COLLISION','lift')
        self.assertIn('上一阶段终点',hint)
        self.assertNotIn('截图初始',hint)

    def test_counts_include_every_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            progress=dict(entries=[dict(status=x) for x in ['failed','generated','failed','pending','running']])
            module.save_progress(Path(directory),progress)
            self.assertEqual(progress['counts'],dict(pending=1,running=1,generated=1,failed=2,skipped=0))


if __name__=='__main__':unittest.main()
