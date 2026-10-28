"""Real CPU MP4 fixtures: fixed camera selection must never become best-of."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import zipfile

import cv2
import numpy as np

from scripts.extensions.replicator import package_reviewable_physical_demos as bundle
from scripts.extensions.replicator import package_trajectory_demos as original


class ReviewableDemoPackagingTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(); self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        for module, key, value in ((bundle, 'ROOT', self.root), (original, 'ROOT', self.root),
                (bundle, 'TOTAL', 3), (original, 'TOTAL', 3), (bundle, 'RETAKE_TOTAL', 2),
                (original, 'KNOWN_WARNINGS', set())):
            mocked = patch.object(module, key, value); mocked.start(); self.addCleanup(mocked.stop)
        self.source = self.root/'reports/run10'
        self.first = self.root/'exports/first'
        self.retakes = self.root/'exports/retakes'
        self.dest = self.root/'exports/bundle'
        self.zip = self.dest.with_suffix('.zip')
        self.plan_path = self.root/'configs/camera_retake.json'
        self.settings = dict(device='cpu', seed=42, physics_threads=8)
        self.sources, self.first_entries, self.retake_entries = [], [], []
        self.ids = [bundle.RETAKE_SCENE_PREFIX+'variant_a', bundle.RETAKE_SCENE_PREFIX+'variant_b', 'other_task__variant_a']
        for index, sid in enumerate(self.ids):
            task = bundle.RETAKE_TASK if index < 2 else 'other_task'
            config = self.write(f'configs/{sid}.json', dict(task_id=task))
            traj = self.root/f'reports/trajectory_{index}.npz'; traj.parent.mkdir(parents=True, exist_ok=True)
            np.savez(traj, q=np.zeros((6,7)), tcp_world=np.repeat(np.eye(4)[None],6,axis=0),
                gripper=np.zeros(6), phase=np.array(original.PHASES), dt=np.array(.1))
            scene = self.write(self.source/'scenes'/sid/'source_scene.usda', b'# Synthetic source scene')
            row = dict(scene_id=sid, task_id=task, variant_id='variant_'+str(index), status='generated',
                variant_name='测试变体', category='robot', config_path=self.rel(config), config_sha256=original.sha(config))
            planning = dict(row, stage='complete', trajectory_complete=True, planning_only=True,
                executed=False, task_success_verified=False, trajectory_samples=6,
                composition=dict(scene_sha256=original.sha(scene)),
                phases=[dict(phase=p,status='generated',solver_success=True) for p in original.PHASES],
                scene_adjustment=dict(scene_quality_issues=['保留的质量提示'] if index==0 else []),
                artifacts=dict(trajectory=dict(path=self.rel(traj),sha256=original.sha(traj))))
            result = self.write(self.source/'scenes'/sid/'result.json', planning)
            entry = dict(row, result_path=self.rel(result), result_sha256=original.sha(result),
                trajectory_path=self.rel(traj), trajectory_sha256=original.sha(traj),
                source_scene_path=self.rel(scene), source_scene_sha256=original.sha(scene),
                planning_status='generated', execution_status='failed' if index==1 else 'success',
                executed=True, task_success=index!=1, attempts=1,
                execution_result_path=f'executions/{sid}/result.json', video_path=f'videos/{sid}.mp4',
                fps=10.,frame_count=6,width=32,height=24,duration_seconds=.6)
            self.video(self.first/entry['video_path'], 60+index*30)
            entry['video_sha256'] = original.sha(self.first/entry['video_path'])
            actual = dict(entry, kind='physical_execution', planning_result_path=entry['result_path'],
                planning_result_sha256=entry['result_sha256'], runtime_settings=deepcopy(self.settings))
            self.bind(self.first,entry,actual)
            self.sources.append(row); self.first_entries.append(entry)
        self.progress = self.write(self.source/'progress.json',dict(stage='complete',entries=self.sources))
        self.first_manifest = dict(schema_version=1,kind='physical_execution',stage='complete',
            source_run=self.rel(self.source),source_progress_sha256=original.sha(self.progress),entries=self.first_entries)
        self.write(self.first/'manifest.json',self.first_manifest)
        self.camera = dict(eye=[1.,2.,3.],renderer_settings={'exposure':.01})
        override = self.write('configs/overrides.json',dict(kind='physical_execution_camera_overrides',tasks={bundle.RETAKE_TASK:self.camera}))
        plans = []
        for old in self.first_entries[:2]:
            plans.append({**{k:old[k] for k in ('scene_id','task_id','variant_id','config_path','config_sha256',
                'trajectory_path','trajectory_sha256','source_scene_path','source_scene_sha256')},
                'physical_attempt_number':2, 'locked_execution_settings':deepcopy(self.settings),
                'first_pass':dict(result_path=self.rel(self.first/old['execution_result_path']),
                    result_sha256=old['execution_result_sha256'],video_path=self.rel(self.first/old['video_path']),
                    video_sha256=old['video_sha256'],execution_status=old['execution_status'],
                    task_success=old['task_success'],executed=True,physical_attempt_number=1)})
        self.plan = dict(schema_version=1,kind='physical_camera_retake',physical_attempt_number=2,
            select_all_retakes_regardless_of_success=True,first_pass_manifest_path=self.rel(self.first/'manifest.json'),
            camera_overrides_path=self.rel(override),camera_overrides_sha256=original.sha(override),entries=plans)
        self.write(self.plan_path,self.plan)
        for index, (old, plan_row) in enumerate(zip(self.first_entries,plans)):
            entry = deepcopy(old)
            entry.update(execution_status='failed',task_success=False,physical_attempt_number=2,
                first_pass=deepcopy(plan_row['first_pass']),camera_retake_plan_path=self.rel(self.plan_path),
                camera_retake_plan_sha256=original.sha(self.plan_path))
            self.video(self.retakes/entry['video_path'],180+index*20)
            entry['video_sha256']=original.sha(self.retakes/entry['video_path'])
            actual = dict(entry,kind='physical_execution',planning_result_path=entry['result_path'],
                planning_result_sha256=entry['result_sha256'],runtime_settings={**self.settings,
                    'camera_overrides_path':self.plan['camera_overrides_path'],
                    'camera_overrides_sha256':self.plan['camera_overrides_sha256'],
                    'camera_retake_plan_path':self.rel(self.plan_path),'camera_retake_plan_sha256':original.sha(self.plan_path)},
                camera_override=dict(path=self.rel(override),sha256=original.sha(override),task_id=bundle.RETAKE_TASK,applied=deepcopy(self.camera)),
                camera_renderer=dict(requested=self.camera['renderer_settings'],applied=self.camera['renderer_settings'],restored=True))
            self.bind(self.retakes,entry,actual);self.retake_entries.append(entry)
        self.retake_manifest = dict(schema_version=1,kind='physical_execution',stage='complete',
            source_run=self.rel(self.source),source_progress_sha256=original.sha(self.progress),entries=self.retake_entries)
        self.save_retake_manifest()

    def rel(self,path): return path.relative_to(self.root).as_posix()

    def write(self,path,value):
        path=self.root/path;path.parent.mkdir(parents=True,exist_ok=True)
        path.write_bytes(value if isinstance(value,bytes) else json.dumps(value).encode());return path

    def video(self,path,color):
        path.parent.mkdir(parents=True,exist_ok=True)
        writer=cv2.VideoWriter(str(path),cv2.VideoWriter_fourcc(*'mp4v'),10.,(32,24))
        self.assertTrue(writer.isOpened())
        for n in range(6):
            frame=np.full((24,32,3),color,dtype=np.uint8);frame[:,n:n+4]=10;writer.write(frame)
        writer.release()

    def bind(self,root,entry,actual):
        path=self.write(root/entry['execution_result_path'],actual);entry['execution_result_sha256']=original.sha(path)

    def save_retake_manifest(self): self.write(self.retakes/'manifest.json',self.retake_manifest)

    def change_actual(self,**changes):
        row=self.retake_entries[0];path=self.retakes/row['execution_result_path'];actual=original.read(path)
        actual.update(changes);row.update({k:v for k,v in changes.items() if k!='runtime_settings'})
        self.bind(self.retakes,row,actual);self.save_retake_manifest()

    def run_package(self,validate_only=False):
        return bundle.package(self.first,self.retakes,self.source,self.plan_path,self.dest,self.zip,workers=1,validate_only=validate_only)

    def test_complete_bundle_keeps_worse_retake_and_all_original_bytes(self):
        first_manifest=(self.first/'manifest.json').read_bytes()
        original_video=(self.first/self.first_entries[0]['video_path']).read_bytes()
        new_video=(self.retakes/self.retake_entries[0]['video_path']).read_bytes()
        report=self.run_package()
        self.assertEqual(report['first_pass_counts'],{'failed':1,'success':2})
        self.assertEqual(report['selected_demo_counts'],{'failed':2,'success':1})
        selection=original.read(self.dest/'demo_selection.json');row=selection['entries'][1]
        # The originally successful A scene must still choose its failed camera retake.
        row=next(x for x in selection['entries'] if x['scene_id']==self.ids[0])
        self.assertTrue(row['first_pass']['task_success']);self.assertFalse(row['selected_demo']['task_success'])
        self.assertEqual(row['selected_demo']['physical_attempt_number'],2)
        self.assertEqual((self.dest/row['selected_demo']['video_package_path']).read_bytes(),new_video)
        self.assertEqual((self.dest/row['first_pass']['video_package_path']).read_bytes(),original_video)
        self.assertEqual((self.first/'manifest.json').read_bytes(),first_manifest)
        self.assertEqual((self.dest/'provenance/first_pass/manifest.json').read_bytes(),first_manifest)
        with zipfile.ZipFile(self.zip) as archive:
            self.assertEqual(sum(n.endswith('.mp4') for n in archive.namelist()),5)
            self.assertEqual(sum(n.startswith('videos/') for n in archive.namelist()),3)
            self.assertEqual(sum(n.startswith('first_pass_results/') for n in archive.namelist()),3)
            self.assertEqual(sum(n.startswith('selected_results/') for n in archive.namelist()),3)
            for line in archive.read('SHA256SUMS.txt').decode().splitlines():
                digest,name=line.split('  ',1);self.assertEqual(hashlib.sha256(archive.read(name)).hexdigest(),digest)
        page=(self.dest/'index.html').read_text();self.assertIn('首轮 154 景成绩',page)
        self.assertIn('固定选择全部 11 次重录',page);self.assertIn('保留的质量提示',page)

    def test_missing_retake_does_not_publish_partial_bundle(self):
        self.retake_manifest['entries'].pop();self.save_retake_manifest()
        with self.assertRaisesRegex(ValueError,'Missing, duplicate or foreign'):
            self.run_package()
        self.assertFalse(self.dest.exists());self.assertFalse(self.zip.exists())

    def test_retake_must_record_actual_attempt_two(self):
        self.change_actual(physical_attempt_number=1)
        with self.assertRaisesRegex(ValueError,'physical attempt 2'):self.run_package()

    def test_forged_first_pass_outcome_is_rejected(self):
        first=deepcopy(self.retake_entries[0]['first_pass']);first['task_success']=False
        self.change_actual(first_pass=first)
        with self.assertRaisesRegex(ValueError,'snapshot differs'):self.run_package()

    def test_locked_controller_setting_change_is_rejected(self):
        path=self.retakes/self.retake_entries[0]['execution_result_path'];settings=original.read(path)['runtime_settings']
        settings['seed']=43;self.change_actual(runtime_settings=settings)
        with self.assertRaisesRegex(ValueError,'settings changed beyond'):self.run_package()

    def test_outcome_selection_and_incomplete_first_run_are_rejected(self):
        self.plan['select_all_retakes_regardless_of_success']=False;self.write(self.plan_path,self.plan)
        with self.assertRaisesRegex(ValueError,'without outcome selection'):self.run_package()
        self.first_manifest['stage']='executing';self.write(self.first/'manifest.json',self.first_manifest)
        with self.assertRaisesRegex(ValueError,'not complete'):self.run_package()

    def test_changed_first_video_and_missing_new_video_are_rejected(self):
        old=self.first/self.first_entries[0]['video_path'];data=old.read_bytes();old.write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError,'SHA256 mismatch'):self.run_package()
        old.write_bytes(data);(self.retakes/self.retake_entries[0]['video_path']).unlink()
        with self.assertRaisesRegex(ValueError,'fixed 11 MP4'):self.run_package()

    def test_validate_only_leaves_all_outputs_unwritten(self):
        report=self.run_package(validate_only=True)
        self.assertEqual(report['videos_fully_decoded'],5)
        self.assertFalse(self.dest.exists());self.assertFalse(self.zip.exists())


if __name__=='__main__': unittest.main()
