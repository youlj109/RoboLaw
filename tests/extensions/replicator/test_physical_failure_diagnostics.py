import copy
import json
from unittest.mock import patch
import tempfile
import unittest
from pathlib import Path
import numpy as np
from scripts.extensions.replicator import render_physical_retry_failures as replay


class DiagnosticReplayTests(unittest.TestCase):
    def states(self):
        phases=np.array(['reset','close','close','lift','transfer','lower','release','final_settle'])
        n=len(phases);poses=np.zeros((n,2,7));poses[...,6]=1
        arrays=dict(physics_step=np.arange(n),phase=phases,action_index=np.arange(n),source_trajectory_index=np.arange(n),
            q=np.zeros((n,1)),robot_body_pose_w=poses[:,:1].copy(),task_object_states=np.zeros((n,1,13)),all_rigid_body_pose_w=poses)
        arrays['task_object_states'][...,6]=1
        metadata=dict(all_rigid_body_paths=['/World/envs/env_0/kitchen/Door','/World/envs/env_0/Robot'],
            robot_body_names=['panda_link0'],joint_names=['joint'],object_names=['banana'],quaternion_convention='xyzw',samples=n)
        return arrays,metadata

    def test_actual_phase_endpoints_and_final_are_selected(self):
        a,m=self.states();replay.validate_states(a,m)
        selected=replay.select_frames(a)
        self.assertEqual([r['state_index'] for r in selected],[2,3,4,5,6,7])
        self.assertEqual(selected[-1]['phase'],'final_settle')

    def test_short_interrupted_execution_uses_only_observed_states(self):
        a,m=self.states();a={k:v[:2] for k,v in a.items()};m['samples']=2
        replay.validate_states(a,m);self.assertEqual([r['state_index'] for r in replay.select_frames(a)],[0,1])

    def test_malformed_or_noncontinuous_states_rejected(self):
        for mutation in ('missing_background','duplicate_paths','gap','bad_quaternion'):
            a,m=self.states()
            if mutation=='missing_background':del a['all_rigid_body_pose_w']
            elif mutation=='duplicate_paths':m['all_rigid_body_paths'][1]=m['all_rigid_body_paths'][0]
            elif mutation=='gap':a['physics_step'][3]+=1
            else:a['all_rigid_body_pose_w'][0,0,6]=0
            with self.subTest(mutation=mutation),self.assertRaises(ValueError):replay.validate_states(a,m)

    def hint_binding(self):
        return dict(execution_result_path='reports/actual/result.json',execution_result_sha256='a'*64,
                    physics_states_path='reports/actual/physics_states.npz',physics_states_sha256='b'*64,
                    physics_metadata_path='reports/actual/physics_metadata.json',physics_metadata_sha256='c'*64)

    def hints(self,frames):
        return dict(scene_id='task__variant',**self.hint_binding(),frames=frames)

    def test_event_frames_preserve_defaults_and_use_actual_phase(self):
        arrays,_=self.states();hint=self.hints([dict(state_index=1,event='load_onset'),dict(state_index=3,event='lift_event',phase='lift')])
        selected=replay.select_frames_with_hints(arrays,self.hint_binding(),hint)
        self.assertEqual([f['state_index'] for f in selected],[1,2,3,4,5,6,7])
        self.assertTrue({f['state_index'] for f in replay.select_frames(arrays)}.issubset({f['state_index'] for f in selected}))
        self.assertEqual(selected[0]['phase'],'close');self.assertEqual(selected[0]['events'][0]['event'],'load_onset')
        self.assertEqual(next(f for f in selected if f['state_index']==3)['selection'],'default_phase_endpoint')

    def test_stale_hash_wrong_label_and_invalid_indices_are_rejected(self):
        arrays,_=self.states()
        for key,value in [('execution_result_sha256','d'*64),('physics_states_sha256','d'*64),('physics_metadata_sha256','d'*64)]:
            hint=self.hints([dict(state_index=1,event='event')]);hint[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):replay.select_frames_with_hints(arrays,self.hint_binding(),hint)
        for frame in [dict(state_index=-1,event='event'),dict(state_index=8,event='event'),dict(state_index=True,event='event'),
                      dict(state_index=1.,event='event'),dict(state_index=1,event='event',phase='lift'),dict(state_index=1,event='event',physics_step=9)]:
            with self.subTest(frame=frame),self.assertRaises(ValueError):replay.select_frames_with_hints(arrays,self.hint_binding(),self.hints([frame]))

    def test_analysis_schema_and_total_ten_cap(self):
        arrays,_=self.states();legacy=dict(scene_id='task__variant',result=dict(path=self.hint_binding()['execution_result_path'],sha256='a'*64),
            physics_states=dict(path=self.hint_binding()['physics_states_path'],sha256='b'*64),suggested_six_diagnostic_frames=[dict(state_index=1,label='contact_event')])
        self.assertEqual(len(replay.select_frames_with_hints(arrays,self.hint_binding(),legacy)),7)
        phases=np.array(['reset']*10+['close','lift','transfer','lower','release','final_settle'])
        extended=dict(phase=phases,physics_step=np.arange(16),action_index=np.arange(16),source_trajectory_index=np.arange(16))
        selected=replay.select_frames_with_hints(extended,self.hint_binding(),self.hints([dict(state_index=i,event='event') for i in range(4)]))
        self.assertEqual(len(selected),10);self.assertEqual(selected[-1]['state_index'],15)
        with self.assertRaisesRegex(ValueError,'limit of 10'):
            replay.select_frames_with_hints(extended,self.hint_binding(),self.hints([dict(state_index=i,event='event') for i in range(5)]))

    def test_duplicate_foreign_or_successful_scene_hints_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory,patch.object(replay,'ROOT',Path(directory)):
            path=Path(directory)/'hints.json';row=dict(scene_id='task__variant',execution_status='failed',executed=True);manifest=dict(entries=[row])
            for entries in [[dict(scene_id='foreign')],[dict(scene_id='task__variant'),dict(scene_id='task__variant')]]:
                path.write_text(json.dumps(dict(entries=entries)))
                with self.assertRaises(ValueError):replay.load_frame_hints(path,manifest)
            path.write_text(json.dumps(dict(entries=[dict(scene_id='task__variant')])));row['execution_status']='success'
            with self.assertRaises(ValueError):replay.load_frame_hints(path,manifest)

    def test_dryrun_rejects_bad_hints_before_any_output_or_stage(self):
        arrays,_=self.states()
        with tempfile.TemporaryDirectory() as directory,patch.object(replay,'ROOT',Path(directory)):
            root=Path(directory);manifest=root/'manifest.json';hints=root/'hints.json';output=root/'out'
            manifest.write_text(json.dumps(dict(stage='complete',entries=[dict(scene_id='task__variant',execution_status='failed',executed=True)])))
            hint=self.hints([dict(state_index=99,event='invalid')]);hints.write_text(json.dumps(dict(entries=[hint])))
            with patch.object(replay.sys,'argv',['renderer','--manifest',str(manifest),'--output',str(output),'--frame-hints',str(hints),'--dry-run']),patch.object(replay,'load_record',return_value=({}, {},arrays,{},self.hint_binding())),patch.object(replay,'prepare_stage') as prepare:
                with self.assertRaises(ValueError):replay.main()
                prepare.assert_not_called();self.assertFalse(output.exists())

    def test_body_mapping_covers_dynamic_background_robot_objects_and_hazard(self):
        values={'kitchen/Cup/Body':'/World/kitchen/Asset/Cup/Body','Robot':'/World/robot/Asset/panda_link0',
            'Robot/panda_link1':'/World/robot/Asset/panda_link1','banana':'/World/objects/banana/Asset',
            'hazard/Body':'/World/hazard/Asset/Body'}
        for source,target in values.items():self.assertEqual(replay.mapped_body_path('/World/envs/env_0/'+source,['banana']),target)
        with self.assertRaises(ValueError):replay.mapped_body_path('/World/envs/env_0/unrecorded',['banana'])

    def test_measured_background_nested_body_motion_and_source_scale_preserved(self):
        from pxr import Gf,Usd,UsdGeom
        stage=Usd.Stage.CreateInMemory();UsdGeom.Xform.Define(stage,'/World')
        parent=UsdGeom.Xform.Define(stage,'/World/kitchen');parent.AddTranslateOp().Set(Gf.Vec3d(80,20,10))
        child=UsdGeom.Xform.Define(stage,'/World/kitchen/Door');child.AddScaleOp().Set(Gf.Vec3d(.01,.01,.01))
        mapping=[dict(source_path='background',target_path='/World/kitchen',world_scale=[1,1,1]),
                 dict(source_path='door',target_path='/World/kitchen/Door',world_scale=[.01,.01,.01])]
        poses=np.array([[1,2,3,0,0,0,1],[4,5,6,0,0,.7071067811865475,.7071067811865475]])
        replay.apply_state(stage,mapping,poses)
        cache=UsdGeom.XformCache();self.assertTrue(np.allclose(cache.GetLocalToWorldTransform(child.GetPrim()).ExtractTranslation(),[4,5,6]))
        poses[1,:3]=[7,8,9];replay.apply_state(stage,mapping,poses)
        cache.Clear();self.assertTrue(np.allclose(cache.GetLocalToWorldTransform(child.GetPrim()).ExtractTranslation(),[7,8,9]))


if __name__=='__main__':unittest.main()
