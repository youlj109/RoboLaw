"""Actual retry lineage must not alter first-pass/camera scores or expose stale images."""
from copy import deepcopy
import json
from pathlib import Path
from http.server import ThreadingHTTPServer
import threading
import urllib.request
import urllib.error
import unittest
from unittest.mock import patch
from scripts.extensions.replicator import progress_dashboard as dashboard
from tests.extensions.replicator import test_camera_retake_dashboard as fixtures
from scripts.extensions.replicator.package_trajectory_demos import sha


class PhysicalRetryDashboardTests(unittest.TestCase):
    def setUp(self):
        fixture=fixtures.CameraRetakeDashboardTests('test_overlay_keeps_first_success_when_retaken_episode_fails')
        fixture.setUp();self.addCleanup(fixture.doCleanups);self.f=fixture.fixture;self.progress=fixture.progress;f=self.f
        self.output=f.root/'exports/retry';self.plan_path=f.root/'configs/retry.json';self.diagnostics=f.root/'reports/diagnostics'
        for key,value in [('PHYSICAL_RETRY_DEMOS',f.rel(self.output)+'/'),('PHYSICAL_RETRY_PLAN',f.rel(self.plan_path)),
                          ('PHYSICAL_RETRY_DIAGNOSTICS',f.rel(self.diagnostics)+'/'),('PHYSICAL_RETRY_TOTAL',1)]:
            mocked=patch.object(dashboard,key,value);mocked.start();self.addCleanup(mocked.stop)
        old=f.first_entries[1];self.sid=old['scene_id'];traj=f.write('reports/adjusted.npz',b'adjusted frozen path')
        keys=('scene_id','task_id','variant_id','config_path','config_sha256','source_scene_path','source_scene_sha256','result_path','result_sha256')
        self.plan_row={k:old[k] for k in keys};self.plan_row.update(source_trajectory_path=old['trajectory_path'],source_trajectory_sha256=old['trajectory_sha256'],
            trajectory_path=f.rel(traj),trajectory_sha256=sha(traj),physical_attempt_number=3,adjustment=dict(description='Timing only',kind='retime'))
        self.plan=dict(kind='physical_failure_retry_plan',source_first_pass_manifest=dict(path=f.rel(f.first/'manifest.json'),sha256=sha(f.first/'manifest.json')),
            source_camera_retake_manifest=dict(path=f.rel(f.retakes/'manifest.json'),sha256=sha(f.retakes/'manifest.json')),entries=[self.plan_row])
        f.write(self.plan_path,self.plan)
        history=[]
        for root,row in [(f.first,old),(f.retakes,f.retake_entries[1])]:
            history.append(dict(result_path=f.rel(root/row['execution_result_path']),result_sha256=row['execution_result_sha256'],
                execution_status='failed',task_success=False,executed=True,physical_attempt_number=len(history)+1,video_sha256=row['video_sha256']))
        self.row=dict(self.plan_row,retry_plan_path=f.rel(self.plan_path),retry_plan_sha256=sha(self.plan_path),prior_physical_attempts=history,
            execution_status='pending',executed=False,task_success=False,attempts=0)
        self.manifest=dict(kind='physical_failure_retry',stage='executing',rendering_enabled=False,video_expected=False,
            retry_plan_path=f.rel(self.plan_path),retry_plan_sha256=sha(self.plan_path),entries=[self.row],
            source_first_pass_manifest=self.plan['source_first_pass_manifest'],source_camera_retake_manifest=self.plan['source_camera_retake_manifest'])
        self.save()

    def save(self):self.f.write(self.output/'manifest.json',self.manifest)

    def terminal(self,status='success'):
        f=self.f;path=self.output/'executions'/self.sid/'result.json'
        states=f.write(path.parent/'physics_states.npz',b'measured states');meta=f.write(path.parent/'physics_metadata.json',dict(samples=20))
        overlay=f.write(self.output/'overlay.usda',b'bound physical background')
        self.row.update(execution_status=status,executed=True,task_success=status=='success',attempts=1)
        self.result=dict(self.row,kind='physical_failure_retry_execution',status='complete',planning_result_path=self.row['result_path'],
            planning_result_sha256=self.row['result_sha256'],rendering_enabled=False,video_expected=False,
            original_task_success=status=='success',final_original_destination_predicate=status=='success',
            physics_evidence=dict(q_path=f.rel(states),q_sha256=sha(states),metadata_path=f.rel(meta),metadata_sha256=sha(meta),complete=True,continuous=True),
            physical_overlay_binding=dict(path=f.rel(overlay),sha256_start=sha(overlay)))
        f.write(path,self.result);self.row.update(execution_result_path=f.rel(path),execution_result_sha256=sha(path));self.manifest['stage']='complete';self.save()

    def diagnostic(self):
        f=self.f;result_path=Path(self.row['execution_result_path']);e=self.result['physics_evidence'];overlay=self.result['physical_overlay_binding']
        directory=self.diagnostics/'scenes'/self.sid;image=f.write(directory/'frame_00.jpg',b'bound diagnostic image')
        binding=dict(execution_result_path=str(result_path),execution_result_sha256=self.row['execution_result_sha256'],config_path=self.row['config_path'],config_sha256=self.row['config_sha256'],
            physics_states_path=e['q_path'],physics_states_sha256=e['q_sha256'],physics_metadata_path=e['metadata_path'],physics_metadata_sha256=e['metadata_sha256'],
            physical_overlay_path=overlay['path'],physical_overlay_sha256=overlay['sha256_start'])
        self.report=dict(kind='physical_failure_diagnostic_state_replay',scene_id=self.sid,diagnostic_only=True,physics_steps_requested=0,physical_episode_started=False,
            binding=binding,frames=[dict(image_path=f.rel(image),image_sha256=sha(image),state_index=19,physics_step=19,phase='final_settle')])
        report=f.write(directory/'diagnostic.json',self.report)
        self.diag_progress=dict(entries=[dict(scene_id=self.sid,status='ready',report_path=f.rel(report),report_sha256=sha(report))])
        f.write(self.diagnostics/'progress.json',self.diag_progress)

    def test_pending_keeps_first_and_camera_counts(self):
        data=self.progress.physical_execution();self.assertEqual(data['counts']['success'],2);self.assertEqual(data['counts']['failed'],1)
        self.assertEqual(data['failure_retry']['counts']['pending'],1);self.assertEqual(data['failure_retry']['counts']['stale'],0)
        self.assertEqual(data['combined_latest_counts']['success'],2);self.assertEqual(data['camera_retakes']['saved'],2)

    def test_success_only_promotes_combined_latest_and_remains_no_video(self):
        self.terminal();data=self.progress.physical_execution();self.assertEqual(data['failure_retry']['counts']['success'],1)
        self.assertEqual(data['combined_latest_counts'],dict(total=3,success=3,failed=0,error=0));self.assertEqual(data['counts']['success'],2)
        self.assertEqual(data['counts']['video_count'],3);row=data['failure_retry']['entries'][0]
        self.assertEqual(row['physical_attempt_number'],3);self.assertTrue(row['result_url']);self.assertNotIn('video_url',row)

    def test_changed_original_or_retry_result_is_stale_without_promoting(self):
        self.terminal();path=self.f.root/self.row['execution_result_path'];path.write_bytes(b'changed result')
        data=self.progress.physical_execution();self.assertEqual(data['failure_retry']['counts']['stale'],1)
        self.assertEqual(data['combined_latest_counts']['success'],2);self.assertIsNone(data['failure_retry']['entries'][0]['result_url'])

    def test_bound_diagnostics_require_same_actual_states_and_zero_execution(self):
        self.terminal('failed');self.diagnostic();data=self.progress.physical_execution();row=data['failure_retry']['entries'][0]
        self.assertEqual(len(row['diagnostics']),1);self.assertTrue(row['diagnostic_report_url']);self.assertEqual(data['counts']['failed'],1)
        self.report['binding']['physics_states_sha256']='0'*64;p=self.f.write(self.diagnostics/'scenes'/self.sid/'diagnostic.json',self.report)
        self.diag_progress['entries'][0]['report_sha256']=sha(p);self.f.write(self.diagnostics/'progress.json',self.diag_progress)
        row=self.progress.physical_execution()['failure_retry']['entries'][0];self.assertFalse(row['diagnostics']);self.assertTrue(row['diagnostic_error'])
        self.assertTrue(row['result_valid'])

    def test_ten_bound_diagnostic_images_allowed_but_eleven_rejected(self):
        self.terminal('failed');self.diagnostic();directory=self.diagnostics/'scenes'/self.sid
        frames=[]
        for i in range(11):
            image=self.f.write(directory/f'frame_{i:02d}.jpg',f'actual-state-frame-{i}'.encode())
            frames.append(dict(image_path=self.f.rel(image),image_sha256=sha(image),state_index=i,physics_step=i,phase='lift'))
        for count in (10,11):
            self.report['frames']=frames[:count];p=self.f.write(directory/'diagnostic.json',self.report)
            self.diag_progress['entries'][0]['report_sha256']=sha(p);self.f.write(self.diagnostics/'progress.json',self.diag_progress)
            row=self.progress.physical_execution()['failure_retry']['entries'][0]
            self.assertEqual(len(row['diagnostics']),10 if count==10 else 0)
            self.assertEqual(bool(row['diagnostic_error']),count==11)

    def publish_visual_review(self):
        report=self.diagnostics/'scenes'/self.sid/'diagnostic.json'
        review=dict(scene_id=self.sid,status='complete',execution_result_sha256=self.row['execution_result_sha256'],diagnostic_report_sha256=sha(report),
            observed_visual_facts_zh=['抬升图中物体仍在台面。'],interpretation_zh='可能未保持夹持，尚不能确认根因。',next_adjustment_zh='核对失持时刻的双侧接触。',
            root_cause_confirmed=False,interpretation_is_inference=True)
        self.published=dict(kind='physical_failure_retry_result_snapshot',partial_preview=False,
            source_retry_manifest=dict(path=self.f.rel(self.output/'manifest.json'),sha256=sha(self.output/'manifest.json')),
            diagnostic_visual_review=dict(status='complete',conclusions_written=True),markdown_report_path=dashboard.PHYSICAL_RETRY_REPORT,
            entries=[dict(scene_id=self.sid,retry=dict(result=dict(path=self.row['execution_result_path'],sha256=self.row['execution_result_sha256']),diagnostic_visual_review=review))])
        self.f.write(dashboard.PHYSICAL_RETRY_REPORT,b'# Bound visual review')
        self.f.write(dashboard.PHYSICAL_RETRY_RESULTS,self.published)

    def test_completed_bound_visual_facts_are_separate_from_manual_notes(self):
        self.terminal('failed');self.diagnostic();self.publish_visual_review()
        payload=dict(scene_id=self.sid,note='My independent note',expected_revision=0,config_sha256=self.row['config_sha256'],result_sha256=self.row['execution_result_sha256'])
        self.progress.save_physical_retry_annotation(payload);notes=self.progress.physical_retry_annotation_path.read_bytes()
        data=self.progress.physical_execution();row=data['failure_retry']['entries'][0];review=row['diagnostic_visual_review']
        self.assertEqual(review['observed_visual_facts_zh'],['抬升图中物体仍在台面。']);self.assertTrue(review['interpretation_is_inference'])
        self.assertTrue(review['markdown_report_url'].endswith('/PHYSICAL_RETRY_ROUND1_20261028.md'))
        self.assertEqual(notes,self.progress.physical_retry_annotation_path.read_bytes());self.assertEqual(data['counts']['failed'],1)

    def test_visual_review_hidden_on_stale_manifest_result_or_pending_review(self):
        self.terminal('failed');self.diagnostic();self.publish_visual_review();original=deepcopy(self.published)
        for case in ('manifest','result','review_pending','unconfirmed_claim'):
            self.published=deepcopy(original)
            if case=='manifest':self.published['source_retry_manifest']['sha256']='0'*64
            elif case=='result':self.published['entries'][0]['retry']['result']['sha256']='0'*64
            elif case=='review_pending':self.published['entries'][0]['retry']['diagnostic_visual_review']['status']='pending'
            else:self.published['entries'][0]['retry']['diagnostic_visual_review'].update(root_cause_confirmed=True,interpretation_is_inference=False)
            self.f.write(dashboard.PHYSICAL_RETRY_RESULTS,self.published)
            row=self.progress.physical_execution()['failure_retry']['entries'][0]
            with self.subTest(case=case):self.assertNotIn('diagnostic_visual_review',row);self.assertTrue(row['result_valid'])

    def test_supplementary_camera_must_bind_same_episode_and_measured_frame(self):
        self.terminal('failed');self.diagnostic();self.publish_visual_review()
        directory=Path(dashboard.PHYSICAL_RETRY_DETAIL_DIAGNOSTICS)/'scenes'/self.sid
        image=self.f.write(directory/'frame_00.jpg',b'oblique same actual state')
        extra=deepcopy(self.report);extra.update(status='ready',timeline_playing=False)
        extra['frames'][0].update(image_path=self.f.rel(image),image_sha256=sha(image))
        report=self.f.write(directory/'diagnostic.json',extra)
        review=self.published['entries'][0]['retry']['diagnostic_visual_review']
        review['supplementary_diagnostic_reports']=[dict(path=self.f.rel(report),sha256=sha(report))]
        self.f.write(dashboard.PHYSICAL_RETRY_RESULTS,self.published)
        row=self.progress.physical_execution()['failure_retry']['entries'][0]
        self.assertEqual(len(row['diagnostics']),2);self.assertTrue(row['diagnostics'][-1]['supplementary'])
        self.assertTrue(dashboard.allowed_artifact(self.f.rel(image)))
        for case in ('frame','episode','image'):
            modified=deepcopy(extra)
            if case=='frame':modified['frames'][0]['physics_step']=123456
            elif case=='episode':modified['binding']['execution_result_sha256']='0'*64
            else:modified['frames'][0]['image_sha256']='0'*64
            self.f.write(directory/'diagnostic.json',modified)
            review['supplementary_diagnostic_reports'][0]['sha256']=sha(report)
            self.f.write(dashboard.PHYSICAL_RETRY_RESULTS,self.published)
            row=self.progress.physical_execution()['failure_retry']['entries'][0]
            with self.subTest(case=case):
                self.assertEqual(len(row['diagnostics']),1)
                self.assertNotIn('diagnostic_visual_review',row)

    def test_visual_publication_absent_or_batch_in_progress_is_silent(self):
        data=self.progress.physical_execution();self.assertNotIn('manifest_error',data['failure_retry'])
        self.assertNotIn('diagnostic_visual_review',data['failure_retry']['entries'][0])
        self.terminal('failed');self.diagnostic();self.manifest['stage']='executing';self.save();self.publish_visual_review()
        row=self.progress.physical_execution()['failure_retry']['entries'][0];self.assertNotIn('diagnostic_visual_review',row)

    def test_note_result_binding_and_revision_conflict_preserve_old_notes(self):
        self.terminal('failed');old=self.f.write('reports/replicator_dashboard/trajectory_annotations.json',b'old trajectory notes')
        payload=dict(scene_id=self.sid,note='Measured failure observation',expected_revision=0,config_sha256=self.row['config_sha256'],result_sha256=self.row['execution_result_sha256'])
        entry=self.progress.save_physical_retry_annotation(payload);self.assertEqual(entry['revision'],1)
        with self.assertRaises(dashboard.AnnotationError) as caught:self.progress.save_physical_retry_annotation(payload)
        self.assertEqual(caught.exception.status,409);self.assertEqual(old.read_bytes(),b'old trajectory notes')
        payload.update(expected_revision=1,result_sha256='0'*64)
        with self.assertRaises(dashboard.AnnotationError):self.progress.save_physical_retry_annotation(payload)
        self.assertEqual(self.progress.physical_retry_annotations()['entries'][0]['note'],'Measured failure observation')

    def test_http_diagnostic_image_disappears_when_actual_result_changes(self):
        self.terminal('failed');self.diagnostic()
        server=ThreadingHTTPServer(('127.0.0.1',0),dashboard.handler_for(self.progress))
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            row=self.progress.physical_execution()['failure_retry']['entries'][0]
            base='http://127.0.0.1:'+str(server.server_port)
            with urllib.request.urlopen(base+row['diagnostics'][0]['url']) as response:
                self.assertEqual(response.status,200);self.assertEqual(response.read(),b'bound diagnostic image')
            (self.f.root/self.row['execution_result_path']).write_bytes(b'changed actual result')
            for url in (row['result_url'],row['diagnostics'][0]['url'],row['diagnostic_report_url']):
                with self.assertRaises(urllib.error.HTTPError) as caught:urllib.request.urlopen(base+url)
                self.assertEqual(caught.exception.code,404);caught.exception.close()
        finally:
            server.shutdown();server.server_close();thread.join()

    def test_diagnostic_path_allowlist_does_not_expose_arbitrary_recorded_files(self):
        self.assertTrue(dashboard.allowed_artifact(self.f.rel(self.output)+'/executions/'+self.sid+'/result.json'))
        self.assertFalse(dashboard.allowed_artifact(self.f.rel(self.output)+'/executions/'+self.sid+'/physics_states.npz'))
        self.assertFalse(dashboard.allowed_artifact(self.f.rel(self.diagnostics)+'/secrets.jpg'))


if __name__=='__main__':unittest.main()
