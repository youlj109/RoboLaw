"""Independent second-round progress, outcome and note bindings."""
from copy import deepcopy
from pathlib import Path
from http.server import ThreadingHTTPServer
import threading
import urllib.request
import urllib.error
import unittest
from unittest.mock import patch
from scripts.extensions.replicator import progress_dashboard as dashboard
from tests.extensions.replicator import test_physical_retry_dashboard as round1
from scripts.extensions.replicator.package_trajectory_demos import sha


class Retry2DashboardTests(unittest.TestCase):
    def setUp(self):
        fixture=round1.PhysicalRetryDashboardTests('test_pending_keeps_first_and_camera_counts')
        fixture.setUp();self.addCleanup(fixture.doCleanups);self.old=fixture
        self.f=fixture.f;self.progress=fixture.progress;fixture.terminal('failed');fixture.diagnostic()
        self.sid=fixture.sid;f=self.f
        for key,value in [('PHYSICAL_RETRY2_DEMOS','exports/retry2/'),('PHYSICAL_RETRY2_PLAN','configs/retry2.json'),
                          ('PHYSICAL_RETRY2_PLANNING','reports/retry2planning/'),('PHYSICAL_RETRY2_DIAGNOSTICS','reports/retry2diagnostic/'),
                          ('PHYSICAL_RETRY2_DETAIL_DIAGNOSTICS','reports/retry2detail/'),('PHYSICAL_RETRY2_DARK_DIAGNOSTICS','reports/retry2dark/'),
                          ('PHYSICAL_RETRY2_TOTAL',1)]:
            p=patch.object(dashboard,key,value);p.start();self.addCleanup(p.stop)
        self.previous=dict(path=f.rel(fixture.output/'manifest.json'),sha256=sha(fixture.output/'manifest.json'))
        raw=f.write('reports/retry2planning/startup_recovery_2/scenes/'+self.sid+'/trajectory.npz',b'new raw command path')
        self.adjusted=f.write('reports/retry2planning/retimed.npz',b'new retimed command path')
        self.candidate={k:fixture.row[k] for k in ('scene_id','task_id','variant_id','config_path','config_sha256')}
        self.candidate.update(status='generated',planning_only=True,executed=False,attempts=1,trajectory_complete=True,
                              artifacts=dict(trajectory=dict(path=f.rel(raw),sha256=sha(raw))))
        self.candidate_path=f.write('reports/retry2planning/startup_recovery_2/scenes/'+self.sid+'/result.json',self.candidate)
        self.planning=dict(source_previous_retry_manifest=self.previous,entries=[dict(scene_id=self.sid,status='generated',
            candidate_planning_result=dict(path=f.rel(self.candidate_path),sha256=sha(self.candidate_path)),
            trajectory=dict(path=f.rel(self.adjusted),sha256=sha(self.adjusted)),recipe={'description':'New grasp'})])
        self.save_planning()

    def save_planning(self):self.f.write(dashboard.PHYSICAL_RETRY2_PLANNING+'progress.json',self.planning)

    def initialize(self, failed=False):
        f=self.f
        if failed:
            self.candidate.update(status='failed',trajectory_complete=False,failure_reason='IK_FAIL',artifacts={})
            f.write(self.candidate_path,self.candidate)
            self.planning['entries'][0].update(status='failed',failure_reason='IK_FAIL',trajectory=None,
                candidate_planning_result=dict(path=f.rel(self.candidate_path),sha256=sha(self.candidate_path)))
            self.save_planning()
        item=self.planning['entries'][0]
        self.planrow=dict(self.old.plan_row,planning_status=item['status'],candidate_planning_result=item['candidate_planning_result'],
            previous_retry_result=dict(path=self.old.row['execution_result_path'],sha256=self.old.row['execution_result_sha256']),
            physical_attempt_number=4,execution_overrides={'close_seconds':3.},failure_reason='IK_FAIL' if failed else None,
            trajectory_path=None if failed else f.rel(self.adjusted),trajectory_sha256=None if failed else sha(self.adjusted))
        self.plan=dict(self.old.plan,schema_version=2,retry_round=2,source_previous_retry_manifest=self.previous,entries=[self.planrow])
        path=f.write(dashboard.PHYSICAL_RETRY2_PLAN,self.plan)
        self.row=dict(self.planrow,retry_schema_version=2,retry_plan_path=f.rel(path),retry_plan_sha256=sha(path),
            prior_physical_attempts=self.old.row['prior_physical_attempts']+[dict(result_path=self.old.row['execution_result_path'],
                result_sha256=self.old.row['execution_result_sha256'],physical_attempt_number=3,execution_status='failed',task_success=False,executed=True,video_sha256=None)],
            execution_status='planning_failed' if failed else 'pending',executed=False,task_success=False,attempts=0)
        self.manifest=dict(self.old.manifest,schema_version=2,retry_round=2,stage='complete' if failed else 'executing',
            source_previous_retry_manifest=self.previous,retry_plan_path=f.rel(path),retry_plan_sha256=sha(path),entries=[self.row])
        f.write(dashboard.PHYSICAL_RETRY2_DEMOS+'manifest.json',self.manifest)

    def terminal(self,status='success'):
        self.initialize();f=self.f;path=f.root/dashboard.PHYSICAL_RETRY2_DEMOS/'executions'/self.sid/'result.json'
        states=f.write(path.parent/'physics_states.npz',b'new measured states');meta=f.write(path.parent/'physics_metadata.json',dict(samples=21))
        self.row.update(execution_status=status,executed=True,task_success=status=='success',attempts=1)
        self.result=dict(self.row,kind='physical_failure_retry_execution',status='complete',planning_result_path=self.row['candidate_planning_result']['path'],
            planning_result_sha256=self.row['candidate_planning_result']['sha256'],
            original_planning_result=dict(path=self.row['result_path'],sha256=self.row['result_sha256']),rendering_enabled=False,video_expected=False,
            original_task_success=status=='success',final_original_destination_predicate=status=='success',
            source_previous_retry_manifest=self.previous,source_inputs_unchanged=True,execution_sources_unchanged=True,
            render_generation_delta=0,render_state_start={'passed':True},render_state_end={'passed':True},
            physics_evidence=dict(q_path=f.rel(states),q_sha256=sha(states),metadata_path=f.rel(meta),metadata_sha256=sha(meta),complete=True,continuous=True),
            physical_overlay_binding=self.old.result['physical_overlay_binding'])
        f.write(path,self.result);self.row.update(execution_result_path=f.rel(path),execution_result_sha256=sha(path))
        self.manifest['stage']='complete';f.write(dashboard.PHYSICAL_RETRY2_DEMOS+'manifest.json',self.manifest)

    def publish(self,retry):
        self.published=dict(kind='physical_failure_retry_result_snapshot',partial_preview=False,retry_round=2,
            source_retry_manifest=dict(path=dashboard.PHYSICAL_RETRY2_DEMOS+'manifest.json',sha256=sha(self.f.root/dashboard.PHYSICAL_RETRY2_DEMOS/'manifest.json')),
            diagnostic_visual_review=dict(status='complete',conclusions_written=True),entries=[dict(scene_id=self.sid,retry_round2=retry)])
        self.f.write(dashboard.PHYSICAL_RETRY2_RESULTS,self.published)

    def publish_visual(self):
        self.terminal('failed');self.diagnostics=self.f.root/dashboard.PHYSICAL_RETRY2_DIAGNOSTICS
        round1.PhysicalRetryDashboardTests.diagnostic(self)
        review=dict(scene_id=self.sid,status='complete',execution_result_sha256=self.row['execution_result_sha256'],
            diagnostic_report_sha256=sha(self.diagnostics/'scenes'/self.sid/'diagnostic.json'),
            observed_visual_facts_zh=['物体仍在台面。'],interpretation_zh='可能没有保持夹持。',next_adjustment_zh='检查夹持接触。',
            root_cause_confirmed=False,interpretation_is_inference=True)
        self.publish(dict(result=dict(path=self.row['execution_result_path'],sha256=self.row['execution_result_sha256']),diagnostic_visual_review=review))

    def supplementary(self,fixture,prefix):
        directory=Path(prefix)/'scenes'/self.sid
        image=self.f.write(directory/'frame_00.jpg',b'same physical state supplemental image')
        extra=deepcopy(fixture.report);extra.update(status='ready',timeline_playing=False)
        extra['frames'][0].update(image_path=self.f.rel(image),image_sha256=sha(image))
        path=self.f.write(directory/'diagnostic.json',extra)
        return dict(path=self.f.rel(path),sha256=sha(path)),image

    def publish_cpu(self):
        self.initialize(failed=True)
        diagnostic=dict(scene_id=self.sid,task_id=self.row['task_id'],variant_id=self.row['variant_id'],status='diagnosed',
            candidate_planning_result=self.row['candidate_planning_result'],failed_phase='lift',solver_status='MotionGenStatus.IK_FAIL',
            root_cause_confirmed=False,scene_infeasibility_established=False,observed_cpu_facts_zh=['固定目标的一次 CPU IK 得到候选解。'],
            interpretation_zh='原 IK 失败可能与锁定关节的碰撞约束有关。',next_adjustment_zh='下一轮检查带碰撞的抓取关节角。')
        self.cpu_path=self.f.root/'reports/replicator_physical_retry_round2_20261028/planning_failure_diagnostics.json'
        self.cpu_report=dict(kind='physical_retry_planning_failure_cpu_diagnostics',retry_round=2,complete=True,
            source_plan=dict(path=self.manifest['retry_plan_path'],sha256=self.manifest['retry_plan_sha256']),
            physical_execution_started=False,kit_started=False,curobo_planning_calls=0,new_target_candidates=0,
            layout_changed=False,source_plan_unchanged=True,entries=[deepcopy(diagnostic)])
        self.f.write(self.cpu_path,self.cpu_report)
        ref=dict(path=self.f.rel(self.cpu_path),sha256=sha(self.cpu_path));diagnostic['evidence']=ref
        self.publish(dict(execution_status='planning_failed',planning_status='failed',attempts=0,executed=False,
            candidate_planning_result=self.row['candidate_planning_result'],planning_failure_diagnostic=diagnostic))
        self.published['source_planning_failure_diagnostics']=ref
        self.f.write(dashboard.PHYSICAL_RETRY2_RESULTS,self.published)

    def test_planning_progress_does_not_promote_success_or_erase_old_diagnostics(self):
        data=self.progress.physical_execution();new=data['failure_retry_round2']
        self.assertFalse(new['manifest_present']);self.assertTrue(new['planning_present'])
        self.assertEqual(new['planning']['counts']['generated'],1);self.assertEqual(new['counts']['pending'],1)
        self.assertEqual(new['counts']['success'],0);self.assertEqual(data['combined_latest_counts']['success'],2)
        self.assertEqual(len(data['failure_retry']['entries'][0]['diagnostics']),1)
        self.assertEqual(new['entries'][0]['attempts'],0)
        self.assertTrue(dashboard.allowed_artifact(self.f.rel(self.candidate_path)))

    def test_planning_failed_is_neither_physical_failure_nor_attempt(self):
        self.initialize(failed=True);data=self.progress.physical_execution();new=data['failure_retry_round2']
        self.assertEqual(new['counts']['planning_failed'],1);self.assertEqual(new['counts']['failed'],0)
        self.assertEqual(new['counts']['stale'],0);self.assertFalse(new['entries'][0]['executed'])
        self.assertEqual(new['entries'][0]['attempts'],0);self.assertEqual(data['combined_latest_counts']['success'],2)

    def test_success_promotes_only_latest_and_round2_notes_do_not_change_round1(self):
        self.terminal();data=self.progress.physical_execution()
        self.assertEqual(data['failure_retry_round2']['counts']['success'],1)
        self.assertEqual(data['failure_retry']['counts']['failed'],1)
        self.assertEqual(data['counts']['failed'],1);self.assertEqual(data['combined_latest_counts']['success'],3)
        payload=dict(scene_id=self.sid,note='Original round note',expected_revision=0,
                     config_sha256=self.old.row['config_sha256'],result_sha256=self.old.row['execution_result_sha256'])
        self.progress.save_physical_retry_annotation(payload);old_bytes=self.progress.physical_retry_annotation_path.read_bytes()
        payload.update(note='Round 2 independent note',result_sha256=self.row['execution_result_sha256'])
        self.progress.save_physical_retry_annotation(payload,retry_round=2)
        self.assertEqual(self.progress.physical_retry_annotation_path.read_bytes(),old_bytes)
        self.assertEqual(self.progress.physical_retry_annotations(2)['entries'][0]['note'],'Round 2 independent note')
        with self.assertRaises(dashboard.AnnotationError):self.progress.save_physical_retry_annotation(payload,retry_round=2)

    def test_candidate_tampering_withholds_latest_success(self):
        self.terminal();self.candidate_path.write_text('{}')
        data=self.progress.physical_execution()
        self.assertEqual(data['failure_retry_round2']['counts']['success'],0)
        self.assertEqual(data['failure_retry_round2']['counts']['stale'],1)
        self.assertEqual(data['combined_latest_counts']['success'],2)
        self.assertEqual(len(data['failure_retry']['entries'][0]['diagnostics']),1)

    def test_round2_detail_and_dark_supplements_bind_the_same_recorded_episode(self):
        self.publish_visual();review=self.published['entries'][0]['retry_round2']['diagnostic_visual_review']
        for prefix in (dashboard.PHYSICAL_RETRY2_DETAIL_DIAGNOSTICS,dashboard.PHYSICAL_RETRY2_DARK_DIAGNOSTICS,dashboard.PHYSICAL_RETRY2_DARK_DIAGNOSTICS+'attempt_01/'):
            ref,image=self.supplementary(self,prefix);review['supplementary_diagnostic_reports']=[ref]
            self.f.write(dashboard.PHYSICAL_RETRY2_RESULTS,self.published)
            row=self.progress.physical_execution()['failure_retry_round2']['entries'][0]
            with self.subTest(prefix=prefix):
                self.assertEqual(len(row['diagnostics']),2);self.assertTrue(row['diagnostics'][-1]['supplementary'])
                self.assertIn('diagnostic_visual_review',row);self.assertTrue(dashboard.allowed_artifact(self.f.rel(image)))

    def test_supplements_from_another_round_are_rejected_in_both_directions(self):
        self.publish_visual();self.old.publish_visual_review()
        for fixture,round_number,prefix,publication,publication_path,key in (
            (self,2,dashboard.PHYSICAL_RETRY_DETAIL_DIAGNOSTICS,self.published,dashboard.PHYSICAL_RETRY2_RESULTS,'retry_round2'),
            (self,2,dashboard.PHYSICAL_RETRY_DARK_DIAGNOSTICS,self.published,dashboard.PHYSICAL_RETRY2_RESULTS,'retry_round2'),
            (self.old,1,dashboard.PHYSICAL_RETRY2_DETAIL_DIAGNOSTICS,self.old.published,dashboard.PHYSICAL_RETRY_RESULTS,'retry'),
            (self.old,1,dashboard.PHYSICAL_RETRY2_DARK_DIAGNOSTICS,self.old.published,dashboard.PHYSICAL_RETRY_RESULTS,'retry')):
            ref,_=self.supplementary(fixture,prefix)
            publication['entries'][0][key]['diagnostic_visual_review']['supplementary_diagnostic_reports']=[ref]
            self.f.write(publication_path,publication)
            field='failure_retry_round2' if round_number==2 else 'failure_retry'
            row=self.progress.physical_execution()[field]['entries'][0]
            with self.subTest(round=round_number,prefix=prefix):
                self.assertEqual(len(row['diagnostics']),1);self.assertNotIn('diagnostic_visual_review',row)

    def test_supplement_artifact_paths_remain_narrow_and_bound_over_http(self):
        self.publish_visual();ref,image=self.supplementary(self,dashboard.PHYSICAL_RETRY2_DETAIL_DIAGNOSTICS)
        self.published['entries'][0]['retry_round2']['diagnostic_visual_review']['supplementary_diagnostic_reports']=[ref]
        self.f.write(dashboard.PHYSICAL_RETRY2_RESULTS,self.published)
        for prefix in (dashboard.PHYSICAL_RETRY2_DETAIL_DIAGNOSTICS,dashboard.PHYSICAL_RETRY2_DARK_DIAGNOSTICS):
            for tail in ('secrets.jpg','scenes/'+self.sid+'/physics_states.npz','scenes/'+self.sid+'/diagnostic.json'):
                self.assertFalse(dashboard.allowed_artifact(prefix+tail))
        unbound=self.f.write(Path(dashboard.PHYSICAL_RETRY2_DETAIL_DIAGNOSTICS)/'scenes'/self.sid/'frame_01.jpg',b'not reviewed')
        server=ThreadingHTTPServer(('127.0.0.1',0),dashboard.handler_for(self.progress))
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            base='http://127.0.0.1:'+str(server.server_port)+'/artifact/'
            with urllib.request.urlopen(base+self.f.rel(image)) as response:self.assertEqual(response.status,200)
            with self.assertRaises(urllib.error.HTTPError) as caught:urllib.request.urlopen(base+self.f.rel(unbound))
            self.assertEqual(caught.exception.code,404);caught.exception.close()
            (self.f.root/self.row['execution_result_path']).write_bytes(b'changed actual result')
            with self.assertRaises(urllib.error.HTTPError) as caught:urllib.request.urlopen(base+self.f.rel(image))
            self.assertEqual(caught.exception.code,404);caught.exception.close()
        finally:
            server.shutdown();server.server_close();thread.join()

    def test_cpu_diagnostic_is_distinct_from_visual_review_and_physical_outcome(self):
        self.publish_cpu();data=self.progress.physical_execution();row=data['failure_retry_round2']['entries'][0]
        diagnosis=row['planning_failure_diagnostic']
        self.assertEqual(diagnosis['observed_cpu_facts_zh'],self.cpu_report['entries'][0]['observed_cpu_facts_zh'])
        self.assertTrue(diagnosis['interpretation_is_inference']);self.assertFalse(diagnosis['scene_infeasibility_established'])
        self.assertNotIn('diagnostic_visual_review',row);self.assertFalse(row['result_valid']);self.assertEqual(row['attempts'],0)
        self.assertEqual(data['combined_latest_counts']['success'],2);self.assertEqual(data['failure_retry_round2']['counts']['planning_failed'],1)

    def test_cpu_diagnostic_rejects_changed_candidate_source_or_embedded_facts(self):
        self.publish_cpu();original=deepcopy(self.published)
        for case in ('candidate','source_sha','publication_source','embedded_facts','physical_attempt','executed','root_claim','infeasibility_claim','manifest'):
            publication=deepcopy(original);retry=publication['entries'][0]['retry_round2'];diagnostic=retry['planning_failure_diagnostic']
            if case=='candidate':diagnostic['candidate_planning_result']=dict(diagnostic['candidate_planning_result'],sha256='0'*64)
            elif case=='source_sha':diagnostic['evidence']=dict(diagnostic['evidence'],sha256='0'*64)
            elif case=='publication_source':publication['source_planning_failure_diagnostics']=dict(diagnostic['evidence'],sha256='0'*64)
            elif case=='embedded_facts':diagnostic['observed_cpu_facts_zh']=['Unbound replacement conclusion']
            elif case=='physical_attempt':retry['attempts']=1
            elif case=='executed':retry['executed']=True
            elif case=='root_claim':diagnostic['root_cause_confirmed']=True
            elif case=='infeasibility_claim':diagnostic['scene_infeasibility_established']=True
            else:publication['source_retry_manifest']['sha256']='0'*64
            self.f.write(dashboard.PHYSICAL_RETRY2_RESULTS,publication)
            row=self.progress.physical_execution()['failure_retry_round2']['entries'][0]
            with self.subTest(case=case):
                self.assertNotIn('planning_failure_diagnostic',row);self.assertEqual(row['execution_status'],'planning_failed')
                self.assertEqual(row['attempts'],0);self.assertFalse(row['executed'])

    def test_cpu_diagnostic_rejects_rebound_report_from_wrong_plan_or_execution(self):
        self.publish_cpu();original=deepcopy(self.cpu_report)
        for case in ('plan','physical','kit','duplicate_scene'):
            report=deepcopy(original)
            if case=='plan':report['source_plan']['sha256']='0'*64
            elif case=='physical':report['physical_execution_started']=True
            elif case=='kit':report['kit_started']=True
            else:report['entries'].append(deepcopy(report['entries'][0]))
            self.f.write(self.cpu_path,report);ref=dict(path=self.f.rel(self.cpu_path),sha256=sha(self.cpu_path))
            self.published['source_planning_failure_diagnostics']=ref
            self.published['entries'][0]['retry_round2']['planning_failure_diagnostic']['evidence']=ref
            self.f.write(dashboard.PHYSICAL_RETRY2_RESULTS,self.published)
            with self.subTest(case=case):self.assertNotIn('planning_failure_diagnostic',self.progress.physical_execution()['failure_retry_round2']['entries'][0])


if __name__=='__main__':unittest.main()
