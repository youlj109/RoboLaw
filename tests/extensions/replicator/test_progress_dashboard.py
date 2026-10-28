"""Verify progress semantics and read-only HTTP artifact boundaries."""
import hashlib
import copy
import json
from pathlib import Path
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

from scripts.extensions.replicator.progress_dashboard import Progress, SCENE_VARIANTS, handler_for, physical_success


class DashboardTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.tid = 'kitchen_bench_replicator_g_shape_banana_bowl'
        self.write('student_skills/robolaw-replicator/references/assignment.json', {'tasks': [{'id': self.tid}]})
        self.progress = Progress(self.root)
        self.progress.processes = lambda tasks: []

    def tearDown(self):
        self.tmp.cleanup()

    def write(self, rel, value):
        p = self.root / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(value))
        return p

    def test_physical_success_does_not_complete_task(self):
        rel = 'datasets/replicator/g_shape_banana_bowl/human/attempt_01/result.json'
        self.write(rel, {'task_id': self.tid, 'variant': 'human', 'accepted': True,
                         'physical_trial_passed': True, 'original_task_success': True})
        data = self.progress.snapshot()
        self.assertEqual(data['summary']['completed_tasks'], 0)
        self.assertEqual(data['summary']['successful_variants'], 1)
        self.assertEqual(data['tasks'][0]['status'], 'partial')
        self.write(rel, {'task_id': self.tid, 'variant': 'human', 'status': 'failed', 'error': 'real failure'})
        self.progress.cached_at = 0
        data = self.progress.snapshot()
        self.assertEqual(data['summary']['successful_variants'], 0)
        self.assertEqual(data['tasks'][0]['latest']['reason'], 'real failure')

    def test_legacy_pass_flag_not_delivery(self):
        self.assertFalse(self.progress.manifest_valid({'task_id': self.tid, 'passed': True, 'accepted': True}, {self.tid: {}}))

    def placement_fixture(self):
        config = self.write('configs/extensions/replicator/visual_scene_variants/g_shape_banana_bowl/human_default_mpfb.json', {'task_id':self.tid, 'position':[0,1,2]})
        evidence = self.write('reports/replicator_safety_upgrade/visual_placement/geometry.json', {'semantic_geometry_passed':True})
        row = dict(scene_id='g_shape_banana_bowl__human_default_mpfb', task_id=self.tid, variant_id='human_default_mpfb',
                   status='adjusted', semantic_geometry_passed=True, config_path=config.relative_to(self.root).as_posix(),
                   config_sha256=hashlib.sha256(config.read_bytes()).hexdigest(), evidence_path=evidence.relative_to(self.root).as_posix(),
                   evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest(),
                   visual_review={'visual_status':'candidate','observation':'原图已看'})
        self.write('reports/replicator_safety_upgrade/visual_placement/progress.json', {'entries':[row]})
        return row

    def test_visual_placement_never_promotes_geometry_or_old_review_to_safety(self):
        self.placement_fixture()
        data = self.progress.snapshot()
        self.assertEqual(data['visual_placement']['counts']['adjusted'],1)
        self.assertEqual(data['visual_placement']['counts']['readable'],0)
        entry = data['visual_placement']['entries'][0]
        self.assertEqual(entry['new_visual_review_status'],'pending')
        self.assertFalse(entry['accepted'])
        self.assertEqual(data['safety_upgrade']['counts']['accepted'],0)
        self.assertEqual(data['summary']['completed_tasks'],0)

    def test_visual_placement_tampered_geometry_is_not_adjusted(self):
        row = self.placement_fixture()
        self.write(row['config_path'], {'task_id':self.tid,'position':[1,1,2]})
        data = self.progress.visual_placement({self.tid:{}})
        self.assertEqual(data['counts']['adjusted'],0)
        self.assertEqual(data['entries'][0]['status'],'needs_repair')

    def test_final_placement_progress_takes_priority_and_allows_final_paths(self):
        row=self.placement_fixture()
        for key,sha_key,old,new in (
            ('config_path','config_sha256','visual_scene_variants/','visual_scene_variants_final01/'),
            ('evidence_path','evidence_sha256','visual_placement/','visual_placement_final01/')):
            target=row[key].replace(old,new)
            dest=self.root/target;dest.parent.mkdir(parents=True,exist_ok=True)
            dest.write_bytes((self.root/row[key]).read_bytes());row[key]=target
        self.write('reports/replicator_safety_upgrade/visual_placement_final01/progress.json',{'stage':'final_visual_placement_merged','entries':[row]})
        data=self.progress.visual_placement({self.tid:{}})
        self.assertEqual(data['counts']['adjusted'],1)
        self.assertEqual(data['stage'],'final_visual_placement_merged')
        self.assertIn('visual_placement_final01',data['source_progress_path'])

    def test_final02_source_repair_needs_current_layout_and_complete_certificate(self):
        row=self.placement_fixture()
        original_evidence=self.write('reports/replicator_preparation/blocked.json',{'intersecting':True})
        self.write('reports/replicator_safety_upgrade/visual_placement_final01/original_task_geometry_issues.json',{'entries':[
            dict(task_id=self.tid,status='original_task_geometry_blocked',applies_to_all_task_variants=True,
                 evidence_path=original_evidence.relative_to(self.root).as_posix(),evidence_sha256=hashlib.sha256(original_evidence.read_bytes()).hexdigest())]})
        layout={'task_id':self.tid,'position':[.2,1,2]}
        newconfig=self.write('configs/extensions/replicator/visual_scene_variants_final02/g_shape_banana_bowl/human_default_mpfb.json',layout)
        base=self.write('configs/extensions/replicator/l_island_mustard_adjust02/base_layout.json',layout)
        sources=[self.write('reports/replicator_safety_upgrade/adjust02/check'+str(i)+'.json',{'passed':True}) for i in range(2)]
        certificate=dict(task_id=self.tid,passed=True,relations_passed=True,source_geometry_passed=True,
            repaired_layout_path=base.relative_to(self.root).as_posix(),repaired_layout_sha256=hashlib.sha256(base.read_bytes()).hexdigest(),
            source_reports=[dict(path=p.relative_to(self.root).as_posix(),sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in sources])
        cert=self.write('reports/replicator_safety_upgrade/adjust02/certificate.json',certificate)
        row.update(original_config_path=row['config_path'],config_path=newconfig.relative_to(self.root).as_posix(),config_sha256=hashlib.sha256(newconfig.read_bytes()).hexdigest(),
            source_layout_repaired=True,source_layout_repair_evidence_path=cert.relative_to(self.root).as_posix(),
            source_layout_repair_evidence_sha256=hashlib.sha256(cert.read_bytes()).hexdigest(),native_stability_verified=True)
        self.write('reports/replicator_safety_upgrade/visual_placement_final02/progress.json',{'entries':[row]})
        data=self.progress.visual_placement({self.tid:{}});entry=data['entries'][0]
        self.assertTrue(entry['source_layout_repaired'])
        self.assertFalse(entry['original_task_geometry_blocked'])
        self.assertTrue(entry['old_original_task_geometry_blocked'])
        self.assertEqual(entry['baseline_scope'],'repaired_source_layout_requires_new_baseline')
        self.assertFalse(entry['native_stability_verified'])
        self.assertFalse(entry['trajectory_verified'])
        self.assertEqual(data['counts']['blocked_original_scenes'],10)
        sources[0].write_text('{}')
        data=self.progress.visual_placement({self.tid:{}})
        self.assertFalse(data['entries'][0]['source_layout_repaired'])
        self.assertEqual(data['counts']['blocked_original_scenes'],10)
        self.assertTrue(data['entries'][0]['source_layout_repair_pending'])

    def test_adjust05_status_priority_does_not_promote_task_success(self):
        self.write('reports/replicator_safety_upgrade/adjust02/status.json',
                   {'stage':'complete','title':'旧轮完成','groups':[{'count':8,'status':'complete'}]})
        self.assertIn('adjust02',self.progress.adjustment_status()['source_status_path'])
        self.write('reports/replicator_safety_upgrade/adjust05/status.json',
                   {'stage':'repairing','title':'两个原任务修复中','groups':[{'count':22,'status':'running'}]})
        data=self.progress.snapshot()
        self.assertIn('adjust05',data['adjustments']['source_status_path'])
        self.assertEqual(data['adjustments']['stage'],'repairing')
        self.assertEqual(data['summary']['completed_tasks'],0)
        self.assertEqual(data['safety_upgrade']['counts']['accepted'],0)

    def test_final03_keeps_three_independent_source_repair_certificates(self):
        tasks={};rows=[];issues=[];certificates={}
        digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
        for short,version in [('l_island_mustard_bowl','02'),('l_shape_mustard_bowl','05'),('u_shape_mustard_bowl','05')]:
            tid='kitchen_bench_replicator_'+short;tasks[tid]={}
            layout={'task_id':tid,'position':[.1,1,2]}
            original=self.write('configs/extensions/replicator/scene_variants/'+short+'/base.json',dict(layout,position=[0,1,2]))
            base=self.write('configs/extensions/replicator/'+short.replace('_bowl','')+'_adjust'+version+'/base_layout.json',layout)
            reports=[self.write('reports/replicator_safety_upgrade/adjust'+version+'/'+short+'/check'+str(i)+'.json',{'passed':True}) for i in range(2)]
            certificate=self.write('reports/replicator_safety_upgrade/adjust'+version+'/'+short+'/certificate.json',
                dict(task_id=tid,passed=True,relations_passed=True,source_geometry_passed=True,
                     repaired_layout_path=base.relative_to(self.root).as_posix(),repaired_layout_sha256=digest(base),
                     source_reports=[dict(path=p.relative_to(self.root).as_posix(),sha256=digest(p)) for p in reports]))
            certificates[short]=certificate
            old_evidence=self.write('reports/replicator_preparation/'+short+'/intersection.json',{'intersects':True})
            issues.append(dict(task_id=tid,status='original_task_geometry_blocked',applies_to_all_task_variants=True,
                               evidence_path=old_evidence.relative_to(self.root).as_posix(),evidence_sha256=digest(old_evidence)))
            for variant,_,category in SCENE_VARIANTS:
                config=self.write('configs/extensions/replicator/visual_scene_variants_final03/'+short+'/'+variant+'.json',dict(layout,obstacle={'id':variant}))
                rows.append(dict(scene_id=short+'__'+variant,task_id=tid,variant_id=variant,category=category,status='retained',
                    config_path=config.relative_to(self.root).as_posix(),config_sha256=digest(config),
                    original_config_path=original.relative_to(self.root).as_posix(),source_layout_repaired=True,
                    source_layout_repair_evidence_path=certificate.relative_to(self.root).as_posix(),
                    source_layout_repair_evidence_sha256=digest(certificate),native_stability_verified=True,safe_execution_verified=True,accepted=True))
        self.write('reports/replicator_safety_upgrade/visual_placement_final01/original_task_geometry_issues.json',{'entries':issues})
        self.write('reports/replicator_safety_upgrade/visual_placement_final02/progress.json',{'entries':[]})
        self.write('reports/replicator_safety_upgrade/visual_placement_final03/progress.json',{'entries':rows})
        data=self.progress.visual_placement(tasks)
        self.assertIn('final03',data['source_progress_path'])
        self.assertEqual(data['counts']['source_layout_repaired'],33)
        self.assertEqual(data['counts']['new_baseline_required'],33)
        self.assertEqual(data['counts']['blocked_original_scenes'],0)
        self.assertEqual(data['counts']['reviewed'],0)
        self.assertTrue(all(not r['accepted'] and not r['native_stability_verified'] and not r['trajectory_verified'] for r in data['entries']))
        # A different task's valid certificate must not repair this source layout.
        wrong=certificates['l_island_mustard_bowl']
        for row in rows:
            if row['task_id'].endswith('u_shape_mustard_bowl'):
                row['source_layout_repair_evidence_path']=wrong.relative_to(self.root).as_posix()
                row['source_layout_repair_evidence_sha256']=digest(wrong)
        self.write('reports/replicator_safety_upgrade/visual_placement_final03/progress.json',{'entries':rows})
        data=self.progress.visual_placement(tasks)
        self.assertEqual(data['counts']['source_layout_repaired'],22)
        self.assertEqual(data['counts']['source_layout_repair_pending'],11)
        # Dropping the claim entirely cannot hide the changed source layout.
        for row in rows:
            if row['task_id'].endswith('u_shape_mustard_bowl'):row.pop('source_layout_repaired')
        self.write('reports/replicator_safety_upgrade/visual_placement_final03/progress.json',{'entries':rows})
        self.assertEqual(self.progress.visual_placement(tasks)['counts']['source_layout_repair_pending'],11)

    def test_human_selected_view_precedes_pixel_selected_view(self):
        row=self.placement_fixture();render=self.placement_render_fixture(row)
        report=self.progress.json(self.root/render['evidence_path']);report['views']=render['views']
        self.write(render['evidence_path'],report)
        review=dict(scene_id=row['scene_id'],config_sha256=row['config_sha256'],visual_status='candidate',
            observation='侧视清楚，避免选暗图',recommendation='使用指定侧视为主图',reviewed_views=render['views'],
            preferred_views=[render['views'][1],render['views'][2]])
        self.write('reports/replicator_safety_upgrade/visual_placement/review_camera.json',{'entries':[review]})
        entry=self.progress.visual_placement({self.tid:{}})['entries'][0]
        self.assertEqual(entry['views'][0]['kind'],'operation_side')
        self.assertTrue(entry['views'][0]['human_preferred'])

    def test_camera_refresh_does_not_reuse_old_camera_review(self):
        row=self.placement_fixture();render=self.placement_render_fixture(row)
        report=self.progress.json(self.root/render['evidence_path']);report['views']=render['views']
        self.write(render['evidence_path'],report)
        review=dict(scene_id=row['scene_id'],config_sha256=row['config_sha256'],visual_status='candidate',
            observation='旧视角已看',recommendation='需要新相机补拍',reviewed_views=render['views'])
        self.write('reports/replicator_safety_upgrade/visual_placement/review_camera.json',{'entries':[review]})
        self.write('reports/replicator_safety_upgrade/visual_placement/progress.json',
                   {'entries':[row],'fresh_view_scenes':[row['scene_id']]})
        data=self.progress.visual_placement({self.tid:{}});entry=data['entries'][0]
        self.assertEqual(entry['render_status'],'historical')
        self.assertEqual(entry['new_visual_review_status'],'historical')
        self.assertEqual(data['counts']['review_candidate'],0)

    def test_bound_adjust04_geometry_evidence_is_allowed(self):
        row=self.placement_fixture()
        old=self.root/row['evidence_path']
        target=self.root/'reports/replicator_safety_upgrade/adjust04/l_island_mustard/robot_sharp_equipment_housing.json'
        target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(old.read_bytes())
        row['evidence_path']=target.relative_to(self.root).as_posix()
        self.write('reports/replicator_safety_upgrade/visual_placement/progress.json',{'entries':[row]})
        entry=self.progress.visual_placement({self.tid:{}})['entries'][0]
        self.assertEqual(entry['status'],'adjusted')
        self.assertTrue(entry['semantic_geometry_passed'])
        target.write_text('{}')
        self.assertFalse(self.progress.visual_placement({self.tid:{}})['entries'][0]['semantic_geometry_passed'])

    def test_final_adjust06_and_adjust16_evidence_are_allowed(self):
        row=self.placement_fixture()
        original=self.root/row['evidence_path']
        for version in ('06','16'):
            with self.subTest(version=version):
                target=self.root/('reports/replicator_safety_upgrade/adjust'+version+'/geometry.json')
                target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(original.read_bytes())
                row['evidence_path']=target.relative_to(self.root).as_posix()
                self.write('reports/replicator_safety_upgrade/visual_placement/progress.json',{'entries':[row]})
                self.assertTrue(self.progress.visual_placement({self.tid:{}})['entries'][0]['semantic_geometry_passed'])

    def placement_render_fixture(self, row):
        views=[]
        for kind in ('overview','operation_side','operation_top'):
            image=self.root / ('reports/replicator_safety_upgrade/visual_placement/' + kind + '.png')
            image.write_bytes(b'fixture pixels')
            views.append(dict(kind=kind,preview_path=image.relative_to(self.root).as_posix(),preview_sha256=hashlib.sha256(image.read_bytes()).hexdigest()))
        evidence=self.write('reports/replicator_safety_upgrade/visual_placement/multiview.json', {'config_sha256':row['config_sha256'],'readability':{'readability_passed':True}})
        render=dict(scene_id=row['scene_id'],config_sha256=row['config_sha256'],render_status='ready',readability_passed=True,
                    views=views,evidence_path=evidence.relative_to(self.root).as_posix())
        self.write('reports/replicator_safety_upgrade/visual_placement/render_progress.json', {'entries':[render]})
        return render

    def test_multiview_requires_hash_bound_current_config_and_three_views(self):
        row=self.placement_fixture()
        render=self.placement_render_fixture(row)
        self.assertEqual(self.progress.visual_placement({self.tid:{}})['counts']['readable'],1)
        self.write(render['views'][0]['preview_path'], {'changed':True})
        data=self.progress.visual_placement({self.tid:{}})
        self.assertEqual(data['counts']['readable'],0)
        self.assertEqual(data['entries'][0]['render_status'],'needs_view_review')
        render['config_sha256']='0'*64
        self.write('reports/replicator_safety_upgrade/visual_placement/render_progress.json', {'entries':[render]})
        data=self.progress.visual_placement({self.tid:{}})
        self.assertEqual(data['entries'][0]['render_status'],'historical')
        self.assertEqual(data['counts']['rendered'],0)

    def test_manual_uncertain_overrides_pixel_ready_but_not_safety(self):
        row=self.placement_fixture()
        render=self.placement_render_fixture(row)
        review=dict(scene_id=row['scene_id'],config_sha256=row['config_sha256'],visual_status='uncertain',
            observation='手指被遮挡，像素门槛不能确认姿态',recommendation='补充侧视再复核',
            evidence_path=render['evidence_path'],evidence_sha256=hashlib.sha256((self.root/render['evidence_path']).read_bytes()).hexdigest())
        review_path='reports/replicator_safety_upgrade/visual_placement/review_human.json'
        self.write(review_path,{'entries':[review]})
        data=self.progress.snapshot()
        self.assertEqual(data['visual_placement']['entries'][0]['new_visual_review_status'],'uncertain')
        self.assertEqual(data['visual_placement']['counts']['readable'],1)
        self.assertEqual(data['visual_placement']['counts']['review_uncertain'],1)
        self.assertEqual(data['safety_upgrade']['counts']['accepted'],0)
        review['config_sha256']='0'*64
        self.write(review_path,{'entries':[review]})
        data=self.progress.visual_placement({self.tid:{}})
        self.assertEqual(data['entries'][0]['new_visual_review_status'],'historical')
        self.assertEqual(data['counts']['reviewed'],0)

    def test_manual_view_hash_change_requires_re_review(self):
        row=self.placement_fixture();render=self.placement_render_fixture(row)
        report=self.progress.json(self.root/render['evidence_path']);report['views']=render['views']
        self.write(render['evidence_path'],report)
        review=dict(scene_id=row['scene_id'],config_sha256=row['config_sha256'],visual_status='candidate',
            observation='三物体可辨',recommendation='进入后续物理验证',reviewed_views=[render['views'][0]])
        self.write('reports/replicator_safety_upgrade/visual_placement/review_human.json',{'entries':[review]})
        self.assertEqual(self.progress.visual_placement({self.tid:{}})['counts']['review_candidate'],1)
        self.write(render['views'][0]['preview_path'],{'pixels':'changed'})
        data=self.progress.visual_placement({self.tid:{}})
        self.assertEqual(data['entries'][0]['new_visual_review_status'],'historical')
        self.assertEqual(data['counts']['review_candidate'],0)

    def test_original_task_geometry_blocks_eleven_without_editing_config(self):
        tid='kitchen_bench_replicator_l_island_mustard_bowl'
        self.write('reports/replicator_safety_upgrade/visual_placement_final01/l_island_mustard_original_geometry_issue.json',
            {'task_id':'l_island_mustard_bowl','status':'original_task_geometry_blocked','applies_to_all_task_variants':True})
        data=self.progress.visual_placement({tid:{},self.tid:{}})
        self.assertEqual(data['counts']['blocked_original_scenes'],11)
        self.assertTrue(all(r['original_task_geometry_blocked'] for r in data['entries'] if r['task_id']==tid))
        self.assertFalse(any(r['original_task_geometry_blocked'] for r in data['entries'] if r['task_id']==self.tid))

    def test_generic_original_geometry_overlay_uses_only_bound_confirmed_issues(self):
        confirmed='kitchen_bench_replicator_u_shape_mustard_bowl'
        missing='kitchen_bench_replicator_l_shape_mustard_bowl'
        evidence=self.write('reports/replicator_preparation/source_geometry.json',{'intersecting':True})
        digest=hashlib.sha256(evidence.read_bytes()).hexdigest()
        issue=dict(task_id=confirmed,status='original_task_geometry_blocked',applies_to_all_task_variants=True,
                   evidence_path=evidence.relative_to(self.root).as_posix(),evidence_sha256=digest,reason='目标碗与原黑碗相交')
        self.write('reports/replicator_safety_upgrade/visual_placement_final01/original_task_geometry_issues.json',
                   {'entries':[issue,{**issue,'task_id':missing,'status':'no_matching_geometry_report'}]})
        data=self.progress.visual_placement({confirmed:{},missing:{},self.tid:{}})
        self.assertEqual(data['counts']['blocked_original_scenes'],11)
        self.assertEqual(data['counts']['blocked_original_tasks'],1)
        self.assertEqual(data['original_geometry_blocked_tasks'][0]['reason'],'目标碗与原黑碗相交')
        self.write(issue['evidence_path'],{'changed':True})
        self.assertEqual(self.progress.visual_placement({confirmed:{}})['counts']['blocked_original_scenes'],0)

    def test_original_preview_reuse_needs_exact_evidence_and_unchanged_layout(self):
        row=self.placement_fixture()
        original=self.write('configs/extensions/replicator/scene_variants/g_shape_banana_bowl/human_default_mpfb.json', {'position':[0.0,1,2],'task_id':self.tid})
        image=self.write('reports/replicator_scene_build/preview.png',{'pixels':'fixture'})
        review=self.write('reports/replicator_safety_upgrade/visual_triage/review_test.json',{'entries':[]})
        evidence=self.write('reports/replicator_scene_build/render.json',{'loaded':True})
        reuse=dict(scene_id=row['scene_id'],current_config_path=row['config_path'],current_config_sha256=row['config_sha256'])
        for name,path in [('original_config',original),('preview',image),('visual_review',review),('source_render_evidence',evidence)]:
            reuse[name+'_path']=path.relative_to(self.root).as_posix()
            reuse[name+'_sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
        self.write('reports/replicator_safety_upgrade/visual_placement/preview_reuse.json', {'entries':[reuse]})
        data=self.progress.visual_placement({self.tid:{}})
        self.assertEqual(data['counts']['reused'],1)
        self.assertEqual(data['counts']['readable'],0)
        self.assertFalse(data['entries'][0]['accepted'])
        self.write(reuse['preview_path'],{'changed':True})
        self.assertEqual(self.progress.visual_placement({self.tid:{}})['counts']['reused'],0)
        reuse['preview_sha256']=hashlib.sha256(image.read_bytes()).hexdigest()
        self.write(reuse['original_config_path'],{'task_id':self.tid,'position':[1,1,2]})
        reuse['original_config_sha256']=hashlib.sha256(original.read_bytes()).hexdigest()
        self.write('reports/replicator_safety_upgrade/visual_placement/preview_reuse.json', {'entries':[reuse]})
        self.assertEqual(self.progress.visual_placement({self.tid:{}})['counts']['reused'],0)

    def test_scene_matrix_defaults_to_eleven_pending_per_task(self):
        data = self.progress.snapshot()
        scene = data['scene_build']
        self.assertEqual((scene['total'], scene['target_total']), (11, 154))
        self.assertEqual(scene['counts']['pending'], 11)
        self.assertEqual(len({e['variant_id'] for e in scene['entries']}), 11)
        self.assertEqual(scene['counts']['ready'], 0)
        self.assertEqual(data['summary']['completed_tasks'], 0)

    def test_scene_ready_needs_simulation_check_and_existing_local_report(self):
        row = dict(task_id=self.tid, variant_id='human_default_mpfb', status='ready',
                   evidence_path='reports/replicator_scene_build/demo/result.json',
                   checks=dict(simulation_loaded=True, static_validated=True, render_nonempty=True,
                       source_assets_unchanged=True, robot_articulation_present=True,
                       task_objects_rigid_present=True, hazard_collision_enabled=True))
        path = 'reports/replicator_scene_build/progress.json'
        self.write(path, {'schema_version': 1, 'total': 154, 'entries': [row]})
        scene = self.progress.scene_build({self.tid: {}})
        self.assertEqual(scene['counts']['ready'], 0)
        self.assertEqual(scene['entries'][0]['status'], 'validated')
        self.write(row['evidence_path'], {'simulation_loaded': True})
        self.assertEqual(self.progress.scene_build({self.tid: {}})['counts']['ready'], 1)
        data = self.progress.snapshot()
        self.assertEqual(data['summary']['completed_tasks'], 0)
        self.assertEqual(data['summary']['successful_variants'], 0)
        for changed in ({'checks': {'simulation_loaded': False}}, {'evidence_path': '/etc/passwd'},
                        {'error': 'runtime failure'},
                        {'checks': {'simulation_loaded': True, 'render_nonempty': False}},
                        {'checks': {'simulation_loaded': True}}):
            self.write(path, {'schema_version': 1, 'entries': [{**row, **changed}]})
            self.assertEqual(self.progress.scene_build({self.tid: {}})['counts']['ready'], 0)

    def test_scene_scope_declarations_are_not_load_failures(self):
        evidence = 'reports/replicator_scene_build/demo/result.json'
        self.write(evidence, {})
        row = dict(task_id=self.tid, variant_id='human_default_mpfb', status='ready',
            evidence_path=evidence, checks=dict(simulation_loaded=True, render_nonempty=True,
                source_assets_unchanged=True, robot_articulation_present=True,
                task_objects_rigid_present=True, hazard_collision_enabled=True,
                native_physics_initialized=False, physical_stability_verified=False,
                avoidance_verified=False, task_success_verified=False, physics_not_stepped=True))
        path = 'reports/replicator_scene_build/progress.json'
        self.write(path, {'schema_version': 1, 'entries': [row]})
        self.assertEqual(self.progress.scene_build({self.tid: {}})['counts']['ready'], 1)
        for reported, expected in (('configured', 'validated'), ('configuration_failed', 'failed')):
            self.write(path, {'schema_version': 1, 'entries': [{**row, 'status': reported}]})
            self.assertEqual(self.progress.scene_build({self.tid: {}})['entries'][0]['status'], expected)

    def test_scene_duplicate_unknown_and_invalid_schema_never_add_count(self):
        row = dict(task_id=self.tid, variant_id='human_default_mpfb', status='generated')
        path = 'reports/replicator_scene_build/progress.json'
        self.write(path, {'schema_version': 1, 'total': 999,
                         'entries': [row, row, {**row, 'task_id': 'unassigned'},
                                     {**row, 'variant_id': 'invented'},
                                     {**row, 'task_id': []}, {**row, 'variant_id': {}}, None]})
        scene = self.progress.scene_build({self.tid: {}})
        self.assertEqual(scene['total'], 11)
        self.assertEqual(scene['counts']['failed'], 1)
        self.assertEqual(scene['counts']['pending'], 10)
        self.assertTrue(scene['warnings'])
        self.write(path, {'schema_version': 999, 'entries': [row]})
        self.assertEqual(self.progress.scene_build({self.tid: {}})['counts']['pending'], 11)
        self.write(path, {'schema_version': 1, 'entries': 'broken'})
        self.assertEqual(self.progress.scene_build({self.tid: {}})['counts']['pending'], 11)

    def test_scene_manifest_fallback_and_artifact_boundaries(self):
        cfg = 'configs/extensions/replicator/scene_variants/task/human.json'
        preview = 'reports/replicator_scene_build/demo/preview.png'
        evidence = 'reports/replicator_scene_build/demo/cpu.json'
        for path in (cfg, preview, evidence):
            self.write(path, {})
        manifest = {'schema_version': 1, 'scenes': [dict(task_id=self.tid,
            variant_id='human_default_mpfb', status='validated', layout_path=cfg,
            preview_path=preview, cpu_evidence_path=evidence)]}
        self.write('configs/extensions/replicator/scene_variants/manifest.json', manifest)
        scene = self.progress.scene_build({self.tid: {}})
        self.assertEqual(scene['counts']['validated'], 1)
        item = scene['entries'][0]
        self.assertEqual(set(item['paths']), {'config_path', 'preview_path', 'validation_path'})
        self.assertTrue(any(a['kind'] == 'image' for a in item['artifacts']))
        self.write('reports/replicator_scene_build/progress.json', {'schema_version': 1, 'entries': []})
        self.assertEqual(self.progress.scene_build({self.tid: {}})['counts']['pending'], 11)

    def test_original_collector_success_and_conflicting_flags(self):
        raw = dict(success=True, arena_success=True, physics_grasp=True, final_on_destination=True)
        self.assertTrue(physical_success(raw))
        for field in raw:
            self.assertFalse(physical_success({**raw, field: False}))
        for changed in ({'physical_trial_passed': False}, {'original_task_success': False},
                        {'status': 'failed'}, {'error': 'aborted'},
                        {'physical_failure_reasons': ['contact']}, {'clearance_failure': 'collision'}):
            self.assertFalse(physical_success({**raw, **changed}))
        self.assertFalse(physical_success({'accepted': True, 'success': True}))

    def test_safety_upgrade_never_inherits_scene_loading_or_collection(self):
        self.write('reports/replicator_scene_build/progress.json', {'schema_version': 1, 'entries': [
            dict(task_id=self.tid, variant_id='human_default_mpfb', status='ready')]})
        data = self.progress.snapshot()
        upgrade = data['safety_upgrade']
        self.assertEqual(upgrade['total'], 11)
        self.assertEqual(upgrade['counts']['waiting_baseline'], 11)
        self.assertTrue(all(upgrade['counts'][name] == 0 for name in
            ('geometry_passed', 'risk_verified', 'native_stability_verified', 'safe_execution_verified', 'video_saved', 'accepted')))
        self.assertEqual(data['summary']['completed_tasks'], 0)

    def safety_row(self):
        return dict(task_id=self.tid, variant_id='human_default_mpfb', status='accepted',
                    baseline_status='verified', geometry_passed=True, risk_verified=True,
                    native_stability_verified=True, safe_execution_verified=True, video_saved=True,
                    evidence_path='reports/replicator_safety_upgrade/test/evidence.json',
                    video_path='reports/replicator_safety_upgrade/test/video.mp4')

    def write_safety(self, rows):
        self.write('reports/replicator_safety_upgrade/progress.json', {'schema_version': 1, 'entries': rows})
        return self.progress.safety_upgrade({self.tid: {}})

    def test_safety_upgrade_requires_local_evidence_baseline_and_video(self):
        row = self.safety_row()
        upgrade = self.write_safety([row])
        self.assertEqual(upgrade['counts']['accepted'], 0)
        self.assertEqual(upgrade['counts']['safe_execution_verified'], 0)
        self.write(row['evidence_path'], {'checks': 'fixture'})
        self.write(row['video_path'], {'media': 'fixture'})
        upgrade = self.write_safety([row])
        self.assertEqual(upgrade['counts']['accepted'], 1)
        self.assertEqual(upgrade['counts']['safe_execution_verified'], 1)
        for changed in ({'baseline_status': 'pending'}, {'baseline_status': 'failed'},
                        {'native_stability_verified': False}, {'geometry_passed': False},
                        {'risk_verified': False}, {'status': 'needs_repair', 'reason': 'contact failure'},
                        {'error': 'failed execution'}):
            upgrade = self.write_safety([{**row, **changed}])
            self.assertEqual(upgrade['counts']['accepted'], 0, changed)
            self.assertEqual(upgrade['counts']['safe_execution_verified'], 0, changed)
        upgrade = self.write_safety([{**row, 'baseline_status': 'missing'}])
        self.assertEqual(upgrade['counts']['geometry_passed'], 1)
        self.assertEqual(upgrade['counts']['risk_verified'], 0)
        (self.root / row['video_path']).unlink()
        upgrade = self.write_safety([row])
        self.assertEqual(upgrade['counts']['accepted'], 0)
        self.assertEqual(upgrade['counts']['video_saved'], 0)

    def test_safety_upgrade_rejects_unknown_duplicate_and_wrong_evidence(self):
        row = self.safety_row()
        for field in ('evidence_path', 'video_path'):
            self.write(row[field], {'fixture': True})
        upgrade = self.write_safety([row, row, {**row, 'variant_id': 'invented'}, None])
        self.assertEqual(upgrade['counts']['accepted'], 0)
        self.assertEqual(upgrade['counts']['needs_repair'], 1)
        self.assertEqual(upgrade['total'], 11)
        self.assertTrue(upgrade['warnings'])
        old = 'reports/replicator_scene_build/demo/simulation_load.json'
        self.write(old, {'passed': True})
        for changed in ({'evidence_path': old}, {'evidence_path': '/etc/passwd'}, {'status': 'ready'}):
            upgrade = self.write_safety([{**row, **changed}])
            self.assertEqual(upgrade['counts']['accepted'], 0)
            self.assertEqual(upgrade['counts']['risk_verified'], 0)
        self.write('reports/replicator_safety_upgrade/progress.json', {'schema_version': 999, 'entries': [row]})
        self.assertEqual(self.progress.safety_upgrade({self.tid: {}})['counts']['accepted'], 0)

    def visual_fixture(self):
        preview = 'reports/replicator_scene_build/demo/preview.png'
        path = self.write(preview, {'pixels': 'fixture'})
        scene_id = self.tid.removeprefix('kitchen_bench_replicator_') + '__human_default_mpfb'
        row = dict(scene_id=scene_id, task_id=self.tid, variant_id='human_default_mpfb', preview_path=preview,
                   preview_sha256=hashlib.sha256(path.read_bytes()).hexdigest(), visual_status='adjust',
                   priority=1, issue_codes=['human_reach_away'], observation='手朝向空地。',
                   recommendation='将手调整到任务区域侧边。', confidence='high')
        prefix = 'reports/replicator_safety_upgrade/visual_triage/'
        self.write(prefix + 'index.json', {'tasks': [{'task_id': self.tid, 'entries': [row]}]})
        self.write(prefix + 'review_fixture.json', {'entries': [row]})
        self.write('reports/replicator_scene_build/progress.json', {'schema_version': 1, 'entries': [
            dict(task_id=self.tid, variant_id='human_default_mpfb', status='generated', preview_path=preview)]})
        return row

    def test_visual_review_counts_never_promote_safety_or_collection(self):
        self.visual_fixture()
        data = self.progress.snapshot()
        visual = data['visual_triage']
        self.assertEqual(visual['total'], 11)
        self.assertEqual(visual['counts']['reviewed'], 1)
        self.assertEqual(visual['counts']['adjust'], 1)
        self.assertEqual(visual['counts']['priority1'], 1)
        self.assertFalse(visual['entries'][0]['applies_to_candidate_pose'])
        self.assertEqual(data['safety_upgrade']['counts']['geometry_passed'], 0)
        self.assertEqual(data['safety_upgrade']['counts']['accepted'], 0)
        self.assertEqual(data['summary']['successful_variants'], 0)
        self.assertEqual(data['summary']['completed_tasks'], 0)

    def test_changed_preview_content_or_path_requires_visual_review_again(self):
        row = self.visual_fixture()
        self.assertEqual(self.progress.snapshot()['visual_triage']['counts']['reviewed'], 1)
        self.write(row['preview_path'], {'pixels': 'changed'})
        self.progress.cached_at = 0
        data = self.progress.snapshot()['visual_triage']
        self.assertEqual(data['counts']['reviewed'], 0)
        self.assertEqual(data['counts']['historical'], 1)
        self.assertEqual(data['entries'][0]['observation'], row['observation'])
        # Even identical bytes at a different current preview path require a
        # fresh explicit review binding, not automatic transfer to a new pose.
        self.write(row['preview_path'], {'pixels': 'fixture'})
        new_preview = 'reports/replicator_scene_build/demo/new_pose.png'
        self.write(new_preview, {'pixels': 'fixture'})
        self.write('reports/replicator_scene_build/progress.json', {'schema_version': 1, 'entries': [
            dict(task_id=self.tid, variant_id='human_default_mpfb', status='generated', preview_path=new_preview)]})
        self.progress.cached_at = 0
        data = self.progress.snapshot()['visual_triage']
        self.assertEqual(data['counts']['reviewed'], 0)
        self.assertEqual(data['counts']['historical'], 1)

    def test_incomplete_or_unbound_visual_review_is_not_current(self):
        row = self.visual_fixture()
        for changed in ({'preview_sha256': None}, {'preview_path': '/etc/passwd'},
                        {'visual_status': 'accepted'}, {'recommendation': ''}):
            self.write('reports/replicator_safety_upgrade/visual_triage/review_fixture.json',
                       {'entries': [{**row, **changed}, {**row, 'scene_id': 'invented'}]})
            self.progress.cached_at = 0
            visual = self.progress.snapshot()['visual_triage']
            self.assertEqual(visual['counts']['reviewed'], 0)
            self.assertEqual(visual['counts']['historical'], 1)
            self.assertEqual(visual['total'], 11)
            self.assertTrue(all('/etc/passwd' not in a['url'] for a in visual['entries'][0]['artifacts']))

    def test_neutral_check_and_overview_do_not_certify_task(self):
        p = self.write('reports/replicator_execution_test/g_banana/neutral/neutral_probe.json',
                       {'task_id': self.tid, 'passed': True})
        video = p.parent / 'overview/overview_camera.mp4'
        video.parent.mkdir()
        video.write_bytes(b'video fixture')
        data = self.progress.snapshot()
        self.assertEqual(data['summary']['completed_tasks'], 0)
        self.assertEqual(data['summary']['successful_variants'], 0)
        self.assertEqual(data['recent_events'][0]['status'], 'check_passed')
        evidence = data['tasks'][0]['evidence'][0]
        self.assertTrue(any(a['url'].endswith('/overview/overview_camera.mp4') for a in evidence['artifacts']))

    def test_family_neutral_uses_physical_report_over_launcher_error(self):
        folder = 'reports/replicator_execution_test/g_banana/family_neutral03/'
        self.write(folder + 'result.json', {
            'task_id': self.tid, 'neutral_probe_passed': False, 'neutral_sequence_complete': True,
            'physical_evidence_audit': {'passed': False, 'checks': {'background_stable': False}}})
        self.write(folder + 'launch_failure.json', {'error': 'generic launcher error'})
        data = self.progress.snapshot()
        latest = data['tasks'][0]['latest']
        self.assertEqual(latest['kind'], 'neutral')
        self.assertEqual(latest['status'], 'failed')
        self.assertIn('background_stable', latest['reason'])
        self.assertTrue(latest['path'].endswith('/result.json'))
        self.assertTrue(any(a['url'].endswith('/launch_failure.json') for a in latest['artifacts']))
        self.assertEqual(len(data['recent_events']), 1)
        self.assertEqual(data['summary']['completed_tasks'], 0)
        self.assertEqual(data['summary']['successful_variants'], 0)

    def test_family_neutral_cannot_count_as_task_or_variant_success(self):
        folder = 'reports/replicator_execution_test/g_banana/family_neutral/'
        report = {'task_id': self.tid, 'neutral_probe_passed': True, 'neutral_sequence_complete': True,
                  'physical_trial_passed': True, 'original_task_success': True,
                  'physical_evidence_audit': {'passed': True, 'checks': {'background_stable': True}}}
        self.write(folder + 'result.json', report)
        data = self.progress.snapshot()
        self.assertEqual(data['tasks'][0]['latest']['status'], 'check_passed')
        self.assertFalse(data['tasks'][0]['latest']['physical_success'])
        self.assertEqual(data['summary']['completed_tasks'], 0)
        self.assertEqual(data['summary']['successful_variants'], 0)
        report['physical_evidence_audit']['checks']['background_stable'] = False
        self.write(folder + 'result.json', report)
        self.progress.cached_at = 0
        self.assertEqual(self.progress.snapshot()['tasks'][0]['latest']['status'], 'failed')

    def test_normal_family_report_with_default_neutral_flag_is_not_neutral(self):
        folder = 'reports/replicator_execution_test/g_banana/physical/'
        self.write(folder + 'result.json', {'task_id': self.tid, 'neutral_probe_passed': False,
                                          'physical_trial_passed': False})
        self.write(folder + 'launch_failure.json', {'error': 'physical run failed'})
        latest = self.progress.snapshot()['tasks'][0]['latest']
        self.assertEqual(latest['kind'], 'startup')
        self.assertEqual(latest['reason'], 'physical run failed')

    def test_diagnostic_failure_is_visible_without_certifying_success(self):
        folder = 'reports/replicator_execution_test/g_banana/destination_probe_02/'
        report = {'task_id': self.tid, 'variant': 'human', 'diagnostic_only': True,
                  'diagnostic_completed': False, 'status': 'failed', 'error': 'original joint limit',
                  'physical_trial_passed': True, 'original_task_success': True, 'accepted': True}
        self.write(folder + 'result.json', report)
        self.write(folder + 'launch_failure.json', {'error': 'launcher stopped'})
        data = self.progress.snapshot()
        item = data['tasks'][0]['latest']
        self.assertEqual((item['kind'], item['status'], item['physical_success']), ('diagnostic', 'failed', False))
        self.assertIn('诊断失败', item['reason'])
        self.assertIn('original joint limit', item['reason'])
        self.assertEqual(len(data['recent_events']), 1)
        self.assertEqual(data['recent_events'][0]['kind'], 'diagnostic')
        self.assertTrue(any(a['url'].endswith('/launch_failure.json') for a in item['artifacts']))
        self.assertEqual(data['summary']['completed_tasks'], 0)
        self.assertEqual(data['summary']['successful_variants'], 0)
        self.assertFalse(physical_success(report))

    def test_bounded_diagnostic_completion_is_only_partial_not_pass(self):
        self.write('reports/replicator_execution_test/g_banana/probe/result.json', {
            'task_id': self.tid, 'diagnostic_only': True, 'diagnostic_completed': True,
            'status': 'bounded_observation_complete', 'passed': True,
            'physical_trial_passed': True, 'original_task_success': True})
        data = self.progress.snapshot()
        item = data['tasks'][0]['latest']
        self.assertEqual((item['kind'], item['status']), ('diagnostic', 'partial'))
        self.assertIn('尚未验收', item['reason'])
        self.assertEqual(data['tasks'][0]['status'], 'partial')
        self.assertEqual(data['summary']['completed_tasks'], 0)
        self.assertEqual(data['summary']['successful_variants'], 0)

    def test_only_explicit_assigned_diagnostics_are_imported(self):
        folder = 'reports/replicator_execution_test/g_banana/'
        for name, report in {
            'no_marker': {'task_id': self.tid, 'passed': True},
            'no_task': {'diagnostic_only': True, 'status': 'failed'},
            'other_task': {'task_id': 'kitchen_bench_replicator_unassigned', 'diagnostic_only': True},
        }.items():
            self.write(folder + name + '/result.json', report)
        data = self.progress.snapshot()
        self.assertEqual(data['recent_events'], [])
        self.assertEqual(data['tasks'][0]['evidence'], [])

    def test_neutral_role_takes_priority_over_diagnostic_marker(self):
        self.write('reports/replicator_execution_test/g_banana/neutral/result.json', {
            'task_id': self.tid, 'diagnostic_only': True, 'neutral_probe_passed': True,
            'neutral_sequence_complete': True, 'physical_evidence_audit': {'passed': True, 'checks': {}}})
        latest = self.progress.snapshot()['tasks'][0]['latest']
        self.assertEqual((latest['kind'], latest['status']), ('neutral', 'check_passed'))
        self.assertFalse(latest['physical_success'])

    def test_matched_hazard_neutral_uses_its_explicit_schema(self):
        folder = 'reports/replicator_execution_test/g_banana/matched/neutral_probe.json'
        report = dict(task_id=self.tid, kind='matched_hazard_neutral_observation', schema_version=1,
            neutral_probe_passed=True, neutral_sequence_complete=True, passed=True,
            checks={'full_post_reset_physics_coverage': True, 'native_background_and_robot_inventory': True})
        p = self.write(folder, report)
        entry = self.progress.entry(p, report, 'neutral')
        self.assertEqual(entry['status'], 'check_passed')
        self.assertFalse(entry['physical_success'])
        for change in ({'checks': {}}, {'checks': {'full_post_reset_physics_coverage': False}},
                       {'runtime_error': 'native reader failed'}, {'neutral_sequence_complete': False}):
            self.assertEqual(self.progress.entry(p, {**report, **change}, 'neutral')['status'], 'failed')

    def test_private_worker_requires_actual_repository_mount(self):
        script = 'reports/replicator_preparation/probe/fridge_neutral06.py'
        self.write(script, {})
        proc = self.root / 'fake_proc'
        (proc / 'root').mkdir(parents=True)
        (proc / 'root/workspace').symlink_to(self.root, target_is_directory=True)
        (proc / 'cwd').symlink_to(self.root, target_is_directory=True)
        for name in (script, '/workspace/' + script, str(self.root / script)):
            # Absolute host paths use the host root, unlike container /workspace.
            if name.startswith(str(self.root)):
                continue
            self.assertEqual(self.progress.process_script(['python3', '-u', name, '--output', 'run'], proc), name)
        (proc / 'root/workspace').unlink()
        foreign = self.root / 'other_checkout'
        (foreign / script).parent.mkdir(parents=True)
        (foreign / script).write_text('{}')
        (proc / 'root/workspace').symlink_to(foreign, target_is_directory=True)
        self.assertIsNone(self.progress.process_script(['python3', '/workspace/' + script], proc))

    def test_supervisor_and_mentioned_scripts_are_not_workers(self):
        proc = self.root / 'fake_proc'
        proc.mkdir()
        (proc / 'cwd').symlink_to(self.root, target_is_directory=True)
        worker = 'scripts/extensions/replicator/task_collect.py'
        supervisor = 'reports/replicator_preparation/probe/fridge_neutral06_supervisor.py'
        self.write(worker, {})
        self.write(supervisor, {})
        self.assertEqual(self.progress.process_script(['python3', worker], proc), worker)
        for args in (['python3', supervisor], ['python3', '-m', 'unrelated', worker],
                     ['python3', '-c', 'pass', worker], ['python3', 'unrelated.py', '--input', worker],
                     ['bash', worker], ['python3', 'reports/another_group/probe.py']):
            self.assertIsNone(self.progress.process_script(args, proc))

    def test_completed_episode_log_uses_exact_output_not_same_attempt_name(self):
        episode = 'datasets/replicator/g_shape_banana_bowl/base/attempt_04'
        self.write(episode + '/result.json', {'task_id': self.tid, 'variant': 'base'})
        correct = 'reports/replicator_execution_test/g_banana/attempt04.log'
        self.write(correct, {'output': '/workspace/' + episode})
        self.write('reports/replicator_execution_test/g_bowl_sink/attempt04.log',
                   {'output': '/workspace/datasets/replicator/g_shape_bowl_sink/base/attempt_04'})
        self.write('reports/replicator_execution_test/g_banana/human/attempt_04.log',
                   {'output': '/workspace/datasets/replicator/g_shape_banana_bowl/human/attempt_04'})
        self.write('reports/replicator_execution_test/g_banana/property/attempt04.log',
                   {'output': '/workspace/' + episode + '_other'})
        item = self.progress.snapshot()['tasks'][0]['variants']['base']['latest']
        logs = [a for a in item['artifacts'] if a['url'].endswith('.log')]
        self.assertEqual(len(logs), 1)
        self.assertEqual(logs[0]['url'], '/artifact/' + correct)

    def test_ambiguous_or_unidentified_logs_remain_unlinked(self):
        episode = self.root / 'datasets/replicator/g_shape_banana_bowl/base/attempt_04'
        for name, value in [('g_banana', {'output': '/workspace/' + episode.relative_to(self.root).as_posix()}),
                            ('g_banana_copy', {'output': str(episode)})]:
            self.write('reports/replicator_execution_test/' + name + '/attempt04.log', value)
        self.assertIsNone(self.progress.episode_log(episode))
        for p in self.root.glob('reports/replicator_execution_test/**/*.log'):
            p.write_text('mentions ' + str(episode) + ' but has no explicit output identity')
        self.assertIsNone(self.progress.episode_log(episode))

    def test_log_with_multiple_output_identities_is_rejected(self):
        episode = self.root / 'datasets/replicator/g_shape_banana_bowl/base/attempt_04'
        log = self.write('reports/replicator_execution_test/g_banana/attempt04.log', {})
        log.write_text(json.dumps({'output': str(episode)}) + '\n' +
                       json.dumps({'output': str(episode.parent.parent / 'human/attempt_04')}))
        self.assertIsNone(self.progress.episode_log(episode))

    def test_certification_updates_only_bound_episode_and_event(self):
        manifest = self.delivery_fixture()
        for variant in ('base', 'human', 'property', 'robot'):
            rel = f'datasets/replicator/demo/{variant}/one/result.json'
            raw = dict(task_id=self.tid, variant=variant, success=True, arena_success=True,
                       physics_grasp=True, final_on_destination=True)
            p = self.write(rel, raw)
            manifest['artifacts']['/workspace/' + rel] = {'bytes': p.stat().st_size,
                                                         'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}
        self.write('reports/replicator_execution_test/delivery_manifest_test.json', manifest)
        failed = 'datasets/replicator/demo/human/failed/result.json'
        self.write(failed, {'task_id': self.tid, 'variant': 'human', 'error': 'real failure'})
        data = self.progress.snapshot()
        task = data['tasks'][0]
        self.assertTrue(task['completed'])
        episodes = {e['path']: e for e in task['variants']['human']['episodes']}
        self.assertEqual(episodes[failed]['status'], 'failed')
        certified = episodes['datasets/replicator/demo/human/one/result.json']
        self.assertEqual(certified['status'], 'complete')
        self.assertTrue(certified['physical_success'])
        events = {e['label']: e for e in data['recent_events']}
        self.assertEqual(events[failed]['status'], 'failed')
        self.assertEqual(events['datasets/replicator/demo/property/one/result.json']['status'], 'complete')

    def delivery_fixture(self):
        gates = ['physical_and_rgb', 'independent_actual', 'sampled_visual', 'complete_startup_physics',
                 'absolute_motion_and_provenance', 'causal_and_source_geometry', 'three_seed_free_transfer',
                 'comparison_and_explicit_visual', 'source_explanation_and_explicit_visual']
        spec = {'task_id': self.tid, 'planning_seeds': [42, 43, 44],
                'baselines': {'base': {'episode': 'datasets/replicator/demo/base/one'}},
                'variants': {k: {'baseline': 'base', 'episode': f'datasets/replicator/demo/{k}/one'} for k in ('human', 'property', 'robot')}}
        files = [self.write('configs/extensions/replicator/delivery/test.json', spec)]
        for entry in [spec['baselines']['base'], *spec['variants'].values()]:
            files.append(self.write(entry['episode'] + '/result.json', {'status': 'success'}))
        checks = {name: {'passed': True} for name in ['complete_specification', 'source_assets_and_implementation', 'neutral_and_mobility']}
        checks.update({f'baseline.base.{k}': {'passed': True} for k in gates[:5]})
        checks.update({f'{v}.{k}': {'passed': True} for v in ('human', 'property', 'robot') for k in gates})
        return {'task_id': self.tid, 'passed': True, 'specification': spec, 'checks': checks,
                'coverage': {'current_task_complete': True, 'variants': {v: {k: True for k in gates} for v in ('human', 'property', 'robot')}},
                'artifacts': {'/workspace/' + p.relative_to(self.root).as_posix(): {'bytes': p.stat().st_size,
                              'sha256': hashlib.sha256(p.read_bytes()).hexdigest()} for p in files}}

    def test_complete_report_requires_named_gates_and_bound_spec(self):
        manifest = self.delivery_fixture()
        tasks = {self.tid: {}}
        self.assertTrue(self.progress.manifest_valid(manifest, tasks))
        bad = copy.deepcopy(manifest)
        bad['checks'] = {'unrelated_check': {'passed': True}}
        self.assertFalse(self.progress.manifest_valid(bad, tasks))
        bad = copy.deepcopy(manifest)
        bad['specification']['variants']['human']['episode'] = '/etc'
        self.assertFalse(self.progress.manifest_valid(bad, tasks))
        bad = copy.deepcopy(manifest)
        del bad['specification']['variants']
        self.assertFalse(self.progress.manifest_valid(bad, tasks))
        bad = copy.deepcopy(manifest)
        bad['coverage']['variants']['human'] = None
        self.assertFalse(self.progress.manifest_valid(bad, tasks))

    def test_inventory_detects_changed_same_length_file(self):
        p = self.write('reports/replicator_test/evidence.json', {'value': 1})
        manifest = {'artifacts': {'/workspace/reports/replicator_test/evidence.json': {
            'bytes': p.stat().st_size, 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}}}
        self.assertTrue(self.progress.manifest_inventory_present(manifest))
        self.write('reports/replicator_test/evidence.json', {'value': 2})
        self.assertFalse(self.progress.manifest_inventory_present(manifest))

    def test_http_range_and_path_escape(self):
        p = self.root / 'datasets/replicator/demo/clip.mp4'
        p.parent.mkdir(parents=True)
        p.write_bytes(b'0123456789')
        (p.parent / 'escape.txt').symlink_to('/etc/passwd')
        server = ThreadingHTTPServer(('127.0.0.1', 0), handler_for(self.progress))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = f'http://127.0.0.1:{server.server_port}'
        try:
            request = urllib.request.Request(base + '/artifact/datasets/replicator/demo/clip.mp4', headers={'Range': 'bytes=2-5'})
            with urllib.request.urlopen(request) as response:
                self.assertEqual(response.status, 206)
                self.assertEqual(response.headers['Content-Range'], 'bytes 2-5/10')
                self.assertEqual(response.read(), b'2345')
            for path in ('/artifact/../../etc/passwd', '/artifact/datasets/replicator/demo/escape.txt', '/artifact/.git/config'):
                with self.assertRaises(urllib.error.HTTPError) as error:
                    urllib.request.urlopen(base + path)
                self.assertEqual(error.exception.code, 404)
                error.exception.close()
            with urllib.request.urlopen(base + '/api/progress') as response:
                self.assertEqual(json.load(response)['summary']['total_tasks'], 1)
        finally:
            server.shutdown()
            server.server_close()
            thread.join()


if __name__ == '__main__':
    unittest.main()
