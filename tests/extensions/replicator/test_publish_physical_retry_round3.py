"""CPU publication gates; synthetic artifacts never execute a simulator."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

ROOT=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('publish_round3',ROOT/'scripts/extensions/replicator/publish_physical_retry_round3.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class PublicationTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        self.patch=patch.object(module,'ROOT',self.root);self.patch.start();self.addCleanup(self.patch.stop)
        self.previous=json.loads((ROOT/module.PREVIOUS).read_text())
        prior_manifest=self.write('prior/manifest.json',{'stage':'complete'})
        self.previous['source_retry_manifest']=prior_manifest
        evidence=self.write('evidence.json',{'cpu_only':True})
        self.plan=dict(retry_round=3,layout_changed=False,success_predicate_changed=False,
            source_previous_retry_manifest=prior_manifest,source_first_pass_manifest=prior_manifest,
            source_camera_retake_manifest=prior_manifest,execution_settings={},entries=[])
        self.manifest=dict(kind='physical_failure_retry',retry_round=3,stage='complete',rendering_enabled=False,
            source_previous_retry_manifest=prior_manifest,entries=[])
        self.audit=dict(passed=True,errors=[],entries=[])
        self.visual=dict(kind='physical_failure_visual_review',entries=[])
        self.planning_diagnostics=dict(kind='physical_retry_planning_failure_cpu_diagnostics',retry_round=3,
            total=9,diagnosed=9,complete=True,source_plan_unchanged=True,errors=0,
            physical_execution_started=False,kit_started=False,curobo_planning_calls=0,new_target_candidates=0,
            cpu_ik_seed_attempts_per_scene=1,layout_changed=False,diagnostic_sources={},
            method='Synthetic single CPU IK fixture',limitations=['Single branch does not establish infeasibility'],entries=[])
        failures=[e for e in self.previous['entries'] if not e['latest_success']]
        self.failed_sid=failures[1]['scene_id']
        for i,old in enumerate(failures):
            sid=old['scene_id'];config=self.write('configs/'+sid+'.json',{'scene_id':sid})
            old.update(config_path=config['path'],config_sha256=config['sha256'])
            old['latest_physical_result']=self.write('prior/'+sid+'.json',{'scene_id':sid,'config_sha256':config['sha256'],'overview':{'camera':'recorded'}})
            actual_layer = old['retry_round2'] if old['retry_round2'].get('executed') else old['retry']
            actual_layer['result'] = deepcopy(old['latest_physical_result'])
            state='failed' if i==1 else 'success' if i<20 else 'planning_failed'
            generated=state!='planning_failed'
            identity=dict(scene_id=sid,task_id=old['task_id'],variant_id=old['variant_id'],config_path=config['path'],config_sha256=config['sha256'])
            planning=self.write('planning/'+sid+'.json',dict(identity,status='generated' if generated else 'failed',attempts=1,
                executed=False,planning_only=True,composition={'source_paths_sha256':{}},failure_reason=None if generated else 'IK_FAIL',
                phases=[] if generated else [dict(phase='approach',solver_status='IK_FAIL')],artifacts={},failed_phase=None if generated else 'approach'))
            scene=self.write('scene/'+sid+'.usda',{'scene':sid})
            candidate=dict(identity,candidate_planning_result=planning,previous_retry_result=old['latest_physical_result'],
                source_scene_path=scene['path'],source_scene_sha256=scene['sha256'],
                physical_attempt_number=3,planning_status='generated' if generated else 'failed',
                trajectory_path=None,trajectory_sha256=None,failure_reason=None if generated else 'IK_FAIL',
                adjustment=dict(description='new grasp',evidence_path=evidence['path'],evidence_sha256=evidence['sha256']))
            if generated:
                trajectory=self.write('trajectories/'+sid+'.npz',{'fixture':'trajectory hash only'})
                candidate.update(trajectory_path=trajectory['path'],trajectory_sha256=trajectory['sha256'])
            row=dict(candidate,execution_status=state,attempts=int(generated),executed=generated,task_success=state=='success')
            if generated:
                episode=self.root/'exports'/module.RUN/'executions'/sid;episode.mkdir(parents=True)
                raw=dict(row,kind='physical_failure_retry_execution',planning_result_path=planning['path'],planning_result_sha256=planning['sha256'],
                    failure_reason=None if state=='success' else 'Final predicate failed',
                    prior_physical_attempts=[dict(result_path=old['latest_physical_result']['path'],result_sha256=old['latest_physical_result']['sha256'])])
                if state=='failed':
                    np.savez(episode/'physics_states.npz',physics_step=np.array([1,2]),phase=np.array(['close','lift']),
                        action_index=np.array([0,1]),source_trajectory_index=np.array([-1,2]))
                    states=module.ref(episode/'physics_states.npz');meta=self.write(episode/'physics_metadata.json',{'samples':2})
                    overlay=self.write('overlay.usda',{'fixture':'overlay hash'})
                    raw['physics_evidence']=dict(q_path=states['path'],q_sha256=states['sha256'],metadata_path=meta['path'],metadata_sha256=meta['sha256'])
                    raw['physical_overlay_binding']=dict(path=overlay['path'],sha256_start=overlay['sha256'],sha256_end=overlay['sha256'],unchanged=True)
                result=self.write(episode/'result.json',raw)
                row.update(execution_result_path=result['path'],execution_result_sha256=result['sha256'],failure_reason=raw['failure_reason'])
                self.audit['entries'].append(dict(scene_id=sid,result=result,no_render_gate_passed=True))
                if state=='failed':
                    binding=module.expected_diagnostic_binding(result,raw)
                    diagnostic=self.root/module.DIAGNOSTICS/'scenes'/sid/'diagnostic.json'
                    image=self.write(diagnostic.parent/'frame_00.jpg',{'fixture':'image bytes'})
                    frame=dict(state_index=0,physics_step=1,phase='close',action_index=0,source_trajectory_index=-1,
                        image_path=image['path'],image_sha256=image['sha256'])
                    diagnostic_ref=self.write(diagnostic,dict(kind=module.DIAGNOSTIC_KIND,scene_id=sid,status='ready',
                        physical_episode_started=False,physics_steps_requested=0,timeline_playing=False,new_task_success_claimed=False,
                        binding=binding,frames=[frame]))
                    self.visual['entries'].append(dict(scene_id=sid,execution_result_sha256=result['sha256'],diagnostic_report_sha256=diagnostic_ref['sha256'],
                        observed_visual_facts_zh=['测试图像事实'],interpretation_zh='测试推断',next_adjustment_zh='测试建议',root_cause_confirmed=False,
                        reviewed_image_paths=[image['path']]))
            else:
                self.audit['entries'].append(dict(scene_id=sid,executed=False,attempts=0))
                self.planning_diagnostics['entries'].append(dict(scene_id=sid,task_id=old['task_id'],variant_id=old['variant_id'],
                    candidate_planning_result=planning,config=config,source_scene=scene,status='diagnosed',
                    failed_phase='approach',solver_status='IK_FAIL',root_cause_confirmed=False,scene_infeasibility_established=False,
                    fixed_goal_one_seed_ik=dict(solved=True,seed_attempts=1,collision_diagnostic=dict(unique_world_pairs=[dict(link='finger',world_object='/World/hazard/fixture')])),
                    failed_phase_start=dict(collision_diagnostic=dict(unique_world_pairs=[])),
                    observed_cpu_facts_zh=['单个CPU目标IK有解','目标保守包络与测试障碍有接触或近邻'],
                    interpretation_zh='仅一个分支，根因未确认',next_adjustment_zh='核对本景finger与fixture，调整抓姿而不改布局'))
            self.plan['entries'].append(candidate);self.manifest['entries'].append(row)
        self.write(module.PREVIOUS,self.previous)
        plan=self.write('plan.json',self.plan);self.manifest.update(retry_plan_path=plan['path'],retry_plan_sha256=plan['sha256'])
        self.planning_diagnostics['source_plan']=plan
        self.flush()

    def write(self,path,value):
        path=module.local(path);path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(value,ensure_ascii=False));return module.ref(path)

    def flush(self):
        manifest=self.write(module.MANIFEST,self.manifest);self.audit['manifest_snapshot']=manifest
        self.write(module.AUDIT,self.audit);self.write(module.VISUAL,self.visual)
        self.write(module.PLANNING_DIAGNOSTICS,self.planning_diagnostics)

    def test_complete_merge_retains_history_and_zero_physics(self):
        data=module.build();text=module.document(data)
        self.assertEqual((len(data['entries']),len(data['tasks'])),(154,14))
        self.assertEqual(data['counts']['latest_success'],144)
        self.assertEqual(data['counts']['physical_attempts'],20)
        self.assertEqual(data['counts']['planning_failed'],9)
        for old,new in zip(self.previous['entries'],data['entries']):
            for key in ('first_pass','camera_retake','retry','retry_round2'):
                self.assertEqual(old.get(key),new.get(key))
        planning=next(e for e in data['entries'] if e['latest_status']=='planning_failed')
        self.assertNotIn('result',planning['retry_round3'])
        old=next(e for e in self.previous['entries'] if e['scene_id']==planning['scene_id'])
        self.assertEqual(planning['latest_physical_result'],old['latest_physical_result'])
        self.assertIsNone(planning['retry_round3']['physical_attempt_number'])
        self.assertEqual(planning['retry_round3']['candidate_investigation'],'核对本景finger与fixture，调整抓姿而不改布局')
        self.assertFalse(planning['retry_round3']['planning_failure_diagnostic']['root_cause_confirmed'])
        self.assertEqual(data['source_planning_failure_diagnostics'],module.ref(module.PLANNING_DIAGNOSTICS))
        self.assertIn('9景规划失败的只读CPU诊断',text)
        self.assertIn('14 × 11',text)
        table_width=None
        for line in text.splitlines():
            if not line.startswith('|'):
                table_width=None
                continue
            if table_width is None:table_width=line.count('|')
            self.assertEqual(line.count('|'),table_width)
        self.assertFalse((self.root/module.JSON_PATH).exists())

    def test_nonterminal_rejected(self):
        self.manifest['stage']='executing';self.flush()
        with self.assertRaisesRegex(ValueError,'terminal'):module.build()

    def test_round2_planning_failure_keeps_actual_history_when_round3_executes(self):
        data=module.build();lookup={e['scene_id']:e for e in data['entries']}
        old=next(e for e in self.previous['entries'] if e.get('retry_round2')
            and e['retry_round2']['execution_status']=='planning_failed'
            and lookup[e['scene_id']]['retry_round3']['executed'])
        new=lookup[old['scene_id']]
        self.assertNotIn('result',new['retry_round2'])
        self.assertEqual(new['retry_round3']['previous_result'],old['latest_physical_result'])
        self.assertEqual(new['latest_physical_result'],new['retry_round3']['result'])
        self.assertNotEqual(new['latest_physical_result'],old['latest_physical_result'])

    def test_round2_planning_failure_without_round3_execution_retains_last_actual(self):
        data=module.build();lookup={e['scene_id']:e for e in data['entries']}
        old=next(e for e in self.previous['entries'] if e.get('retry_round2')
            and e['retry_round2']['execution_status']=='planning_failed'
            and lookup[e['scene_id']]['retry_round3']['execution_status']=='planning_failed')
        new=lookup[old['scene_id']]
        self.assertEqual(new['latest_physical_result'],old['latest_physical_result'])
        self.assertNotIn('result',new['retry_round2'])
        self.assertNotIn('result',new['retry_round3'])
        self.assertEqual(new['retry_round3']['attempts'],0)

    def test_zero_planning_failure_needs_no_diagnostic_artifact(self):
        module.local(module.PLANNING_DIAGNOSTICS).unlink()
        result=module.planning_diagnostic_gate(module.PLANNING_DIAGNOSTICS,{}, {},
            {'scene':{'execution_status':'success'}})
        self.assertIsNone(result[0]);self.assertEqual(result[2],{})

    def test_zero_planning_failure_rejects_stale_nonempty_diagnostic(self):
        with self.assertRaisesRegex(ValueError,'complete current CPU-only'):
            module.planning_diagnostic_gate(module.PLANNING_DIAGNOSTICS,
                self.planning_diagnostics['source_plan'],{}, {'scene':{'execution_status':'success'}})

    def test_stale_audit_rejected(self):
        self.audit['manifest_snapshot']['sha256']='0'*64;self.write(module.AUDIT,self.audit)
        with self.assertRaisesRegex(ValueError,'Audit does not bind'):module.build()

    def test_missing_actual_failure_review_rejected(self):
        self.visual['entries']=[];self.write(module.VISUAL,self.visual)
        with self.assertRaisesRegex(ValueError,'Every actual failed'):module.build()

    def test_wrong_result_image_binding_rejected(self):
        self.visual['entries'][0]['execution_result_sha256']='0'*64;self.write(module.VISUAL,self.visual)
        with self.assertRaisesRegex(ValueError,'different result'):module.build()

    def test_forged_zero_attempt_planning_result_rejected(self):
        self.manifest['entries'][20]['attempts']=1;self.flush()
        with self.assertRaisesRegex(ValueError,'zero physical attempts'):module.build()

    def test_forged_frame_label_rejected(self):
        path=module.local(module.DIAGNOSTICS)/'scenes'/self.failed_sid/'diagnostic.json'
        diagnostic=module.read(path);diagnostic['frames'][0]['physics_step']=123
        binding=self.write(path,diagnostic);self.visual['entries'][0]['diagnostic_report_sha256']=binding['sha256'];self.write(module.VISUAL,self.visual)
        with self.assertRaisesRegex(ValueError,'differs from native state'):module.build()

    def test_missing_planning_diagnostic_rejected(self):
        self.planning_diagnostics['entries'].pop();self.flush()
        with self.assertRaisesRegex(ValueError,'cover exactly all current'):module.build()

    def test_stale_planning_candidate_diagnostic_rejected(self):
        self.planning_diagnostics['entries'][0]['candidate_planning_result']=dict(path='old.json',sha256='0'*64);self.flush()
        with self.assertRaisesRegex(ValueError,'candidate_planning_result binding differs'):module.build()

    def test_wrong_planning_diagnostic_phase_rejected(self):
        self.planning_diagnostics['entries'][0]['failed_phase']='retreat';self.flush()
        with self.assertRaisesRegex(ValueError,'identity/uncertainty differs'):module.build()

    def test_planning_diagnostic_cannot_claim_confirmed_root_cause(self):
        self.planning_diagnostics['entries'][0]['root_cause_confirmed']=True;self.flush()
        with self.assertRaisesRegex(ValueError,'identity/uncertainty differs'):module.build()

    def test_planning_diagnostic_cannot_add_solver_attempt(self):
        self.planning_diagnostics['curobo_planning_calls']=1;self.flush()
        with self.assertRaisesRegex(ValueError,'CPU-only fixed-goal cohort'):module.build()

    def make_zero_actual_failures(self):
        row=next(r for r in self.manifest['entries'] if r['scene_id']==self.failed_sid)
        raw=module.read(row['execution_result_path'])
        raw.update(execution_status='success',task_success=True,failure_reason=None)
        result=self.write(row['execution_result_path'],raw)
        row.update(execution_status='success',task_success=True,failure_reason=None,execution_result_sha256=result['sha256'])
        next(a for a in self.audit['entries'] if a['scene_id']==self.failed_sid)['result']=result
        self.visual['entries']=[];self.flush()

    def assert_zero_failure_publication(self,data):
        text=module.document(data)
        self.assertEqual(data['counts']['latest_success'],145)
        self.assertEqual(data['counts']['latest_failed'],9)
        self.assertEqual(data['counts']['actual_failed_executions'],0)
        self.assertEqual(data['counts']['planning_failed'],9)
        self.assertFalse(data['diagnostic_visual_review']['review_required'])
        self.assertEqual(data['diagnostic_visual_review']['image_sets_bound'],0)
        self.assertFalse(data['safety_accepted'])
        self.assertFalse(data['safety_certified'])
        self.assertFalse(data['long_term_stability_verified'])
        self.assertTrue(all(e['safety_accepted'] is False for e in data['entries']))
        self.assertIn('本轮实际执行失败为0景',text)
        self.assertIn('9景规划失败没有新增物理测试，不能记为物理成功',text)
        self.assertNotIn('](../../../reports/',text)

    def test_zero_actual_failure_with_empty_review(self):
        self.make_zero_actual_failures();data=module.build()
        self.assert_zero_failure_publication(data)
        self.assertEqual(data['diagnostic_visual_review']['source_visual_review'],module.ref(module.VISUAL))

    def test_zero_actual_failure_without_review_artifact(self):
        self.make_zero_actual_failures();module.local(module.VISUAL).unlink()
        data=module.build();self.assert_zero_failure_publication(data)
        self.assertIsNone(data['diagnostic_visual_review']['source_visual_review'])

    def test_zero_actual_failure_rejects_stale_nonempty_review(self):
        stale=deepcopy(self.visual['entries']);self.make_zero_actual_failures()
        self.visual['entries']=stale;self.write(module.VISUAL,self.visual)
        with self.assertRaisesRegex(ValueError,'Every actual failed execution'):module.build()


if __name__=='__main__':unittest.main()
