#!/usr/bin/env python3
"""Publish round7 only after terminal execution, audit and bound image review.

The original-layout publication and all six historical retry layers remain unchanged.
The selected hazard-pose revisions have a separate current-version result/count.
--dry-run performs the same final gates and writes no publication artifacts.
"""
from __future__ import annotations
import argparse
from collections import Counter
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from scripts.extensions.replicator.scene_revision_review import validate_scene_revision
RUN = 'replicator_physical_retry_round7_20261029'
MANIFEST = f'exports/{RUN}/manifest.json'
PREVIOUS = 'configs/extensions/replicator/physical_retry_results_round6_20261029.json'
AUDIT = f'reports/{RUN}/audit_execution.json'
VISUAL = f'reports/{RUN}/failure_visual_review.json'
PLANNING_DIAGNOSTICS = f'reports/{RUN}/planning_failure_diagnostics.json'
DIAGNOSTICS = 'reports/replicator_physical_retry_round7_diagnostics_20261029'
JSON_PATH = 'configs/extensions/replicator/physical_retry_results_round7_20261029.json'
DOC_PATH = 'docs/extensions/replicator/PHYSICAL_RETRY_ROUND7_20261029.md'
TERMINAL = {'success', 'failed', 'error', 'planning_failed'}
DIAGNOSTIC_KIND = 'physical_failure_diagnostic_state_replay'


def local(value):
    path = (ROOT / str(value).removeprefix('/workspace/')).resolve()
    if not path.is_relative_to(ROOT): raise ValueError('Path outside checkout')
    return path


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1024*1024), b''): h.update(chunk)
    return h.hexdigest()


def read(path): return json.loads(local(path).read_text())
def ref(path): return dict(path=str(local(path).relative_to(ROOT)), sha256=sha(local(path)))


def bound(binding):
    if (not isinstance(binding, dict) or not isinstance(binding.get('path'), str)
            or not re.fullmatch('[0-9a-f]{64}', str(binding.get('sha256', '')))):
        raise ValueError('Invalid source binding')
    path = local(binding['path'])
    if sha(path) != binding['sha256']: raise ValueError('Bound source changed: '+str(path))
    return path


def rows(document):
    values = document['entries']; result = {r['scene_id']:r for r in values}
    if len(result) != len(values): raise ValueError('Duplicate scene IDs')
    return result


def assert_diagnostic(report, sid, expected_binding, report_path):
    if (report.get('kind') != DIAGNOSTIC_KIND or report.get('scene_id') != sid
            or report.get('status') != 'ready' or report.get('physical_episode_started') is not False
            or report.get('physics_steps_requested') != 0 or report.get('timeline_playing') is not False
            or report.get('new_task_success_claimed') is not False or report.get('binding') != expected_binding
            or not 1 <= len(report.get('frames', [])) <= 10):
        raise ValueError('Diagnostic must bind this actual episode with zero new physics: '+sid)
    for frame in report['frames']:
        image = bound(dict(path=frame['image_path'], sha256=frame['image_sha256']))
        if image.parent != report_path.parent:
            raise ValueError('Image is outside the bound diagnostic directory')


def expected_diagnostic_binding(result_ref, raw):
    """Same immutable inputs as render_physical_retry_failures.load_record."""
    revision=validate_scene_revision(ROOT,raw['scene_revision'],expected_scene_id=raw['scene_id'],current=raw)
    config = bound(dict(path=raw['config_path'], sha256=raw['config_sha256']))
    planning = bound(dict(path=raw['planning_result_path'], sha256=raw['planning_result_sha256']))
    planned = read(planning)
    for path, digest in planned['composition']['source_paths_sha256'].items():
        bound(dict(path=path, sha256=digest))
    evidence = raw['physics_evidence']
    states = bound(dict(path=evidence['q_path'], sha256=evidence['q_sha256']))
    metadata = bound(dict(path=evidence['metadata_path'], sha256=evidence['metadata_sha256']))
    result_path = bound(result_ref)
    if states.parent != result_path.parent or metadata.parent != result_path.parent:
        raise ValueError('Native records belong to a different physical episode')
    overlay = raw['physical_overlay_binding']
    if overlay.get('unchanged') is not True or overlay.get('sha256_start') != overlay.get('sha256_end'):
        raise ValueError('Physical background changed')
    overlay_path = bound(dict(path=overlay['path'], sha256=overlay['sha256_start']))
    def rel(path): return str(path.relative_to(ROOT))
    binding = dict(execution_result_path=rel(result_path), execution_result_sha256=result_ref['sha256'],
        config_path=rel(config), config_sha256=raw['config_sha256'],
        planning_result_path=rel(planning), planning_result_sha256=raw['planning_result_sha256'],
        physics_states_path=rel(states), physics_states_sha256=evidence['q_sha256'],
        physics_metadata_path=rel(metadata), physics_metadata_sha256=evidence['metadata_sha256'],
        physical_overlay_path=rel(overlay_path), physical_overlay_sha256=overlay['sha256_start'],
        source_paths_sha256=planned['composition']['source_paths_sha256'],scene_revision=raw['scene_revision'],layout_changed=True,camera_reference_scope='historical_camera_pose_only')
    for prior in reversed(raw['prior_physical_attempts']):
        prior_ref = dict(path=prior['result_path'], sha256=prior['result_sha256'])
        prior_path = bound(prior_ref); observation = read(prior_path)
        if observation.get('scene_id') != raw['scene_id'] or observation.get('config_sha256') != revision['document']['old_config']['sha256']:
            raise ValueError('Recorded camera history differs from this scene/config')
        if observation.get('overview') or observation.get('camera_override', {}).get('applied'):
            binding.update(observation_reference_path=rel(prior_path), observation_reference_sha256=prior_ref['sha256'])
            break
    else: raise ValueError('No actual camera observation in prior physical history')
    return binding


def validate_review(item, sid, result_ref, raw, diagnostics_root):
    import numpy as np
    path = local(diagnostics_root)/'scenes'/sid/'diagnostic.json'
    report_ref = ref(path); report = read(path)
    expected = expected_diagnostic_binding(result_ref, raw)
    assert_diagnostic(report, sid, expected, path)
    if (item.get('scene_id') != sid or item.get('execution_result_sha256') != result_ref['sha256']
            or item.get('diagnostic_report_sha256') != report_ref['sha256']):
        raise ValueError('Visual review binds a different result/diagnostic: '+sid)
    facts = item.get('observed_visual_facts_zh')
    if not isinstance(facts, list) or not facts or any(not isinstance(x, str) or not x.strip() for x in facts):
        raise ValueError('Concrete image observations are required: '+sid)
    for key in ('interpretation_zh', 'next_adjustment_zh'):
        if not isinstance(item.get(key), str) or not item[key].strip(): raise ValueError('Missing '+key)
    if type(item.get('root_cause_confirmed')) is not bool: raise ValueError('Explicit root cause uncertainty is required')
    if item['root_cause_confirmed']:
        evidence = item.get('confirmation_evidence')
        if not isinstance(evidence, list) or not evidence or not item.get('confirmation_basis_zh'):
            raise ValueError('A confirmed root cause needs additional bound evidence')
        for binding in evidence: bound(binding)
    # Read only four small native arrays; large body-state arrays stay on disk.
    with np.load(local(expected['physics_states_path']), allow_pickle=False) as archive:
        state = {k:archive[k] for k in ('physics_step', 'phase', 'action_index', 'source_trajectory_index')}
    def actual_frame(frame):
        index = frame.get('state_index')
        if type(index) is not int or not 0 <= index < len(state['phase']): raise ValueError('Invalid measured frame index')
        for key in state:
            actual = str(state[key][index]) if key == 'phase' else int(state[key][index])
            if frame.get(key) != actual: raise ValueError('Image label differs from native state: '+key)
    for frame in report['frames']: actual_frame(frame)
    image_paths = {str(local(f['image_path'])) for f in report['frames']}
    for reference in item.get('supplementary_diagnostic_reports', []):
        extra_path = bound(reference); extra = read(extra_path)
        assert_diagnostic(extra, sid, expected, extra_path)
        for frame in extra['frames']:
            actual_frame(frame)
            if not any(all(frame.get(k) == primary.get(k) for k in ('state_index','physics_step','phase','action_index')) for primary in report['frames']):
                raise ValueError('Supplementary image does not use a primary reviewed state')
        image_paths.update(str(local(f['image_path'])) for f in extra['frames'])
    reviewed = item.get('reviewed_image_paths')
    if reviewed is not None and (not isinstance(reviewed, list) or not reviewed
            or any(str(local(p)) not in image_paths for p in reviewed)):
        raise ValueError('Review names an unbound image')
    review = dict(item, status='complete', interpretation_is_inference=not item['root_cause_confirmed'])
    images = dict(status='ready', report=report_ref, frames=report['frames'],
        kind='measured_state_diagnostic_replay', not_execution_camera_frames=True,
        visual_interpretation_not_yet_written=False)
    return review, images


def audit_gate(audit, manifest_ref, cohort):
    if (audit.get('kind') != 'physical_retry_round7_independent_audit'
            or audit.get('passed') is not True or audit.get('errors')
            or audit.get('all3_audited') is not True or audit.get('audited_scenes') != 3
            or audit.get('require_complete') is not True or audit.get('selected_scene') is not None):
        raise ValueError('Round7 execution audit did not pass')
    if audit.get('manifest_snapshot') != manifest_ref or audit.get('manifest_changed_during_audit') is True:
        raise ValueError('Audit does not bind the unchanged final manifest')
    indexed = rows(audit)
    if set(indexed) != set(cohort): raise ValueError('Audit must cover exactly all 3 retry rows')
    for sid,item in indexed.items():
        expected = cohort[sid]
        if item.get('candidate_evidence_limitations'):
            raise ValueError('Round7 cannot inherit an inapplicable compensation exception')
        if item.get('status') != 'passed': raise ValueError('Incomplete per-scene independent audit')
        for key in ('planning_status','execution_status','executed','attempts','task_success'):
            if item.get(key) != expected.get(key): raise ValueError('Audit outcome differs from final manifest: '+sid+' '+key)
        revision=validate_scene_revision(ROOT,expected['scene_revision'],expected_scene_id=sid,current=expected)
        if (item.get('scene_revision_audit',{}).get('scene_revision') != expected['scene_revision']
                or item.get('scene_revision_audit',{}).get('layout_changed') is not True):
            raise ValueError('Audit does not bind the selected new layout revision')
        if item.get('candidate_planning_result') != expected.get('candidate_planning_result'):
            raise ValueError('Audit candidate differs from final manifest: '+sid)
    return indexed


def planning_diagnostic_gate(path, plan_ref, planned, actual):
    """Bind each explicit CPU-only diagnostic method to the current failed-plan cohort."""
    failed = {sid for sid, row in actual.items() if row['execution_status'] == 'planning_failed'}
    count = len(failed)
    if not failed and not local(path).exists():
        return None, dict(method='No planning failures; no CPU diagnosis required', limitations=[]), {}
    report_ref = ref(path); report = read(path)
    if (report.get('kind') != 'physical_retry_planning_failure_cpu_diagnostics'
            or report.get('retry_round') != 7 or report.get('source_plan') != plan_ref
            or report.get('complete') is not True or report.get('source_plan_unchanged') is not True
            or report.get('total') != count or report.get('diagnosed') != count or report.get('errors')
            or report.get('physical_execution_started') is not False or report.get('kit_started') is not False
            or report.get('curobo_planning_calls') != 0 or report.get('new_target_candidates') != 0
            or report.get('cpu_ik_seed_attempts_per_scene') not in (0,1) or report.get('layout_changed') is not True):
        raise ValueError('Planning diagnostics must bind the complete current CPU-only fixed-goal cohort')
    indexed = rows(report)
    if set(indexed) != failed:
        raise ValueError('Planning diagnostics must cover exactly all current planning failures')
    bound(report['source_plan'])
    for source, digest in report.get('diagnostic_sources', {}).items():
        bound(dict(path=source, sha256=digest))
    for sid, item in indexed.items():
        row = planned[sid]
        expected = dict(scene_revision=row['scene_revision'],candidate_planning_result=row['candidate_planning_result'],
            config=dict(path=row['config_path'], sha256=row['config_sha256']),
            source_scene=dict(path=row['source_scene_path'], sha256=row['source_scene_sha256']))
        for key, binding in expected.items():
            if item.get(key) != binding:
                raise ValueError('Planning diagnostic '+key+' binding differs: '+sid)
            bound(binding)
        candidate = read(local(row['candidate_planning_result']['path']))
        if (any(item.get(k) != row[k] for k in ('scene_id','task_id','variant_id'))
                or item.get('status') != 'diagnosed' or candidate.get('status') != 'failed'
                or item.get('failed_phase') != candidate.get('failed_phase')
                or item.get('root_cause_confirmed') is not False
                or item.get('scene_infeasibility_established') is not False):
            raise ValueError('Planning diagnostic identity/uncertainty differs: '+sid)
        phase = next((p for p in candidate['phases'] if p['phase'] == item['failed_phase']), None)
        if phase is None or item.get('solver_status') != phase.get('solver_status'):
            raise ValueError('Planning diagnostic solver status differs: '+sid)
        partial = candidate.get('artifacts', {}).get('partial_trajectory')
        if item.get('partial_trajectory') != partial:
            raise ValueError('Planning diagnostic partial trajectory differs: '+sid)
        if partial is not None: bound(partial)
        facts = item.get('observed_cpu_facts_zh')
        if not isinstance(facts, list) or not facts or any(not isinstance(s, str) or not s.strip() for s in facts):
            raise ValueError('Planning diagnostic needs concrete CPU observations')
        for key in ('interpretation_zh','next_adjustment_zh'):
            if not isinstance(item.get(key), str) or not item[key].strip():
                raise ValueError('Planning diagnostic missing '+key)
        if item.get('diagnostic_method')=='independent_source_fk_segment_gate':
            raise ValueError('Round7 removed the additional straight-line gate; an old gate diagnosis cannot describe this run')
        elif item.get('diagnostic_method')=='solver_failure_without_failed_segment_path':
            segment=phase.get('segments',[phase])[-1]
            expected=dict(phase=phase['phase'],segment_index=segment.get('segment_index'),solver_status=segment['solver_status'],
                solver_success=False,samples=0,trajectory_saved=False)
            if (report['cpu_ik_seed_attempts_per_scene']!=0 or segment.get('solver_success') is not False
                    or segment.get('status')!='failed' or segment.get('samples',0)!=0 or segment.get('trajectory')
                    or item.get('failed_segment_trajectory_available') is not False
                    or item.get('failed_solver_record')!=expected):
                raise ValueError('Failed solver diagnosis must not fabricate a path or IK check')
        elif item.get('diagnostic_method')=='fixed_goal_one_seed_cpu_ik':
            if (item.get('fixed_goal_one_seed_ik', {}).get('seed_attempts') != 1
                    or type(item['fixed_goal_one_seed_ik'].get('solved')) is not bool):
                raise ValueError('Planning diagnostic is not a single fixed-goal CPU IK')
        else:
            raise ValueError('Unknown planning diagnostic method')
    return report_ref, report, indexed


def planning_diagnostic_summary(item):
    if item.get('diagnostic_method') in ('independent_source_fk_segment_gate','solver_failure_without_failed_segment_path'):
        return '；'.join(item['observed_cpu_facts_zh'])+'。'+item['interpretation_zh']
    goal = item['fixed_goal_one_seed_ik']
    prefix = '固定原目标的单个CPU IK有解' if goal.get('solved') else '单次CPU IK未得到目标解'
    target = goal.get('collision_diagnostic', {}).get('unique_world_pairs', [])
    start = item.get('failed_phase_start', {}).get('collision_diagnostic', {}).get('unique_world_pairs', [])
    pairs = target or start
    if pairs:
        location = '该单分支目标' if target else '失败阶段起点'
        detail = '；'.join(p['link']+' ↔ '+p['world_object'] for p in pairs)
        return prefix+'；'+location+'保守包络接触或1mm内近邻：'+detail+'。未证明真实物理碰撞或全局根因。'
    return prefix+'；所查端点未检出世界包络接触或1mm内近邻，连续路径与其他IK分支尚未穷尽。'


def previous_actual_review(entry):
    """Resolve the latest *executed* observation through unexecuted retries."""
    latest = entry['latest_physical_result']
    for key in ('retry_round6', 'retry_round5', 'retry_round4', 'retry_round3', 'retry_round2', 'retry', 'camera_retake', 'first_pass'):
        layer = entry.get(key)
        if layer and layer.get('result') == latest:
            return deepcopy(layer.get('diagnostic_visual_review'))
    return None


def build(manifest_path=MANIFEST, previous_path=PREVIOUS, audit_path=AUDIT,
          visual_path=VISUAL, diagnostics_root=DIAGNOSTICS, planning_diagnostics_path=PLANNING_DIAGNOSTICS):
    mp=local(manifest_path); initial=mp.read_bytes(); manifest=json.loads(initial)
    manifest_ref=dict(path=str(mp.relative_to(ROOT)),sha256=hashlib.sha256(initial).hexdigest())
    if (manifest.get('kind')!='physical_failure_retry' or manifest.get('retry_round')!=7
            or manifest.get('stage')!='complete' or manifest.get('rendering_enabled') is not False):
        raise ValueError('All round7 scenes must be terminal before publication')
    actual=rows(manifest)
    if len(actual)!=3 or any(r['execution_status'] not in TERMINAL for r in actual.values()):
        raise ValueError('Expected exactly 3 terminal round7 rows')
    previous_ref=ref(previous_path); previous=read(previous_path); prior=rows(previous)
    if (previous.get('kind')!='physical_failure_retry_result_snapshot' or previous.get('retry_round')!=6
            or previous.get('partial_preview') is not False or len(prior)!=154
            or sum(bool(r['latest_success']) for r in prior.values())!=151
            or {sid for sid,r in prior.items() if not r['latest_success']}!=set(actual)):
        raise ValueError('Previous completed original-layout snapshot must contain 151 successes and the same 3 failures')
    if manifest.get('source_previous_retry_manifest')!=previous['source_retry_manifest']:
        raise ValueError('Previous manifest differs from published original-layout history')
    bound(previous['source_retry_manifest'])
    plan_ref=dict(path=manifest['retry_plan_path'],sha256=manifest['retry_plan_sha256']); plan=read(bound(plan_ref)); planned=rows(plan)
    if (set(planned)!=set(actual) or plan.get('retry_round')!=7 or plan.get('layout_changed') is not True
            or plan.get('success_predicate_changed') is not False
            or plan.get('source_previous_retry_manifest')!=previous['source_retry_manifest']):
        raise ValueError('Round7 plan/cohort/revision contract differs')
    audit_ref=ref(audit_path);audit=read(audit_path);audited=audit_gate(audit,manifest_ref,actual)
    diagnostic_ref,diagnostic_report,diagnosed=planning_diagnostic_gate(planning_diagnostics_path,plan_ref,planned,actual)
    actual_failures={sid for sid,r in actual.items() if r['execution_status'] in ('failed','error') and r.get('executed') is True}
    visual_ref=None;reviews={}
    if actual_failures or local(visual_path).exists():
        visual_ref=ref(visual_path);visual=read(visual_path)
        if visual.get('kind')!='physical_failure_visual_review':raise ValueError('Invalid visual review kind')
        reviews=rows(visual)
        if set(reviews)!=actual_failures:raise ValueError('Every new-layout actual failure needs exactly one source-bound image review')
    entries=[];counts=Counter(r['execution_status'] for r in actual.values())
    original_keys=('config_path','config_sha256','latest_status','latest_success','latest_physical_result','latest_physical_status','annotation_zh','first_pass','camera_retake','retry','retry_round2','retry_round3','retry_round4','retry_round5','retry_round6')
    for old in previous['entries']:
        sid=old['scene_id'];entry=deepcopy(old)
        entry.update(retry_round7=None,current_revision_success=bool(old['latest_success']),current_revision_status=old['latest_status'],
            current_revision_result=deepcopy(old['latest_physical_result']),current_revision_config=dict(path=old['config_path'],sha256=old['config_sha256']),
            current_revision=None,current_revision_annotation_zh=old.get('annotation_zh',''))
        if sid not in actual:entries.append(entry);continue
        row=actual[sid];candidate=planned[sid];state=row['execution_status'];analysis=audited[sid]
        for key,value in candidate.items():
            if key!='failure_reason' and row.get(key)!=value:raise ValueError('Execution row differs from frozen plan: '+sid+' '+key)
        revision=validate_scene_revision(ROOT,row['scene_revision'],expected_scene_id=sid,current=row)
        if (revision['document']['old_config']!={'path':old['config_path'],'sha256':old['config_sha256']}
                or candidate['previous_retry_result']!=old['latest_physical_result']):
            raise ValueError('Revision historical layout or prior actual result differs')
        planning_ref=row['candidate_planning_result'];planning=read(bound(planning_ref))
        if any(planning.get(k)!=row[k] for k in ('scene_id','task_id','variant_id','config_path','config_sha256','scene_revision','layout_changed')):
            raise ValueError('Candidate planning identity/new revision differs')
        if planning.get('composition')!=revision['document']['new_composition']:
            raise ValueError('Candidate new composition differs from revision')
        if planning.get('attempts')!=1 or planning.get('executed') is not False or planning.get('planning_only') is not True:
            raise ValueError('Exactly one non-executing planning candidate required')
        recipe=planning.get('recipe',{});policy=recipe.get('planning_path_policy',{})
        if (policy.get('kind')!='collision_checked_free_path' or policy.get('previous_straight_line_gate_removed') is not True
                or not policy.get('reason_zh') or recipe.get('segment_deviation_limits_m') or recipe.get('joint_space_target_q_by_phase')
                or recipe.get('planning_options_by_phase')):
            raise ValueError('New revision must use declared default collision-checked Cartesian free-path policy')
        adjustment=row['adjustment'];bound(dict(path=adjustment['evidence_path'],sha256=adjustment['evidence_sha256']))
        retry=dict(execution_status=state,task_success=bool(row['task_success']),executed=bool(row['executed']),attempts=row['attempts'],
            adjustment=deepcopy(adjustment),candidate_planning_result=planning_ref,planning_status=row['planning_status'],candidate_count=1,
            recorded_audit=analysis,previous_result=candidate['previous_retry_result'],scene_revision=row['scene_revision'],layout_changed=True,
            config=revision['document']['new_config'],source_scene=revision['document']['new_source_scene'],
            previous_config=revision['document']['old_config'],previous_source_scene=revision['document']['old_source_scene'],
            changed_fields=revision['changed_fields'],planning_path_policy=deepcopy(policy),diagnostic_visual_review=dict(status='not_applicable'))
        entry.update(current_revision_success=False,current_revision_status='planning_failed',current_revision_result=None,
            current_revision_config=deepcopy(retry['config']),current_revision=deepcopy(row['scene_revision']))
        if state=='planning_failed':
            if (row.get('attempts')!=0 or row.get('executed') is not False or row.get('task_success') is not False
                    or row.get('planning_status')!='failed' or planning.get('status')!='failed' or row.get('execution_result_path')
                    or row.get('trajectory_path') is not None or analysis.get('executed') is not False or analysis.get('attempts')!=0):
                raise ValueError('Planning failure must consume zero physical attempts and no executable trajectory')
            reason=planning.get('failure_reason') or row.get('failure_reason')
            if not reason:raise ValueError('Planning failure reason required')
            d=diagnosed[sid]
            retry.update(physical_attempt_number=None,planned_physical_attempt_number=row['physical_attempt_number'],failure_reason=reason,
                failed_phase=planning.get('failed_phase',planning.get('stage')),planning_failure_diagnostic=dict(deepcopy(d),evidence=diagnostic_ref),
                diagnostic_visual_review=dict(status='prior_round_reference_only',reason='新布局本轮规划失败，没有新物理结果；旧图只说明旧布局历史。',
                    previous_execution_result=old['latest_physical_result'],prior_review=previous_actual_review(old)),candidate_investigation=d['next_adjustment_zh'])
            entry['current_revision_annotation_zh']='新布局规划失败、零新物理执行：'+reason+'。'+planning_diagnostic_summary(d)
        else:
            if row.get('attempts')!=1 or row.get('planning_status')!='generated' or planning.get('status')!='generated':
                raise ValueError('Executed row must consume exactly one generated candidate')
            result_ref=dict(path=row['execution_result_path'],sha256=row['execution_result_sha256']);raw=read(bound(result_ref))
            if local(result_ref['path'])!=mp.parent/'executions'/sid/'result.json':raise ValueError('Physical result outside current round scene directory')
            for key in ('scene_id','task_id','variant_id','config_path','config_sha256','execution_status','executed','task_success','attempts','physical_attempt_number','scene_revision','layout_changed'):
                if raw.get(key)!=row.get(key):raise ValueError('Actual result/manifest differ: '+sid+' '+key)
            if analysis.get('result')!=result_ref:raise ValueError('Audit binds a different actual result')
            if raw.get('executed') is True and analysis.get('no_render_gate_passed') is not True:raise ValueError('Actual result lacks audited no-render gate')
            if state in ('success','failed') and raw.get('executed') is not True:raise ValueError('Task outcome requires actual physics')
            if (state=='success')!=(row.get('task_success') is True):raise ValueError('Physical success status/Boolean differ')
            retry.update(result=result_ref,physical_attempt_number=row['physical_attempt_number'],adjusted_trajectory=dict(path=row['trajectory_path'],sha256=row['trajectory_sha256']),
                no_render_gate_passed=analysis.get('no_render_gate_passed',False),failure_reason=raw.get('failure_reason'))
            bound(retry['adjusted_trajectory'])
            if raw['executed']:entry['current_revision_result']=result_ref
            entry.update(current_revision_success=state=='success',current_revision_status='retry_round7_passed' if state=='success' else state,
                current_revision_annotation_zh='新布局第七轮原生物理任务通过；不计入原布局成功。' if state=='success' else '新布局本轮实际物理失败。' if raw['executed'] else '新布局执行初始化异常，无新物理结果。')
            if state!='success':
                if raw['executed']:
                    review,images=validate_review(reviews[sid],sid,result_ref,raw,diagnostics_root)
                    retry.update(diagnostic_visual_review=review,diagnostic_images=images,candidate_investigation=review['next_adjustment_zh'])
                else:retry.update(candidate_investigation='检查初始化错误；没有新物理状态可用于图像诊断。')
        entry['retry_round7']=retry;entry['safety_accepted']=False;entry['scene_infeasibility_established']=False
        if any(entry.get(key)!=old.get(key) for key in original_keys):raise ValueError('Historical original-layout result was modified')
        entries.append(entry)
    lookup={e['scene_id']:e for e in entries};variant_set={e['variant_id'] for e in entries};tasks=[]
    if len(previous['tasks'])!=14 or len(variant_set)!=11:raise ValueError('Expected fourteen tasks by eleven variants')
    for oldtask in previous['tasks']:
        group=[lookup[sid] for sid in oldtask['scene_ids']]
        if len(group)!=11 or {r['variant_id'] for r in group}!=variant_set:raise ValueError('Incomplete task matrix')
        task=deepcopy(oldtask);task.update(original_layout_success=sum(e['latest_success'] for e in group),round7_success=sum(bool(e['retry_round7'] and e['retry_round7']['task_success']) for e in group),
            current_revision_success=sum(e['current_revision_success'] for e in group),current_revision_remaining_failed=sum(not e['current_revision_success'] for e in group));tasks.append(task)
    if (mp.read_bytes()!=initial or ref(previous_path)!=previous_ref or ref(audit_path)!=audit_ref
            or (diagnostic_ref and ref(planning_diagnostics_path)!=diagnostic_ref) or (visual_ref and ref(visual_path)!=visual_ref)):
        raise ValueError('Publication source changed while building')
    original_success=sum(e['latest_success'] for e in entries);current_success=sum(e['current_revision_success'] for e in entries)
    if original_success!=151 or current_success!=151+counts['success']:raise ValueError('Original/new revision success counts mixed')
    return dict(schema_version=7,kind='physical_failure_retry_result_snapshot',retry_round=7,created_at=datetime.now(timezone.utc).isoformat(),partial_preview=False,markdown_report_path=DOC_PATH,
        scope='三个剩余场景各冻结一个新增障碍位姿版本，原布局历史151/154保持不变；当前采用版本另计成绩。',
        source_previous_snapshot=previous_ref,source_retry_manifest=manifest_ref,source_retry_plan=plan_ref,source_execution_audit=audit_ref,
        source_planning_failure_diagnostics=diagnostic_ref,source_first_pass_manifest=plan['source_first_pass_manifest'],source_camera_retake_manifest=plan['source_camera_retake_manifest'],
        original_layout_counts=dict(total=154,success=151,failed=3,unchanged=True),current_revision_counts=dict(total=154,success=current_success,failed=154-current_success,revised_scenes=3),
        counts=dict(previous_success=151,previous_failed=3,retry_cohort=3,retry_completed=3,retry_outcomes=dict(counts),new_successes=counts['success'],
            original_layout_success=151,original_layout_failed=3,latest_success=151,latest_failed=3,current_revision_success=current_success,current_revision_failed=154-current_success,
            planning_failed=counts['planning_failed'],physical_failed=counts['failed'],execution_errors=counts['error'],actual_failed_executions=len(actual_failures),physical_attempts=sum(r['attempts'] for r in actual.values()),remaining_pending=0),
        planning_failure_diagnostics=dict(diagnosed=len(diagnosed),solver_failure_without_path_diagnosed=sum(d.get('diagnostic_method')=='solver_failure_without_failed_segment_path' for d in diagnosed.values()),
            fixed_goal_cpu_ik_checked=sum(d.get('diagnostic_method')=='fixed_goal_one_seed_cpu_ik' for d in diagnosed.values()),method=diagnostic_report['method'],limitations=diagnostic_report['limitations']),
        layout_changed=True,success_predicate_changed=False,safety_accepted=False,safety_certified=False,long_term_stability_verified=False,
        candidate_evidence_limitations=deepcopy(audit.get('candidate_evidence_limitations',[])),all3_fully_verified_candidates=audit.get('all3_fully_verified_candidates',False),
        startup_failure_reviews=deepcopy(audit.get('startup_failure_reviews',[])),execution_settings=plan['execution_settings'],
        planning_method=dict(api='MotionGen.plan_single',path_policy='collision_checked_free_path',previous_straight_line_gate_removed=True,default_constraint_weight=1,enable_graph=True),
        rendering=dict(execution='No-render native PhysX; no new demo videos',later_failure_images='Measured-state reconstruction of the new selected layout, zero new physics; prior camera used only as a viewpoint'),
        diagnostic_visual_review=dict(status='complete',conclusions_written=True,reviewed_scenes=len(actual_failures),source_visual_review=visual_ref,review_required=bool(actual_failures),
            planning_failures_have_no_new_physical_images=True,completion_reason='All actual failure scenes have source-bound image reviews' if actual_failures else 'No actual failed executions; no image review required'),tasks=tasks,entries=entries)


def escape(value):return str(value).replace('|','／').replace('\n',' ')
def link(path):return '../../../'+str(path)


def document(data):
    c=data['counts'];lookup={e['scene_id']:e for e in data['entries']};variants=[]
    for e in data['entries']:
        if e['variant_id'] not in [v[0] for v in variants]:variants.append((e['variant_id'],e['variant_name_zh']))
    symbols={'first_pass_passed':'初✓','retry_passed':'一✓','retry_round2_passed':'二✓','retry_round3_passed':'三✓','retry_round4_passed':'四✓','retry_round5_passed':'五✓','retry_round6_passed':'六✓','retry_round7_passed':'新版七✓','planning_failed':'新版规划失败','failed':'新版失败','error':'新版异常','execution_error':'异常'}
    lines=['# Replicator 154 景：第七轮新增障碍位姿版本','',
        f"**原布局成绩保持151/154；本轮新版本通过{c['new_successes']}/3，物理失败{c['physical_failed']}景，规划失败{c['planning_failed']}景。当前采用版本{c['current_revision_success']}/154通过，剩余{c['current_revision_failed']}景。**",'',
        f"[机器可读状态与SHA]({link(JSON_PATH)}) · [保留的第六轮原布局记录](PHYSICAL_RETRY_ROUND6_20261029.md) · [冻结新版本计划]({link(data['source_retry_plan']['path'])})",'',
        '每个新版本只修改新增障碍物的位置或朝向。原厨房、机器人、任务物品、资产内容、物理参数和任务成功判据保持不变。旧配置、旧结果及 retry 至 retry_round6 均原样保留；新版通过不能表述为旧版通过。', '',
        f"每景一个冻结候选，规划通过者至多一次无画面原生PhysX，共{c['physical_attempts']}次。{c['planning_failed']}景规划失败没有新增物理尝试。",'',
        '本轮使用默认笛卡尔 `MotionGen.plan_single` 并保留碰撞检查，允许规划器绕行；取消先前额外的5毫米直线路径门槛。该门槛不是原任务谓词，移除后仍按原生物理记录的原任务判据验收。CPU理想采样不代表实际规划曲线或真实避障。', '',
        '任务通过不等于安全验收、避障验收或长期稳定通过；有限次失败不证明场景无解。本轮没有实际失败，因此未生成新失败诊断图；历史相机记录仅提供视角，不能代替新版物理证据。','',
        '## 14 × 11 当前采用版本矩阵','',
        '初✓及一✓至六✓沿用原布局历史；新版七✓仅表示新布局本轮通过。旧布局矩阵与成绩保留在第六轮报告。','',
        '| 原始任务 | '+' | '.join(escape(v[1]) for v in variants)+' | 原布局通过 | 当前版本通过 |',
        '| --- | '+' | '.join('---' for _ in variants)+' | --- | --- |']
    for t in data['tasks']:
        group={lookup[s]['variant_id']:lookup[s] for s in t['scene_ids']}
        cells=[symbols.get(group[v]['current_revision_status'],group[v]['current_revision_status']) for v,_ in variants]
        lines.append('| '+escape(t['task_name_zh'])+' | '+' | '.join(cells)+f" | {t['original_layout_success']}/11 | {t['current_revision_success']}/11 |")
    lines+=['','## 三个新版本的调整与结果','','| 场景 ID | 冻结调整 | 新配置 | 规划 / 物理次数 | 新版结果 | 后续建议 |','| --- | --- | --- | --- | --- | --- |']
    for e in data['entries']:
        r=e['retry_round7']
        if not r:continue
        lines.append('| `'+e['scene_id']+'` | '+escape(r['adjustment']['description'])+' | [新布局]('+link(r['config']['path'])+') | '+escape(r['planning_status'])+f" / {r['attempts']}次 | "+escape(e['current_revision_annotation_zh'])+' | '+escape(r.get('candidate_investigation','—'))+' |')
    lines+=['','## 本轮规划失败的只读诊断','','这些诊断没有追加cuRobo或物理尝试。来源绑定本轮新配置、场景版本和候选；一次IK分支检查或求解失败不能证明全局无解。','','| 场景 ID | 阶段 / 状态 | 可核实事实 | 推断 | 下一步建议 |','| --- | --- | --- | --- | --- |']
    for e in data['entries']:
        r=e['retry_round7']
        if not r or r['execution_status']!='planning_failed':continue
        d=r['planning_failure_diagnostic'];lines.append('| `'+e['scene_id']+'` | '+escape(d['failed_phase']+' / '+d['solver_status'])+' | '+escape('；'.join(d['observed_cpu_facts_zh']))+' | '+escape(d['interpretation_zh'])+' | '+escape(d['next_adjustment_zh'])+' |')
    lines+=['','## 本轮实际失败的图像复核','']
    if not data['diagnostic_visual_review']['review_required']:
        lines += ['本轮实际执行失败为0景，无需新失败诊断图或图像复核；规划失败不是物理成功。','']
    else:
        lines+=['| 场景 ID | 直接可见事实 | 推断 | 下一次调整建议 | 绑定证据 |','| --- | --- | --- | --- | --- |']
        for e in data['entries']:
            r=e['retry_round7']
            if not r or r['diagnostic_visual_review'].get('status')!='complete':continue
            v=r['diagnostic_visual_review'];lines.append('| `'+e['scene_id']+'` | '+escape('；'.join(v['observed_visual_facts_zh']))+' | '+escape(v['interpretation_zh'])+' | '+escape(v['next_adjustment_zh'])+' | 结果SHA `'+v['execution_result_sha256'][:12]+'`；报告SHA `'+v['diagnostic_report_sha256'][:12]+'` |')
    lines+=['','全部原布局记录、新版记录、源文件SHA、只读审计和诊断路径写入状态JSON；大体积本地证据需单独恢复。本报告不覆盖前六轮报告。','']
    return '\n'.join(lines)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',default=MANIFEST);p.add_argument('--previous',default=PREVIOUS);p.add_argument('--audit',default=AUDIT)
    p.add_argument('--visual-review',default=VISUAL);p.add_argument('--diagnostics-root',default=DIAGNOSTICS)
    p.add_argument('--planning-diagnostics',default=PLANNING_DIAGNOSTICS);p.add_argument('--dry-run',action='store_true');a=p.parse_args()
    data=build(a.manifest,a.previous,a.audit,a.visual_review,a.diagnostics_root,a.planning_diagnostics);markdown=document(data)
    if a.dry_run:
        print(json.dumps(dict(dry_run=True,publication_files_written=False,entries=len(data['entries']),counts=data['counts'],original_layout_counts=data['original_layout_counts'],current_revision_counts=data['current_revision_counts']),ensure_ascii=False));return
    jp,dp=local(JSON_PATH),local(DOC_PATH)
    if jp.exists() or dp.exists():raise FileExistsError('Existing publication retained')
    jp.parent.mkdir(parents=True,exist_ok=True);dp.parent.mkdir(parents=True,exist_ok=True)
    with jp.open('x') as f:json.dump(data,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
    with dp.open('x') as f:f.write(markdown)
    print(json.dumps(dict(json=ref(jp),markdown=ref(dp),counts=data['counts']),ensure_ascii=False))

if __name__=='__main__':main()
