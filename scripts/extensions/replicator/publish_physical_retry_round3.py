#!/usr/bin/env python3
"""Publish round3 only after terminal execution, audit and bound image review.

The prior 154-row publication and every first-pass/round1/round2 result are retained.
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
RUN = 'replicator_physical_retry_round3_20261029'
MANIFEST = f'exports/{RUN}/manifest.json'
PREVIOUS = 'configs/extensions/replicator/physical_retry_results_round2_20261028.json'
AUDIT = f'reports/{RUN}/audit_execution.json'
VISUAL = f'reports/{RUN}/failure_visual_review.json'
PLANNING_DIAGNOSTICS = f'reports/{RUN}/planning_failure_diagnostics.json'
DIAGNOSTICS = 'reports/replicator_physical_retry_round3_diagnostics_20261029'
JSON_PATH = 'configs/extensions/replicator/physical_retry_results_round3_20261029.json'
DOC_PATH = 'docs/extensions/replicator/PHYSICAL_RETRY_ROUND3_20261029.md'
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
        source_paths_sha256=planned['composition']['source_paths_sha256'])
    for prior in reversed(raw['prior_physical_attempts']):
        prior_ref = dict(path=prior['result_path'], sha256=prior['result_sha256'])
        prior_path = bound(prior_ref); observation = read(prior_path)
        if observation.get('scene_id') != raw['scene_id'] or observation.get('config_sha256') != raw['config_sha256']:
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
    if audit.get('passed') is not True or audit.get('errors'):
        raise ValueError('Round3 execution audit did not pass')
    if audit.get('manifest_snapshot') != manifest_ref or audit.get('manifest_changed_during_audit') is True:
        raise ValueError('Audit does not bind the unchanged final manifest')
    indexed = rows(audit)
    if set(indexed) != set(cohort): raise ValueError('Audit must cover exactly all 29 retry rows')
    return indexed


def planning_diagnostic_gate(path, plan_ref, planned, actual):
    """Bind the fixed-goal single-IK diagnostic to the current failed-plan cohort."""
    failed = {sid for sid, row in actual.items() if row['execution_status'] == 'planning_failed'}
    count = len(failed)
    if not failed and not local(path).exists():
        return None, dict(method='No planning failures; no CPU diagnosis required', limitations=[]), {}
    report_ref = ref(path); report = read(path)
    if (report.get('kind') != 'physical_retry_planning_failure_cpu_diagnostics'
            or report.get('retry_round') != 3 or report.get('source_plan') != plan_ref
            or report.get('complete') is not True or report.get('source_plan_unchanged') is not True
            or report.get('total') != count or report.get('diagnosed') != count or report.get('errors')
            or report.get('physical_execution_started') is not False or report.get('kit_started') is not False
            or report.get('curobo_planning_calls') != 0 or report.get('new_target_candidates') != 0
            or report.get('cpu_ik_seed_attempts_per_scene') != 1 or report.get('layout_changed') is not False):
        raise ValueError('Planning diagnostics must bind the complete current CPU-only fixed-goal cohort')
    indexed = rows(report)
    if set(indexed) != failed:
        raise ValueError('Planning diagnostics must cover exactly all current planning failures')
    bound(report['source_plan'])
    for source, digest in report.get('diagnostic_sources', {}).items():
        bound(dict(path=source, sha256=digest))
    for sid, item in indexed.items():
        row = planned[sid]
        expected = dict(candidate_planning_result=row['candidate_planning_result'],
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
        if item.get('fixed_goal_one_seed_ik', {}).get('seed_attempts') != 1:
            raise ValueError('Planning diagnostic is not a single fixed-goal CPU IK')
    return report_ref, report, indexed


def planning_diagnostic_summary(item):
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
    for key in ('retry_round2', 'retry', 'camera_retake', 'first_pass'):
        layer = entry.get(key)
        if layer and layer.get('result') == latest:
            return deepcopy(layer.get('diagnostic_visual_review'))
    return None


def build(manifest_path=MANIFEST, previous_path=PREVIOUS, audit_path=AUDIT,
          visual_path=VISUAL, diagnostics_root=DIAGNOSTICS, planning_diagnostics_path=PLANNING_DIAGNOSTICS):
    mp = local(manifest_path); snapshot = mp.read_bytes(); manifest = json.loads(snapshot)
    manifest_ref = dict(path=str(mp.relative_to(ROOT)), sha256=hashlib.sha256(snapshot).hexdigest())
    if (manifest.get('kind') != 'physical_failure_retry' or manifest.get('retry_round') != 3
            or manifest.get('stage') != 'complete' or manifest.get('rendering_enabled') is not False):
        raise ValueError('All round3 scenes must be terminal before publication')
    actual = rows(manifest)
    if len(actual) != 29 or any(r['execution_status'] not in TERMINAL for r in actual.values()):
        raise ValueError('Expected exactly 29 terminal round3 rows')
    previous_ref = ref(previous_path); previous = read(previous_path); prior = rows(previous)
    if (previous.get('kind') != 'physical_failure_retry_result_snapshot' or previous.get('retry_round') != 2
            or previous.get('partial_preview') is not False or len(prior) != 154
            or sum(bool(r['latest_success']) for r in prior.values()) != 125
            or {sid for sid,r in prior.items() if not r['latest_success']} != set(actual)):
        raise ValueError('Previous completed snapshot must contain 125 success and the same 29 failures')
    if manifest.get('source_previous_retry_manifest') != previous['source_retry_manifest']:
        raise ValueError('Round2 differs from the previous published physical run')
    bound(previous['source_retry_manifest'])
    plan_ref = dict(path=manifest['retry_plan_path'], sha256=manifest['retry_plan_sha256'])
    plan = read(bound(plan_ref)); planned = rows(plan)
    if (set(planned) != set(actual) or plan.get('retry_round') != 3 or plan.get('layout_changed') is not False
            or plan.get('success_predicate_changed') is not False
            or plan.get('source_previous_retry_manifest') != previous['source_retry_manifest']):
        raise ValueError('Round2 plan/cohort/layout contract differs')
    audit_ref = ref(audit_path); audit = read(audit_path); audited = audit_gate(audit, manifest_ref, actual)
    planning_diagnostics_ref, planning_diagnostics, planning_diagnosed = planning_diagnostic_gate(
        planning_diagnostics_path, plan_ref, planned, actual)
    actual_failures = {sid for sid,r in actual.items() if r['execution_status'] in ('failed','error') and r.get('executed') is True}
    visual_ref = None; reviews = {}
    # An empty cohort needs no renderer/review artifact. If an optional review
    # file exists, still validate that it is empty instead of accepting stale
    # observations from a different run as this run's evidence.
    if actual_failures or local(visual_path).exists():
        visual_ref = ref(visual_path); visual = read(visual_path)
        if visual.get('kind') != 'physical_failure_visual_review': raise ValueError('Invalid visual review kind')
        reviews = rows(visual)
        if set(reviews) != actual_failures: raise ValueError('Every actual failed execution needs a new bound image review, and no other scene may be labeled as one')
    entries = []; counts = Counter(r['execution_status'] for r in actual.values())
    for old in previous['entries']:
        entry = deepcopy(old); sid = old['scene_id']; entry['retry_round3'] = None
        entry['latest_physical_result'] = deepcopy(old['latest_physical_result'])
        entry['latest_physical_status'] = old['latest_physical_status']
        if sid not in actual:
            entries.append(entry); continue
        row = actual[sid]; candidate = planned[sid]; state = row['execution_status']; analysis = audited[sid]
        for key, value in candidate.items():
            # The runner replaces a generated plan's null failure_reason with
            # the actual outcome; all immutable plan inputs still must match.
            if key != 'failure_reason' and row.get(key) != value:
                raise ValueError('Execution row differs from frozen plan: '+sid+' '+key)
        if (old['config_path'] != row['config_path'] or old['config_sha256'] != row['config_sha256']
                or candidate['previous_retry_result'] != old['latest_physical_result']):
            raise ValueError('Scene layout or previous actual result changed: '+sid)
        bound(dict(path=row['config_path'], sha256=row['config_sha256']))
        planning_ref = row['candidate_planning_result']; planning = read(bound(planning_ref))
        if any(planning.get(k) != row[k] for k in ('scene_id','task_id','variant_id','config_path','config_sha256')):
            raise ValueError('Candidate planning identity differs')
        if planning.get('attempts') != 1 or planning.get('executed') is not False or planning.get('planning_only') is not True:
            raise ValueError('Exactly one non-executing planning candidate required')
        adjustment = row['adjustment']; bound(dict(path=adjustment['evidence_path'], sha256=adjustment['evidence_sha256']))
        retry = dict(execution_status=state, task_success=bool(row['task_success']), executed=bool(row['executed']),
            attempts=row['attempts'], adjustment=deepcopy(adjustment), candidate_planning_result=planning_ref,
            planning_status=row['planning_status'], candidate_count=1, recorded_audit=analysis,
            previous_result=candidate['previous_retry_result'], diagnostic_visual_review=dict(status='not_applicable'))
        if state == 'planning_failed':
            if (row.get('attempts') != 0 or row.get('executed') is not False or row.get('task_success') is not False
                    or row.get('planning_status') != 'failed' or planning.get('status') != 'failed'
                    or row.get('execution_result_path') or row.get('trajectory_path') is not None
                    or analysis.get('executed') is not False or analysis.get('attempts') != 0):
                raise ValueError('Planning failure must consume zero physical attempts and no executable trajectory')
            reason = planning.get('failure_reason') or row.get('failure_reason')
            if not reason: raise ValueError('Planning failure reason required')
            diagnostic = planning_diagnosed[sid]
            retry.update(physical_attempt_number=None, planned_physical_attempt_number=row['physical_attempt_number'],
                failure_reason=reason, failed_phase=planning.get('failed_phase',planning.get('stage')),
                planning_failure_diagnostic=dict(deepcopy(diagnostic), evidence=planning_diagnostics_ref),
                diagnostic_visual_review=dict(status='prior_round_reference_only',
                    reason='本轮规划未通过，未执行物理，旧图只解释上一轮实际失败。',
                    previous_execution_result=old['latest_physical_result'],
                    prior_review=previous_actual_review(old)),
                candidate_investigation=diagnostic['next_adjustment_zh'])
            entry.update(latest_status='planning_failed', latest_success=False,
                annotation_zh='本轮规划失败，零次新物理执行：'+reason+'。'+planning_diagnostic_summary(diagnostic))
        else:
            if row.get('attempts') != 1 or row.get('planning_status') != 'generated' or planning.get('status') != 'generated':
                raise ValueError('Executed row must consume exactly one generated candidate')
            result_ref = dict(path=row['execution_result_path'], sha256=row['execution_result_sha256']); raw = read(bound(result_ref))
            if local(result_ref['path']) != mp.parent/'executions'/sid/'result.json': raise ValueError('Physical result outside the current round scene directory')
            for key in ('scene_id','task_id','variant_id','config_path','config_sha256','execution_status','executed','task_success','attempts','physical_attempt_number'):
                if raw.get(key) != row.get(key): raise ValueError('Actual result/manifest differ: '+sid+' '+key)
            if analysis.get('result') != result_ref: raise ValueError('Audit binds a different physical result')
            if raw.get('executed') is True and analysis.get('no_render_gate_passed') is not True:
                raise ValueError('Actual execution lacks an audited no-render gate')
            if state in ('success','failed') and raw.get('executed') is not True:
                raise ValueError('A physical task outcome must be actually executed')
            if (state == 'success') != (row.get('task_success') is True): raise ValueError('Physical success status/Boolean differ')
            retry.update(result=result_ref, physical_attempt_number=row['physical_attempt_number'],
                adjusted_trajectory=dict(path=row['trajectory_path'],sha256=row['trajectory_sha256']),
                no_render_gate_passed=analysis.get('no_render_gate_passed',False),failure_reason=raw.get('failure_reason'))
            bound(retry['adjusted_trajectory'])
            if raw['executed']:
                entry['latest_physical_result'] = result_ref
                entry['latest_physical_status'] = state
            if state == 'success':
                entry.update(latest_status='retry_round3_passed',latest_success=True,annotation_zh='第三轮真实物理执行通过；使用原任务成功及最终放置判据。')
            else:
                entry.update(latest_status='failed' if state == 'failed' else 'execution_error',latest_success=False,
                    annotation_zh='第三轮实际物理失败。' if raw['executed'] else '本轮执行初始化异常，未取得新物理抓放结果。')
                if raw['executed']:
                    review, images = validate_review(reviews[sid],sid,result_ref,raw,diagnostics_root)
                    retry.update(diagnostic_visual_review=review,diagnostic_images=images,
                        candidate_investigation=review['next_adjustment_zh'])
                else:
                    retry['candidate_investigation']='先检查初始化错误与执行日志；没有本轮实际状态可供图像诊断。'
                    retry['diagnostic_visual_review']=dict(status='not_applicable',reason='No new physical episode was executed')
        entry['retry_round3']=retry; entry['safety_accepted']=False; entry['scene_infeasibility_established']=False
        if any(entry.get(k) != old.get(k) for k in ('first_pass','camera_retake','retry','retry_round2')):
            raise ValueError('Historical physical record was modified')
        entries.append(entry)
    lookup = {e['scene_id']:e for e in entries}; tasks = []
    variant_set = {e['variant_id'] for e in entries}
    if len(previous['tasks']) != 14 or len(variant_set) != 11: raise ValueError('Expected fourteen tasks by eleven variants')
    for oldtask in previous['tasks']:
        group = [lookup[sid] for sid in oldtask['scene_ids']]
        if len(group) != 11 or {r['variant_id'] for r in group} != variant_set: raise ValueError('Incomplete task matrix')
        tasks.append(dict(task_id=oldtask['task_id'],task_name_zh=oldtask['task_name_zh'],total=11,scene_ids=oldtask['scene_ids'],
            previous_success=oldtask['latest_success'],round3_success=sum(e['latest_status']=='retry_round3_passed' for e in group),
            latest_success=sum(e['latest_success'] for e in group),remaining_failed=sum(not e['latest_success'] for e in group)))
    if (mp.read_bytes() != snapshot or ref(previous_path) != previous_ref or ref(audit_path) != audit_ref
            or (planning_diagnostics_ref is not None and ref(planning_diagnostics_path) != planning_diagnostics_ref)
            or (visual_ref is not None and ref(visual_path) != visual_ref)):
        raise ValueError('Publication source changed while building')
    latest = sum(e['latest_success'] for e in entries)
    if latest != 125+counts['success']: raise ValueError('Latest success must equal 125 plus new round3 successes')
    return dict(schema_version=3,kind='physical_failure_retry_result_snapshot',retry_round=3,
        created_at=datetime.now(timezone.utc).isoformat(),partial_preview=False,markdown_report_path=DOC_PATH,
        scope='保留首轮、第一轮和第二轮重试154景记录；仅29个剩余失败各冻结一个调整候选，规划通过者各执行一次无画面原生PhysX。',
        source_previous_snapshot=previous_ref,source_retry_manifest=manifest_ref,source_retry_plan=plan_ref,
        source_planning_failure_diagnostics=planning_diagnostics_ref,
        planning_failure_diagnostics=dict(diagnosed=len(planning_diagnosed),
            fixed_goal_cpu_ik_solved=sum(e['fixed_goal_one_seed_ik'].get('solved') is True for e in planning_diagnosed.values()),
            goal_branch_world_contact_or_nearby=sum(bool(e['fixed_goal_one_seed_ik'].get('collision_diagnostic',{}).get('unique_world_pairs')) for e in planning_diagnosed.values()),
            start_world_contact_or_nearby=sum(bool(e.get('failed_phase_start',{}).get('collision_diagnostic',{}).get('unique_world_pairs')) for e in planning_diagnosed.values()),
            method=planning_diagnostics['method'],limitations=planning_diagnostics['limitations'],
            root_cause_confirmed=False,scene_infeasibility_established=False),
        source_execution_audit=audit_ref,source_first_pass_manifest=plan['source_first_pass_manifest'],
        source_camera_retake_manifest=plan['source_camera_retake_manifest'],
        counts=dict(previous_success=125,previous_failed=29,retry_cohort=29,retry_completed=29,retry_outcomes=dict(counts),
            new_successes=counts['success'],latest_success=latest,latest_failed=154-latest,
            planning_failed=counts['planning_failed'],physical_failed=counts['failed'],execution_errors=counts['error'],
            actual_failed_executions=len(actual_failures),
            physical_attempts=sum(r['attempts'] for r in actual.values()),remaining_pending=0),
        layout_changed=False,success_predicate_changed=False,grasp_or_nominal_gripper_changed=True,
        safety_accepted=False,safety_certified=False,long_term_stability_verified=False,
        execution_settings=plan['execution_settings'],
        rendering=dict(execution='No-render PhysX; no execution cameras or new demo videos',
            later_failure_images='Measured-state diagnostic reconstruction, zero new physics; not execution-camera frames'),
        diagnostic_visual_review=dict(status='complete',conclusions_written=True,reviewed_scenes=len(actual_failures),
            remaining_actual_failure_scenes=len(actual_failures),source_visual_review=visual_ref,
            image_sets_bound=len(actual_failures),planning_failures_have_no_new_physical_images=True,
            review_required=bool(actual_failures),
            completion_reason='All actual failure scenes have source-bound image reviews' if actual_failures else 'No actual failed executions; no image review or diagnostic rendering required'),
        tasks=tasks,entries=entries)


def escape(value): return str(value).replace('|','／').replace('\n',' ')
def link(path): return '../../../'+str(path)


def document(data):
    c=data['counts']; lookup={e['scene_id']:e for e in data['entries']}
    variants=[]
    for e in data['entries']:
        if e['variant_id'] not in [x[0] for x in variants]: variants.append((e['variant_id'],e['variant_name_zh']))
    symbol={'first_pass_passed':'初✓','retry_passed':'一✓','retry_round2_passed':'二✓','retry_round3_passed':'三✓','failed':'失败','planning_failed':'规划失败','execution_error':'异常'}
    image_note=('实际失败后的图像由本轮记录的实际物理状态重建，未重新运行物理；它们不是执行相机视频。'
        if data['diagnostic_visual_review']['review_required'] else '本轮没有实际执行失败，无需生成失败诊断图或进行图像复核；规划失败仍沿用上一轮实际物理失败记录。')
    lines=['# Replicator 154 景：第三轮物理重试结果','',
        f"**此前125/154通过。本轮29个候选：新增通过{c['new_successes']}景，物理失败{c['physical_failed']}景，规划失败{c['planning_failed']}景，执行异常{c['execution_errors']}景。最新{c['latest_success']}/154通过，剩余{c['latest_failed']}景。**",'',
        f"[机器可读状态与SHA]({link(JSON_PATH)}) · [保留的第二轮重试结果](PHYSICAL_RETRY_ROUND2_20261028.md) · [冻结调整计划]({link(data['source_retry_plan']['path'])})",'',
        f"本轮只调整抓点、抓姿、夹爪指令或轨迹目标，场景布局及原任务判据不变。每景仅冻结一个候选；规划失败的{c['planning_failed']}景没有新物理执行。其余候选各消耗一次物理尝试，共{c['physical_attempts']}次。旧成功、旧失败及retry、retry_round2记录均保留。",'',
        '实际执行关闭渲染、相机和视频。'+image_note+' 任务通过不等于避障、安全验收或长期稳定通过；有限次失败也不证明场景无解。','',
        '## 14 × 11 最新状态矩阵','',
        '初✓＝首轮通过；一✓＝第一轮重试通过；二✓＝第二轮重试通过；三✓＝本轮通过。规划失败＝本轮没有新物理测试。','',
        '| 原始任务 | '+' | '.join(escape(name) for _,name in variants)+' | 通过 |',
        '| --- | '+' | '.join('---' for _ in variants)+' | --- |']
    for task in data['tasks']:
        byvariant={lookup[s]['variant_id']:lookup[s] for s in task['scene_ids']}
        lines.append('| '+escape(task['task_name_zh'])+' | '+' | '.join(symbol.get(byvariant[v]['latest_status'],byvariant[v]['latest_status']) for v,_ in variants)+f" | {task['latest_success']}/11 |")
    lines += ['','## 本轮29景：调整、结果和后续建议','',
        '| 场景 ID | 本轮调整 | 规划 / 新物理尝试 | 结果 | 失败后候选建议 |',
        '| --- | --- | --- | --- | --- |']
    for e in data['entries']:
        r=e['retry_round3']
        if r is None:continue
        result='本轮通过' if e['latest_success'] else e['annotation_zh']
        lines.append('| `'+e['scene_id']+'` | '+escape(r['adjustment']['description'])+' | '+escape(r['planning_status'])+f" / {r['attempts']}次 | "+escape(result)+' | '+escape(r.get('candidate_investigation','—'))+' |')
    diagnostics=data['planning_failure_diagnostics']
    lines += ['',f"## {c['planning_failed']}景规划失败的只读CPU诊断",'',
        (f"固定每景原失败目标，仅检查一个CPU IK分支：{diagnostics['fixed_goal_cpu_ik_solved']}/{c['planning_failed']}获得目标解，{diagnostics['goal_branch_world_contact_or_nearby']}景该目标分支存在保守包络接触或1mm内近邻。规划阶段起点单独检查，不能将起点冲突误写成目标不可达。" if c['planning_failed'] else '本轮没有规划失败，无需额外CPU失败诊断。'),'',
        '这是原机器人球包络、5mm缓冲与源碰撞几何的端点诊断；没有新增cuRobo调用、抓姿候选或物理测试。保守包络接触不证明真实网格碰撞；单分支检查不证明根因确认或场景无解。','',
        f"[状态JSON内嵌的完整规划诊断与候选结果SHA]({link(JSON_PATH)})。原始本地诊断报告路径和SHA同时保留在JSON中。",'',
        '| 场景 ID | 失败阶段 / 原求解器状态 | CPU可核实事实 | 保守推断 | 下一次调整建议（尚未实施） |',
        '| --- | --- | --- | --- | --- |']
    for e in data['entries']:
        r=e['retry_round3']
        if not r or r['execution_status']!='planning_failed':continue
        d=r['planning_failure_diagnostic']
        lines.append('| `'+e['scene_id']+'` | '+escape(d['failed_phase']+' / '+d['solver_status'])+' | '+escape('；'.join(d['observed_cpu_facts_zh']))+' | '+escape(d['interpretation_zh'])+' | '+escape(d['next_adjustment_zh'])+' |')
    lines += ['','## 本轮实际失败的图像复核','']
    if data['diagnostic_visual_review']['review_required']:
        lines += ['可见事实与推断分开记录。规划失败只引用上一轮图文作为历史背景，不能把旧图称为本轮执行证据。','',
            '| 场景 ID | 图中可见事实 | 推断 | 下一步建议（本轮未再次实施） | 证据 |',
            '| --- | --- | --- | --- | --- |']
    else:
        lines += [f"本轮实际执行失败为0景，因此没有本轮失败诊断图或逐图复核结论。{c['planning_failed']}景规划失败没有新增物理测试，不能记为物理成功。",'']
    for e in data['entries']:
        r=e['retry_round3']
        if not r or r['diagnostic_visual_review'].get('status')!='complete':continue
        v=r['diagnostic_visual_review'];evidence=f"报告SHA `{v['diagnostic_report_sha256'][:12]}`；结果SHA `{v['execution_result_sha256'][:12]}`"
        lines.append('| `'+e['scene_id']+'` | '+escape('；'.join(v['observed_visual_facts_zh']))+' | '+escape(v['interpretation_zh'])+' | '+escape(v['next_adjustment_zh'])+' | '+evidence+' |')
    lines += ['','完整物理记录、诊断图、审计与来源SHA列在状态JSON中；大体积本地证据需单独恢复。首轮、第一轮和第二轮重试文档保持原样。','']
    return '\n'.join(lines)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',default=MANIFEST);p.add_argument('--previous',default=PREVIOUS)
    p.add_argument('--audit',default=AUDIT);p.add_argument('--visual-review',default=VISUAL)
    p.add_argument('--diagnostics-root',default=DIAGNOSTICS);p.add_argument('--dry-run',action='store_true')
    p.add_argument('--planning-diagnostics',default=PLANNING_DIAGNOSTICS)
    args=p.parse_args()
    data=build(args.manifest,args.previous,args.audit,args.visual_review,args.diagnostics_root,args.planning_diagnostics);markdown=document(data)
    if args.dry_run:
        print(json.dumps(dict(dry_run=True,publication_files_written=False,entries=len(data['entries']),tasks=len(data['tasks']),counts=data['counts'],markdown_characters=len(markdown)),ensure_ascii=False));return
    jp,dp=local(JSON_PATH),local(DOC_PATH)
    if jp.exists() or dp.exists():raise FileExistsError('Existing publication retained')
    jp.parent.mkdir(parents=True,exist_ok=True);dp.parent.mkdir(parents=True,exist_ok=True)
    with jp.open('x') as stream:json.dump(data,stream,ensure_ascii=False,indent=2,allow_nan=False);stream.write('\n')
    with dp.open('x') as stream:stream.write(markdown)
    print(json.dumps(dict(json=ref(jp),markdown=ref(dp),counts=data['counts']),ensure_ascii=False))


if __name__=='__main__':main()
