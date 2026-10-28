"""One physical attempt per reviewed scene, with matched baselines and web progress."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from urllib.request import ProxyHandler, Request, build_opener

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from kitchen_safety.extensions.controls.execution_contract import (
    controller_digest, controller_signature, frozen_task_differences)

API = 'http://127.0.0.1:8765/api/scenes'
DATA = ROOT / 'datasets/controls_completion_20261027'
REPORTS = ROOT / 'reports/controls_completion_20261027'


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
    temporary.replace(path)


def runtime_argument(value):
    path = Path(str(value))
    if path.is_absolute():
        return path.resolve().relative_to(ROOT).as_posix()
    return str(value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--round', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--episodes', type=Path, required=True)
    parser.add_argument('--left-baseline','--baseline',dest='left_baseline',type=Path,
                        help='Previously recorded physical baseline; exact controller signature must match')
    parser.add_argument('--baseline-progress', type=Path, nargs='*', default=[])
    parser.add_argument('--config-overrides', type=Path)
    parser.add_argument('--held-gripper-scenes', nargs='*', default=[])
    parser.add_argument('--balanced-lamp-scenes', nargs='*', default=[])
    parser.add_argument('--scenes', nargs='+')
    parser.add_argument('--max-steps',type=int,default=1000)
    args = parser.parse_args()
    container = os.environ.get('ARENA_CONTAINER', 'arena-kitchen-dev')
    check = subprocess.run(['docker', 'inspect', '--format', '{{.State.Running}}', container],
                           capture_output=True, text=True)
    if check.returncode or check.stdout.strip() != 'true':
        raise RuntimeError('Arena container is not running: '+container+'; set ARENA_CONTAINER before starting')
    args.output.mkdir(parents=True, exist_ok=False)
    args.episodes.mkdir(parents=True, exist_ok=True)
    client = build_opener(ProxyHandler({}))

    def scenes():
        return json.load(client.open(API, timeout=15))['scenes']

    def update(sid, changes, row=None):
        if row is None:
            row = next(s for s in scenes() if s['id'] == sid)
        request = Request(API+'/'+sid, json.dumps(dict(changes=changes,
            base={key: row[key] for key in changes}, actor='physical-completion')).encode(),
            headers={'Content-Type': 'application/json'}, method='PATCH')
        with client.open(request, timeout=15) as response:
            return json.load(response)

    def reference(path, label):
        path = path.resolve()
        for prefix, root in (('/execution-data/', DATA), ('/execution-reports/', REPORTS)):
            if root.resolve() in path.parents:
                return dict(url=prefix+path.relative_to(root.resolve()).as_posix(), label=label)
        raise ValueError('Evidence outside completion roots: '+str(path))

    progress = dict(round=args.round, state='running', container=container, cases=[], baselines={})
    ledger = args.output/'progress.json'

    def save():
        progress['updated_utc'] = datetime.now(timezone.utc).isoformat()
        write_json(ledger, progress)

    def refs_for(episode, label):
        names = ('result.json', 'all_cameras.mp4', 'independent_validation.json',
                 'independent_temperature_validation.json', 'contact_causality_review.json',
                 'audit.json', 'mechanism_audit.json', 'fixed_base_hazard_audit.json',
                 'self_collision_detailed_audit.json', 'property_motion_review.json', 'trajectory.npz')
        return [reference(episode/name, label+' '+name) for name in names if (episode/name).exists()]

    def publish(sid, changes, new_refs=()):
        row = next(s for s in scenes() if s['id'] == sid)
        old_refs = row['execution_artifacts']
        new = [reference(ledger, '完整执行进度 JSON'), *new_refs]
        changes['execution_artifacts'] = new+[r for r in old_refs
            if not any(n['url'] == r['url'] for n in new)]
        update(sid, changes, row=row)

    def run(sid, name, script, arguments, gpu=False):
        command = (['bash', 'scripts/extensions/controls/run.sh', script] if gpu else
                   ['bash', 'scripts/arena_python.sh', 'scripts/extensions/controls/'+script])
        command += [runtime_argument(value) for value in arguments]
        log = args.output/(name+'.log')
        print('START', sid, name, flush=True)
        with log.open('x') as stream:
            process = subprocess.Popen(command, cwd=ROOT, env=os.environ, stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, text=True)
            for line in process.stdout:
                stream.write(line)
                stream.flush()
                if line.startswith('CONTROL_STEP '):
                    tokens = line.split()
                    publish(sid, dict(execution_hint=f'第{args.round}轮 {name}：'
                        f'已记录{tokens[1]}步，当前阶段{tokens[2]}。保留真实接触与执行安全门槛。'),
                        [reference(log, name+' 日志')])
                    print(sid, ' '.join(tokens[:5]), flush=True)
            code = process.wait()
        print('DONE', sid, name, code, flush=True)
        return code, log

    current = scenes()
    rows = [s for s in current if s['id'] in args.scenes] if args.scenes else [
        s for s in current if s['execution_status'] != 'passed']
    progress['retained_passed_scenes'] = [s['id'] for s in current if s not in rows and s['execution_status']=='passed']
    cache = {}
    explicit_baseline_signature = None
    if args.left_baseline:
        baseline_config = json.loads((args.left_baseline/'config.json').read_text())
        digest = controller_digest(baseline_config)
        cache[digest] = args.left_baseline
        explicit_baseline_signature = digest
    for previous in args.baseline_progress:
        data = json.loads(previous.read_text())
        for saved in data['baselines'].values():
            directory = Path(saved['path'])
            result_file = directory/'result.json'
            if result_file.exists() and json.loads(result_file.read_text()).get('accepted'):
                config = json.loads((directory/'config.json').read_text())
                digest = controller_digest(config)
                if digest != explicit_baseline_signature:
                    cache[digest] = directory
    overrides = json.loads(args.config_overrides.read_text()) if args.config_overrides else {}
    save()
    for row in rows:
        sid = row['id']
        source = ROOT/row['visual_config']
        config = json.loads(source.read_text())
        if sid in overrides:
            override = json.loads(Path(overrides[sid]).read_text())
            if frozen_task_differences(config, override):
                raise ValueError('Execution override changed a frozen task/layout field: '+sid)
            config = override
        source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
        task = {'left_cancel': 'left', 'right_cancel': 'right', 'oven_temperature': 'temperature'}[row['task']]
        variant = 'robot' if row['role'] == 'obstacle' else row['role']
        # Use the accepted approach branch, not the failed old obstacle branch.
        if sid in ('03_left_cancel_obstacle', '06_right_cancel_obstacle') and sid not in overrides:
            report = ROOT/'reports/controls_planning_quick_20261027/run02_fixed_layout'/sid/'result.json'
            planned = json.loads(report.read_text())
            if planned['status'] != 'generated' or planned['config_sha256'] != source_hash:
                raise ValueError('Selected approach/config mismatch: '+sid)
            config['prepress_planning']['ik_branch_seed'] = planned['goal_q']
            config['prepress_planning']['source_provenance']=config['prepress_planning'].get('provenance')
            config['prepress_planning']['provenance']='Fixed endpoint branch from '+str(report.relative_to(ROOT))
        if sid == '08_oven_temperature_property' and sid not in overrides:
            report = ROOT/'reports/controls_planning_quick_20261027/round90_auxiliary_roll_height_20261027/round90'/sid/'result.json'
            planned = json.loads(report.read_text())
            if planned['status'] != 'generated' or planned['config_sha256'] != source_hash:
                raise ValueError('Auxiliary approach/config mismatch')
            config['pregrasp_tcp_world_m'] = [planned['requested_tcp'][i][3] for i in range(3)]
            config['pregrasp_roll_deg'] = -165.
            config['prepress_planning']['ik_branch_seed'] = planned['goal_q']
        if sid in args.held_gripper_scenes:
            config['planning_gripper_geometry']='held_articulated'
        if sid in args.balanced_lamp_scenes:
            if task!='temperature' or variant!='property':
                raise ValueError('Lamp mass repair is scoped to oven/property only')
            config['property_physics_recipe']='weighted_reading_lamp_v1'
        if config['variants'] != json.loads(source.read_text())['variants']:
            raise ValueError('Layouts must remain frozen')
        runtime_config = ROOT/'configs/extensions/controls/completion_20261027'/f'round{args.round:02d}'/(sid+'.json')
        if runtime_config.exists():
            raise FileExistsError(runtime_config)
        config['execution_revision'] = dict(source=str(source.relative_to(ROOT)), source_sha256=source_hash,
            layout_modified=False, scope='Controller/contact-reference only; original task, scene and nominal grasp anchor unchanged')
        write_json(runtime_config, config)
        digest = controller_digest(config)
        case = dict(scene=sid, task=task, variant=variant, source_config=str(source.relative_to(ROOT)),
                    source_sha256=source_hash, execution_config=str(runtime_config.relative_to(ROOT)),
                    controller_signature=digest, state='running', validations={})
        progress['cases'].append(case)
        save()
        baseline = cache.get(digest, args.episodes/(task+'_base_'+digest[:8]))
        publish(sid, dict(execution_status='baseline', execution_hint='采集/验证匹配控制参数的无新增实体基线。'))
        if digest not in cache:
            run(sid, sid+'_base', 'collect_trial.py', ['--controls-task', task, '--config',runtime_config,
                '--variant', 'base', '--output', baseline, '--seed',42,'--max-steps',args.max_steps,'--device','cpu'], gpu=True)
            cache[digest] = baseline
        case['baseline'] = str(baseline)
        baseline_result = json.loads((baseline/'result.json').read_text()) if (baseline/'result.json').exists() else {}
        progress['baselines'][digest] = dict(path=str(baseline), accepted=bool(baseline_result.get('accepted')))
        baseline_checks={}
        if baseline_result.get('accepted'):
            baseline_steps=[('data','validate_temperature.py' if task=='temperature' else 'validate_trial.py',
                'independent_temperature_validation.json' if task=='temperature' else 'independent_validation.json',False)]
            if task!='temperature':
                baseline_steps.append(('contact','review_contact_causality.py','contact_causality_review.json',False))
            baseline_steps.append(('self','audit_self_collision.py','self_collision_detailed_audit.json',True))
            for key,script,filename,gpu in baseline_steps:
                destination=baseline/filename
                if not destination.exists():
                    run(sid,sid+'_base_'+key,script,[baseline],gpu=gpu)
                baseline_checks[key]=destination.exists() and json.loads(destination.read_text()).get('passed') is True
        case['baseline_validations']=baseline_checks
        baseline_passed=bool(baseline_result.get('accepted')) and bool(baseline_checks) and all(baseline_checks.values())
        progress['baselines'][digest]['accepted']=baseline_passed
        progress['baselines'][digest]['validations']=baseline_checks
        save()
        if not baseline_passed:
            case.update(state='failed', failure='MATCHED_PHYSICAL_BASELINE_FAILED')
            save()
            publish(sid, dict(execution_status='failed', execution_hint='匹配基线失败；不将该场景视为成功。下一轮先修复同一控制策略。',
                execution_log=json.dumps(baseline_result, ensure_ascii=False, indent=2)), refs_for(baseline,'失败基线'))
            continue
        ep = args.episodes/sid
        case['episode'] = str(ep)
        save()
        publish(sid, dict(execution_status='running', execution_hint=f'第{args.round}轮真实执行与三路RGB采集中：{ep}'), refs_for(baseline,'匹配基线'))
        code, log = run(sid, sid+'_collect', 'collect_trial.py', ['--controls-task',task,'--config',runtime_config,
            '--variant',variant,'--output',ep,'--seed',42,'--max-steps',args.max_steps,'--device','cpu'], gpu=True)
        result = json.loads((ep/'result.json').read_text()) if (ep/'result.json').exists() else dict(accepted=False, failure='COLLECTOR_NO_RESULT', exit_code=code)
        case['collector_accepted'] = bool(result.get('accepted'))
        if result.get('accepted'):
            checks = [('data', 'validate_temperature.py' if task == 'temperature' else 'validate_trial.py', [ep], False)]
            if task != 'temperature':
                checks.append(('contact','review_contact_causality.py',[ep],False))
            if variant=='property':
                checks.append(('property_pose','review_property_motion.py',[ep],False))
            checks.extend([('fixed','audit_fixed_base.py',[ep,'--stand-height', '.7' if task=='temperature' else '.8'],False),
                           ('self','audit_self_collision.py',[ep],True)])
            for key, script, arguments, gpu in checks:
                rc, _ = run(sid,sid+'_'+key,script,arguments,gpu=gpu)
                case['validations'][key] = rc == 0
                save()
        passed = case['collector_accepted'] and bool(case['validations']) and all(case['validations'].values())
        case.update(state='passed' if passed else 'failed', failure=result.get('failure'))
        if hashlib.sha256(source.read_bytes()).hexdigest() != source_hash:
            case.update(state='failed', failure='SOURCE_CHANGED_DURING_EXECUTION')
            passed = False
        save()
        publish(sid,dict(execution_status='passed' if passed else 'failed',
            execution_hint=('原任务真实执行、数据/接触/底座/自碰撞验证通过；有效避障对照尚未新增验收。' if passed else
                '本轮执行或验证失败；保留视频、记录和失败原因，下一轮才修改，不移动布局。'),
            execution_log=json.dumps(dict(result=result, checks=case['validations']),ensure_ascii=False,indent=2)),
            refs_for(ep,'本轮实采')+[reference(log,'采集完整日志')])
    progress['state'] = 'complete' if all(c['state']=='passed' for c in progress['cases']) else 'round_finished_with_failures'
    save()
    if progress['cases']:
        summary=args.output/'reviewed_summary.json'
        subprocess.run([sys.executable,str(ROOT/'scripts/extensions/controls/summarize_reviewed_completion.py'),
            '--progress',*[str(path) for path in args.baseline_progress],str(ledger),
            '--output',str(summary)],cwd=ROOT,check=True)
        for case in progress['cases']:
            publish(case['scene'],{},[reference(summary,'自动验收汇总（含未验证项）')])
    print('PHYSICAL_ROUND_END',progress['state'],flush=True)


if __name__ == '__main__':
    main()
