"""Sequential rounds: one declared modification and at most one path query per case."""
import argparse
import json
import os
from pathlib import Path
import subprocess
from urllib.request import ProxyHandler, Request, build_opener

ROOT = Path(__file__).resolve().parents[3]
PLAN_ROOT = ROOT/'reports/controls_planning_quick_20261027'


def write_json(path, data):
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
    temp.replace(path)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--original-run', type=Path, required=True)
    p.add_argument('--first-round', type=int, default=3)
    p.add_argument('--scene', nargs='+', help='Explicit scenes, including generated-but-incomplete approach targets')
    p.add_argument('--policies', type=Path, help='Declared sequential single-change policies')
    p.add_argument('--previous-history', type=Path)
    p.add_argument('--require-original-tcp-position', action='store_true')
    p.add_argument('--pose-reference', type=Path)
    p.add_argument('--target-kind', choices=['original_precontact','actual_grasp_arrival'], default='original_precontact')
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    client = build_opener(ProxyHandler({}))
    api = 'http://127.0.0.1:8765'

    def scenes():
        return json.load(client.open(api+'/api/scenes'))['scenes']

    def update(sid, changes):
        row = next(s for s in scenes() if s['id'] == sid)
        body = dict(changes=changes, base={k:row[k] for k in changes}, actor='single-change-rounds')
        req = Request(api+'/api/scenes/'+sid, json.dumps(body, ensure_ascii=False).encode(),
                      headers={'Content-Type':'application/json'}, method='PATCH')
        return json.load(client.open(req))

    pending = [s for s in scenes() if (s['id'] in a.scene if a.scene else s['planning_status'] == 'failed')]
    write_json(a.output/'input_scenes.json', pending)
    history = dict(scope='Approach only; fixed layouts. One change, one IK and at most one MotionGen per case/round.',
                   original_run=str(a.original_run), rounds=[], state='running')
    if a.previous_history:
        history['rounds'] = json.loads(a.previous_history.read_text())['rounds']
    history['require_original_tcp_position'] = a.require_original_tcp_position
    history['target_kind'] = a.target_kind
    ledger = a.output/'round_history.json'
    ledger_ref = dict(url='/plans/'+ledger.resolve().relative_to(PLAN_ROOT).as_posix(), label='连续轮次记录 JSON')
    policies = [dict(change='standoff', value=v) for v in (.09, .12, .15)]
    policies += [dict(change='roll', value=v) for v in (-90., 90., -45., 45.)]
    if a.policies:
        policies = json.loads(a.policies.read_text())
        if not isinstance(policies, list) or not policies:
            raise ValueError('Policies must be a nonempty list')
        write_json(a.output/'declared_policies.json', policies)
    for offset, policy in enumerate(policies):
        if not pending:
            break
        number = a.first_round+offset
        round_row = dict(round=number, state='running', cases=[])
        history['rounds'].append(round_row)
        write_json(ledger, history)
        next_pending = []
        for scene in pending:
            sid = scene['id']
            directory = a.output/f'round{number:02d}'/sid
            directory.mkdir(parents=True)
            original_file = a.original_run/sid/'result.json'
            original = json.loads(original_file.read_text())
            policy_file = directory/'single_policy.json'
            write_json(policy_file, policy)
            if policy['change']=='standoff':
                description='仅把预抓取/预按压的轴向退让增加'+str(round(policy['value']*100))+'厘米'
            elif policy['change']=='pregrasp_tilt_y':
                description='保持目标坐标，仅相对声明参考姿态绕世界Y轴偏转'+str(policy['value'])+'度'
            elif policy['change']=='pregrasp_yaw_z':
                description='保持目标坐标，仅相对声明参考姿态绕世界Z轴偏航'+str(policy['value'])+'度'
            elif policy['change']=='ik_seed':
                description='保持已声明的目标TCP与姿态，仅替换一个IK初值，不改变机器人实际起始关节'
            elif policy['change']=='arrival_tcp_position':
                description='只把规划终点改为配置中原有的真实抓取TCP坐标，保留参考姿态和IK初值'
            elif policy['change']=='pregrasp_height':
                description='只把辅助预抓取点相对参考位置上移'+str(round(policy['value']*100))+'厘米，真实抓取点不变'
            else:
                description='仅把工具绕操作轴的roll偏移'+str(policy['value'])+'度'
            case = dict(scene=sid, title=scene['title'], action=description,
                        policy=policy, status='running', motiongen_calls=0)
            round_row['cases'].append(case)
            write_json(ledger, history)
            old_refs = next(s for s in scenes() if s['id']==sid)['planning_artifacts']
            refs = [ledger_ref]+[r for r in old_refs if r['url']!=ledger_ref['url']]
            update(sid, dict(planning_status='running', planning_artifacts=refs,
                             planning_log=f'第{number}轮：'+description+'。本轮只有一个候选，没有循环筛选。',
                             planning_hint=f'第{number}轮进行中：'+description+'。布局、初态、真实接触点与原任务目标保持不变。一次IK、至多一次MotionGen；失败进入下一轮，不在本轮重调。'))
            print('ROUND_START', number, sid, description, flush=True)
            cmd = ['docker','exec','-i','-w','/workspace','-e','PYTHONPATH=/workspace',
                   os.environ.get('ARENA_CONTAINER','robolaw-arena'),
                   'flock','-w','10','/workspace/reports/skill_trials/gpu.lock',
                   'timeout','--kill-after=5s','100s','/workspace/.venv-arena/bin/python',
                   'scripts/extensions/controls/quick_plan_scene.py', '--config', original['config'],
                   '--baseline',original['baseline'],'--stage',original['stage'],
                   '--variant','property' if scene['role']=='property' else 'robot',
                   '--output',str(directory),'--refine-from',str(original_file),
                   '--single-policy',str(policy_file),'--round-number',str(number),'--seed','42']
            cmd += ['--target-kind',a.target_kind]
            if a.pose_reference:
                cmd += ['--pose-reference',str(a.pose_reference)]
            try:
                with (directory/'worker.log').open('w') as stream:
                    proc = subprocess.run(cmd, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, timeout=120)
                if not (directory/'result.json').exists():
                    write_json(directory/'result.json', dict(status='failed', motiongen_calls=0,
                        failure_code='TIMEOUT' if proc.returncode==124 else 'WORKER_ERROR', exit_code=proc.returncode))
                result = json.loads((directory/'result.json').read_text())
            except Exception as exc:
                result = dict(status='failed', motiongen_calls=0, failure_code=type(exc).__name__, error=str(exc))
                if not (directory/'result.json').exists():
                    write_json(directory/'result.json', result)
            case.update(status=result['status'], motiongen_calls=result.get('motiongen_calls',0),
                        failure_code=result.get('failure_code'), layout_files_unchanged=result.get('layout_files_unchanged'),
                        result_file=str(directory/'result.json'))
            new_refs = []
            for name,label in [('preview.json','运动学预览'),('positions.csv','关节轨迹 CSV'),
                               ('positions.npy','关节轨迹 NPY'),('result.json','规划结果'),
                               ('single_policy.json','唯一改动'),('worker.log','完整日志')]:
                file = directory/name
                if file.exists():
                    new_refs.append(dict(url='/plans/'+file.resolve().relative_to(PLAN_ROOT).as_posix(), label=f'第{number}轮 · '+label))
            case['artifacts'] = new_refs
            success = result['status']=='generated'
            if success and a.require_original_tcp_position:
                original_xyz = [original['requested_tcp'][i][3] for i in range(3)]
                if a.target_kind=='actual_grasp_arrival':
                    original_xyz=json.loads((ROOT/original['config']).read_text())['grasp_tcp_world_m']
                selected_xyz = [result['requested_tcp'][i][3] for i in range(3)]
                case['required_tcp_position_reached'] = all(abs(x-y)<1e-9 for x,y in zip(original_xyz,selected_xyz))
                success = case['required_tcp_position_reached']
                if not success:
                    case.update(status='failed', failure_code='ORIGINAL_TCP_POSITION_NOT_REACHED')
            case['summary'] = '返回接近轨迹；未验收' if success else '失败；本轮不再调整'
            if success and policy['change']=='pregrasp_height':
                case['summary']='返回上移后的辅助预抓取轨迹；原终点未达，不代表抓取成功'
            prior = next(s for s in scenes() if s['id']==sid)['planning_artifacts']
            refs = new_refs+[ledger_ref]+[r for r in prior if r['url']!=ledger_ref['url'] and
                                          not any(n['url']==r['url'] for n in new_refs)]
            hint = f'第{number}轮：'+description+'。\n'+case['summary']+'。'
            hint += '\n一次IK、'+str(case['motiongen_calls'])+'次MotionGen。布局和初态未改。'
            if success:
                hint += '\n这是接近轨迹，真实抓取点仍保留；到抓取点的后续连接、旋转3档和过程避障均未验证。'
                if policy['change']=='pregrasp_height':
                    hint += '\n仅调整辅助预抓取点高度，不能计作到达原预抓取点或真实抓取点。'
            else:
                hint += '\n失败原因：'+str(result.get('failure_code', result.get('error')))+'；下一轮才采用另一项改动。'
                next_pending.append(scene)
            if a.require_original_tcp_position:
                hint += ('\n本轮要求到配置中真实抓取TCP坐标的候选到达姿态，不是实际抓住旋钮；闭夹爪、对轴与旋转仍未验证。'
                         if a.target_kind=='actual_grasp_arrival' else
                         '\n本阶段要求到达原预接近TCP坐标，不能仅退到更远点后算成功；最终接触与操作仍未验证。')
            update(sid, dict(planning_status='generated' if success else 'failed', planning_hint=hint,
                             planning_log=json.dumps(result,ensure_ascii=False,indent=2), planning_artifacts=refs))
            write_json(ledger, history)
            print('ROUND_DONE',number,sid,result['status'],result.get('failure_code'),flush=True)
        round_row['state']='complete'
        pending = next_pending
        history['state'] = 'complete' if not pending else 'running'
        write_json(ledger, history)
    if pending:
        history['state']='declared_policies_exhausted_not_complete'
        write_json(ledger, history)
    print('ROUND_SEQUENCE_END',history['state'],flush=True)


if __name__=='__main__':
    main()
