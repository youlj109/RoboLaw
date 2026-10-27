"""Serial one-shot planning for the nine visually revised scenes; skip failures."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
from urllib.request import ProxyHandler, Request, build_opener

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0,str(ROOT))
from kitchen_safety.extensions.controls.execution_contract import frozen_task_differences
BASELINES = {
    'left_cancel': ('datasets/controls_zxw_20261023/full_recorded/base', 'reports/skill_trials/controls/inspection/stage.usda'),
    'right_cancel': ('datasets/controls_right_zxw_20261023/final_raw/base', 'reports/controls_right_zxw_20261023/inspection/stage.usda'),
    'oven_temperature': ('datasets/controls_temperature_zxw_20261023/base04_fixed', 'reports/controls_temperature_zxw_20261023/inspection03/stage.usda'),
}


def select_scenes(scenes, identifiers):
    if not identifiers:return scenes
    unknown=set(identifiers)-{row['id'] for row in scenes}
    if unknown:raise ValueError('Unknown scenes: '+', '.join(sorted(unknown)))
    return [row for row in scenes if row['id'] in identifiers]


def planning_inputs(scene,references):
    reference=references.get(scene['id'],{})
    config=reference.get('config',scene['visual_config'])
    if not scene['visual_config'] or not config:raise ValueError('Missing visual config')
    source=json.loads((ROOT/scene['visual_config']).read_text())
    candidate=json.loads((ROOT/config).read_text())
    if frozen_task_differences(source,candidate):
        raise ValueError('Planning reference changed the frozen scene/task')
    baseline,stage=BASELINES[scene['task']]
    return config,reference.get('baseline',baseline),reference.get('stage',stage)


def failure_hint(code, error=''):
    value = (code + ' ' + error).upper()
    if 'START' in value and ('COLLISION' in value or 'INVALID' in value):
        return '布局已冻结：优先核对初始关节读取与模型。无效起点不能靠抓点或中间路点补救；不移动实体、不降低间隙。若真实初态非法，明确标为受阻，不能伪造新的起点冒充原任务。'
    if 'GOAL' in value and ('COLLISION' in value or 'INVALID' in value):
        return '优先标注终点：考虑不同冗余肘姿、按压前/抓取前接近方向或接近距离。保留原按钮/旋钮位置与操作轴；不要直接移动任务目标或降低碰撞约束。'
    if 'IK FAILED' in value:
        return '固定IK分支未解出目标。先核对目标坐标/工具姿态，下一轮可尝试其他冗余IK种子；不要把IK失败直接当作场景碰撞或任务无解。'
    if 'GRAPH' in value or 'TRAJOPT' in value or 'OPTIM' in value:
        return '本次图搜索/优化未找到轨迹，不代表无解。人工先看起终姿态与可走侧，再标注候选绕行侧、中间路点或冗余肘姿；下一轮可增加预算，但本轮不重试。'
    if 'TIMEOUT' in value:
        return '本轮到时跳过。先看日志区分加载耗时与搜索耗时，下一轮再决定增加预算或复用规划上下文；不要直接改场景。'
    return '先查看结果与完整日志，区分资源/资产/SDK错误和真正的路径失败，再标注修改起点、终点或中间路点。仅凭一次失败不能判定任务无解。'


def scene_hint(sid):
    return {
        '03_left_cancel_obstacle': '本场景的人工候选（未尝试）：标注从横杆上方或自由端外侧绕行的中间区域；也可换肘部冗余姿态，但保持左按钮最后按压轴与目标不变。',
        '06_right_cancel_obstacle': '本场景的人工候选（未尝试）：标注从立柱另一侧或横杆上方接近，再回到右按钮按压前姿态；若终点拥挤，先换冗余肘姿，不把中间路点放进支架。',
        '08_oven_temperature_property': '本场景的人工候选（未尝试）：布局冻结，不移动灯。考虑不同肘姿、合法抓点/接近姿态或从灯臂外侧接近；若终点本身无效，中间路点不能修复。原旋钮轴与目标档位保持。',
    }.get(sid, '')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--resume', action='store_true', help='Publish existing results without re-planning completed scenes')
    p.add_argument('--refine-failed-from', type=Path, help='Only refine failed cases in this saved run; fixed layouts')
    p.add_argument('--scenes',nargs='+',help='Retain all other scenes without re-planning')
    p.add_argument('--references',type=Path,help='Per-scene config/baseline/stage references; layouts must match visual configs')
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=a.resume)
    client = build_opener(ProxyHandler({}))
    api = 'http://127.0.0.1:8765'

    def get():
        return json.load(client.open(api+'/api/scenes'))['scenes']

    def update(sid, changes):
        row = next(s for s in get() if s['id'] == sid)
        payload = dict(changes=changes, base={k: row[k] for k in changes}, actor='curobo-one-shot')
        req = Request(api+'/api/scenes/'+sid, json.dumps(payload, ensure_ascii=False).encode(),
                      headers={'Content-Type': 'application/json'}, method='PATCH')
        return json.load(client.open(req))

    scenes = (json.loads((a.output/'input_scenes.json').read_text())
              if a.resume and (a.output/'input_scenes.json').exists() else get())
    scenes=select_scenes(scenes,a.scenes)
    references=json.loads(a.references.read_text()) if a.references else {}
    if a.refine_failed_from:
        scenes = [s for s in scenes if (a.refine_failed_from/s['id']/'result.json').is_file() and
                  json.loads((a.refine_failed_from/s['id']/'result.json').read_text()).get('status') == 'failed']
    if not (a.output/'input_scenes.json').exists():
        (a.output/'input_scenes.json').write_text(json.dumps(scenes, ensure_ascii=False, indent=2)+'\n')
    runs = []
    for scene in scenes:
        sid = scene['id']
        directory = a.output/sid
        directory.mkdir(exist_ok=a.resume)
        print('QUICK_START', sid, flush=True)
        update(sid, dict(planning_status='running',
                         planning_log=('第2轮：布局与起点固定；有界接近终点筛选后，只尝试一次MotionGen。'
                                       if a.refine_failed_from else '固定seed42；单次MotionGen查询；仅接近段；失败跳过，不执行。'),
                         planning_hint=('正在修改规划策略：优先同TCP冗余IK，其次轴向roll及接近退让；最多64个IK候选/25秒，路径规划不重试。'
                                        if a.refine_failed_from else '正在生成，请勿把规划器输出理解为任务/安全验收。')))
        try:
            config,baseline,stage=planning_inputs(scene,references)
            variant = 'robot' if scene['role'] == 'obstacle' else scene['role']
            cmd = ['docker', 'exec', '-i', '-w', '/workspace', '-e', 'PYTHONPATH=/workspace',
                   os.environ.get('ARENA_CONTAINER', 'robolaw-arena'),
                   'flock', '-w', '10', '/workspace/reports/skill_trials/gpu.lock',
                   'timeout', '--signal=TERM', '--kill-after=5s', '100s',
                   '/workspace/.venv-arena/bin/python', 'scripts/extensions/controls/quick_plan_scene.py',
                   '--config', config, '--baseline', baseline, '--stage', stage,
                   '--variant', variant, '--output', str(directory), '--seed', '42']
            if a.refine_failed_from:
                cmd += ['--refine-from', str(a.refine_failed_from/sid/'result.json')]
            if not (a.resume and (directory/'result.json').exists()):
                with (directory/'worker.log').open('w') as stream:
                    result = subprocess.run(cmd, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, timeout=120)
            file = directory/'result.json'
            if file.is_file():
                report = json.loads(file.read_text())
            else:
                report = dict(status='failed', failure_code='TIMEOUT' if result.returncode == 124 else 'WORKER_ERROR',
                              error='Worker exit '+str(result.returncode), safety_validated=False)
        except Exception as exc:
            report = dict(status='failed', failure_code=type(exc).__name__, error=str(exc), safety_validated=False)
        if not (directory/'result.json').is_file():
            (directory/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        artifacts = []
        for name, label in [('preview.json', '运动学预览'), ('positions.npy', '关节轨迹 NPY'),
                            ('positions.csv', '时间与关节轨迹 CSV'), ('result.json', '规划结果 JSON'),
                            ('worker.log', '完整规划日志'), ('endpoint_candidates.json', '接近终点候选筛选记录')]:
            file = directory/name
            if file.is_file():
                artifacts.append(dict(url='/plans/'+file.resolve().relative_to(ROOT/'reports/controls_planning_quick_20261027').as_posix(),
                                      label=('第2轮 · ' if a.refine_failed_from else '')+label))
        success = report.get('status') == 'generated'
        hint = ('已返回接近段。请人工检查走向和起终姿态，并填写标注；按压、旋转、避障效果与物理执行均未验证。'
                if success else failure_hint(report.get('failure_code', ''), report.get('error', '')))
        if a.refine_failed_from:
            if success:
                hint = ('第2轮已返回接近轨迹。布局、底座与初态未改，真实接触点未改。所选策略：' +
                        json.dumps(report.get('selected_endpoint_policy', {}), ensure_ascii=False) +
                        '\n本轮只调用一次MotionGen；未执行，也未验收过程避障、接触动作或任务成功。')
            elif report.get('failure_code') == 'NO_VALID_ENDPOINT_IN_BOUNDED_CANDIDATES':
                hint = '第2轮仍失败：限定候选/时间内未找到可用接近终点，本轮直接跳过，未调用路径规划器。不代表整个固定布局问题无解；布局和初态保留，不自动改场景或再试。'
            else:
                hint = ('第2轮仍失败，已跳过：'+str(report.get('failure_code',report.get('error')))+
                        '\n布局不改。本轮不再重试；请看候选日志和规划日志，下一轮再决定调整冗余姿态/轴向roll/接近距离或合法路点。')
        elif not success and scene_hint(sid):
            hint += '\n' + scene_hint(sid)
        old_refs = next(s for s in get() if s['id']==sid)['planning_artifacts']
        artifacts += [r for r in old_refs if not any(n['url']==r['url'] for n in artifacts)]
        summary = dict(report, scene=sid)
        runs.append(summary)
        update(sid, dict(planning_status='generated' if success else 'failed',
                         planning_log=json.dumps(report, ensure_ascii=False, indent=2),
                         planning_hint=hint, planning_artifacts=artifacts))
        (a.output/'batch_results.json').write_text(json.dumps(dict(scope='one-shot approach planning only',
            safety_validated=False, completed=len(runs), total=len(scenes), runs=runs), ensure_ascii=False, indent=2)+'\n')
        print('QUICK_DONE', sid, report['status'], report.get('failure_code'), flush=True)


if __name__ == '__main__':
    main()
