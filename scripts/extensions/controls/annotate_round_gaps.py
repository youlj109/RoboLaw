"""One annotation per scene: record approach coverage without changing plans."""
import argparse
import json
from pathlib import Path
from urllib.request import ProxyHandler, Request, build_opener

ROOT = Path(__file__).resolve().parents[3]
PLANS = ROOT/'reports/controls_planning_quick_20261027'


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--previous-history', type=Path, required=True)
    p.add_argument('--round-number', type=int, default=6)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    client = build_opener(ProxyHandler({}))
    api = 'http://127.0.0.1:8765'
    rows = json.load(client.open(api+'/api/scenes'))['scenes']
    history = json.loads(a.previous_history.read_text())
    history['scope'] = 'Per scene/round: annotation OR one planning modification. No layout edits.'
    history['state'] = 'awaiting_next_stage_choice'
    round_row = dict(round=a.round_number, state='complete', action_kind='annotation_only', cases=[])
    history['rounds'].append(round_row)
    annotations = []
    for row in rows:
        reference = next(r for r in row['planning_artifacts'] if r['url'].endswith('/result.json'))
        result_file = PLANS/reference['url'][len('/plans/'):]
        result = json.loads(result_file.read_text())
        config = json.loads((ROOT/row['visual_config']).read_text())
        if row['task'] == 'oven_temperature':
            current = result['requested_tcp']
            gap = config['grasp_tcp_world_m'][0]-current[0][3]
            note = (f'第{a.round_number}轮只标注，不修改或重规划。现有文件仅到预抓取TCP；'
                    f'沿原旋钮轴到真实抓取TCP尚有约{gap*100:.0f}厘米连接未生成。'
                    '随后夹爪闭合、围绕原轴转至3档、释放/撤离也未包含在本轮轨迹中。'
                    '预抓取成功不能作为旋钮任务完成。')
            action = f'仅标注：预抓取后约{gap*100:.0f}厘米连接及抓取/旋转段缺失'
            next_step = '根据选择的后续范围，下一轮只补一个阶段；保持旋钮轴、真实抓取点和原3档目标不变。'
        else:
            note = (f'第{a.round_number}轮只标注，不修改或重规划。现有文件只到按钮按压前TCP，'
                    '未拼接或生成从该终点接续的最后接近、实际按压和保持段。'
                    '到达按压前位置不能作为原is_pressed任务成功；旧物理录像不自动验证新的规划连接。')
            action = '仅标注：按压前之后的最后接近、按压/保持段缺失'
            next_step = '根据选择的后续范围，下一轮只补一个阶段；保留原按钮、按压轴和成功阈值，禁止设置按钮关节来代替操作。'
        entry = dict(scene=row['id'], title=row['title'], round=a.round_number,
                     action_kind='annotation_only', source_result=str(result_file.relative_to(ROOT)),
                     annotation=note, next_step=next_step, ik_calls=0, motiongen_calls=0,
                     layout_modified=False, trajectory_modified=False)
        annotations.append(entry)
        round_row['cases'].append(dict(scene=row['id'], title=row['title'], action=action,
                                      status='annotated', motiongen_calls=0, ik_calls=0))
    annotation_file = a.output/'annotations.json'
    annotation_file.write_text(json.dumps(annotations, ensure_ascii=False, indent=2)+'\n')
    ledger = a.output/'round_history.json'
    ledger.write_text(json.dumps(history, ensure_ascii=False, indent=2)+'\n')
    ledger_ref = dict(url='/plans/'+ledger.resolve().relative_to(PLANS).as_posix(), label='连续轮次记录（至第6轮）')
    note_ref = dict(url='/plans/'+annotation_file.resolve().relative_to(PLANS).as_posix(), label='第6轮：阶段缺口标注 JSON')
    for row, annotation in zip(rows, annotations):
        refs = [ledger_ref, note_ref]+row['planning_artifacts']
        changes = dict(planning_hint=annotation['annotation']+'\n'+annotation['next_step']+
                       '\n布局及已有轨迹保留；下一轮范围待选择，不自动执行仿真动作。', planning_artifacts=refs[:30])
        payload = dict(changes=changes, base={k:row[k] for k in changes}, actor='round06-annotation-only')
        req = Request(api+'/api/scenes/'+row['id'], json.dumps(payload, ensure_ascii=False).encode(),
                      headers={'Content-Type':'application/json'}, method='PATCH')
        updated = json.load(client.open(req))
        assert updated['planning_status'] == row['planning_status']
        assert updated['planning_notes'] == row['planning_notes']
        assert updated['planning_review'] == row['planning_review']
        print('ANNOTATED_ONCE', a.round_number, row['id'], flush=True)


if __name__ == '__main__':
    main()
