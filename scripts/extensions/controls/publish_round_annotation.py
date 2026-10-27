"""Publish one annotation-only action for one scene, without changing a plan."""
import argparse
import json
from pathlib import Path
from urllib.request import ProxyHandler, Request, build_opener
from repeat_plan_rounds import write_json, PLAN_ROOT


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene', required=True)
    p.add_argument('--round-number', type=int, required=True)
    p.add_argument('--previous-history', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--note', required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    history = json.loads(a.previous_history.read_text())
    history['rounds'].append(dict(round=a.round_number, state='complete', action_kind='annotation_only',
        cases=[dict(scene=a.scene, action=a.note, status='annotated', ik_calls=0, motiongen_calls=0)]))
    history['state']='annotation_complete_next_round_pending'
    ledger=a.output/'round_history.json'
    write_json(ledger,history)
    client=build_opener(ProxyHandler({}));api='http://127.0.0.1:8765'
    row=next(s for s in json.load(client.open(api+'/api/scenes'))['scenes'] if s['id']==a.scene)
    reference=dict(url='/plans/'+ledger.resolve().relative_to(PLAN_ROOT).as_posix(),label=f'连续轮次记录（至第{a.round_number}轮）')
    changes=dict(planning_hint=f'第{a.round_number}轮仅标注：'+a.note+'\n本轮零IK、零MotionGen；不改布局、初态或轨迹。',
                 planning_artifacts=[reference]+row['planning_artifacts'])
    payload=dict(changes=changes,base={k:row[k] for k in changes},actor='single-round-annotation')
    request=Request(api+'/api/scenes/'+a.scene,json.dumps(payload,ensure_ascii=False).encode(),
                    headers={'Content-Type':'application/json'},method='PATCH')
    updated=json.load(client.open(request))
    assert updated['planning_status']==row['planning_status']
    assert updated['planning_notes']==row['planning_notes']
    print('ANNOTATED_ONCE',a.round_number,a.scene)


if __name__=='__main__':main()
