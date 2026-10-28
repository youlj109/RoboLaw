"""Publish an offline contact diagnosis without touching manual review fields."""
import argparse
import json
from pathlib import Path
from urllib.request import ProxyHandler, Request, build_opener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--round', type=int, required=True)
    parser.add_argument('--note', required=True)
    parser.add_argument('--reports', type=Path, nargs='+', required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[3]
    plans = root / 'reports/controls_planning_quick_20261027'
    artifacts = []
    for report in args.reports:
        path = report.resolve()
        data = json.loads(path.read_text())
        if data.get('accepted') is not False:
            raise ValueError('Expected an explicitly non-accepted diagnostic')
        artifacts.append(dict(url='/plans/' + path.relative_to(plans).as_posix(),
                              label=f'第{args.round}轮离线诊断：{path.parent.name}/{path.name}（非验收）'))
    client = build_opener(ProxyHandler({}))
    api = 'http://127.0.0.1:8765/api/scenes'
    row = next(s for s in json.load(client.open(api))['scenes']
               if s['id'] == '07_oven_temperature_human')
    urls = {a['url'] for a in artifacts}
    changes = dict(planning_hint=f'第{args.round}轮：{args.note}\n'
                   '布局未改；诊断不等同于物理碰撞、实采成功或安全验收。',
                   planning_artifacts=artifacts + [a for a in row['planning_artifacts']
                                                  if a['url'] not in urls])
    payload = dict(changes=changes, base={k: row[k] for k in changes},
                   actor='offline-contact-diagnosis')
    request = Request(api + '/' + row['id'], json.dumps(payload, ensure_ascii=False).encode(),
                      headers={'Content-Type': 'application/json'}, method='PATCH')
    updated = json.load(client.open(request))
    for key in ('planning_notes', 'notes', 'execution_notes'):
        assert updated.get(key) == row.get(key)
    print('PUBLISHED_DIAGNOSIS', args.round)


if __name__ == '__main__':
    main()
