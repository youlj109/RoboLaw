"""Publish a manually determined visual review; never infers a verdict."""
import argparse
import json
from datetime import datetime
from pathlib import Path
from urllib.request import ProxyHandler, Request, build_opener


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene', required=True)
    p.add_argument('--state', choices=['reviewing', 'adjusting', 'passed', 'blocked'], required=True)
    p.add_argument('--log', required=True)
    p.add_argument('--config')
    p.add_argument('--render-dir')
    p.add_argument('--main', default='external_camera')
    a = p.parse_args()
    client = build_opener(ProxyHandler({}))
    base = 'http://127.0.0.1:8765'
    rows = json.load(client.open(base + '/api/scenes'))['scenes']
    row = next(s for s in rows if s['id'] == a.scene)
    stamp = datetime.now().strftime('%m-%d %H:%M:%S')
    changes = {'visual_status': a.state,
               'visual_log': (row['visual_log'] + '\n\n' if row['visual_log'] else '') + stamp + '\n' + a.log}
    if a.config:
        changes['visual_config'] = a.config
    if a.render_dir:
        root = Path(__file__).resolve().parents[3] / 'reports/controls_visual_20261027'
        directory = root / a.render_dir
        labels = {'external_camera': '原相机1', 'external_camera_2': '原相机2',
                  'wide': '全景检查', 'side': '侧面检查', 'body': '全身检查',
                  'lamp_detail': '灯具低角度细节', 'wrist_camera': '腕部视角'}
        entries = list(row['visual_images'])
        views = [v for v in ('wide', 'side', 'body', 'lamp_detail', 'external_camera', 'external_camera_2') if v != a.main] + [a.main]
        for view in views:
            file = directory / (view + '.jpg')
            if not file.is_file():
                continue
            url = '/renders/' + file.relative_to(root).as_posix()
            if not any(x['url'] == url for x in entries):
                entries.append({'url': url, 'label': directory.name + ' · ' + labels.get(view, view)})
        changes['visual_images'] = entries
    payload = {'changes': changes, 'base': {k: row[k] for k in changes}, 'actor': 'assistant-visual-only'}
    req = Request(base + '/api/scenes/' + a.scene, json.dumps(payload, ensure_ascii=False).encode(),
                  headers={'Content-Type': 'application/json'}, method='PATCH')
    result = json.load(client.open(req))
    print(result['id'], result['visual_status'], 'revision', result['revision'])


if __name__ == '__main__':
    main()
