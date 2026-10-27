"""Host-side independent avoidance audits for version-matching physical evidence."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from urllib.request import ProxyHandler, Request, build_opener

ROOT=Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from kitchen_safety.extensions.controls.execution_contract import frozen_task_differences


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--round',type=int,required=True)
    p.add_argument('--physical',type=Path,default=Path('docs/extensions/controls/scene_review/physical_final_20261027.json'))
    p.add_argument('--scenes',nargs='+',required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    api='http://127.0.0.1:8765/api/scenes';client=build_opener(ProxyHandler({}))
    def rows():return json.load(client.open(api,timeout=15))['scenes']
    current={r['id']:r for r in rows()}
    saved={r['scene']:r for r in json.loads(a.physical.read_text())['scenes']}
    if set(a.scenes)-set(current):raise ValueError('Unknown scenes')
    progress=dict(round=a.round,state='running',safety_benchmark_accepted=False,cases=[])
    def save():
        progress['updated_utc']=datetime.now(timezone.utc).isoformat()
        temp=a.output/'progress.tmp';temp.write_text(json.dumps(progress,ensure_ascii=False,indent=2)+'\n')
        temp.replace(a.output/'progress.json')
    for sid in a.scenes:
        print('AUDIT_START',sid,flush=True)
        old=saved[sid];ep=ROOT/old['episode'];baseline=ROOT/old['baseline']
        visual=ROOT/current[sid]['visual_config']
        config=json.loads((ep/'config.json').read_text())
        case=dict(scene=sid,episode=old['episode'],baseline=old['baseline'],state='running')
        progress['cases'].append(case);save()
        differences=frozen_task_differences(json.loads(visual.read_text()),config)
        if differences:
            case.update(state='skipped',reason='Physical evidence belongs to a different frozen scene',differences=differences)
            save();continue
        report_name=f'benchmark_avoidance{a.round}.json';destination=ep/report_name
        if destination.exists():raise FileExistsError(destination)
        stage=config.get('inspection_stage','reports/skill_trials/controls/inspection/stage.usda')
        log=a.output/(sid+'.log')
        command=['bash','scripts/arena_python.sh','scripts/extensions/controls/audit_avoidance.py',
            str(ep.relative_to(ROOT)),'--baseline',str(baseline.relative_to(ROOT)),
            '--stage',stage,'--report-name',report_name]
        with log.open('x') as stream:
            process=subprocess.run(command,cwd=ROOT,env=os.environ,stdout=stream,stderr=subprocess.STDOUT)
        report=json.loads(destination.read_text()) if destination.exists() else {}
        case.update(state='passed' if report.get('accepted') is True else 'failed',exit_code=process.returncode,
            visual_config=str(visual.relative_to(ROOT)),visual_sha256=hashlib.sha256(visual.read_bytes()).hexdigest(),
            report=str(destination.relative_to(ROOT)),failed_gates=[k for k,v in report.get('gates',{}).items() if not v])
        save()
        row=next(r for r in rows() if r['id']==sid)
        refs=[dict(url='/execution-data/'+str(destination.relative_to(ROOT/'datasets/controls_completion_20261027')),
                   label=f'第{a.round}轮：独立避障审计（含拒绝项）'),
              dict(url='/execution-reports/'+str((a.output/'progress.json').resolve().relative_to(ROOT/'reports/controls_completion_20261027')),
                   label=f'第{a.round}轮：安全审计进度')]
        hint=(f'第{a.round}轮独立避障审计：'+('该单基线门禁通过，仍需多 seed 和中性验证。' if case['state']=='passed' else
            '未通过；拒绝项：'+(', '.join(case['failed_gates']) or '脚本/报告失败')+'。')+'\n此前执行记录：\n'+row['execution_hint'])
        changes=dict(execution_hint=hint,execution_artifacts=refs+[r for r in row['execution_artifacts'] if not any(n['url']==r['url'] for n in refs)])
        payload=dict(changes=changes,base={key:row[key] for key in changes},actor='independent-benchmark-audit')
        request=Request(api+'/'+sid,json.dumps(payload,ensure_ascii=False).encode(),
            headers={'Content-Type':'application/json'},method='PATCH')
        with client.open(request,timeout=15) as response:json.load(response)
        print('AUDIT_DONE',sid,case['state'],case['failed_gates'],flush=True)
    progress['state']='complete';save()


if __name__=='__main__':main()
