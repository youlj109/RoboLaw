"""One frozen-layout neutral hold per reviewed scene, with durable web evidence."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener

ROOT=Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from kitchen_safety.extensions.controls.execution_contract import frozen_task_differences


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--profiles',type=Path,required=True,help='Scene IDs mapped to explicit execution configs')
    p.add_argument('--seconds',type=float,default=50.)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    profiles=json.loads(a.profiles.read_text())
    api='http://127.0.0.1:8765/api/scenes';client=build_opener(ProxyHandler({}))
    def rows():return json.load(client.open(api,timeout=15))['scenes']
    progress=dict(state='running',cases=[],safety_benchmark_accepted=False)
    def save():
        progress['updated_utc']=datetime.now(timezone.utc).isoformat()
        temp=a.output/'progress.tmp';temp.write_text(json.dumps(progress,ensure_ascii=False,indent=2)+'\n');temp.replace(a.output/'progress.json')
    for sid,profile in profiles.items():
        row=next(r for r in rows() if r['id']==sid)
        config=json.loads((ROOT/profile).read_text())
        visual=json.loads((ROOT/row['visual_config']).read_text())
        if row['visual_status']!='passed' or frozen_task_differences(visual,config):
            raise ValueError('Neutral hold requires current image-reviewed frozen scene: '+sid)
        task=dict(left_cancel='left',right_cancel='right',oven_temperature='temperature')[row['task']]
        variant='robot' if row['role']=='obstacle' else row['role']
        inspection=dict(left='reports/skill_trials/controls/inspection',
            right='reports/controls_right_zxw_20261023/inspection',
            temperature='reports/controls_temperature_zxw_20261023/inspection03')[task]
        output=a.output/sid;log=a.output/(sid+'.log')
        case=dict(scene=sid,config=profile,output=str(output),state='running')
        progress['cases'].append(case);save();print('NEUTRAL_START',sid,flush=True)
        command=['bash','scripts/extensions/controls/run.sh','probe_neutral.py',
            '--controls-task',task,'--config',profile,'--pose-from-config',
            '--variant',variant,'--seconds',str(a.seconds),'--output',str(output),'--device','cpu']
        with log.open('x') as stream:
            process=subprocess.run(command,cwd=ROOT,env=os.environ,stdout=stream,stderr=subprocess.STDOUT)
        report=json.loads((output/'validation.json').read_text()) if (output/'validation.json').exists() else {}
        case.update(state='passed' if report.get('accepted') is True else 'failed',exit_code=process.returncode,
            seconds=report.get('neutral_seconds'),failure=report.get('failure'))
        save()
        prefix='/execution-reports/'
        def ref(path,label):return dict(url=prefix+str(path.resolve().relative_to(ROOT/'reports/controls_completion_20261027')),label=label)
        refs=[ref(a.output/'progress.json','当前场景中性静置验证进度')]
        for filename in ('validation.json','background_mechanism_audit.json','all_cameras.mp4','hazard_physics_poses.json'):
            if (output/filename).exists():refs.append(ref(output/filename,'当前场景中性对照 '+filename))
        for attempt in range(3):
            row=next(r for r in rows() if r['id']==sid)
            changes=dict(execution_artifacts=refs+[r for r in row['execution_artifacts'] if not any(n['url']==r['url'] for n in refs)])
            if row['execution_status'] not in ('baseline','running'):
                changes['execution_hint']='当前场景中性静置：'+('通过' if case['state']=='passed' else '未通过')+f'，实测 {case["seconds"]} s。此项不替代任务或避障验收。\n'+row['execution_hint']
            request=Request(api+'/'+sid,json.dumps(dict(changes=changes,base={k:row[k] for k in changes},
                actor='benchmark-neutral-verification'),ensure_ascii=False).encode(),
                headers={'Content-Type':'application/json'},method='PATCH')
            try:
                with client.open(request,timeout=15) as response:json.load(response)
                break
            except HTTPError as exc:
                if exc.code!=409 or attempt==2:raise
        print('NEUTRAL_DONE',sid,case['state'],case['seconds'],flush=True)
    progress['state']='complete';save()


if __name__=='__main__':main()
