"""Finalize one physical scene without recollecting, replanning or changing layouts."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener

ROOT=Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from kitchen_safety.extensions.controls.execution_contract import frozen_task_differences


def completed_case(progress, scene):
    cases=[row for row in progress.get('cases',[]) if row['scene']==scene]
    if len(cases)>1:raise ValueError('Duplicate physical scene entries')
    if not cases:return None
    case=cases[0]
    if case['state']=='passed':return case
    if case['state']!='running':raise RuntimeError('Physical scene did not pass: '+str(case.get('failure') or case['state']))
    return None


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene',required=True)
    parser.add_argument('--progress',type=Path,required=True)
    parser.add_argument('--summary-progress',type=Path,nargs='*',default=[])
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--wait-seconds',type=float,default=1800.)
    args=parser.parse_args()
    if not 0<=args.wait_seconds<=7200:raise ValueError('Expected a wait budget from0 to7200s')
    args.output.mkdir(parents=True,exist_ok=False)
    client=build_opener(ProxyHandler({}));api='http://127.0.0.1:8765/api/scenes'
    read=lambda p:json.loads(Path(p).read_text())
    sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
    def run(name,command):
        with (args.output/(name+'.log')).open('x') as log:
            subprocess.run(command,cwd=ROOT,env=os.environ,stdout=log,stderr=subprocess.STDOUT,check=True)
    state=dict(scene=args.scene,state='waiting_for_physical',safety_benchmark_accepted=False)
    def save():
        temporary=args.output/'progress.tmp'
        temporary.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n')
        temporary.replace(args.output/'progress.json')
    save();print('FINALIZE_WAIT',args.scene,flush=True)
    summary_command=[sys.executable,'scripts/extensions/controls/build_benchmark_report.py',
        '--manifest',str(args.manifest),'--progress',*[str(p) for p in args.summary_progress],str(args.progress),
        '--output',str(args.output/'report.json'),'--publish']
    try:
        deadline=time.monotonic()+args.wait_seconds
        while True:
            case=completed_case(read(args.progress),args.scene)
            if case:break
            if time.monotonic()>=deadline:raise TimeoutError('Physical validation wait budget exhausted')
            time.sleep(2)
        scene=next(s for s in json.load(client.open(api,timeout=15))['scenes'] if s['id']==args.scene)
        ep=ROOT/case['episode'];baseline=ROOT/case['baseline'];config=read(ep/'config.json')
        if (scene['visual_status']!='passed' or frozen_task_differences(read(ROOT/scene['visual_config']),config)
                or sha(ROOT/scene['visual_config'])!=case['source_sha256']):
            raise ValueError('Physical evidence differs from the current frozen reviewed scene')
        if config.get('planning_fixture_cylinder_segments',1)!=1:
            raise ValueError('Planner-only primitive refinement cannot receive strict acceptance')
        state.update(state='auditing',episode=case['episode'],baseline=case['baseline']);save()
        inputs=read(args.manifest)[args.scene];audit_name=inputs['avoidance_name'];audit=ep/audit_name
        if not audit.exists():
            run('avoidance',['bash','scripts/arena_python.sh','scripts/extensions/controls/audit_avoidance.py',
                case['episode'],'--baseline',case['baseline'],'--stage',config.get('inspection_stage',
                'reports/skill_trials/controls/inspection/stage.usda'),'--report-name',audit_name])
        evidence=read(audit)
        if (not evidence.get('accepted') or evidence.get('episode_sha256')!=sha(ep/'records.json')
                or evidence.get('baseline_sha256')!=sha(baseline/'records.json')):
            raise ValueError('Independent avoidance report failed or is stale; no overwrite/retry')
        stem='benchmark_first_violation_final';witness=ep/(stem+'_evidence.json')
        if not witness.exists():
            run('witness',['bash','scripts/arena_python.sh','scripts/extensions/controls/render_first_violation.py',
                case['episode'],'--audit-name',audit_name,'--output-stem',stem])
        proof=read(witness);picture=ROOT/proof['picture']
        if proof.get('audit_sha256')!=sha(audit) or proof.get('picture_sha256')!=sha(picture):
            raise ValueError('Witness does not bind the current audit/image')
        render=ROOT/'reports/controls_visual_20261027'/args.scene/(args.output.name+'_evidence.png')
        render.parent.mkdir(parents=True,exist_ok=True)
        if render.exists() and sha(render)!=sha(picture):raise FileExistsError(render)
        if not render.exists():shutil.copy2(picture,render)
        refs=[dict(url='/execution-data/'+str(p.relative_to(ROOT/'datasets/controls_completion_20261027')),
            label='当前严格证据 · '+p.name) for p in (audit,witness)]
        image=dict(url='/renders/'+str(render.relative_to(ROOT/'reports/controls_visual_20261027')),
            label='证据图，非视觉审核图：首次源形状保护距离违规')
        for attempt in range(3):
            scene=next(s for s in json.load(client.open(api,timeout=15))['scenes'] if s['id']==args.scene)
            changes=dict(execution_artifacts=refs+[r for r in scene['execution_artifacts'] if r not in refs],
                visual_images=[image]+[r for r in scene['visual_images'] if r!=image])
            request=Request(api+'/'+args.scene,json.dumps(dict(changes=changes,base={k:scene[k] for k in changes},
                actor='automatic-scene-finalizer'),ensure_ascii=False).encode(),
                headers={'Content-Type':'application/json'},method='PATCH')
            try:
                with client.open(request,timeout=15) as response:json.load(response)
                break
            except HTTPError as error:
                if error.code!=409 or attempt==2:raise
        run('strict_summary',summary_command)
        report=read(args.output/'report.json')
        accepted=next(r['accepted'] for r in report['scenes'] if r['scene']==args.scene)
        state.update(state='complete' if accepted else 'failed_strict_gates',scene_accepted=accepted,
            safety_benchmark_accepted=report['accepted'],strict_passed=report['passed']);save()
        print('FINALIZE_DONE',args.scene,accepted,report['passed'],flush=True)
        if not accepted:raise SystemExit(2)
    except Exception as error:
        state.update(state='failed',reason=f'{type(error).__name__}: {error}')
        if not (args.output/'report.json').exists():
            try:run('failed_summary',summary_command)
            except Exception as summary_error:state['summary_error']=str(summary_error)
        save();raise


if __name__=='__main__':main()
