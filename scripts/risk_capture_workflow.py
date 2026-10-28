"""Default raw-online / risk-offline workflow for scene-supplied episode specs.

Run under scripts/arena_python.sh. Capture never starts offline phases.
Each stage keeps its own output and status; raw success is not risk acceptance.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))


def read(path):return json.loads(Path(path).read_text())

def write(path,value):
    Path(path).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')


def run(args):
    stage,base=args.stage,args.output.resolve()
    if getattr(args,'ylj_scene',None) and (stage!='capture' or not args.task_contract):
        raise ValueError('--ylj-scene requires capture with an explicit --task-contract; later stages replay frozen inputs')
    if args.task_contract or (base/'native_workflow.json').exists():
        from kitchen_safety.native_workflow import run as run_native
        return run_native(args)
    if any(getattr(args,key,None) is not None for key in ('appearance_config','appearance_seed','camera_config')):
        raise ValueError('Native appearance/camera options require a native task workflow')
    policy_path=args.policy.resolve()
    if stage=='capture':
        if not args.episode_specs:raise ValueError('capture requires --episode-specs; no scenario is hard-coded')
        specs=read(args.episode_specs)
        if not isinstance(specs,list) or not specs:raise ValueError('episode-specs must be a nonempty list')
        if len({s['obstacle_id'] for s in specs})!=1:
            raise ValueError('A native batch must have homogeneous asset topology; use separate runs for different scene IDs')
        policy=read(policy_path)
        if policy['capture_mode']!='raw-only':raise ValueError('Expected saved raw-only workflow')
        base.mkdir(parents=True,exist_ok=False)
        write(base/'workflow_policy.json',policy)
        write(base/'episode_specs.json',specs)
        if args.collector_overrides:shutil.copy2(args.collector_overrides,base/'collector_overrides.json')
        r=policy['default_runtime']
        command=[sys.executable,str(ROOT/'scripts/collect_kitchen_vectorized.py'),
            '--episode-specs',str(base/'episode_specs.json'),'--num-envs',str(len(specs)),
            '--output',str(base/'capture'),'--collection-purpose','diagnostic',
            '--observer-config',str(ROOT/policy['observer_config']),
            '--run-id',args.run_id or base.name,'--capture-workflow-policy',str(base/'workflow_policy.json'),
            '--risk-capture-mode','raw-only','--record-render-state',
            '--defer-post-collection-checks','--defer-clearance-diagnostics',
            '--device',str(r['device']),'--physics-threads',str(r['physics_threads']),
            '--torch-threads',str(r['torch_threads']),'--episode-length-s',str(r['episode_length_s']),
            '--max-steps',str(r['max_steps'])]
        if args.collector_overrides:command+=['--collector-overrides',str(base/'collector_overrides.json')]
        jobs=[('capture',command)]
    else:
        policy=read(base/'workflow_policy.json')
        episodes=sorted((base/'capture').glob('env_*'))
        episodes=[p for p in episodes if (p/'raw_evidence.jsonl').is_file()]
        if not episodes:raise ValueError('No captured raw episodes')
        jobs=[]
        for episode in episodes:
            name=episode.name
            if stage=='evaluate':
                jobs.extend([(name+'_raw_integrity',[sys.executable,str(ROOT/'scripts/validate_raw_capture.py'),
                    '--episode',str(episode),'--output',str(base/(name+'_raw_integrity.json'))]),
                    (name+'_risk',[sys.executable,str(ROOT/'scripts/evaluate_raw_capture.py'),
                    '--episode',str(episode),'--output',str(base/'risk'/name),'--geometry-backend',args.geometry_backend])])
                (base/'risk').mkdir(exist_ok=True)
            elif stage=='render':
                (base/'rendered').mkdir(exist_ok=True)
                jobs.append((name+'_render',[sys.executable,str(ROOT/'scripts/render_recorded_episode.py'),
                    '--source-episode',str(episode),'--output',str(base/'rendered'/name),
                    '--num_envs','1','--device','cpu','--camera-device','cuda:0','--background-rgb']))
            elif stage=='audit':
                from kitchen_safety.vectorized_validation import validate_physics_archive
                frozen=read(episode/'safety_observer_binding.json');b=frozen['binding']
                result=read(episode/'result.json')
                job=dict(stage_path=str(base/'capture/validation_scene.usdc'),
                    physics_states_path=str(episode/'physics_states.npz'),
                    episode_spec=read(episode/'episode_spec.json'),
                    env_root=b['robot_prim_path'].rsplit('/',1)[0],
                    env_origin=b['environment_origin_world_m'],state_frame='env_local',
                    settled_poses=result.get('settled_poses',{}),
                    action_clearance_states_path=str(episode/'action_clearance_states.npz'))
                target=base/'audit'/name;target.mkdir(parents=True,exist_ok=False)
                started=time.perf_counter();report=validate_physics_archive(job)
                report.update(evaluation_mode='offline_archive_audit',elapsed_s=time.perf_counter()-started,
                    source_episode=str(episode),scope='Original independent execution/clearance audit; separate from HS/PT/RS Warp rules and user exemptions')
                write(target/'clearance_audit.json',report)
                # A completed audit may find violations. Preserve its verdict;
                # never rewrite raw capture status or call this production acceptance.
            elif stage=='video':
                risk=base/'risk'/name;render=base/'rendered'/name
                if not (risk/'offline_risk_summary.json').is_file():raise ValueError('Run evaluate before video')
                report=read(render/'render_report.json')
                if not report.get('complete_trajectory') or Path(report['source_episode']).resolve()!=episode.resolve():
                    raise ValueError('RGB must be a completed rendering of this exact raw episode')
                view=base/'review_input'/name;view.mkdir(parents=True,exist_ok=False)
                for file in ('trajectory.h5','result.json','episode_spec.json','camera_config.json','render_state_metadata.json'):
                    (view/file).symlink_to(episode/file)
                effective_binding=risk/'safety_observer_binding.json'
                (view/'safety_observer_binding.json').symlink_to(effective_binding if effective_binding.exists() else episode/'safety_observer_binding.json')
                (view/'safety_decisions.jsonl').symlink_to(risk/'safety_decisions.jsonl')
                (view/'rendered').symlink_to(render)
                write(view/'result_axes.json',dict(task=dict(legacy_success=read(episode/'result.json').get('success')),source='Raw task result only; no risk or quota acceptance inferred'))
                write(view/'composition_manifest.json',dict(geometry_backend=read(risk/'offline_risk_summary.json').get('geometry_backend'),evaluation_mode='offline_replay',raw_episode=str(episode),risk_output=str(risk),rgb_output=str(render)))
                (base/'video').mkdir(exist_ok=True)
                jobs.append((name+'_video',[sys.executable,str(ROOT/'scripts/export_risk_review_video.py'),
                    '--episode',str(view),'--output',str(base/'video'/name)]))
    logdir=base/'workflow_logs';logdir.mkdir(exist_ok=True)
    ledger_path=base/'workflow_runs.json';ledger=read(ledger_path) if ledger_path.exists() else []
    entry=dict(stage=stage,started_wall_ns=time.time_ns(),status='running',commands=[],
        entrypoint_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        workflow_policy_sha256=hashlib.sha256((base/'workflow_policy.json').read_bytes()).hexdigest())
    ledger.append(entry);write(ledger_path,ledger)
    for name,command in jobs:
        tick=time.perf_counter()
        with (logdir/(name+'_'+str(entry['started_wall_ns'])+'.log')).open('x') as log:
            result=subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,
                env=os.environ|{'PYTHONPATH':str(ROOT),'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','MKL_NUM_THREADS':'1'})
        entry['commands'].append(dict(name=name,command=command,exit_code=result.returncode,wall_s=time.perf_counter()-tick))
        write(ledger_path,ledger)
        if result.returncode:
            entry.update(status='failed',completed_wall_ns=time.time_ns());write(ledger_path,ledger)
            return result.returncode
    if stage=='video':
        for episode in episodes:
            target=base/'video'/episode.name
            composition=read(base/'review_input'/episode.name/'composition_manifest.json')
            write(target/'composition_manifest.json',composition)
            for filename in ('manifest.json','risk_review.json'):
                report=read(target/filename);report.update(composition)
                write(target/filename,report)
    entry.update(status='completed',completed_wall_ns=time.time_ns(),
        risk_evaluation_status='pending_offline' if stage=='capture' else 'see_per_episode_reports',production_acceptance='not_inferred')
    write(ledger_path,ledger)
    return 0


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('stage',choices=['capture','evaluate','audit','render','video'])
    p.add_argument('--output',type=Path,required=True,help='Run root shared by all stages')
    p.add_argument('--episode-specs',type=Path,help='Scene-supplied JSON list, capture only')
    p.add_argument('--collector-overrides',type=Path)
    p.add_argument('--policy',type=Path,default=ROOT/'configs/risk_capture_workflow.json')
    p.add_argument('--run-id')
    p.add_argument('--task-contract',type=Path,help='Native multi-task contract; separate from legacy scene episode specs')
    p.add_argument('--policy-factory',help='Explicit native task controller module:function')
    p.add_argument('--policy-config',type=Path)
    p.add_argument('--ylj-scene',help='Exact ylj scene ID; use frozen ylj layout/NPZ with the current reviewed risk scope')
    p.add_argument('--asset-cache',type=Path)
    p.add_argument('--obstacle-binding',type=Path)
    p.add_argument('--max-actions',type=int,default=3000)
    p.add_argument('--seed',type=int,default=42,help='Native environment and default placement seed')
    p.add_argument('--placement-seed',type=int,help='Explicit native placement sampler seed')
    p.add_argument('--camera-config',type=Path,help='Native render only: task-bound offline camera configuration')
    p.add_argument('--appearance-config',type=Path,help='Native render only: exact-path visual-only appearance recipe')
    p.add_argument('--appearance-seed',type=int,help='Native render only: explicit appearance draw seed')
    p.add_argument('--geometry-backend',choices=['frozen','gdist','warp'],default='frozen',help='Evaluation only: explicit historical backend migration; new captures freeze gDist by default')
    args=p.parse_args()
    return run(args)

if __name__=='__main__':raise SystemExit(main())
