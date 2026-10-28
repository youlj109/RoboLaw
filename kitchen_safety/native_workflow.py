"""Native task routing for the saved online/offline workflow entry point."""
import hashlib,json,os,shutil,subprocess,sys,time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]

def run(args):
    base=args.output.resolve();stage=args.stage
    camera_config=getattr(args,'camera_config',None)
    if camera_config and stage!='render':raise ValueError('Camera configuration is render-only')
    appearance_config=getattr(args,'appearance_config',None)
    appearance_seed=getattr(args,'appearance_seed',None)
    if (appearance_config is None)!=(appearance_seed is None) or (appearance_config and stage!='render'):
        raise ValueError('Visual appearance config/seed are paired render-stage options')
    def read(path):return json.loads(Path(path).read_text())
    def write(path,value):Path(path).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
    def record_stage(started,tick,exit_code,**details):
        ledger=base/'native_workflow_runs.json';entries=read(ledger) if ledger.exists() else []
        entries.append(dict(stage=stage,started_wall_ns=started,wall_s=time.perf_counter()-tick,
            exit_code=exit_code,status='completed' if exit_code==0 else 'failed',
            production_acceptance='not inferred from stage execution',**details))
        write(ledger,entries)
    if stage=='capture':
        ylj_scene=getattr(args,'ylj_scene',None)
        if not args.task_contract or not (args.policy_factory or ylj_scene) or not args.asset_cache:
            raise ValueError('Native capture requires --task-contract, --asset-cache and either --policy-factory or --ylj-scene')
        if ylj_scene and (args.policy_factory or args.policy_config or args.obstacle_binding):
            raise ValueError('ylj scene/trajectory already freezes its controller and obstacle; do not overlay a second policy')
        if args.episode_specs:raise ValueError('Choose native task contract or legacy episode specs')
        base.mkdir(parents=True,exist_ok=False);policy=read(args.policy)
        if policy['capture_mode']!='raw-only':raise ValueError('Native safety collection is raw-only')
        required=dict(physics_hz=120,observation_hz=30,decision_hz=30,mesh_distance_backend='gdist',
            observer_config='configs/safety_observer_arena.json')
        for key,value in required.items():
            if policy.get(key)!=value:raise ValueError('Unsupported native workflow policy override: '+key)
        if 'IR' not in policy.get('disabled_categories',[]):raise ValueError('Native IR must remain disabled')
        if args.max_actions<=0:raise ValueError('Native capture needs a positive action limit')
        write(base/'workflow_policy.json',policy)
        binding=dict(version='robolaw-native-workflow-1',task_contract=str(args.task_contract.resolve()),
            task_sha256=hashlib.sha256(args.task_contract.read_bytes()).hexdigest(),
            asset_cache=str(args.asset_cache.resolve()),policy_factory=args.policy_factory,ylj_scene=ylj_scene)
        write(base/'native_workflow.json',binding)
        command=[sys.executable,str(ROOT/'scripts/collect_native_task.py'),'--task-contract',str(args.task_contract.resolve()),
            '--output',str(base/'capture/native_000'),'--asset-cache',binding['asset_cache'],
            '--policy-factory',args.policy_factory,'--max-actions',str(args.max_actions),'--device','cpu','--num_envs','1']
        if args.policy_config:command+=['--policy-config',str(args.policy_config.resolve())]
        command+=['--seed',str(args.seed)]
        if args.placement_seed is not None:command+=['--placement_seed',str(args.placement_seed)]
        if args.obstacle_binding:command+=['--obstacle-binding',str(args.obstacle_binding.resolve())]
        if ylj_scene:
            command=[sys.executable,str(ROOT/'scripts/collect_ylj_risk_task.py'),
                '--task-contract',str(args.task_contract.resolve()),'--scene',ylj_scene,
                '--output',str(base/'capture/native_000'),'--asset-cache',binding['asset_cache'],
                '--max-actions',str(args.max_actions),'--seed',str(args.seed),'--device','cpu','--num_envs','1']
    else:
        binding=read(base/'native_workflow.json');episode=base/'capture/native_000/raw_episode'
        risk=base/'risk/native_000';render=base/'rendered/native_000'
        if stage=='evaluate':
            risk.parent.mkdir(exist_ok=True)
            command=[sys.executable,str(ROOT/'scripts/evaluate_native_task.py'),'--episode',str(episode),'--output',str(risk)]
            if args.geometry_backend!='frozen':raise ValueError('Native migration requires a separately frozen task/rules/build binding; no implicit backend conversion')
        elif stage=='render':
            render.parent.mkdir(exist_ok=True)
            command=[sys.executable,str(ROOT/'scripts/render_native_task.py'),'--episode',str(episode),'--risk',str(risk),
                '--output',str(render),'--asset-cache',binding['asset_cache'],'--device','cpu','--num_envs','1']
            if camera_config:command+=['--camera-config',str(camera_config.resolve())]
            if appearance_config:
                command+=['--appearance-config',str(appearance_config.resolve()),'--appearance-seed',str(appearance_seed)]
        elif stage=='audit':
            (base/'audit').mkdir(exist_ok=True)
            command=[sys.executable,str(ROOT/'scripts/audit_native_task.py'),'--episode',str(episode),
                '--output',str(base/'audit/native_000.json')]
        elif stage=='video':
            started=time.time_ns();tick=time.perf_counter()
            report=read(render/'render_report.json');raw=read(episode/'raw_capture_summary.json')
            if report.get('preview_only') or report.get('mode')!='offline_measured_USD_pose_render':
                raise ValueError('Selected-frame previews cannot be exported as full aligned videos')
            if not report['complete'] or report['source_raw_sha256']!=raw['raw_writer']['sha256'] or report['physics_steps_advanced']!=0:
                raise ValueError('Video requires complete source-time RGB alignment with zero physics advancement')
            attachments=[]
            if report.get('appearance'):
                appearance=report['appearance']
                if appearance.get('report_file')!='appearance_report.json':
                    raise ValueError('Unexpected appearance report filename')
                path=render/'appearance_report.json'
                if hashlib.sha256(path.read_bytes()).hexdigest()!=appearance.get('report_sha256'):
                    raise ValueError('Appearance report is missing or changed; rerender to a new output')
                actual=read(path)
                if (actual.get('source_raw_sha256')!=report['source_raw_sha256'] or
                    actual.get('source_reconstruction_sha256')!=report['source_reconstruction_sha256']):
                    raise ValueError('Appearance report belongs to another source')
                attachments.append(path)
            camera=render/'camera_config.json'
            if report.get('camera_config_sha256'):
                if hashlib.sha256(camera.read_bytes()).hexdigest()!=report['camera_config_sha256']:
                    raise ValueError('Camera report is missing or changed')
                attachments.append(camera)
            target=base/'video/native_000';target.mkdir(parents=True,exist_ok=False)
            shutil.copy2(render/'risk_review.mp4',target/'risk_review.mp4');write(target/'render_report.json',report)
            for path in attachments:shutil.copy2(path,target/path.name)
            record_stage(started,tick,0,operation='copy_aligned_offline_video',
                source_raw_sha256=report['source_raw_sha256'],
                source=str(render/'risk_review.mp4'),output=str(target/'risk_review.mp4'))
            return 0
        else:raise ValueError('Unsupported native stage')
    logs=base/'workflow_logs';logs.mkdir(exist_ok=True)
    started=time.time_ns();tick=time.perf_counter()
    with (logs/(stage+'_'+str(started)+'.log')).open('x') as log:
        result=subprocess.run(command,cwd=ROOT,env=os.environ|{'PYTHONPATH':str(ROOT)},stdout=log,stderr=subprocess.STDOUT)
    record_stage(started,tick,result.returncode,command=command)
    return result.returncode
