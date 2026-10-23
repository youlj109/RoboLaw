"""Render native raw obs[t] with zero physics advancement and offline HS/PT/RS.

Uses the frozen flattened scene and measured world poses, not the task policy.
The three overview cameras are explicitly derived offline and saved with this
render; they are not represented as original wrist-camera observations.
"""
import argparse,json,sys,time,traceback,os,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))


def main():
    from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
    from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
    parser=get_isaaclab_arena_cli_parser();parser.add_argument('--episode',type=Path,required=True)
    parser.add_argument('--risk',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--asset-cache',type=Path,required=True)
    parser.add_argument('--camera-config',type=Path,help='Task-bound three-camera offline view configuration')
    parser.add_argument('--appearance-config',type=Path,help='Explicit task-bound visual-only offline augmentation')
    parser.add_argument('--appearance-seed',type=int,help='Required with --appearance-config; actual draws are saved')
    parser.add_argument('--preview-action-indices',type=int,nargs='+',
        help='Explicit source samples for camera/pose review only; not a full aligned video')
    args=parser.parse_args();args.enable_cameras=True
    if (args.appearance_config is None)!=(args.appearance_seed is None):
        raise ValueError('Appearance config and seed must be supplied together')
    args.output.mkdir(parents=True,exist_ok=False)
    for var,suffix in [('WARP_CACHE_PATH','warp_kernels'),('CUDA_CACHE_PATH','cuda_cache'),('TMPDIR','tmp')]:
        folder=args.asset_cache/suffix;folder.mkdir(parents=True,exist_ok=True);os.environ[var]=str(folder)
    raw_summary=json.loads((args.episode/'raw_capture_summary.json').read_text())
    risk_summary=json.loads((args.risk/'evaluation_summary.json').read_text())
    if not risk_summary.get('complete') or risk_summary['raw_sha256']!=raw_summary['raw_writer']['sha256']:
        raise ValueError('Risk result must be complete replay of this exact raw archive')
    preview=args.preview_action_indices is not None
    selected=set(args.preview_action_indices or [])
    if preview and (not selected or len(selected)!=len(args.preview_action_indices) or min(selected)<0 or max(selected)>=risk_summary['frames']):
        raise ValueError('Preview indices must be unique source sample indices within this episode')
    inventory=json.loads((args.episode/'mesh_inventory.json').read_text());scope=json.loads((args.episode/'relation_scope.json').read_text())
    report=dict(complete=False,mode='offline_measured_USD_pose_preview' if preview else 'offline_measured_USD_pose_render',source_episode=str(args.episode.resolve()),
        physics_steps_advanced=0,frames=0,full_task_execution_verified=raw_summary.get('full_task_execution_verified',False),
        source_raw_sha256=raw_summary['raw_writer']['sha256'],preview_only=preview,
        requested_action_indices=sorted(selected) if preview else None,rendered_action_indices=[],source_samples_checked=0,
        render_implementation_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    frozen_scene=args.episode/'reconstruction.usda'
    frozen_scene_sha256=hashlib.sha256(frozen_scene.read_bytes()).hexdigest()
    report['source_reconstruction_sha256']=frozen_scene_sha256
    started=time.perf_counter()
    def save():
        report['wall_s']=time.perf_counter()-started
        (args.output/'render_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    save()
    from contextlib import contextmanager
    @contextmanager
    def receipt():
        try:yield
        except BaseException as exc:report.update(error=repr(exc),traceback=traceback.format_exc());save();raise
    with SimulationAppContext(args) as app,receipt():
        import numpy as np
        from pxr import Usd,UsdGeom,Gf
        from PIL import Image,ImageDraw
        import omni.usd,omni.timeline,omni.physx
        import omni.replicator.core as rep
        import carb
        from kitchen_safety.hs_geometry import transform
        from kitchen_safety.dataset import video_encoder,close_encoders
        context=omni.usd.get_context();context.open_stage(str(args.episode/'reconstruction.usda'))
        for _ in range(5):app.app_launcher.app.update()
        stage=context.get_stage()
        if stage is None:raise ValueError('Frozen reconstruction stage did not open')
        if args.appearance_config:
            from kitchen_safety.task_appearance import apply_offline_appearance
            appearance_config=json.loads(args.appearance_config.read_text())
            appearance=apply_offline_appearance(stage,json.loads((args.episode/'task_contract.json').read_text()),
                appearance_config,args.appearance_seed)
            appearance.update(source_raw_sha256=report['source_raw_sha256'],source_reconstruction_sha256=frozen_scene_sha256)
            appearance_file=args.output/'appearance_report.json'
            appearance_file.write_text(json.dumps(appearance,ensure_ascii=False,indent=2)+'\n')
            report['appearance']=dict(sha256=appearance['appearance_sha256'],dimensions_applied=appearance['dimensions_applied'],
                semantics='visual_only',report_file='appearance_report.json',
                report_sha256=hashlib.sha256(appearance_file.read_bytes()).hexdigest())
        from isaaclab_physx.renderers.isaac_rtx_renderer_utils import ensure_rtx_hydra_engine_attached
        ensure_rtx_hydra_engine_attached()
        if 'rtx' not in context.get_attached_hydra_engine_names():
            raise RuntimeError('RTX Hydra engine did not attach to frozen USD stage')
        timeline=omni.timeline.get_timeline_interface();timeline.pause();timeline.set_auto_update(False);timeline.commit()
        # This standalone USD render has no native PhysX/Fabric render clock.
        # Use the synchronous RTX path, rather than waiting for absent ticks.
        carb.settings.get_settings().set('/rtx/hydra/supportMultiTickRate',False)
        carb.settings.get_settings().set('/rtx/hydra/readTransformsFromFabricInRenderDelegate',False)
        rep.settings.set_render_rtx_realtime(antialiasing='DLAA')
        carb.settings.get_settings().set('/rtx-transient/dldenoiser/enabled',True)
        # Assert no simulator steps, even if a render extension changes timeline state.
        event_count=[0]
        def on_step(dt):event_count[0]+=1
        step_subscription=omni.physx.get_physx_interface().subscribe_physics_step_events(on_step)
        rep.orchestrator.set_capture_on_play(False)
        actors=sorted({e['actor'] for e in inventory['colliders'] if e['dynamic']},key=lambda p:(p.count('/'),p))
        cache=UsdGeom.XformCache();stretches={};ops={}
        for actor in actors:
            prim=stage.GetPrimAtPath(actor)
            if not prim:raise ValueError('Frozen actor missing: '+actor)
            matrix=np.asarray(cache.GetLocalToWorldTransform(prim)).T
            u,_,vt=np.linalg.svd(matrix[:3,:3]);rot=u@vt
            if np.linalg.det(rot)<0:u[:,-1]*=-1;rot=u@vt
            stretches[actor]=rot.T@matrix[:3,:3]
            xf=UsdGeom.Xformable(prim);xf.ClearXformOpOrder()
            name='xformOp:transform:measuredReplay'
            if prim.HasAttribute(name):prim.RemoveProperty(name)
            ops[actor]=xf.AddTransformOp(opSuffix='measuredReplay')
        with (args.episode/'raw_evidence.jsonl').open() as source:
            first=next(json.loads(l) for l in source if json.loads(l)['kind']=='initial_state')['state']
        robot=scope['bindings']['robot_root']
        root=transform(first['articulations'][robot]['root_state_world'])
        forward=root[:3,0].copy();forward[2]=0.;forward/=np.linalg.norm(forward)
        left=np.array([-forward[1],forward[0],0.]);up=np.array([0.,0.,1.])
        basis=np.column_stack((forward,left,up));base=root[:3,3]
        # Task-relative close views stay near the working volume, rather than
        # placing fixed global cameras behind kitchen walls or above ceilings.
        center=base+basis@np.array([.30,0.,.30])
        positions=[base+basis@np.array(v) for v in ((.55,-.70,.70),(.55,.70,.70),(.30,.01,1.15))]
        targets=[center.copy(),center.copy(),center.copy()];focal_lengths=[20.]*3
        camera_mode='derived_offline_overview'
        if args.camera_config:
            from kitchen_safety.task_camera import validate_camera_config
            supplied=validate_camera_config(json.loads(args.camera_config.read_text()),
                json.loads((args.episode/'task_contract.json').read_text()),report['source_raw_sha256'])
            positions=[v['eye'] for v in supplied['cameras']];targets=[v['look_at'] for v in supplied['cameras']]
            focal_lengths=[v['focal_length_mm'] for v in supplied['cameras']]
            camera_mode='explicit_task_bound_offline_overview'
        cameras=[];annotators=[];products=[]
        for i,(eye,target) in enumerate(zip(positions,targets)):
            camera=rep.create.camera(position=tuple(eye),look_at=tuple(target),focal_length=focal_lengths[i],clipping_range=(.01,100.))
            product=rep.create.render_product(camera,(640,480));rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach(product)
            cameras.append(dict(name=['front','side','overhead'][i],eye=list(eye),look_at=list(target),focal_length_mm=focal_lengths[i]))
            products.append(product);annotators.append(rgb)
        (args.output/'camera_config.json').write_text(json.dumps(dict(mode=camera_mode,cameras=cameras),indent=2)+'\n')
        report['camera_config_sha256']=hashlib.sha256((args.output/'camera_config.json').read_bytes()).hexdigest()
        report['visual_acceptance']='requires target visibility and occlusion review; nonblack alone is insufficient'
        baseline_time=timeline.get_current_time()
        def restore(state):
            missing=set(actors)-set(state['actor_states_world'])
            if missing:raise ValueError('Missing source-time dynamic poses: '+str(sorted(missing)))
            for actor in actors:
                world=transform(state['actor_states_world'][actor]);world[:3,:3]=world[:3,:3]@stretches[actor]
                prim=stage.GetPrimAtPath(actor);current=UsdGeom.XformCache()
                parent=np.asarray(current.GetLocalToWorldTransform(prim.GetParent())).T
                local=np.linalg.inv(parent)@world;ops[actor].Set(Gf.Matrix4d(local.T.tolist()))
            check=UsdGeom.XformCache();maximum=0.
            for actor in actors:
                actual=np.asarray(check.GetLocalToWorldTransform(stage.GetPrimAtPath(actor))).T
                expected=transform(state['actor_states_world'][actor]);expected[:3,:3]=expected[:3,:3]@stretches[actor]
                maximum=max(maximum,float(np.abs(actual-expected).max()))
            if maximum>1e-7:raise ValueError('Reconstructed actor transform differs from measured source')
            return maximum
        restore(first)
        def render_frame():
            # Annotator render products are driven directly by Kit, as in the
            # native Isaac RTX camera backend. The Replicator orchestrator's
            # capture scheduler requires a simulation clock in this Kit build.
            # This replay has no clock and must never advance one for RGB.
            carb.settings.get_settings().set('/app/player/playSimulations',False)
            for _ in range(4):app.app_launcher.app.update()
        for _ in range(16):
            render_frame()
            probe=annotators[0].get_data()
            if isinstance(probe,dict):probe=probe.get('data',[])
            if np.asarray(probe).size==0:raise RuntimeError('RTX did not produce a capture; stop instead of repeating empty frames')
        if event_count[0] or timeline.get_current_time()!=baseline_time:raise ValueError('Physics/time advanced during RGB warmup')
        video=args.output/('preview.mp4' if preview else 'risk_review.mp4')
        writer=video_encoder(video,1920,544,1/30)
        max_error=0.;dark=[]
        try:
            with (args.episode/'raw_evidence.jsonl').open() as source,(args.risk/'safety_decisions.jsonl').open() as labels:
                for line in source:
                    packet=json.loads(line)
                    if packet['kind'] not in ('observation','terminal_observation'):continue
                    label=json.loads(next(labels))
                    if label['action_index']!=packet['action_index'] or label['source_physics_step']!=packet['source_physics_step']:
                        raise ValueError('RGB/label source-time identity differs')
                    report['source_samples_checked']+=1
                    if preview and packet['action_index'] not in selected:continue
                    max_error=max(max_error,restore(packet['state']))
                    render_frame()
                    if event_count[0] or timeline.get_current_time()!=baseline_time:raise ValueError('Physics/time advanced during offline rendering')
                    images=[]
                    for i,annotator in enumerate(annotators):
                        data=annotator.get_data()
                        if isinstance(data,dict):data=data['data']
                        rgb=np.asarray(data)
                        if rgb.shape[:2]!=(480,640) or rgb.shape[2] not in (3,4):raise ValueError('Incomplete RGB readback')
                        im=Image.fromarray(rgb[:,:,:3].astype(np.uint8));images.append(im)
                        if float(rgb[:,:,:3].mean())<1:dark.append(dict(frame=packet['action_index'],camera=i))
                    panel=Image.new('RGB',(1920,544),(15,18,24))
                    for i,im in enumerate(images):panel.paste(im,(640*i,0))
                    draw=ImageDraw.Draw(panel)
                    categories=label['report']['categories']
                    caption='  '.join(c+': '+('N/A' if categories[c].get('status')=='not_applicable' else categories[c]['level'] or 'UNKNOWN')+
                        (' (partial)' if categories[c].get('data_complete') is False else '') for c in ('HS','PT','RS'))
                    draw.text((20,494),caption,fill='white',font_size=24)
                    sample='terminal' if packet['kind']=='terminal_observation' else f"obs[{packet['action_index']}]"
                    suffix='SELECTED FRAME PREVIEW; not continuous video' if preview else 'offline replay'
                    draw.text((20,524),f"t={packet['time_s']:.3f}s | {packet['phase']} | measured {sample} | {suffix}",fill='white')
                    writer.stdin.write(np.asarray(panel).tobytes());report['frames']+=1
                    report['rendered_action_indices'].append(packet['action_index'])
                    if report['frames']==1:panel.save(args.output/'first_frame.png')
                    if preview:panel.save(args.output/f"source_frame_{packet['action_index']:06d}.png")
            expected=len(selected) if preview else risk_summary['frames']
            if report['frames']!=expected or report['source_samples_checked']!=risk_summary['frames']:
                raise ValueError('RGB selection or source frame count differs from labels')
        finally:close_encoders({'risk_review':writer})
        if hashlib.sha256(frozen_scene.read_bytes()).hexdigest()!=frozen_scene_sha256:
            raise ValueError('Offline appearance/rendering modified the frozen source scene')
        report.update(complete=not dark,physics_steps_advanced=event_count[0],pose_matrix_max_error=max_error,dark_frames=dark,
            video=str(video),RGB_source='frozen scene with actual observed actor transforms, no simulated trajectory replay')
        save()
        for annotator,product in zip(annotators,products):annotator.detach(product)
        step_subscription=None
        if dark:raise ValueError('Black RGB frames require render repair')

if __name__=='__main__':main()
