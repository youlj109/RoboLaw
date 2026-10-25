"""Render frozen door layouts before planning; never a physical success episode.

One scene/app startup for one task and several immutable candidate configurations.
Only the normal source reset advances physics. Subsequent recorded baseline poses
are explicitly assigned for static inspection, without actions or physics steps.
No motion planner or trajectory generator is invoked. The unchanged source
placement setup can invoke its standard cuRobo reachability validator once.
"""
import hashlib
import json
import time
from pathlib import Path

from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext

p = get_isaaclab_arena_cli_parser()
p.add_argument('--manifest', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
a.enable_cameras = True
if a.output.exists():
    raise FileExistsError(a.output)
manifest = json.loads(a.manifest.read_text())
a.output.mkdir(parents=True)
started = time.monotonic()
report = dict(scope='FROZEN LAYOUT / KINEMATIC PREVIEW ONLY. Not physical execution, planning or safety acceptance.',
              manifest=str(a.manifest), manifest_sha256=hashlib.sha256(a.manifest.read_bytes()).hexdigest(),
              trajectory_planning_invoked=False, physics_after_reset=False,
              source_reset_placement_validation='Unchanged source placement setup can invoke cuRobo IK reachability once; no motion trajectory is generated.',
              complete=False, candidates=[])
(a.output/'manifest_snapshot.json').write_text(json.dumps(manifest, indent=2)+'\n')
(a.output/'preview_source_snapshot.py').write_bytes(Path(__file__).read_bytes())

with SimulationAppContext(a):
    import numpy as np
    import torch
    import warp as wp
    from types import SimpleNamespace
    from PIL import Image, ImageDraw
    from pxr import Gf, Usd, UsdGeom
    from scipy.spatial.transform import Rotation
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from isaaclab_physx.renderers.isaac_rtx_renderer_utils import ensure_isaac_rtx_render_update
    from kitchen_safety.extensions.doors.task_environment import build, ROOT
    from kitchen_safety.extensions.doors.review_camera_lwx import add_review_camera, NAME
    from kitchen_safety.extensions.doors.video import DoorCameraWriter
    from kitchen_safety.dataset import numpy as arr

    torch.set_num_threads(4)
    if not hasattr(wp, 'torch'):
        wp.torch = SimpleNamespace(device_from_torch=wp.device_from_torch)
    task = manifest['task_id']
    env_spec, graph = build(task, 'base', True, manifest['base_config'])
    builder = ArenaEnvBuilder(env_spec, ArenaEnvBuilderCfg(
        device='cpu', seed=a.seed or 42, presets='physx', num_envs=1,
        solve_relations=True, resolve_on_reset=False,
        recorder_dataset_export_dir_path=str(a.output/'arena')))
    cfg, kw = builder.compose_manager_cfg()
    cfg.observations.camera_obs = None
    cfg.sim.render_interval = cfg.decimation
    cfg.terminations.success = None
    observer = add_review_camera(cfg, manifest['observer'])
    # The supplementary camera belongs only to this clearly labelled preview.
    if manifest['observer'].get('focal_length'):
        getattr(cfg.scene, NAME).spawn.focal_length = manifest['observer']['focal_length']
    env = builder.make_registered(cfg, kw)
    env.reset()
    e = env.unwrapped
    scene, sim, robot = e.scene, e.sim, e.scene['robot']
    names = list(env_spec.embodiment.camera_config.camera_names())+[NAME]
    physics_step = int(sim.get_physics_step_count())
    report.update(physics_step_count_after_source_reset=physics_step,
                  camera_names=names, observer=observer, task_id=task)
    original_q = robot.data.joint_pos.clone()

    def freeze_guard():
        if int(sim.get_physics_step_count()) != physics_step:
            raise RuntimeError('Physics unexpectedly advanced in frozen layout preview')

    def render(count):
        for _ in range(count):
            sim.render()
            ensure_isaac_rtx_render_update(force=True)
        for name in names:
            scene[name].cfg.update_latest_camera_pose = True
            scene[name].update(0., force_recompute=True)
        freeze_guard()

    # Reuse only the appearance routine, without creating a fake episode/HDF5.
    warm_writer = SimpleNamespace(frames=0, closed=False, scene=scene, names=names,
                                  timings={'appearance_warmup_s':0.},
                                  file=SimpleNamespace(attrs={}))
    warm = DoorCameraWriter.warm_up(warm_writer, sim)
    (a.output/'preview_appearance_warmup.json').write_text(json.dumps(warm, indent=2)+'\n')
    if not warm['passed'] or warm['physics_advanced']:
        raise RuntimeError('Frozen preview appearance did not converge')

    root = UsdGeom.Xform.Define(sim.stage, '/World/LayoutPreviewOnly')
    root.CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)

    def add_reference(parent, name, spec):
        asset = (ROOT/spec['asset']).resolve()
        if not asset.is_file():
            raise FileNotFoundError(asset)
        prim = UsdGeom.Xform.Define(sim.stage, parent+'/'+name)
        if not prim.GetPrim().GetReferences().AddReference(str(asset)):
            raise RuntimeError('USD reference failed: '+str(asset))
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*spec['position']))
        yaw = np.deg2rad(spec.get('yaw_deg', 0.))
        xf.AddOrientOp().Set(Gf.Quatf(float(np.cos(yaw/2)), 0., 0., float(np.sin(yaw/2))))
        return dict(asset=str(asset), sha256=hashlib.sha256(asset.read_bytes()).hexdigest(),
                    position=spec['position'], yaw_deg=spec.get('yaw_deg', 0.))

    def projected(points, camera):
        pos = arr(camera.data.pos_w)[0]
        quat = arr(camera.data.quat_w_ros)[0]
        local = (np.asarray(points)-pos) @ Rotation.from_quat(quat).as_matrix()
        k = arr(camera.data.intrinsic_matrices)[0]
        uv = np.column_stack([k[0,0]*local[:,0]/np.maximum(local[:,2],1e-9)+k[0,2],
                              k[1,1]*local[:,1]/np.maximum(local[:,2],1e-9)+k[1,2]])
        return uv, local[:,2]

    def annotated(raw, camera, paths, label):
        im = raw.copy()
        draw = ImageDraw.Draw(im)
        for points, color, width in paths:
            uv, depth = projected(points, camera)
            for j in range(len(uv)-1):
                if min(depth[j:j+2]) > .05 and np.max(np.abs(uv[j:j+2])) < 10000:
                    draw.line([tuple(uv[j]),tuple(uv[j+1])],fill=color,width=width)
            for j in (0,len(uv)-1):
                u,v = uv[j]
                if depth[j]>.05:
                    draw.ellipse((u-4,v-4,u+4,v+4),fill=color)
        draw.rectangle((0,0,im.width,52), fill='black')
        draw.text((8,5),label,fill='white')
        draw.text((8,21),'STATIC PREVIEW ONLY | blue: original hand route; orange: original forearm route',fill='white')
        draw.text((8,37),'Recorded baseline poses assigned with physics frozen. No planner / no success claim.',fill='white')
        return im

    try:
        for ci, item in enumerate(manifest['candidates']):
            label = item['id']
            directory = a.output/label
            directory.mkdir()
            config = json.loads(Path(item['config']).read_text())
            spec = config[item['variant']]
            previous = sim.stage.GetPrimAtPath('/World/LayoutPreviewOnly/Candidate')
            if previous:
                sim.stage.RemovePrim(previous.GetPath())
            parent = '/World/LayoutPreviewOnly/Candidate'
            UsdGeom.Xform.Define(sim.stage, parent)
            provenance = [add_reference(parent,'Obstacle',spec)]
            if spec.get('support'):
                provenance.append(add_reference(parent,'Support',spec['support']))
            root.MakeVisible()
            ep = Path(item['baseline'])
            metadata = json.loads((ep/'metadata.json').read_text())
            if metadata.get('task_id', metadata.get('task')) not in (task, None):
                raise ValueError('Baseline task mismatch')
            if not np.allclose(arr(robot.data.root_pos_w)[0], metadata['robot_base'],atol=2e-6,rtol=0):
                raise ValueError('Source robot base changed')
            trajectory = json.loads((ep/'trajectory.json').read_text())
            pre = [i for i,row in enumerate(trajectory) if row['phase']=='pregrasp']
            if not pre:
                raise ValueError('Baseline has no pregrasp')
            body_names = metadata['robot_body_names']
            hand_index = body_names.index('base_link')
            fore_index = body_names.index('panda_link5')
            hand = [trajectory[i]['next_obs']['robot_body_pose'][hand_index][:3] for i in pre]
            fore = [trajectory[i]['next_obs']['robot_body_pose'][fore_index][:3] for i in pre]
            # Paths are measured body origins, not swept volumes or collision proof.
            paths = [(hand,(20,140,255),4),(fore,(255,150,20),3)]
            intersections = spec.get('baseline_intersection_actions',[])
            middle = intersections[len(intersections)//2] if intersections else pre[len(pre)//2]
            middle = min(pre[-1], max(pre[0], middle))
            poses = [('initial',pre[0]),('baseline_channel',middle),('pregrasp_goal',pre[-1])]
            candidate = dict(id=label, variant=item['variant'], config=item['config'],
                             config_sha256=hashlib.sha256(Path(item['config']).read_bytes()).hexdigest(),
                             source_assets=provenance, baseline=str(ep),
                             baseline_sha256=hashlib.sha256((ep/'trajectory.json').read_bytes()).hexdigest(),
                             poses=[], visual_review='pending', physical_acceptance='not_run')
            for phase, frame in poses:
                q = robot.data.joint_pos.clone()
                source_q = trajectory[frame]['next_obs']['q']
                for j,name in enumerate(robot.joint_names):
                    q[0,j] = source_q[metadata['robot_joint_names'].index(name)]
                robot.write_joint_state_to_sim(q, torch.zeros_like(q))
                sim.forward()
                render(24)
                panels=[]
                for name in names:
                    rgb = arr(scene[name].data.output['rgb'])[0,:,:,:3].astype(np.uint8)
                    if int(np.ptp(rgb[::8,::8],axis=(0,1)).max()) <= 2:
                        raise RuntimeError('Uniform RGB preview: '+name)
                    raw = Image.fromarray(rgb)
                    raw.save(directory/(phase+'_'+name+'_raw.jpg'),quality=92)
                    overlay = annotated(raw,scene[name],paths,label+' | '+phase+' | baseline frame '+str(frame))
                    overlay.save(directory/(phase+'_'+name+'_paths.jpg'),quality=92)
                    panels.append(overlay.resize((640,360)))
                sheet = Image.new('RGB',(1280,720),'black')
                for j,panel in enumerate(panels):
                    sheet.paste(panel,((j%2)*640,(j//2)*360))
                sheet.save(directory/(phase+'_all_views.jpg'),quality=92)
                candidate['poses'].append(dict(phase=phase,baseline_frame=frame,physics_step_count=int(sim.get_physics_step_count())))
            report['candidates'].append(candidate)
            (a.output/'preview_report.json').write_text(json.dumps(report,indent=2)+'\n')
            print('LAYOUT_PREVIEW',ci+1,len(manifest['candidates']),label,flush=True)
        freeze_guard()
        report.update(complete=True,physics_step_count_final=int(sim.get_physics_step_count()),
                      elapsed_seconds=time.monotonic()-started)
    finally:
        (a.output/'preview_report.json').write_text(json.dumps(report,indent=2)+'\n')
        env.close()
print('LAYOUT_PREVIEW_COMPLETE',report['complete'],flush=True)
