"""Frozen layout review only: no relation solver, IK, cuRobo, actions or collection."""
import argparse
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
manifest = json.loads(a.manifest.read_text())
if a.output.exists():
    raise FileExistsError(a.output)
a.output.mkdir(parents=True)
report = dict(scope='STATIC SCENE DESIGN. Await human approval before planning.',
              curobo_invoked=False, ik_invoked=False, solve_relations=False,
              actions_executed=0, physical_collection=False, complete=False,
              manifest=str(a.manifest), candidates=[])
(a.output/'manifest_snapshot.json').write_text(json.dumps(manifest, indent=2))
(a.output/'preview_source_snapshot.py').write_bytes(Path(__file__).read_bytes())

with SimulationAppContext(a):
    import copy
    import numpy as np
    import torch
    import warp as wp
    from types import SimpleNamespace
    from PIL import Image, ImageDraw
    from pxr import Gf, UsdGeom
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
    spec, _ = build(manifest['task_id'], 'base', True, manifest['base_config'])
    # Reuse the frozen source robot base. Crucially do NOT run relation placement
    # or its cuRobo reachability validator, even during reset.
    builder = ArenaEnvBuilder(spec, ArenaEnvBuilderCfg(
        device='cpu', seed=42, presets='physx', num_envs=1,
        solve_relations=False, resolve_on_reset=False,
        recorder_dataset_export_dir_path=str(a.output/'arena')))
    cfg, kw = builder.compose_manager_cfg()
    cfg.observations.camera_obs = None
    cfg.sim.render_interval = cfg.decimation
    cfg.terminations.success = None
    add_review_camera(cfg, manifest['observer'])
    getattr(cfg.scene, NAME).spawn.focal_length = manifest['observer'].get('focal_length', 2.8)
    if manifest.get('second_observer'):
        original = getattr(cfg.scene, NAME)
        add_review_camera(cfg, manifest['second_observer'])
        second = getattr(cfg.scene, NAME)
        second.prim_path = '{ENV_REGEX_NS}/LayoutSideCamera'
        second.spawn.focal_length = manifest['second_observer'].get('focal_length', 2.8)
        cfg.scene.layout_side_camera = second
        setattr(cfg.scene, NAME, original)
    env = builder.make_registered(cfg, kw)
    env.reset()
    e = env.unwrapped
    scene, sim, robot = e.scene, e.sim, e.scene['robot']
    names = list(spec.embodiment.camera_config.camera_names())+[NAME]
    if manifest.get('second_observer'):
        names.append('layout_side_camera')
    count_after_reset = int(sim.get_physics_step_count())
    metadata = json.loads((Path(manifest['baseline'])/'metadata.json').read_text())
    if not np.allclose(arr(robot.data.root_pos_w)[0], manifest.get('proposed_robot_base_world_m', metadata['robot_base']), atol=2e-6, rtol=0):
        raise RuntimeError('Frozen source robot base mismatch')
    q = robot.data.joint_pos.clone()
    source_q = manifest.get('preview_initial_q', metadata['initial_q'])
    for j, name in enumerate(robot.joint_names):
        q[0, j] = source_q[metadata['robot_joint_names'].index(name)]
    robot.write_joint_state_to_sim(q, torch.zeros_like(q))
    sim.forward()

    def render(n):
        for _ in range(n):
            sim.render()
            ensure_isaac_rtx_render_update(force=True)
        for name in names:
            scene[name].cfg.update_latest_camera_pose = True
            scene[name].update(0., force_recompute=True)
        if int(sim.get_physics_step_count()) != count_after_reset:
            raise RuntimeError('Physics advanced in frozen preview')

    warm_writer = SimpleNamespace(frames=0, closed=False, scene=scene, names=names,
                                 timings={'appearance_warmup_s': 0.}, file=SimpleNamespace(attrs={}))
    warm = DoorCameraWriter.warm_up(warm_writer, sim)
    if not warm['passed'] or warm['physics_advanced']:
        raise RuntimeError('Static appearance warmup failed')
    report['appearance_warmup'] = warm

    def reference(parent, name, item):
        asset = (ROOT/item['asset']).resolve()
        prim = UsdGeom.Xform.Define(sim.stage, parent+'/'+name)
        if not prim.GetPrim().GetReferences().AddReference(str(asset)):
            raise RuntimeError('Cannot reference '+str(asset))
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*item['position']))
        xf.AddRotateZOp().Set(float(item.get('yaw_deg', 0.)))
        return dict(asset=str(asset), sha256=hashlib.sha256(asset.read_bytes()).hexdigest(),
                    position=item['position'], yaw_deg=item.get('yaw_deg', 0.))

    def project(points, camera):
        pos = arr(camera.data.pos_w)[0]
        quat = arr(camera.data.quat_w_ros)[0]
        local = (np.asarray(points)-pos) @ Rotation.from_quat(quat).as_matrix()
        k = arr(camera.data.intrinsic_matrices)[0]
        uv = np.column_stack([k[0, 0]*local[:, 0]/np.maximum(local[:, 2], 1e-9)+k[0, 2],
                              k[1, 1]*local[:, 1]/np.maximum(local[:, 2], 1e-9)+k[1, 2]])
        return uv, local[:, 2]

    def annotate(raw, camera, label):
        im = raw.copy()
        draw = ImageDraw.Draw(im)
        # These are design guides, not computed robot trajectories.
        pts = [manifest['start_tcp_world_m'], manifest['old_handle_world_m'], manifest['new_handle_world_m']]
        uv, depth = project(pts, camera)
        if min(depth[:2]) > .03:
            draw.line([tuple(uv[0]), tuple(uv[1])], fill=(255, 160, 30), width=4)
        for i, color, text in [(1, (255, 160, 30), 'old handle'), (2, (20, 255, 90), 'proposed legal handle')]:
            x, y = uv[i]
            if depth[i] > .03 and 0 <= x < im.width and 0 <= y < im.height:
                draw.ellipse((x-7, y-7, x+7, y+7), outline=color, width=3)
                draw.text((x+9, y-16), text, fill=color)
        draw.rectangle((0, 0, im.width, 48), fill='black')
        draw.text((8, 5), label+' | STATIC DESIGN - NOT PLANNED', fill='white')
        draw.text((8, 23), 'Orange: direct design reference; green: proposed grasp. Await user approval.', fill='white')
        return im

    try:
        for item in manifest['candidates']:
            label = item['id']
            directory = a.output/label
            directory.mkdir()
            previous = sim.stage.GetPrimAtPath('/World/LayoutDesignOnly')
            if previous:
                sim.stage.RemovePrim(previous.GetPath())
            UsdGeom.Xform.Define(sim.stage, '/World/LayoutDesignOnly')
            config = json.loads(Path(item['config']).read_text())
            obstacle = config[item['variant']]
            provenance = [reference('/World/LayoutDesignOnly', 'Obstacle', obstacle)]
            if obstacle.get('support'):
                provenance.append(reference('/World/LayoutDesignOnly', 'Support', obstacle['support']))
            render(45)
            panels = []
            blank = []
            for name in names:
                rgb = arr(scene[name].data.output['rgb'])[0, :, :, :3].astype(np.uint8)
                if int(np.ptp(rgb[::8, ::8], axis=(0, 1)).max()) <= 2:
                    blank.append(name)
                raw = Image.fromarray(rgb)
                raw.save(directory/(name+'_raw.jpg'), quality=94)
                overlay = annotate(raw, scene[name], label+' | '+name)
                overlay.save(directory/(name+'_design.jpg'), quality=94)
                panels.append((name, overlay))
            overview = dict(panels)[NAME]
            side = dict(panels).get('layout_side_camera', dict(panels)['external_camera_2'])
            sheet = Image.new('RGB', (1920, 1080), 'black')
            sheet.paste(overview.resize((960, 540)), (0, 0))
            sheet.paste(side.resize((960, 540)), (960, 0))
            sheet.paste(dict(panels)['external_camera'].resize((960, 540)), (0, 540))
            sheet.paste(dict(panels)['external_camera_2'].resize((960, 540)), (960, 540))
            sheet.save(directory/'scene_review.jpg', quality=94)
            report['candidates'].append(dict(id=label, config=item['config'], variant=item['variant'],
                config_sha256=hashlib.sha256(Path(item['config']).read_bytes()).hexdigest(),
                source_assets=provenance, blank_views=blank, user_approval='pending', planning='not_run'))
            (a.output/'preview_report.json').write_text(json.dumps(report, indent=2))
            print('SCENE_READY', label, flush=True)
        report.update(complete=True, physics_steps_after_reset=int(sim.get_physics_step_count())-count_after_reset,
                      frozen_robot_base_world_m=arr(robot.data.root_pos_w)[0].tolist())
    finally:
        (a.output/'preview_report.json').write_text(json.dumps(report, indent=2))
        env.close()
print('SCENE_DESIGN_COMPLETE', report['complete'], flush=True)
