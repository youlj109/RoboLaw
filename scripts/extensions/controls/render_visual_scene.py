"""Render a single controls layout for visual review only; no motion or audit."""
import hashlib
import json
from pathlib import Path
from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext

parser = get_isaaclab_arena_cli_parser()
parser.add_argument('--controls-task', choices=['left', 'right', 'temperature'], required=True)
parser.add_argument('--variant', choices=['human', 'property', 'robot'], required=True)
parser.add_argument('--config', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
args.enable_cameras = True
args.output.mkdir(parents=True, exist_ok=False)
config = json.loads(args.config.read_text())

with SimulationAppContext(args):
    import numpy as np
    from PIL import Image
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from isaaclab_physx.renderers.isaac_rtx_renderer_utils import ensure_isaac_rtx_render_update
    from kitchen_safety.extensions.controls.environment import build_environment
    from kitchen_safety.dataset import numpy as arr

    envdef, _ = build_environment(variant=args.variant, fixed_pose=config['robot_pose_xyzw'],
        trial_config=config, task=args.controls_task,
        repair_oven_racks=config.get('repair_oven_racks', False),
        balance_temperature_knob=config.get('balance_temperature_knob', False))
    builder = ArenaEnvBuilder(envdef, ArenaEnvBuilderCfg(num_envs=1, seed=42,
        device=args.device, presets='physx', solve_relations=False))
    cfg, kwargs = builder.compose_manager_cfg()
    cfg.observations.camera_obs = None
    env = builder.make_registered(cfg, kwargs)
    env.reset()
    e = env.unwrapped

    def capture(camera, filename):
        for _ in range(64):
            e.sim.render()
        ensure_isaac_rtx_render_update(force=True)
        camera.cfg.update_latest_camera_pose = True
        camera.update(0., force_recompute=True)
        rgb = arr(camera.data.output['rgb'])[0, :, :, :3].astype(np.uint8)
        Image.fromarray(rgb).save(args.output / filename, quality=95)

    for name in envdef.embodiment.camera_config.camera_names():
        capture(e.scene[name], name + '.jpg')
    if args.controls_task == 'temperature':
        views = {'wide': ([3.15, -4.95, 2.25], [4.95, -3.35, .90]),
                 'side': ([4.35, -2.15, 1.75], [5.15, -3.35, 1.05])}
    else:
        views = {'wide': ([1.05, -2.65, 2.15], [2.55, -.55, .90]),
                 'side': ([3.55, -1.85, 1.95], [2.40, -.40, 1.10])}
        if args.variant == 'human':
            views['body'] = ([4.20, -2.70, 1.95], [2.65, -.70, .80])
    views.update(config.get('visual_review_views', {}))
    camera = e.scene['external_camera']
    for name, (eye, target) in views.items():
        camera.set_world_poses_from_view([eye], [target])
        capture(camera, name + '.jpg')
    metadata = {
        'scope': 'Visual-only static render. No planning, collision/distance audit, '
                 'success evaluation, physics stability test or data collection.',
        'accepted_for_collection': False, 'task': args.controls_task, 'variant': args.variant,
        'config': str(args.config),
        'config_sha256': hashlib.sha256(args.config.read_bytes()).hexdigest(),
        'layout': config['variants'][args.variant], 'review_camera_only': views,
        'note': 'Temporary review views; original task camera configs are unchanged.',
    }
    (args.output / 'render_metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print('VISUAL_RENDER_COMPLETE', str(args.output), flush=True)
    env.close()
