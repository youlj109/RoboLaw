"""Record every configured DROID RGB sensor while holding the initial pose."""
import argparse
import json
import subprocess
from pathlib import Path
from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext

parser = get_isaaclab_arena_cli_parser()
parser.add_argument('--variant', choices=['human','property','robot'], required=True)
parser.add_argument('--seconds', type=float, default=10)
parser.add_argument('--human_model', choices=['mpfb','nvidia','blockout'], default='mpfb')
parser.add_argument('--output_tag', default=None)
parser.add_argument('--obstacle_id', default='', help='Optional asset-library entry; keeps the shared task and cameras')
args = parser.parse_args()
args.enable_cameras = True

with SimulationAppContext(args):
    import torch
    import numpy as np
    import imageio_ffmpeg
    from PIL import Image, ImageDraw
    from kitchen_safety.environment import KitchenSafetyCfg, KitchenSafetyEnvironment
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    environment = KitchenSafetyEnvironment().build(KitchenSafetyCfg(variant=args.variant,enable_cameras=True,
                                                                   human_model=args.human_model,
                                                                   obstacle_id=args.obstacle_id))
    names = environment.embodiment.camera_config.camera_names()
    builder = ArenaEnvBuilder(environment,ArenaEnvBuilderCfg(device=args.device,presets='physx',
                               num_envs=1,solve_relations=False,disable_fabric=False))
    env = builder.make_registered()
    env.reset()
    scene = env.unwrapped.scene
    robot = scene['robot']
    ids,_ = robot.find_joints(['panda_joint[1-7]'],preserve_order=True)
    pos = robot.data.joint_pos
    if hasattr(pos,'torch'): pos=pos.torch
    action = torch.zeros((1,8),device=env.unwrapped.device)
    action[:,:7] = pos[:,ids]
    def rigid_states():
        values={}
        for name, obj in scene.rigid_objects.items():
            state=obj.data.root_state_w
            if hasattr(state,'torch'): state=state.torch
            values[name]=state[0,:7].detach().cpu().numpy().tolist()
        return values
    initial_rigid_states=rigid_states()
    for _ in range(30): env.step(action)
    dt = env.unwrapped.step_dt
    fps = 1.0/dt
    frames = round(args.seconds/dt)
    out = Path('/workspace/videos/robot_cameras')/(args.output_tag or args.variant)
    out.mkdir(parents=True,exist_ok=True)
    encoders = {}
    metadata = {'variant':args.variant,'human_model':args.human_model,'mode':'hold_initial_pose_no_policy',
                'obstacle_id':args.obstacle_id,'fps':fps,'frames':frames,'cameras':{},
                'initial_rigid_states':initial_rigid_states}
    if args.obstacle_id:
        from kitchen_safety.obstacle_library import get_asset
        metadata['obstacle']=get_asset(args.obstacle_id)
    def encoder(name, width, height):
        return subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(),'-y','-loglevel','error',
            '-f','rawvideo','-pix_fmt','rgb24','-s',f'{width}x{height}','-r',str(fps),'-i','-',
            '-an','-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p',
            '-movflags','+faststart',str(out/f'{name}.mp4')],stdin=subprocess.PIPE)
    try:
        for i in range(frames):
            with torch.inference_mode(): env.step(action)
            panels=[]
            for name in names:
                camera=scene[name]
                value=camera.data.output['rgb']
                if hasattr(value,'torch'): value=value.torch
                arr=value[0,:,:,:3].detach().cpu().numpy().astype(np.uint8)
                if name not in encoders:
                    h,w=arr.shape[:2]
                    encoders[name]=encoder(name,w,h)
                    def cpu_list(value):
                        if hasattr(value,'torch'): value=value.torch
                        return value[0].detach().cpu().numpy().tolist()
                    metadata['cameras'][name]={'prim_path':camera.cfg.prim_path,'width':w,'height':h,
                        'intrinsics':cpu_list(camera.data.intrinsic_matrices),
                        'offset':{'position':list(camera.cfg.offset.pos),'quaternion_xyzw':list(camera.cfg.offset.rot),
                                  'convention':camera.cfg.offset.convention},
                        'position_world':cpu_list(camera.data.pos_w),
                        'quaternion_world_ros_xyzw':cpu_list(camera.data.quat_w_ros)}
                encoders[name].stdin.write(arr.tobytes())
                if i==frames//2: Image.fromarray(arr).save(out/f'{name}.jpg')
                panel=Image.fromarray(arr).resize((640,360))
                draw=ImageDraw.Draw(panel)
                draw.rectangle((0,0,640,28),fill=(0,0,0))
                draw.text((10,7),f'{args.variant} | {name} | HOLD POSE',fill='white')
                panels.append(np.asarray(panel))
            combined=np.concatenate(panels,axis=1)
            if 'all_cameras' not in encoders:
                encoders['all_cameras']=encoder('all_cameras',combined.shape[1],combined.shape[0])
            encoders['all_cameras'].stdin.write(combined.tobytes())
            if i % 30==0: print('CAMERA_PROGRESS',args.variant,i,frames,flush=True)
    finally:
        for proc in encoders.values(): proc.stdin.close()
        codes=[proc.wait() for proc in encoders.values()]
    if any(codes): raise RuntimeError(f'Video encoding failed: {codes}')
    metadata['final_rigid_states']=rigid_states()
    (out/'metadata.json').write_text(json.dumps(metadata,indent=2))
    print('CAMERA_COMPLETE',args.variant,flush=True)
    env.close()
