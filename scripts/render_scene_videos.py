"""Render camera tours of the three standalone scenes, not policy rollouts."""
import argparse
import json
import math
import subprocess
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--variants', nargs='+', default=['human', 'property', 'robot'])
parser.add_argument('--frames', type=int, default=240)
args = parser.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'width': 960, 'height': 720,
                     'renderer': 'RaytracedLighting'})
import omni.usd
import omni.timeline
import omni.replicator.core as rep
import imageio_ffmpeg
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from pxr import Gf, UsdGeom

root = Path('/workspace')
out = root / 'videos'
out.mkdir(exist_ok=True)
layout = json.loads((root / 'configs/layout.json').read_text())
labels = {'human': 'HUMAN SAFETY | Reaching person',
          'property': 'PROPERTY SAFETY | Glass, plate, phone',
          'robot': 'ROBOT SAFETY | Fixed panel and crossbar'}
try:
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 20)
except OSError:
    font = ImageFont.load_default()

for variant in args.variants:
    omni.usd.get_context().open_stage(str(root / 'scenes' / f'{variant}.usda'))
    for _ in range(30): app.update()
    stage = omni.usd.get_context().get_stage()
    camera = UsdGeom.Xformable(stage.GetPrimAtPath('/World/Overview'))
    transform = camera.GetOrderedXformOps()[0]
    product = rep.create.render_product('/World/Overview', (960, 720))
    rgb = rep.AnnotatorRegistry.get_annotator('rgb')
    rgb.attach(product)
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(120): app.update()
    for _ in range(12): rep.orchestrator.step(rt_subframes=4)
    # Keep the settled scene fixed while moving the inspection camera.
    timeline.pause()
    command = [imageio_ffmpeg.get_ffmpeg_exe(), '-y', '-loglevel', 'error',
               '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', '960x720', '-r', '24',
               '-i', '-', '-an', '-c:v', 'libx264', '-crf', '19', '-preset', 'fast',
               '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
               str(out / f'{variant}.mp4')]
    encoder = subprocess.Popen(command, stdin=subprocess.PIPE)
    try:
        for i in range(args.frames):
            t = i / max(args.frames - 1, 1)
            u = min(max((t - .15) / .65, 0), 1)
            u = u*u*(3-2*u)
            start = np.array(layout['camera_eye'])
            end = np.array([2.55, -2.65, 2.05])
            eye = (1-u)*start + u*end
            eye[0] += .18*math.sin(math.pi*t)
            target = Gf.Vec3d(*layout['camera_target'])
            transform.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), target,
                                                Gf.Vec3d(0,0,1)).GetInverse())
            rep.orchestrator.step(rt_subframes=4, delta_time=0.0)
            arr = rgb.get_data()
            if arr.size == 0: raise RuntimeError('Empty render')
            frame = Image.fromarray(arr[:, :, :3]).convert('RGB')
            draw = ImageDraw.Draw(frame)
            draw.rectangle((0,0,960,68), fill=(15,20,27))
            draw.text((18,9), labels[variant], font=font, fill='white')
            draw.text((18,37), 'SCENE PREVIEW | Camera tour | No grasp policy',
                      font=font, fill=(190,200,210))
            encoder.stdin.write(frame.tobytes())
            if i in (0,args.frames//2,args.frames-1):
                frame.save(out / f'{variant}_frame_{i:03d}.jpg')
            if i % 48 == 0: print('VIDEO_PROGRESS',variant,i,args.frames,flush=True)
    finally:
        encoder.stdin.close()
        result = encoder.wait()
    if result: raise RuntimeError(f'ffmpeg exited with {result}')
    rgb.detach(product)
    product.destroy()
    timeline.stop()
    print('VIDEO_COMPLETE',variant,flush=True)
app.close()
