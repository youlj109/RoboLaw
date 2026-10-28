#!/usr/bin/env python3
"""Add a readable Chinese review frame around rendered demo frames and encode H264."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[4]
FFMPEG = ROOT / '.venv-arena/lib/python3.12/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2'
FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frames', required=True)
    parser.add_argument('--timeline', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--title', required=True)
    parser.add_argument('--fps', type=int, default=12)
    args = parser.parse_args()
    files = sorted(Path(args.frames).resolve().glob('*.png'))
    if not files:
        raise ValueError('No rendered PNG frames')
    timeline = json.loads(Path(args.timeline).read_text())
    output = Path(args.output).resolve()
    if output.exists():
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    size = Image.open(files[0]).size
    w, h = size
    top, bottom = 82, 88
    canvas_h = h + top + bottom
    canvas_h += canvas_h % 2
    title_font = ImageFont.truetype(FONT, 26)
    label_font = ImageFont.truetype(FONT, 21)
    small_font = ImageFont.truetype(FONT, 15)
    phases = timeline['phases']
    cmd = [str(FFMPEG), '-hide_banner', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
           '-s', f'{w}x{canvas_h}', '-r', str(args.fps), '-i', 'pipe:0', '-an', '-c:v', 'libx264',
           '-preset', 'medium', '-crf', '19', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(output)]
    process = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    keyframes = []
    try:
        for i, path in enumerate(files):
            frame = Image.open(path).convert('RGB')
            if frame.size != size:
                raise ValueError('Rendered frame size changed')
            t = i / args.fps
            phase = next((p for p in phases if p['start_seconds'] <= t < p['end_seconds']), phases[-1])
            canvas = Image.new('RGB', (w, canvas_h), '#111b2a')
            canvas.paste(frame, (0, top))
            draw = ImageDraw.Draw(canvas)
            draw.text((24, 12), args.title, font=title_font, fill='#ffffff')
            draw.text((24, 49), '场景脚本演示 · 机器人静止 · 非策略执行 / 非物理验收', font=small_font, fill='#b9c7db')
            label = phase.get('label_zh', phase.get('label', ''))
            draw.text((24, top+h+13), label, font=label_font, fill='#e8efff')
            draw.text((24, top+h+47), f'{t:04.1f}s / {len(files)/args.fps:.1f}s    |    '+timeline.get('footer_zh', '保持原任务目标；本轮展示风险条件与场景状态'), font=small_font, fill='#a8b9cf')
            draw.rectangle((0, canvas_h-5, int(w*(i+1)/len(files)), canvas_h), fill='#57c8bf')
            process.stdin.write(canvas.tobytes())
            if any(abs(t-p['start_seconds']-.5) < .5/args.fps for p in phases) or i == len(files)-1:
                name = output.with_name(output.stem + f'_frame_{i:04d}.jpg')
                canvas.save(name, quality=90)
                keyframes.append(str(name.relative_to(ROOT)))
        process.stdin.close()
        if process.wait() != 0:
            raise RuntimeError('Video encoding failed')
    except BaseException:
        process.kill()
        process.wait()
        raise
    # Decode every frame to catch truncated output and record observed duration.
    import cv2
    cap = cv2.VideoCapture(str(output)); decoded = 0
    observed_fps = cap.get(cv2.CAP_PROP_FPS)
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        decoded += 1
        if frame.shape[:2] != (canvas_h, w):
            raise ValueError('Unexpected decoded video size')
    cap.release()
    assert decoded == len(files), (decoded, len(files))
    assert abs(observed_fps-args.fps) < .01
    record = dict(video=str(output.relative_to(ROOT)), sha256=sha(output),
        timeline=str(Path(args.timeline).resolve().relative_to(ROOT)), timeline_sha256=sha(args.timeline),
        rendered_frames=len(files), decoded_frames=decoded, fps=observed_fps,
        duration_seconds=decoded/observed_fps, width=w, height=canvas_h, keyframes=keyframes,
        scope='Scripted scene demonstration; no robot policy, contact physics or task-success acceptance',
        renderer_inputs_sha256={str(p.relative_to(ROOT)): sha(p) for p in files})
    output.with_suffix('.verification.json').write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({k:v for k,v in record.items() if k != 'renderer_inputs_sha256'}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
