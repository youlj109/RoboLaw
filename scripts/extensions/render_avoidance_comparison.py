"""Pair two real recordings; never fabricate a hazardous baseline execution."""
import argparse
import hashlib
import json
from pathlib import Path
import imageio_ffmpeg
from PIL import Image, ImageDraw, ImageFont
from kitchen_safety.dataset import decode_video, video_encoder, close_encoders

parser=argparse.ArgumentParser()
parser.add_argument('episode',type=Path)
parser.add_argument('--baseline',type=Path,required=True)
parser.add_argument('--camera',default='external_camera_2')
args=parser.parse_args()
evidence=json.loads((args.episode/'avoidance_audit.json').read_text())
if evidence.get('accepted') is not True:
    raise SystemExit('Refusing final comparison without accepted avoidance evidence')
destination=args.episode/'comparison.mp4'
if destination.exists():raise FileExistsError(destination)
font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',15)
sources=[args.baseline/(args.camera+'.mp4'),args.episode/(args.camera+'.mp4')]
videos=[];rates=[]
for path in sources:
    reader=imageio_ffmpeg.read_frames(str(path),pix_fmt='rgb24')
    metadata=next(reader);rates.append(metadata['fps'])
    try:videos.append([Image.frombytes('RGB',metadata['size'],frame).resize((640,360)) for frame in reader])
    finally:reader.close()
if rates[0]!=rates[1]:raise ValueError('Different source rates require explicit synchronization')
rate=rates[0];total=max(map(len,videos))
encoder=video_encoder(destination,1280,472,1/rate)
try:
    for index in range(total):
        image=Image.new('RGB',(1280,472),'#152133');draw=ImageDraw.Draw(image)
        draw.text((15,12),'BASELINE: actual recording without the added hazard',font=font,fill='white')
        draw.text((655,12),'REPLANNED: actual recording with the added hazard',font=font,fill='white')
        for side,frames in enumerate(videos):
            i=min(index,len(frames)-1);image.paste(frames[i],(side*640,48))
            caption=f't={i/rate:.2f}s'+(' (finished, final frame held)' if index>=len(frames) else '')
            draw.text((side*640+15,414),caption,font=small,fill='white')
        draw.text((15,444),'Original timing. Unsafe baseline is checked in geometry only; no collision was physically replayed.',font=small,fill='#d8e3f0')
        encoder.stdin.write(image.tobytes())
finally:close_encoders({'comparison':encoder})
validation=decode_video(destination,total,expected_dt=1/rate)
(args.episode/'comparison_validation.json').write_text(json.dumps(dict(video=validation,sources=[dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),frames=len(videos[i])) for i,p in enumerate(sources)],frames=total,fps=rate,scope='Real recordings, no time scaling; shorter stream holds its last frame. Baseline has no added entity.'),indent=2))
print(destination,validation)
