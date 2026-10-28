"""Evidence visualization: real videos plus labeled counterfactual clearance.

The baseline video contains no added hazard. Only its recorded geometry is
evaluated against the hazard; this renderer never synthesizes a collision video.
"""
import argparse
import json
from pathlib import Path
import imageio_ffmpeg
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from kitchen_safety.dataset import video_encoder, close_encoders

parser=argparse.ArgumentParser()
parser.add_argument('episode',type=Path)
parser.add_argument('--camera',default='external_camera_2')
args=parser.parse_args()
audit=json.loads((args.episode/'avoidance_audit.json').read_text())
if audit.get('accepted') is not True:
    raise SystemExit('Only accepted avoidance evidence may be rendered as a final comparison')
baseline=Path(audit['baseline_episode'])
destination=args.episode/'comparison.mp4'
if destination.exists():raise FileExistsError(destination)
font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',15)

def frames(path):
    reader=imageio_ffmpeg.read_frames(str(path),pix_fmt='rgb24');metadata=next(reader)
    try:return [Image.frombytes('RGB',metadata['size'],frame).resize((640,360)) for frame in reader]
    finally:reader.close()

left=frames(baseline/(args.camera+'.mp4'));right=frames(args.episode/(args.camera+'.mp4'))
def series(key):
    return [row['distance_m']*1000 for row in audit[key]['measurements'] if row['state']=='after']
bd=series('baseline_counterfactual');ad=series('augmented_actual')
assert len(left)==len(bd) and len(right)==len(ad)
threshold=audit['augmented_actual']['required_m']*1000
total=max(len(left),len(right));dt=json.loads((args.episode/'result.json').read_text())['dt']
def plot(draw, values, colour, x0, index):
    upper=max(100.,threshold*3)
    y0,y1=490,580
    draw.rectangle((x0,y0,x0+600,y1),outline='#8e98a6')
    yt=y1-threshold/upper*(y1-y0)
    draw.line((x0,yt,x0+600,yt),fill='#facc15',width=2)
    pts=[(x0+i/max(1,total-1)*600,y1-min(value,upper)/upper*(y1-y0)) for i,value in enumerate(values)]
    draw.line(pts,fill=colour,width=2)
    i=min(index,len(values)-1);px,py=pts[i]
    draw.ellipse((px-4,py-4,px+4,py+4),fill='white')
    draw.text((x0,y1+5),f'Clearance to new hazard: {values[i]:.1f} mm | required {threshold:.0f} mm',font=small,fill='white')

encoder=video_encoder(destination,1280,620,dt)
try:
    for index in range(total):
        image=Image.new('RGB',(1280,620),'#152133');draw=ImageDraw.Draw(image)
        draw.text((20,12),'BASELINE: actual recording without the added hazard',font=font,fill='white')
        draw.text((660,12),'REPLANNED: actual recording with the added hazard',font=font,fill='white')
        image.paste(left[min(index,len(left)-1)],(0,48));image.paste(right[min(index,len(right)-1)],(640,48))
        draw.text((20,416),f't={min(index,len(left)-1)*dt:.2f}s'+(' (finished, held frame)' if index>=len(left) else ''),font=small,fill='white')
        draw.text((660,416),f't={min(index,len(right)-1)*dt:.2f}s'+(' (finished, held frame)' if index>=len(right) else ''),font=small,fill='white')
        draw.text((20,445),'Red: baseline evaluated against new hazard (geometry only)',font=small,fill='#ff9c9c')
        draw.text((660,445),'Green: measured new trajectory; no speed scaling',font=small,fill='#90e6b0')
        plot(draw,bd,'#ff7373',20,index);plot(draw,ad,'#5bdc8d',660,index)
        encoder.stdin.write(image.tobytes())
finally:close_encoders({'comparison':encoder})
from kitchen_safety.dataset import decode_video
validation=decode_video(destination,total,expected_dt=dt)
(args.episode/'comparison_validation.json').write_text(json.dumps(dict(video=validation,baseline_video=str(baseline/(args.camera+'.mp4')),actual_video=str(args.episode/(args.camera+'.mp4')),frames=total,dt=dt,scope='Original timing, same camera; shorter recording holds final frame. Red curve is CPU counterfactual, never a physically executed collision.'),indent=2))
print(destination,validation)
