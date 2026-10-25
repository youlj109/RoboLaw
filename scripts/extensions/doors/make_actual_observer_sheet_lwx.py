import argparse,json,imageio_ffmpeg
from pathlib import Path
from PIL import Image,ImageDraw
p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('output',type=Path);a=p.parse_args();assert not a.output.exists();rows=json.loads((a.episode/'trajectory.json').read_text());pre=[i for i,r in enumerate(rows) if r['phase']=='pregrasp'];pull=[i for i,r in enumerate(rows) if r['phase']=='pull_door'];indices=[pre[int(f*(len(pre)-1))] for f in [0,.35,.7,1]]+([pull[len(pull)//2],pull[-1]] if pull else [len(rows)//2,len(rows)-1]);frames={};reader=imageio_ffmpeg.read_frames(str(a.episode/'supplementary_observer/visible_review_camera.mp4'),pix_fmt='rgb24');meta=next(reader);size=meta['size']
for i,raw in enumerate(reader):
 if i in indices:
  im=Image.frombytes('RGB',size,raw).resize((640,360));draw=ImageDraw.Draw(im);draw.rectangle((0,0,640,26),fill='black');draw.text((8,6),rows[i]['phase']+' | frame '+str(i),fill='white');frames[i]=im
 if i>=max(indices):break
reader.close();canvas=Image.new('RGB',(1920,720))
for j,i in enumerate(indices):
 if i not in frames:raise RuntimeError('Frame unavailable '+str(i))
 canvas.paste(frames[i],((j%3)*640,(j//3)*360))
canvas.save(a.output,quality=92);a.output.with_suffix('.json').write_text(json.dumps({'episode':str(a.episode),'frame_indices':indices,'source':'actual synchronized observer video','scope':'Sampled visual aid only; not complete continuous review'},indent=2))
