"""Extract measured RGB frames for human visual review, without inferring acceptance."""
import argparse,json,subprocess
from pathlib import Path
from PIL import Image,ImageDraw
import imageio_ffmpeg
p=argparse.ArgumentParser();p.add_argument('episodes',nargs='+');p.add_argument('--output-dir',type=Path,default=Path('reports/doors/lwx_C02_C03_20261023'));a=p.parse_args();out=a.output_dir;out.mkdir(parents=True,exist_ok=True)
for directory in a.episodes:
 ep=Path(directory);rows=json.loads((ep/'trajectory.json').read_text());dest=out/(ep.name+'_visual.jpg')
 if dest.exists():continue
 indices=[0]
 for phase in ['pregrasp','grasp_handle','pull_door','release']:
  ids=[i for i,r in enumerate(rows) if r['phase']==phase]
  if ids:indices.append(ids[len(ids)//2] if phase in ['pregrasp','pull_door'] else ids[-1])
 indices=sorted(set(indices+[len(rows)-1]));frames=[]
 for idx in indices:
  jpg=out/(ep.name+'_frame'+str(idx)+'.jpg')
  if not jpg.exists():subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-v','error','-i',str(ep/'all_cameras.mp4'),'-vf',f'select=eq(n\\,{idx}),scale=1200:-1','-frames:v','1',str(jpg)],check=True)
  if not jpg.exists():raise RuntimeError('Missing extracted image: '+str(jpg))
  im=Image.open(jpg).convert('RGB');frames.append((idx,im.copy()))
 canvas=Image.new('RGB',(1200,sum(im.height+24 for _,im in frames)),(245,245,245));draw=ImageDraw.Draw(canvas);y=0
 for idx,im in frames:
  draw.text((8,y),ep.name+' actual frame '+str(idx)+' phase '+rows[idx]['phase'],fill='black');canvas.paste(im,(0,y+24));y+=im.height+24
 canvas.save(dest,quality=92);print(dest,flush=True)
