"""Compose synchronized original cameras and real observer; no replay or retiming."""
import argparse,json,subprocess,hashlib
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
import imageio_ffmpeg
from kitchen_safety.dataset import decode_video
p=argparse.ArgumentParser(description=__doc__);p.add_argument('episode',type=Path);a=p.parse_args();ep=a.episode
out=ep/'review_all_four_cameras.mp4';report=out.with_suffix('.json')
if out.exists() or report.exists():raise FileExistsError(out)
alignment=json.loads((ep/'supplementary_observer_alignment.json').read_text());m=json.loads((ep/'metadata.json').read_text());n=len(json.loads((ep/'trajectory.json').read_text()));dt=m['dt']
if not alignment.get('same_action_alignment') or alignment['native_frames']!=n or alignment['observer_frames']!=n:raise ValueError('Camera alignment mismatch')
paths=[ep/'external_camera.mp4',ep/'external_camera_2.mp4',ep/'wrist_camera.mp4',ep/'supplementary_observer/visible_review_camera.mp4'];names=['original external camera','original external camera 2','original wrist camera','supplementary real observer'];headers=[];filters=[]
for i,name in enumerate(names):
 h=ep/('review_four_cam_header_'+str(i)+'.png');im=Image.new('RGB',(960,30),'black');draw=ImageDraw.Draw(im)
 try:font=ImageFont.truetype('DejaVuSans.ttf',18)
 except OSError:font=ImageFont.load_default()
 draw.text((8,5),name+' | '+ep.name,fill='white',font=font);im.save(h);headers.append(h)
 filters.append(f'[{i}:v]scale=960:540:force_original_aspect_ratio=decrease,pad=960:540:(ow-iw)/2:(oh-ih)/2,setsar=1[v{i}]')
 filters.append(f'[{i+4}:v][v{i}]vstack=inputs=2:shortest=1[x{i}]')
filters.extend(['[x0][x1]hstack=inputs=2:shortest=1[top]','[x2][x3]hstack=inputs=2:shortest=1[bottom]','[top][bottom]vstack=inputs=2:shortest=1[out]'])
cmd=[imageio_ffmpeg.get_ffmpeg_exe(),'-v','error','-filter_complex_threads','2']
for f in paths:cmd+=['-i',str(f)]
for h in headers:cmd+=['-loop','1','-framerate',str(1/dt),'-i',str(h)]
cmd+=['-filter_complex',';'.join(filters),'-map','[out]','-an','-c:v','libx264','-threads','2','-crf','20','-pix_fmt','yuv420p','-r',str(1/dt),'-frames:v',str(n),str(out)]
subprocess.run(cmd,check=True);validation=decode_video(out,n,expected_dt=dt)
report.write_text(json.dumps(dict(episode=str(ep),sources=[dict(path=str(f),sha256=hashlib.sha256(f.read_bytes()).hexdigest()) for f in paths],same_action_alignment=True,original_speed=True,physics_replay=False,source_pixels='only proportional scaling and headers; no overlays replacing geometry',frames=n,video_validation=validation,sha256=hashlib.sha256(out.read_bytes()).hexdigest()),indent=2)+'\n');print(out,flush=True)
