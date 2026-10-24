"""First nine real-time seconds of matched recorded RGB; no physics replay or speed changes."""
import json,subprocess,hashlib
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
import imageio_ffmpeg
from kitchen_safety.dataset import decode_video
r=Path('reports/doors/lwx_C07_obvious_demo_20261025/physical_visual_review13');b=Path('datasets/doors/lwx_C07_release_clearance_retry120_20261024/C07_base_01');e=Path('datasets/doors/lwx_C07_obvious_demo_20261025/C07_robot_pilot13');out=r/'C07_approach_compare_first9s.mp4'
if out.exists():raise FileExistsError(out)
dt=json.loads((e/'metadata.json').read_text())['dt'];font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',24);headers=[]
for i,label in enumerate(['BASELINE - no added obstacle | original speed','NEW - obstacle enabled | original speed']):
 p=r/('approach_label'+str(i)+'.png');im=Image.new('RGB',(960,36),'black');ImageDraw.Draw(im).text((10,4),label,fill='white',font=font);im.save(p);headers.append(p)
cmd=[imageio_ffmpeg.get_ffmpeg_exe(),'-v','error','-i',str(b/'external_camera_2.mp4'),'-i',str(e/'external_camera_2.mp4')]
for p in headers:cmd+=['-loop','1','-framerate',str(1/dt),'-i',str(p)]
cmd+=['-filter_complex','[0:v]scale=960:540[v0];[1:v]scale=960:540[v1];[2:v][v0]vstack=inputs=2:shortest=1[x0];[3:v][v1]vstack=inputs=2:shortest=1[x1];[x0][x1]hstack=inputs=2:shortest=1[out]','-map','[out]','-an','-c:v','libx264','-crf','20','-pix_fmt','yuv420p','-r',str(1/dt),'-frames:v',str(round(9/dt)),str(out)];subprocess.run(cmd,check=True);validation=decode_video(out,round(9/dt),expected_dt=dt);out.with_suffix('.json').write_text(json.dumps({'scope':'First nine seconds only; original real simulation footage and original speed. No unsafe baseline physical replay with an obstacle.','baseline':str(b),'episode':str(e),'camera':'external_camera_2','original_speed':True,'physics_replay':False,'video_validation':validation,'sha256':hashlib.sha256(out.read_bytes()).hexdigest()},indent=2));print(out)
