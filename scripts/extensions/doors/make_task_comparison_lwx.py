"""Original-speed, same-camera baseline/obstacle video with explicit ending hold."""
import argparse,json,subprocess,hashlib
from pathlib import Path
import imageio_ffmpeg
from PIL import Image,ImageDraw,ImageFont
from kitchen_safety.dataset import decode_video
p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('--baseline',required=True,type=Path);p.add_argument('--camera',default='external_camera_2');p.add_argument('--run-label',default='02');a=p.parse_args()
out=a.episode/('baseline_comparison_'+a.camera+'_'+a.run_label+'.mp4');report=out.with_suffix('.json')
if out.exists() or report.exists():raise FileExistsError(out)
paths=[a.baseline,a.episode];meta=[json.loads((ep/'metadata.json').read_text()) for ep in paths];counts=[len(json.loads((ep/'trajectory.json').read_text())) for ep in paths]
if abs(meta[0]['dt']-meta[1]['dt'])>1e-9:raise ValueError('Mismatched action/video periods')
dt=meta[0]['dt'];total=max(counts);filters=[];headers=[]
for i,(ep,n) in enumerate(zip(paths,counts)):
 label=('BASELINE' if i==0 else 'OBSTACLE')+' '+ep.name+' | original speed - shorter ending held'
 header=out.with_name(out.stem+'_label'+str(i)+'.png');im=Image.new('RGB',(960,36),'black');draw=ImageDraw.Draw(im)
 try:font=ImageFont.truetype('DejaVuSans.ttf',18)
 except OSError:font=ImageFont.load_default()
 draw.text((8,8),label,fill='white',font=font);im.save(header);headers.append(header)
 filters.append(f'[{i}:v]scale=960:-2,tpad=stop_mode=clone:stop_duration={(total-n)*dt:.9f}[v{i}]')
 filters.append(f'[{i+2}:v][v{i}]vstack=inputs=2:shortest=1[x{i}]')
filters.append('[x0][x1]hstack=inputs=2:shortest=1[out]')
cmd=[imageio_ffmpeg.get_ffmpeg_exe(),'-v','error','-i',str(paths[0]/(a.camera+'.mp4')),'-i',str(paths[1]/(a.camera+'.mp4'))]
for header in headers:cmd.extend(['-loop','1','-framerate',str(1/dt),'-i',str(header)])
cmd.extend(['-filter_complex',';'.join(filters),'-map','[out]','-an','-c:v','libx264','-crf','20','-pix_fmt','yuv420p','-r',str(1/dt),'-frames:v',str(total),str(out)])
subprocess.run(cmd,check=True);decoded=decode_video(out,total,expected_dt=dt)
report.write_text(json.dumps({'camera':a.camera,'original_speed':True,'same_view':True,'physics_replay':False,'baseline':str(a.baseline),'episode':str(a.episode),'input_frames':counts,'ending_hold_frames':[total-n for n in counts],'video_validation':decoded,'sha256':hashlib.sha256(out.read_bytes()).hexdigest()},indent=2)+'\n');print(out,flush=True)
