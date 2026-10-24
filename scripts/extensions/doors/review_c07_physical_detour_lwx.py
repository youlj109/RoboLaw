"""Measure actual recorded TCP detours and extract unaltered RGB comparisons; manual review stays pending."""
import argparse,hashlib,json,subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from PIL import Image,ImageDraw,ImageFont
import imageio_ffmpeg
from kitchen_safety.planning import DroidKinematics,transform
from scripts.extensions.doors.matched_scene_lwx import require_matching_scene
p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True);p.add_argument('--episode',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
a.output.mkdir(parents=True);paths=[a.baseline,a.episode];metas=[json.loads((b/'metadata.json').read_text()) for b in paths];rows=[json.loads((b/'trajectory.json').read_text()) for b in paths];cameras=[json.loads((b/'cameras.json').read_text()) for b in paths];require_matching_scene(metas[0],metas[1]['variant_config'],metas[1]);assert metas[0]['controller_parameters']==metas[1]['controller_parameters'];assert cameras[0]==cameras[1]
pre_indices=[[i for i,r in enumerate(rs) if r['phase']=='pregrasp'] for rs in rows];points=[];joints=[]
for m,rs,ids in zip(metas,rows,pre_indices):
 kin=DroidKinematics(m['robot_base'],m['robot_yaw_deg']);qids=[m['robot_joint_names'].index('panda_joint'+str(i)) for i in range(1,8)];q=np.asarray([rs[ids[0]]['obs']['q']]+[rs[i]['next_obs']['q'] for i in ids])[:,qids];joints.append(q);points.append(np.asarray([kin.fk(v)[:3,3] for v in q]))
def curve_distance(points,curve):
 start,end=curve[:-1],curve[1:];delta=end-start;v=points[:,None,:]-start[None,:,:];t=np.clip((v*delta[None,:,:]).sum(-1)/np.maximum((delta*delta).sum(-1),1e-15),0,1);closest=start[None,:,:]+t[:,:,None]*delta[None,:,:];d=np.linalg.norm(points[:,None,:]-closest,axis=-1);index=np.argmin(d,axis=1);return d[np.arange(len(points)),index],closest[np.arange(len(points)),index]
off,on=points;dist,closest=curve_distance(on,off);m=metas[0];base=transform(m['robot_base'],Rotation.from_euler('z',m['robot_yaw_deg'],degrees=True).as_quat())
report={'scope':'Actual recorded joint-state FK during pregrasp and original-camera projection. Source collision audits remain separate. No dangerous baseline was physically replayed with an obstacle.','episode':str(a.episode),'baseline':str(a.baseline),'original_cameras_preserved':True,'same_controller_parameters':True,'visual_review_pending':True,'physics_result_accepted':bool(json.loads((a.episode/'result.json').read_text()).get('accepted')),'baseline_tcp_length_m':float(np.linalg.norm(np.diff(off,axis=0),axis=1).sum()),'new_tcp_length_m':float(np.linalg.norm(np.diff(on,axis=0),axis=1).sum()),'max_actual_tcp_distance_to_baseline_polyline_m':float(dist.max()),'pregrasp_frames':[len(x) for x in pre_indices],'curves_world_m':{'baseline':off.tolist(),'new':on.tolist()},'cameras':{},'input_hashes':{str(b/f):hashlib.sha256((b/f).read_bytes()).hexdigest() for b in paths for f in ['metadata.json','trajectory.json','cameras.json','result.json']}}
report['added_actual_tcp_length_m']=report['new_tcp_length_m']-report['baseline_tcp_length_m']
for name,c in cameras[0].items():
 if name=='wrist_camera':continue
 pose=base@transform(c['offset']['position'],c['offset']['quaternion_xyzw']);k=np.array(c['intrinsics'])
 def project(p):
  q=(p-pose[:3,3])@pose[:3,:3];z=-q[:,2];uv=np.column_stack((k[0,0]*q[:,0]/np.maximum(z,1e-9)+k[0,2],k[1,2]-k[1,1]*q[:,1]/np.maximum(z,1e-9)));inside=(z>.05)&(uv[:,0]>0)&(uv[:,0]<c['width'])&(uv[:,1]>0)&(uv[:,1]<c['height']);return uv,inside
 uoff,ioff=project(off);uon,ion=project(on);dp,_=curve_distance(uon,uoff);report['cameras'][name]={'baseline_visible_fraction':float(ioff.mean()),'new_visible_fraction':float(ion.mean()),'max_actual_projected_distance_to_baseline_polyline_px':float(dp[ion].max()) if ion.any() else None,'baseline_tcp_pixels':uoff.tolist(),'new_tcp_pixels':uon.tolist()}
(a.output/'actual_detour_metrics.json').write_text(json.dumps(report,indent=2)+'\n')
# Each still comes directly from recorded video; only labels and tiling are added.
fontpath='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
try:font=ImageFont.truetype(fontpath,20)
except OSError:font=ImageFont.load_default()
canvas=Image.new('RGB',(2560,800),'#111111');draw=ImageDraw.Draw(canvas)
for row,(b,rs,indices) in enumerate(zip(paths,rows,pre_indices)):
 for col,fraction in enumerate([.0,.33,.66,1.]):
  idx=indices[round(fraction*(len(indices)-1))];out=a.output/('native_'+str(row)+'_'+str(col)+'.jpg');subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-v','error','-ss',str(idx*metas[row]['dt']),'-i',str(b/'external_camera_2.mp4'),'-frames:v','1',str(out)],check=True);im=Image.open(out).convert('RGB');im.thumbnail((640,360));canvas.paste(im,(col*640,row*400+35));draw.text((col*640+8,row*400+8),('BASELINE' if row==0 else 'NEW OBSTACLE')+' frame '+str(idx)+' | '+str(round(idx*metas[row]['dt'],2))+'s',font=font,fill='white')
canvas.save(a.output/'native_pregrasp_comparison.jpg',quality=93);print('ACTUAL_DETOUR',report['max_actual_tcp_distance_to_baseline_polyline_m'],report['added_actual_tcp_length_m'],flush=True)
