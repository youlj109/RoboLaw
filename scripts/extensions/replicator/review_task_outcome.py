"""Independent raw contact/state extraction for one recorded physical grasp/place."""
import argparse,hashlib,io,json
from pathlib import Path
import h5py,numpy as np
from PIL import Image,ImageDraw
p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);r=json.loads((a.episode/'result.json').read_text());contacts=json.loads((a.episode/'grasp_contacts.json').read_text())
with h5py.File(a.episode/'trajectory.h5') as h:
 phase=h['phase'].asstr()[:];obs=h['obs/mustard'][:];nxt=h['next_obs/mustard'][:];q=h['next_obs/q'][:];names=json.loads(h.attrs['joint_names']);fi=names.index('finger_joint');dt=float(h.attrs['dt']);lift=np.flatnonzero(phase=='lift');release=np.flatnonzero(phase=='release');transfer=np.flatnonzero(phase=='above_bowl');settle=np.flatnonzero(phase=='settle');settled=obs[settle[-1],:3];count=len(phase)
 blocks=[]
 for name in dict.fromkeys(phase):
  indices=np.flatnonzero(phase==name);blocks.append(dict(phase=name,frames=len(indices),first=int(indices[0]),last=int(indices[-1]),object_initial_xyz=obs[indices[0],:3].tolist(),object_final_xyz=nxt[indices[-1],:3].tolist()))
 loaded=[]
 for row in contacts:
  flags=[]
  for finger in row['fingers'].values():
   force=np.abs(np.asarray(finger['normal_forces_n']));points=np.asarray(finger['points_bottle_local_m']).reshape(-1,3);mask=force>.01;flags.append(bool(mask.any() and ((points[mask,2]>=-.059025203526020045)&(points[mask,2]<=.019974796473980053)).all()))
  loaded.append(dict(phase=row['phase'],bilateral_body_load=all(flags),source_index=row.get('index',row.get('action_index'))))
 support=np.asarray(r['final_destination_contact_force_matrix_w_n']).reshape(-1,3).sum(0)
 report=dict(episode=str(a.episode),dt=dt,frames=count,source_predicate=bool(r['arena_success']),current_final_on_destination=bool(r['final_on_destination']),lift_height_m=float(nxt[lift[-1],2]-obs[0,2]),release_to_final_seconds=float((count-release[-1]-1)*dt),final_gripper_joint_rad=float(q[-1,fi]),final_object_speed_m_s=float(np.linalg.norm(nxt[-1,7:10])),last_1s_position_diameter_upper_bound_m=float(np.linalg.norm(np.ptp(nxt[-15:,:3],axis=0))),final_destination_support_force_n=support.tolist(),raw_contact_counts={name:dict(recorded_actions=sum(x['phase']==name for x in loaded),bilateral_body_loaded_actions=sum(x['phase']==name and x['bilateral_body_load'] for x in loaded)) for name in ['close','lift','above_bowl','lower','release']},phases=blocks,sha256={name:hashlib.sha256((a.episode/name).read_bytes()).hexdigest() for name in ['trajectory.h5','grasp_contacts.json','result.json']},visual_review='Pending human/model inspection of the generated raw-frame montage; montage generation alone is not visual validation.',limitations='Contact and task-state traces sampled at action rate; do not resolve within-step causality. Continuous physics geometry and actual source task predicate remain separate gates. No simulator weld/pose setter introduced; attachment is planner geometry only.')
 indices=[0,int(lift[-1]),int(transfer[len(transfer)//2]),int(release[-1]),count-1];im=Image.new('RGB',(640*5,388*2),'white');draw=ImageDraw.Draw(im)
 with h5py.File(a.episode/'images.h5') as frames:
  for row,cam in enumerate(['external_camera_2','wrist_camera']):
   for col,index in enumerate(indices):
    pic=Image.open(io.BytesIO(frames[cam+'/jpeg'][index].tobytes())).convert('RGB');pic.thumbnail((640,360));im.paste(pic,(col*640,row*388+28));draw.text((col*640+5,row*388+5),f'{cam} {index} {phase[index]}',fill='black')
 im.save(a.output/'raw_review.jpg',quality=93);report['raw_montage']='raw_review.jpg';report['montage_indices']=indices
 (a.output/'task_review.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k not in ['sha256','phases']},indent=2))
