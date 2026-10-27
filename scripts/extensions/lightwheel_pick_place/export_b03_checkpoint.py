"""Export a measured, native-passed first-pick checkpoint and its real action prefix."""
import argparse,json,hashlib
from pathlib import Path
import h5py,numpy as np
p=argparse.ArgumentParser();p.add_argument('--episode',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
r=json.loads((a.episode/'result.json').read_text());c=r['placement_checkpoints']['banana_1'];assert c['passed'];i=int(c['action_index']);a.output.mkdir(parents=True,exist_ok=False)
with h5py.File(a.episode/'trajectory.h5') as h:
 state={'q':h['next_obs/q'][i].tolist(),'dq':h['next_obs/dq'][i].tolist(),'rigid':{n:h['next_obs/'+n][i].tolist() for n in r['initial_object_states']}}
 initial={'q':h['obs/q'][0].tolist(),'rigid':{n:h['obs/'+n][0].tolist() for n in r['initial_object_states']}}
 np.savez_compressed(a.output/'prefix.npz',actions=h['action'][:i+1],phases=h['phase'].asstr()[:i+1].astype(str))
manifest={'kind':'B03 verified first-pick checkpoint; state restoration for development only','source_episode':str(a.episode.resolve()),'source_result_sha256':hashlib.sha256((a.episode/'result.json').read_bytes()).hexdigest(),'trajectory_sha256':hashlib.sha256((a.episode/'trajectory.h5').read_bytes()).hexdigest(),'prefix_sha256':hashlib.sha256((a.output/'prefix.npz').read_bytes()).hexdigest(),'prefix_actions':i+1,'dt':r['dt'],'variant':r['variant'],'variant_config_source':r.get('variant_config_source'),'robot_joint_names':r['robot_joint_names'],'native_first_checkpoint':c,'state':state,'initial_state':initial,'scope':'Includes measured robot joint positions/velocities and all task rigid-body poses/velocities. Does not include hidden PhysX solver/contact warm-start state. Final acceptance must replay prefix from original reset and recheck first native predicate.'}
bounds=r.get('pick_action_ranges',{}).get('banana_2')
if bounds and r.get('extra_lift_contacts') and r['extra_lift_contacts'][0]['both_fingers_on_body'] and r['extra_lift_m'][0]>=.08:
 with h5py.File(a.episode/'trajectory.h5') as h:
  ph=h['phase'].asstr()[:];hit=np.flatnonzero((ph=='lift')&(np.arange(len(ph))>=bounds['start'])&(np.arange(len(ph))<bounds['end']));start=bounds['start'];end=int(hit[-1])+1
  np.savez_compressed(a.output/'second_grasp.npz',actions=h['action'][start:end],phases=ph[start:end].astype(str))
 manifest['second_grasp_prefix']={'file':'second_grasp.npz','sha256':hashlib.sha256((a.output/'second_grasp.npz').read_bytes()).hexdigest(),'start':start,'end':end,'measured_lift_m':r['extra_lift_m'][0],'both_fingers_loaded':True}
(a.output/'checkpoint.json').write_text(json.dumps(manifest,indent=2));print('CHECKPOINT_EXPORTED',a.output,i+1)
