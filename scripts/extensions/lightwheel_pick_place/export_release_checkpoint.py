"""Export measured supported pre-release state; never a whole-task success."""
import json,hashlib,argparse
from pathlib import Path
import numpy as np,h5py
p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('output',type=Path);a=p.parse_args();r=json.loads((a.episode/'result.json').read_text());a.output.mkdir(parents=True,exist_ok=False)
assert r['supported_placements']['green_ladle']['supported']
with h5py.File(a.episode/'trajectory.h5') as h:
 phases=h['phase'].asstr()[:];i=int(np.flatnonzero(phases=='release')[0]);state={'q':h['obs/q'][i].tolist(),'dq':h['obs/dq'][i].tolist(),'rigid':{n:h['obs/'+n][i].tolist() for n in r['initial_object_states']}}
 np.savez_compressed(a.output/'prefix.npz',actions=h['action'][:i],phases=phases[:i].astype(str));grip=float(h['action'][i-1,7])
m={'kind':'Measured supported pre-release development checkpoint; NOT native success','source_episode':str(a.episode.resolve()),'source_result_sha256':hashlib.sha256((a.episode/'result.json').read_bytes()).hexdigest(),'prefix_sha256':hashlib.sha256((a.output/'prefix.npz').read_bytes()).hexdigest(),'dt':r['dt'],'variant':r['variant'],'variant_config_source':r['variant_config_source'],'robot_joint_names':r['robot_joint_names'],'state':state,'grip':grip,'prefix_actions':i,'support_evidence':r['supported_placements']['green_ladle'],'scope':'Reset-only restore for suffix development. Final acceptance requires real original-reset prefix replay and fresh native/safety results.'}
(a.output/'checkpoint.json').write_text(json.dumps(m,indent=2));print('EXPORTED',i,grip)
