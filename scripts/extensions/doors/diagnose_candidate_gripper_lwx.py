"""CPU diagnosis of source gripper versus offline hazard candidates; no acceptance."""
import argparse,json,hashlib,itertools
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from scripts.extensions.doors.physical_hazard_geometry import physical_hazards,physical_robot_shapes
from scripts.extensions.doors.search_avoidance_layouts import clearance
p=argparse.ArgumentParser();p.add_argument('--search',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
s=json.loads(a.search.read_text());base=Path(s['baseline']);m=json.loads((base/'metadata.json').read_text());rows=json.loads((base/'trajectory.json').read_text());config=json.loads(Path(s['search_parameters']['config']).read_text());cs=s['candidates']
if not cs:raise RuntimeError('No candidates')
spec=dict(cs[0]);spec['position']=[0.,0.,0.];spec['yaw_deg']=0.;spec.pop('proposed_support',None)
stage,kin,w,g=reconstruct(m,config,spec);hz=physical_hazards(stage,w,spec['name']);refs=[(n,o,np.asarray(o.getRotation()).copy(),np.asarray(o.getTranslation()).copy(),lo.copy(),hi.copy()) for n,o,lo,hi in hz]
pre=[i for i,r in enumerate(rows) if r['phase']=='pregrasp'];indices=sorted(set([pre[-1],*range(pre[-1],len(rows),12),len(rows)-1]));hands=[]
for i in indices:
 q=np.asarray(rows[i]['next_obs']['q']);f={n:q[m['robot_joint_names'].index(n)] for n in w.GRIPPER_JOINT_NAMES};sh=[x for x in physical_robot_shapes(w.shapes(q[:7],f)) if x[0].startswith('gripper/')]
 if not sh:raise RuntimeError('Missing articulated source gripper')
 hands.append((i,rows[i]['phase'],sh))
out={'scope':'Finite source gripper screening diagnostic only. Baseline measured hand states approximate the same Cartesian policy; no IK feasibility or final acceptance claim.','search':str(a.search),'search_sha256':hashlib.sha256(a.search.read_bytes()).hexdigest(),'margin_m':w.clearances[spec['name']],'sampled_actions':indices,'candidates':[]}
for j,c in enumerate(cs):
 R=Rotation.from_euler('z',c['yaw_deg'],degrees=True).as_matrix();pos=np.asarray(c['position']);moved=[]
 for n,o,rr,tt,lo,hi in refs:
  o.setRotation(R@rr);o.setTranslation(R@tt+pos);corners=np.asarray(list(itertools.product(*zip(lo,hi))));v=corners@R.T+pos;moved.append((n,o,v.min(0),v.max(0)))
 checks=[{'action':i,'phase':phase,**clearance(sh,moved,w.clearances[spec['name']])} for i,phase,sh in hands];bad=[x for x in checks if not x['valid']]
 entry={'candidate':j,'all_sampled_hand_states_clear':not bad,'first_violation':bad[0] if bad else None,'checks':checks};out['candidates'].append(entry)
 print('HAND',j,'clear',not bad,'first',None if not bad else {k:v for k,v in bad[0].items() if k not in ['positions']},flush=True)
a.output.write_text(json.dumps(out,indent=2)+'\n')
