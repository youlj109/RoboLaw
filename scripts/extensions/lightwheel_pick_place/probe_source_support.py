"""Recompute a saved rollout's exact lime hull/source-support and finger evidence."""
import argparse,json
from pathlib import Path
import h5py,numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
from kitchen_safety.extensions.lightwheel_pick_place.contacts import LimeGraspContacts
parser=argparse.ArgumentParser();parser.add_argument('episode',type=Path);parser.add_argument('--output',type=Path,required=True);a=parser.parse_args()
ep=a.episode;r=json.loads((ep/'result.json').read_text());b=r['robot_base_pose'];w=CollisionWorld(Usd.Stage.Open(str(ep/'scene.usda')),DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]));contact=LimeGraspContacts.__new__(LimeGraspContacts);contact.records=json.loads((ep/'contacts.json').read_text())
source=r['source_support_contact_model']['surface'];surfaces=[x for x in w.objects if w._matches(x[0],source)];top=max(x[3][2] for x in surfaces)
def minz(p):return float((w.lime_vertices@Rotation.from_quat(p[3:7]).as_matrix().T+p[:3])[:,2].min())
with h5py.File(ep/'trajectory.h5') as f:
 rows=[]
 for i,p in enumerate(f['next_obs/lime'][:,:7]):rows.append({'action':i,'phase':f['phase'][i].decode(),'minimum_z_m':minz(p),'root_pose':p.tolist(),'source_depth_m':float(top-minz(p))})
close=[x for x in rows if x['phase']=='close']
out={'episode':str(ep),'source_surface_top_m':float(top),'initial_source_support':r['source_support_contact_model'],'contact_summary':contact.summarize([-.001,.061],'close',last_actions=5),'initial_minimum_z_m':minz(np.array(r['initial_lime_pose'][:7])),'last_state':rows[-1],'close_last_5':close[-5:],'maximum_source_depth':max(rows,key=lambda x:x['source_depth_m'])}
a.output.write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
