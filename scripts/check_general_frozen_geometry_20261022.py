import json,tempfile,time
from pathlib import Path
import h5py,numpy as np
from kitchen_safety.frozen_geometry import load_geometry,run_frame
from scripts.benchmark_warp_frame_batch import load_geometry as old_load,run_frame as old_frame,features
root=Path('/workspace');ep=root/'datasets/diagnostic/raw_capture_only_full_20261022/env_000';cache=ep/'raw_meshes'
start=time.perf_counter();b,h,p,r=load_geometry(ep,cache);_,oh,op,orr=old_load(ep,cache)
links=json.load(open(ep/'render_state_metadata.json'))['kitchen']['link_paths'];origin=np.asarray(b['environment_origin_world_m'])
rows=[];max_error=0.
with h5py.File(ep/'trajectory.h5') as f:
 for index in [0,573,1146,1719,2291]:
  poses=dict(zip(b['body_names'],np.asarray(f['obs/robot_body_pose_w'][index])))
  kp=np.asarray(f['obs/kitchen_link_pose'][index]);kp[:,:3]+=origin
  objects=dict(zip(links,kp));objects.update(mustard_bottle=np.asarray(f['obs/mustard'][index]),bowl=np.asarray(f['obs/bowl'][index]))
  a=run_frame(h,p,r,poses,objects);old=old_frame(oh,op,orr,poses,objects,True)
  x,c=features(a);y,d=features(old);assert x.keys()==y.keys() and c==d
  error=max(abs(x[k]-y[k]) for k in x);max_error=max(max_error,error);assert error<1e-7
  assert a[0]['complete'] and a[1]['complete'] and a[2]['coverage_complete']
  rows.append(dict(frame=index,numeric_features=len(x),maximum_difference_m=error))
 # Explicit synthetic no-human loader branch; not a real scene acceptance run.
 with tempfile.TemporaryDirectory() as tmp:
  binding=json.load(open(ep/'safety_observer_binding.json'))
  bb=binding['binding'];meta=bb['hs_geometry'].copy();meta['geometry']=[e for e in meta['geometry'] if e['group']=='robot'];meta['excluded_pairs']=[]
  bb['rs_authored_geometry']=meta;bb['hs_geometry']={'applicable':False};bb['human_inventory'].update(observed_human_paths=[],verified_absent=True)
  dest=Path(tmp);(dest/'safety_observer_binding.json').write_text(json.dumps(binding))
  _,hh,pp,rr=load_geometry(dest,cache);assert hh is None
  absent=run_frame(hh,pp,rr,poses,objects)
  assert absent[0] is None and absent[1]['complete'] and absent[2]['coverage_complete']
  assert absent[2]['self_collision_flag_gt']==a[2]['self_collision_flag_gt']
report=dict(passed=True,human_frames=rows,max_numeric_difference_m=max_error,synthetic_nonhuman_geometry_branch=True,scope='Five recorded human poses against the previous validated geometry adapter, plus a synthetic absent-human binding branch; not an end-to-end test of other scenes',wall_s=time.perf_counter()-start)
(root/'reports/general_frozen_geometry_validation_20261022.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)
