"""Verify native BVTT capacity handling against exhaustive triangle pairs.
Run with a separately built small-capacity candidate, never the installed backend.
"""
import argparse,importlib.util,json,time
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--binary',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
assert not a.output.exists()
spec=importlib.util.spec_from_file_location('gdist_native',a.binary);n=importlib.util.module_from_spec(spec);spec.loader.exec_module(n)
cap=n.capacity_stats()['capacity'];assert cap==256,'This stress check requires isolated GD_CAP=256 build'
rng=np.random.default_rng(72317);identity=np.r_[np.eye(3).reshape(-1),np.zeros(3)].astype(np.float32)
tri=np.array([[[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]]],dtype=np.float32)
cases=[]
for na,nb in [(31,129),(3,1025),(63,257),(7,65),(1,17)]:
 for gap in [0.,.125,2.]:
  aa=np.repeat(tri,na,axis=0);bb=np.repeat(tri,nb,axis=0);bb[:,:,2]+=gap
  cases.append((f'planes_{na}_{nb}_{gap}',aa,bb,identity.copy(),gap))
for i,(na,nb) in enumerate([(7,35),(17,67),(31,129),(3,257),(63,257)]):
 aa=rng.normal(size=(na,3,3)).astype(np.float32);bb=rng.normal(size=(nb,3,3)).astype(np.float32)
 pose=identity.copy();angle=.37;pose[:9]=np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]]).reshape(-1);pose[9:]=[.7,-.4,3.+i*.2]
 cases.append((f'random_pose_{i}',aa,bb,pose,None))
results=[];models=[];poses=[]
for name,aa,bb,pose,expected in cases:
 # Each tile fits in the direct TriDistPQP product buffer. This path has no BVTT,
 # no pruning, and enumerates the complete original pair set across all tiles.
 begin=time.perf_counter();ref=float('inf')
 for i in range(0,len(aa),32):
  for j in range(0,len(bb),32):
   ac,bc=aa[i:i+32],bb[j:j+32];at=pose
   if len(ac)>len(bc):
    r=pose[:9].reshape(3,3);at=np.r_[r.T.reshape(-1),-r.T@pose[9:]].astype(np.float32);ac,bc=bc,ac
   ref=min(ref,n.query(n.Mesh(ac),n.Mesh(bc),at,65536,8,1024)[0])
 ma,mb=n.Mesh(aa),n.Mesh(bb);models.append((ma,mb));poses.append(pose)
 serial=n.query(ma,mb,pose,65536,8,0)[0]
 parallel=float(n.batch_parallel([ma],[mb],np.array([pose]),65536,8,0,1,64)[0][0])
 assert np.isclose(serial,ref,atol=3e-5,rtol=3e-5),(name,serial,ref)
 assert np.isclose(parallel,ref,atol=3e-5,rtol=3e-5),(name,parallel,ref)
 if expected is not None:assert np.isclose(ref,expected,atol=3e-5,rtol=3e-5),(name,ref,expected)
 results.append(dict(case=name,triangles=[len(aa),len(bb)],exhaustive=ref,serial=serial,parallel=parallel,wall_s=time.perf_counter()-begin))
# Multiple waves and repeated calls expose stale queue/stream state.
for concurrency in (1,4,7):
 for repeat in range(2):
  values=n.batch_parallel([x[0] for x in models],[x[1] for x in models],np.array(poses),65536,8,0,concurrency,64)[0]
  assert np.allclose(values,[x['exhaustive'] for x in results],atol=3e-5,rtol=3e-5),(concurrency,repeat,values)
stats=n.capacity_stats();assert stats['exact_frontier_closures']>0;assert stats['asymmetric_depth_closures']>0
out=dict(passed=True,cases=results,capacity_stats=stats,parallel_concurrency=[1,4,7],repeats=2,scope='candidate tree scheduling only; triangle primitive shared by exhaustive reference; real-scene acceptance separate')
a.output.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out),flush=True)
