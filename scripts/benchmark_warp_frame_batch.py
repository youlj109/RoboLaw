"""Measure scalar versus frame-batched Warp on identical recorded real meshes.

No simulator or changed trajectories. Cold compilation excluded; both paths
use the same full triangle math. This is geometry latency, not end-to-end FPS.
"""
import argparse,itertools,json,time,threading,subprocess,os
from pathlib import Path
import h5py
import numpy as np
from kitchen_safety import warp_mesh_backend as backend
from kitchen_safety.warp_batch import RUNNER
from kitchen_safety.fcl_distance import GeometryBatch
from kitchen_safety.hs_geometry import HumanDistanceGeometry
from kitchen_safety.pt_geometry import PropertyDistanceGeometry
from kitchen_safety.rs_evidence import SelfCollisionGeometry

class GpuSampler:
    def __init__(self):
        self.values=[]
        self.process=subprocess.Popen(['nvidia-smi','--query-gpu=utilization.gpu,memory.used,power.draw','--format=csv,noheader,nounits','-lms','100'],stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True)
        def read():
            for line in self.process.stdout:
                try:self.values.append(list(map(float,line.strip().split(','))))
                except ValueError:pass
        self.thread=threading.Thread(target=read,daemon=True);self.thread.start()
    def stop(self):
        self.process.terminate();self.process.wait();self.thread.join(timeout=2)
        if not self.values:return {}
        v=np.asarray(self.values)
        return dict(samples=len(v),device_util_mean_percent=float(v[:,0].mean()),device_util_max_percent=float(v[:,0].max()),device_memory_peak_mib=float(v[:,1].max()),device_power_mean_w=float(v[:,2].mean()),scope='Whole device nvidia-smi samples, including other processes; not kernel occupancy')

def load_geometry(episode,cache):
    b=json.loads((episode/'safety_observer_binding.json').read_text())['binding']
    hs=HumanDistanceGeometry.__new__(HumanDistanceGeometry)
    pt=PropertyDistanceGeometry.__new__(PropertyDistanceGeometry)
    models={}
    for geometry,meta in ((hs,b['hs_geometry']),(pt,b['pt_geometry'])):
        geometry.fcl=backend;geometry.root=b['robot_prim_path'].rsplit('/',1)[0]
        geometry.errors=[];geometry.entries=[];geometry.metadata=meta
        for original in meta['geometry']:
            e=dict(original);key=e['fingerprint']
            if key not in models:
                data=np.load(cache/(key+'.npz'));m=backend.BVHModel()
                m.beginModel();m.addSubModel(data['vertices'],data['faces']);m.endModel();models[key]=m
            m=models[key];e['model']=m;e['object']=backend.CollisionObject(m)
            lo=m.vertices.min(0);hi=m.vertices.max(0)
            e['corners']=np.array(list(itertools.product(*zip(lo,hi))))
            if geometry is pt:
                e['last_pose']=None
                e['key']=(e['actor'].rsplit('/',1)[-1] if e['group']=='robot' else e['actor'][len(geometry.root)+1:]) if e['actor'] else None
            geometry.entries.append(e)
    hs.ee_names=set(hs.metadata['ee_body_names']);hs.fixed_body_names=set(hs.metadata['fixed_body_names'])
    hs.excluded_pairs={(e['resolved_robot_collider'],e['resolved_human_collider']):e
                       for e in hs.metadata['excluded_pairs'] if e['matched']}
    inventory=b['pt_asset_inventory'];pt.assets=inventory['objects'];pt.target=inventory['target_object_ids'][0]
    rs=SelfCollisionGeometry.__new__(SelfCollisionGeometry);rs.geometry=hs
    rs.errors=[];rs.metadata=b['rs_self_geometry'];rs.allowed={tuple(p) for p in rs.metadata['allowed_pairs']}
    rs.owners={}
    for e in hs.entries:
        if e['group']=='robot':rs.owners.setdefault(e['actor'],[]).append(e)
    rs.pairs=[p for p in itertools.combinations(sorted(rs.owners),2) if p not in rs.allowed]
    return b,hs,pt,rs

def features(result):
    hs,pt,rs=result
    out={**hs['features'],**hs['scoped_features']}
    for p in pt['pairs']:
        key='PT/'+p['actor']+'/'+p['object_id'];out[key]=p['distance_m']
        for link,value in p['link_distances_m'].items():out[key+'/'+link]=value
    return out,rs['self_collision_flag_gt']

def evidence_signature(result):
    hs,pt,rs=result
    signature={}
    def save(key,value):
        if value is not None:
            signature[key]=(value['source_collider'],value['destination_collider'],
                            value['mesh_intersection'],value['nearest_points_valid'])
    for scope in ('nearest','raw_nearest'):
        for key,value in hs[scope].items():save('HS/'+scope+'/'+key,value)
    for pair in pt['pairs']:
        prefix='PT/'+pair['actor']+'/'+pair['object_id']
        save(prefix,pair['nearest'])
        for key,value in pair['link_distance_evidence'].items():save(prefix+'/'+key,value)
    signature['RS']=rs['pairs']
    return signature

def run_frame(hs,pt,rs,poses,objects,batched):
    if not batched:return (hs._sample_scalar(poses,objects),pt._sample_scalar(poses,objects),rs._sample_scalar(poses,objects))
    batch=GeometryBatch(backend)
    p=pt.prepare(poses,objects,batch);h=hs.prepare(poses,objects,batch);r=rs.prepare(poses,objects,batch)
    batch.execute()
    return (h(),p(),r())

def main():
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument('--episode',type=Path,required=True);a.add_argument('--cache',type=Path,required=True)
    a.add_argument('--frames',default='0,256,573,1146,1719,2291');a.add_argument('--rounds',type=int,default=2)
    a.add_argument('--output',type=Path,required=True);args=a.parse_args()
    b,hs,pt,rs=load_geometry(args.episode,args.cache)
    metadata=json.loads((args.episode/'render_state_metadata.json').read_text())
    links=metadata['kitchen']['link_paths'];origin=np.asarray(b['environment_origin_world_m'])
    frames=list(map(int,args.frames.split(',')));states=[]
    with h5py.File(args.episode/'trajectory.h5') as f:
        for frame in frames:
            poses=dict(zip(b['body_names'],np.asarray(f['obs/robot_body_pose_w'][frame])))
            kitchen=np.asarray(f['obs/kitchen_link_pose'][frame]);kitchen[:,:3]+=origin
            objects=dict(zip(links,kitchen))
            objects.update(mustard_bottle=np.asarray(f['obs/mustard'][frame])[:7],bowl=np.asarray(f['obs/bowl'][frame])[:7])
            states.append((poses,objects))
    # Compile/capture initial paths outside measured intervals.
    run_frame(hs,pt,rs,*states[0],False)
    run_frame(hs,pt,rs,*states[0],True)
    measurements={};reference={};max_error=0.;equivalent=True
    for mode in ('scalar','batch'):
        backend._CACHE.clear();backend._COLLISION_CACHE.clear()
        elapsed=[];samples=[];before=dict(RUNNER.stats);backend_before=dict(backend.STATS);gpu=GpuSampler()
        for repeat in range(args.rounds):
            for frame,state in zip(frames,states):
                start=time.perf_counter();result=run_frame(hs,pt,rs,*state,mode=='batch');seconds=time.perf_counter()-start
                h,p,r=result
                complete=h['complete'] and p['complete'] and r['coverage_complete']
                if not complete:raise RuntimeError(str((h.get('errors'),p.get('errors'),r.get('errors'))))
                numeric,collision=features(result)
                if mode=='scalar':reference[frame]=(numeric,collision,evidence_signature(result))
                else:
                    expected,expected_collision,signature=reference[frame]
                    if numeric.keys()!=expected.keys() or collision!=expected_collision:equivalent=False
                    if evidence_signature(result)!=signature:equivalent=False
                    for key,value in numeric.items():max_error=max(max_error,abs(value-expected[key]))
                elapsed.append(seconds);samples.append(dict(frame=frame,round=repeat,seconds=seconds))
                print(json.dumps(dict(mode=mode,frame=frame,seconds=round(seconds,6))),flush=True)
        measurements[mode]=dict(mean_s=float(np.mean(elapsed)),median_s=float(np.median(elapsed)),p95_s=float(np.percentile(elapsed,95)),samples=samples,
                                batch_counters={k:RUNNER.stats[k]-before[k] for k in before},scalar_readbacks=backend.STATS['readbacks']-backend_before['readbacks'],gpu=gpu.stop())
    report=dict(episode=str(args.episode),frames=frames,rounds=args.rounds,geometry_only=True,
                frontier=int(os.environ.get('ROBOLAW_WARP_BATCH_FRONTIER','8')),
                cold_compilation_excluded=True,measurements=measurements,
                mean_speedup=measurements['scalar']['mean_s']/measurements['batch']['mean_s'],
                max_distance_difference_m=max_error,coverage_and_self_collision_equal=equivalent,
                equivalent=equivalent and max_error<1.e-6,gpu='RTX 5090',
                scope='Identical recorded poses and complete authored meshes; no end-to-end collection speed claim')
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False),flush=True)

if __name__=='__main__':main()
