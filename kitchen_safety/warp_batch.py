"""Frame-wide CUDA mesh queries. Scalar reference arithmetic is retained.

One flattened triangle grid covers all selected collider pairs. No per-pair
host readbacks. Persistent buffers and CUDA graphs amortize launch overhead.
"""
import time
from collections import OrderedDict
import numpy as np
import warp as wp
from kitchen_safety.warp_mesh_backend import (DEVICE, point_tri, relative_point,
    segment_pair, tri_crosses)

@wp.struct
class Pair:
    av: wp.array(dtype=wp.vec3)
    ai: wp.array(dtype=wp.int32)
    bv: wp.array(dtype=wp.vec3)
    bi: wp.array(dtype=wp.int32)
    mesh: wp.uint64
    rot: wp.mat33
    tr: wp.vec3
    pad: wp.float32
    badlo: wp.vec3
    badhi: wp.vec3
    start: wp.int32
    count: wp.int32
    collision: wp.int32

@wp.kernel
def reset_seed(upper:wp.array(dtype=wp.float32), count:int):
    i=wp.tid()
    if i<count: upper[i]=wp.float32(1.e30)

@wp.kernel
def batch_seed(pairs:wp.array(dtype=Pair), count:int, upper:wp.array(dtype=wp.float32)):
    pair,k=wp.tid()
    if pair>=count:return
    spec=pairs[pair]
    if spec.collision!=0:return
    idx=k*(spec.av.shape[0]-1)//63
    p=relative_point(spec.av[idx],spec.rot,spec.tr)
    res=wp.mesh_query_point_no_sign(spec.mesh,p,wp.float32(1.e20))
    close=point_tri(p,spec.bv[spec.bi[0]],spec.bv[spec.bi[1]],spec.bv[spec.bi[2]])
    value=wp.length(p-close)
    if res.result:
        f=res.face
        close=point_tri(p,spec.bv[spec.bi[3*f]],spec.bv[spec.bi[3*f+1]],spec.bv[spec.bi[3*f+2]])
        value=wp.length(p-close)
    wp.atomic_min(upper,pair,value)

@wp.kernel
def batch_surface(pairs:wp.array(dtype=Pair), owners:wp.array(dtype=wp.int32), total:int,
                  upper:wp.array(dtype=wp.float32), dist:wp.array(dtype=wp.float32),
                  outa:wp.array(dtype=wp.vec3), outb:wp.array(dtype=wp.vec3), hit:wp.array(dtype=wp.int32)):
    tid=wp.tid()
    if tid>=total:return
    pair=owners[tid];spec=pairs[pair];i=tid-spec.start
    av=spec.av;ai=spec.ai;bv=spec.bv;bi=spec.bi;mesh=spec.mesh;rot=spec.rot;tr=spec.tr;pad=spec.pad
    badlo=spec.badlo;badhi=spec.badhi
    if spec.collision!=0:
        a=rot*av[ai[3*i]]+tr;b=rot*av[ai[3*i+1]]+tr;c=rot*av[ai[3*i+2]]+tr
        query=wp.mesh_query_aabb(mesh,wp.vec3(wp.min(a,wp.min(b,c))-wp.vec3(pad)),wp.vec3(wp.max(a,wp.max(b,c))+wp.vec3(pad)))
        f=int(0);found=int(0)
        while wp.mesh_query_aabb_next(query,f):
            u=bv[bi[3*f]];v=bv[bi[3*f+1]];w=bv[bi[3*f+2]]
            na=wp.cross(b-a,c-a);nb=wp.cross(v-u,w-u)
            if tri_crosses(a,b,c,u,v,w):
                found=1;break
            for j in range(3):
                p=a
                if j==1:p=b
                if j==2:p=c
                close=point_tri(p,u,v,w)
                if wp.length(p-close)<=wp.float32(1.e-10):found=1
                p=u
                if j==1:p=v
                if j==2:p=w
                close=point_tri(p,a,b,c)
                if wp.length(p-close)<=wp.float32(1.e-10):found=1
                for k in range(3):
                    x=a;y=b
                    if j==1:x=b;y=c
                    if j==2:x=c;y=a
                    z=u;t=v
                    if k==1:z=v;t=w
                    if k==2:z=w;t=u
                    ea,eb=segment_pair(x,y,z,t)
                    if wp.length(ea-eb)<=wp.float32(1.e-10):found=1
            if found!=0:break
        dist[tid]=wp.float32(1-found);outa[tid]=wp.vec3(0.0);outb[tid]=wp.vec3(0.0);hit[tid]=found
        return
    bound=upper[pair]
    a=relative_point(av[ai[3*i]],rot,tr);b=relative_point(av[ai[3*i+1]],rot,tr);c=relative_point(av[ai[3*i+2]],rot,tr)
    center=(a+b+c)/wp.float32(3.0)
    radius=wp.max(wp.length(a-center),wp.max(wp.length(b-center),wp.length(c-center)))
    # Point query supplies a lower bound after accounting for discarded slivers.
    # The exact triangle search below still supplies the final minimum.
    closest=wp.mesh_query_point_no_sign(mesh,wp.vec3(center),wp.float32(1.0e20))
    lower=wp.float32(1.0e30)
    if closest.result:
        point=wp.mesh_eval_position(mesh,closest.face,closest.u,closest.v)
        lower=wp.length(center-wp.vec3(point))-pad*wp.float32(8.0)
    delta_bad=wp.max(wp.vec3(0.0),wp.max(badlo-center,center-badhi))
    lower=wp.min(lower,wp.length(delta_bad))
    if lower>radius+bound+pad:
        dist[tid]=wp.float32(1.0e30);outa[tid]=a;outb[tid]=a;hit[tid]=0
        return
    lo=wp.min(a,wp.min(b,c))-wp.vec3(bound+pad)
    hi=wp.max(a,wp.max(b,c))+wp.vec3(bound+pad)
    query=wp.mesh_query_aabb(mesh,wp.vec3(lo),wp.vec3(hi))
    face=int(0);best=wp.float32(1.0e30);pa=wp.vec3(0.0);pb=wp.vec3(0.0);inter=int(0)
    while wp.mesh_query_aabb_next(query,face):
        u=bv[bi[3*face]];v=bv[bi[3*face+1]];w=bv[bi[3*face+2]]
        # Cheap triangle-box lower bound before exact primitive tests.
        blo=wp.min(u,wp.min(v,w));bhi=wp.max(u,wp.max(v,w))
        delta=wp.max(wp.vec3(0.0),wp.max(wp.min(a,wp.min(b,c))-bhi,blo-wp.max(a,wp.max(b,c))))
        if wp.dot(delta,delta)>(bound+pad)*(bound+pad):continue
        projection=point_tri(center,u,v,w)
        if wp.length(center-projection)>radius+bound+pad:continue
        na=wp.cross(b-a,c-a);nb=wp.cross(v-u,w-u)
        if tri_crosses(a,b,c,u,v,w):
            best=wp.float32(0.0);inter=1;break
        for j in range(3):
            p=a
            if j==1:p=b
            if j==2:p=c
            q=point_tri(p,u,v,w);d=wp.dot(p-q,p-q)
            if d<best:best=d;pa=p;pb=q
            p=u
            if j==1:p=v
            if j==2:p=w
            q=point_tri(p,a,b,c);d=wp.dot(p-q,p-q)
            if d<best:best=d;pa=q;pb=p
            for k in range(3):
                x=a;y=b
                if j==1:x=b;y=c
                if j==2:x=c;y=a
                z=u;t=v
                if k==1:z=v;t=w
                if k==2:z=w;t=u
                ea,eb=segment_pair(x,y,z,t);d=wp.dot(ea-eb,ea-eb)
                if d<best:best=d;pa=ea;pb=eb
    dist[tid]=best;outa[tid]=pa;outb[tid]=pb;hit[tid]=inter


@wp.kernel
def batch_reduce(pairs:wp.array(dtype=Pair), count:int, dist:wp.array(dtype=wp.float32),
                 values:wp.array(dtype=wp.float32), indices:wp.array(dtype=wp.int32)):
    pair,lane=wp.tid()
    if pair>=count:return
    spec=pairs[pair];best=wp.float32(1.e30);index=int(spec.start)
    for local in range(lane,spec.count,256):
        i=spec.start+local
        if dist[i]<best:best=dist[i];index=i
    values[pair*256+lane]=best;indices[pair*256+lane]=index

@wp.kernel
def batch_gather(count:int, values:wp.array(dtype=wp.float32), indices:wp.array(dtype=wp.int32),
                 pa:wp.array(dtype=wp.vec3),pb:wp.array(dtype=wp.vec3),out:wp.array2d(dtype=wp.float32)):
    pair=wp.tid()
    if pair>=count:return
    best=wp.float32(1.e30);index=int(0)
    for j in range(256):
        k=pair*256+j
        if values[k]<best:best=values[k];index=indices[k]
    a=pa[index];b=pb[index]
    out[pair,0]=best
    out[pair,1]=a[0];out[pair,2]=a[1];out[pair,3]=a[2]
    out[pair,4]=b[0];out[pair,5]=b[1];out[pair,6]=b[2]

class BatchRunner:
    def __init__(self):
        self.pair_capacity=0;self.triangle_capacity=0;self.graphs=OrderedDict()
        self.stats=dict(batches=0,pairs=0,triangles=0,readbacks=0,seconds=0.,graph_launches=0)

    def reserve(self,pairs,triangles):
        pc=1<<max(0,(pairs-1).bit_length());tc=1<<max(0,(triangles-1).bit_length())
        if pc<=self.pair_capacity and tc<=self.triangle_capacity:return
        pc=max(pc,self.pair_capacity);tc=max(tc,self.triangle_capacity)
        self.graphs.clear();self.pair_capacity=pc;self.triangle_capacity=tc
        self.hp=wp.empty(pc,dtype=Pair,device='cpu',pinned=True)
        self.ho=wp.empty(tc,dtype=wp.int32,device='cpu',pinned=True)
        self.pairs=wp.empty(pc,dtype=Pair,device=DEVICE)
        self.owners=wp.empty(tc,dtype=wp.int32,device=DEVICE)
        self.upper=wp.empty(pc,dtype=wp.float32,device=DEVICE)
        self.dist=wp.empty(tc,dtype=wp.float32,device=DEVICE)
        self.pa=wp.empty(tc,dtype=wp.vec3,device=DEVICE);self.pb=wp.empty_like(self.pa)
        self.hit=wp.empty(tc,dtype=wp.int32,device=DEVICE)
        self.values=wp.empty(pc*256,dtype=wp.float32,device=DEVICE)
        self.indices=wp.empty(pc*256,dtype=wp.int32,device=DEVICE)
        self.output=wp.empty((pc,7),dtype=wp.float32,device=DEVICE)
        self.host_output=wp.empty((pc,7),dtype=wp.float32,device='cpu',pinned=True)

    def launch(self,count,total):
        wp.launch(reset_seed,dim=count,inputs=[self.upper,count],device=DEVICE)
        wp.launch(batch_seed,dim=(count,64),inputs=[self.pairs,count,self.upper],device=DEVICE)
        wp.launch(batch_surface,dim=total,inputs=[self.pairs,self.owners,total,self.upper,self.dist,self.pa,self.pb,self.hit],device=DEVICE)
        wp.launch(batch_reduce,dim=(count,256),inputs=[self.pairs,count,self.dist,self.values,self.indices],device=DEVICE)
        wp.launch(batch_gather,dim=count,inputs=[count,self.values,self.indices,self.pa,self.pb,self.output],device=DEVICE)

    def run(self,jobs,graphs=True):
        from kitchen_safety.warp_mesh_backend import _inputs
        if not jobs:return []
        started=time.perf_counter();prepared=[];total=0
        for left,right,collision in jobs:
            a,b,rot,tr,swapped=_inputs(left,right);m=a.model;n=b.model
            q=Pair();q.av=m.v;q.ai=m.idx;q.bv=n.v;q.bi=n.idx;q.mesh=n.mesh.id
            q.rot=rot;q.tr=tr;q.pad=m.pad+n.pad;q.badlo=n.badlo;q.badhi=n.badhi
            q.start=total;q.count=m.n;q.collision=int(collision)
            prepared.append((q,b,swapped));total+=m.n
        count=len(prepared);self.reserve(count,total)
        hp=self.hp.numpy();ho=self.ho.numpy()
        for j,(q,_,_) in enumerate(prepared):
            hp[j]=q.numpy_value();ho[q.start:q.start+q.count]=j
        wp.copy(self.pairs,self.hp,count=count);wp.copy(self.owners,self.ho,count=total)
        key=(count,total)
        if graphs:
            graph=self.graphs.get(key)
            if graph is None:
                wp.load_module(module=__name__,device=DEVICE)
                with wp.ScopedCapture(device=DEVICE) as capture:self.launch(count,total)
                graph=capture.graph;self.graphs[key]=graph
                if len(self.graphs)>16:self.graphs.popitem(last=False)
            else:self.graphs.move_to_end(key)
            wp.capture_launch(graph);self.stats['graph_launches']+=1
        else:self.launch(count,total)
        wp.copy(self.host_output,self.output,count=count*7)
        wp.synchronize_stream(wp.get_stream(DEVICE))
        data=self.host_output.numpy()[:count].copy();results=[]
        for row,(_,b,swapped),(_,_,collision) in zip(data,prepared,jobs):
            if not np.isfinite(row).all():raise ValueError('Nonfinite batched GPU result')
            if collision:results.append(bool(row[0]==0));continue
            value=float(np.sqrt(max(0.,float(row[0]))))
            if not np.isfinite(value) or value>=1.e14:raise ValueError('Batched Warp query found no candidate')
            points=[np.zeros(3),np.zeros(3)]
            if value>0:
                pa=b.pose.rotation@row[1:4].astype(float)+b.pose.translation
                pb=b.pose.rotation@row[4:7].astype(float)+b.pose.translation
                points=[pb,pa] if swapped else [pa,pb]
            results.append((value,points))
        self.stats['batches']+=1;self.stats['pairs']+=count;self.stats['triangles']+=total
        self.stats['readbacks']+=1;self.stats['seconds']+=time.perf_counter()-started
        return results

RUNNER=BatchRunner()
