"""Full triangle-surface distance on CUDA using Warp; no FCL runtime calls.

FCL-shaped value classes only preserve existing observer integration. Every
source triangle is covered. Point queries seed a search upper bound; they never
serve as the final mesh distance. Expanded triangle AABBs conservatively select
candidate triangles, followed by triangle intersection, six vertex/face and nine
edge/edge tests in float32. GPU BVH uses float32 with padded query bounds.
"""
import time
import numpy as np
import warp as wp

wp.init()
DEVICE='cuda:0'
if not wp.is_cuda_available():
    raise RuntimeError('Warp mesh observer requires CUDA; CPU fallback disabled')
_CACHE={}
_COLLISION_CACHE={}
_NEXT_ID=0
STATS={'cache_hits':0,'calls':0,'seconds':0.,'triangles':0,'readbacks':0,'collision_calls':0,'backend':'warp_cuda_full_triangle_surface_batch_v2'}

@wp.func
def clamp01(x:wp.float32):
    return wp.min(wp.max(x,wp.float32(0.0)),wp.float32(1.0))

@wp.func
def point_tri(p:wp.vec3,a:wp.vec3,b:wp.vec3,c:wp.vec3):
    ab=b-a;ac=c-a;ap=p-a
    normal=wp.cross(ab,ac)
    if wp.dot(normal,normal)<=wp.float32(1.0e-30):
        best=wp.float32(1.0e30);out=a
        for k in range(3):
            x=a;y=b
            if k==1:x=b;y=c
            if k==2:x=c;y=a
            edge=y-x;den=wp.dot(edge,edge);t=wp.float32(0.0)
            if den>wp.float32(1.0e-30):t=clamp01(wp.dot(p-x,edge)/den)
            q=x+t*edge;d=wp.dot(p-q,p-q)
            if d<best:best=d;out=q
        return out
    d1=wp.dot(ab,ap);d2=wp.dot(ac,ap)
    if d1<=0.0 and d2<=0.0:return a
    bp=p-b;d3=wp.dot(ab,bp);d4=wp.dot(ac,bp)
    if d3>=0.0 and d4<=d3:return b
    vc=d1*d4-d3*d2
    if vc<=0.0 and d1>=0.0 and d3<=0.0:
        return a+ab*(d1/(d1-d3))
    cp=p-c;d5=wp.dot(ab,cp);d6=wp.dot(ac,cp)
    if d6>=0.0 and d5<=d6:return c
    vb=d5*d2-d1*d6
    if vb<=0.0 and d2>=0.0 and d6<=0.0:
        return a+ac*(d2/(d2-d6))
    va=d3*d6-d5*d4
    if va<=0.0 and d4-d3>=0.0 and d5-d6>=0.0:
        return b+(c-b)*((d4-d3)/((d4-d3)+(d5-d6)))
    den=va+vb+vc
    if wp.abs(den)<wp.float32(1.0e-30):return a
    return a+ab*(vb/den)+ac*(vc/den)

@wp.func
def segment_pair(p:wp.vec3,q:wp.vec3,r:wp.vec3,s:wp.vec3):
    d1=q-p;d2=s-r;v=p-r
    a=wp.dot(d1,d1);e=wp.dot(d2,d2);f=wp.dot(d2,v)
    t=wp.float32(0.0);u=wp.float32(0.0)
    eps=wp.float32(1.0e-30)
    if a<=eps and e<=eps:return p,r
    if a<=eps:
        u=clamp01(f/e)
    else:
        c=wp.dot(d1,v)
        if e<=eps:t=clamp01(-c/a)
        else:
            b=wp.dot(d1,d2);den=a*e-b*b
            if den>eps:t=clamp01((b*f-c*e)/den)
            u=(b*t+f)/e
            if u<0.0:
                u=wp.float32(0.0);t=clamp01(-c/a)
            elif u>1.0:
                u=wp.float32(1.0);t=clamp01((b-c)/a)
    return p+d1*t,r+d2*u

@wp.func
def segment_triangle(p:wp.vec3,q:wp.vec3,a:wp.vec3,b:wp.vec3,c:wp.vec3):
    d=q-p;e1=b-a;e2=c-a;h=wp.cross(d,e2);det=wp.dot(e1,h)
    scale=wp.length(d)*wp.length(e1)*wp.length(e2)
    if wp.abs(det)<=scale*wp.float32(1.e-13):return False
    inv=wp.float32(1.0)/det;s=p-a;u=wp.dot(s,h)*inv
    if u<0.0 or u>1.0:return False
    cross=wp.cross(s,e1);v=wp.dot(d,cross)*inv
    if v<0.0 or u+v>1.0:return False
    t=wp.dot(e2,cross)*inv
    return t>=0.0 and t<=1.0

@wp.func
def tri_crosses(a:wp.vec3,b:wp.vec3,c:wp.vec3,u:wp.vec3,v:wp.vec3,w:wp.vec3):
    return (segment_triangle(a,b,u,v,w) or segment_triangle(b,c,u,v,w) or segment_triangle(c,a,u,v,w)
        or segment_triangle(u,v,a,b,c) or segment_triangle(v,w,a,b,c) or segment_triangle(w,u,a,b,c))

@wp.func
def relative_point(v:wp.vec3,rot:wp.mat33,tr:wp.vec3):
    return rot*v+tr

@wp.kernel
def seed_distance(av:wp.array(dtype=wp.vec3),mesh:wp.uint64,bv:wp.array(dtype=wp.vec3),bi:wp.array(dtype=wp.int32),rot:wp.mat33,tr:wp.vec3,seeds:wp.array(dtype=wp.float32)):
    k=wp.tid()
    idx=k*(av.shape[0]-1)//(seeds.shape[0]-1)
    p=relative_point(av[idx],rot,tr)
    res=wp.mesh_query_point_no_sign(mesh,wp.vec3(p),wp.float32(1.0e20))
    initial=point_tri(p,bv[bi[0]],bv[bi[1]],bv[bi[2]])
    value=wp.length(p-initial)
    if res.result:
        f=res.face
        # Re-evaluate distance to returned authored triangle in float32 precision.
        close=point_tri(p,bv[bi[3*f]],bv[bi[3*f+1]],bv[bi[3*f+2]])
        value=wp.length(p-close)
    wp.atomic_min(seeds,0,value)

@wp.kernel
def full_distance(av:wp.array(dtype=wp.vec3),ai:wp.array(dtype=wp.int32),mesh:wp.uint64,bv:wp.array(dtype=wp.vec3),bi:wp.array(dtype=wp.int32),rot:wp.mat33,tr:wp.vec3,upper:wp.array(dtype=wp.float32),pad:wp.float32,badlo:wp.vec3,badhi:wp.vec3,dist:wp.array(dtype=wp.float32),outa:wp.array(dtype=wp.vec3),outb:wp.array(dtype=wp.vec3),hit:wp.array(dtype=wp.int32)):
    i=wp.tid()
    bound=upper[0]
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
        dist[i]=wp.float32(1.0e30);outa[i]=a;outb[i]=a;hit[i]=0
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
    dist[i]=best;outa[i]=pa;outb[i]=pb;hit[i]=inter

@wp.kernel
def full_intersection(av:wp.array(dtype=wp.vec3),ai:wp.array(dtype=wp.int32),mesh:wp.uint64,bv:wp.array(dtype=wp.vec3),bi:wp.array(dtype=wp.int32),rot:wp.mat33,tr:wp.vec3,pad:wp.float32,hit:wp.array(dtype=wp.int32)):
    i=wp.tid()
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
    hit[i]=found

@wp.kernel
def reduce_chunks(dist:wp.array(dtype=wp.float32),values:wp.array(dtype=wp.float32),indices:wp.array(dtype=wp.int32)):
    k=wp.tid();best=wp.float32(1.e30);index=int(0)
    for i in range(k,dist.shape[0],values.shape[0]):
        if dist[i]<best:best=dist[i];index=i
    values[k]=best;indices[k]=index

@wp.kernel
def gather_min(values:wp.array(dtype=wp.float32),indices:wp.array(dtype=wp.int32),pa:wp.array(dtype=wp.vec3),pb:wp.array(dtype=wp.vec3),out:wp.array(dtype=wp.float32)):
    best=wp.float32(1.e30);index=int(0)
    for j in range(values.shape[0]):
        if values[j]<best:best=values[j];index=indices[j]
    a=pa[index];b=pb[index]
    out[0]=best;out[1]=a[0];out[2]=a[1];out[3]=a[2];out[4]=b[0];out[5]=b[1];out[6]=b[2]

class Transform:
    def __init__(self,rotation=None,translation=None):
        self.rotation=np.eye(3) if rotation is None else np.array(rotation,dtype=float)
        self.translation=np.zeros(3) if translation is None else np.array(translation,dtype=float)
class BVHModel:
    def beginModel(self,*args):pass
    def addSubModel(self,vertices,faces):
        self.vertices=np.ascontiguousarray(vertices,dtype=np.float64)
        self.faces=np.ascontiguousarray(faces,dtype=np.int32)
        a=self.vertices[self.faces[:,1]]-self.vertices[self.faces[:,0]]
        b=self.vertices[self.faces[:,2]]-self.vertices[self.faces[:,0]]
        self.degenerate_count=int(np.sum(np.linalg.norm(np.cross(a,b),axis=1)<1.e-18))
        # Retain authored zero-area triangles as segments/points for distance queries.
    def endModel(self):
        import os,hashlib
        cache=os.environ.get('ROBOLAW_WARP_MESH_CACHE')
        if cache:
            from pathlib import Path
            folder=Path(cache);folder.mkdir(parents=True,exist_ok=True)
            key=hashlib.sha256(self.vertices.tobytes()+self.faces.tobytes()).hexdigest()
            path=folder/(key+'.npz')
            if not path.exists():np.savez_compressed(path,vertices=self.vertices,faces=self.faces)
        self.v=wp.array(self.vertices,dtype=wp.vec3,device=DEVICE)
        self.v32=wp.array(self.vertices.astype(np.float32),dtype=wp.vec3,device=DEVICE)
        self.idx=wp.array(self.faces.ravel(),dtype=wp.int32,device=DEVICE)
        self.mesh=wp.Mesh(points=self.v32,indices=self.idx)
        self.pad=max(1.e-5,float(np.abs(self.vertices).max())*2.e-6)
        # Warp point queries skip sliver faces; their union box protects pruning.
        vf=self.vertices.astype(np.float32);f=self.faces
        e0=vf[f[:,1]]-vf[f[:,0]];e1=vf[f[:,2]]-vf[f[:,0]];e2=vf[f[:,2]]-vf[f[:,1]]
        quality=np.linalg.norm(np.cross(e0,e1),axis=1)/np.maximum(np.sum(e0*e0+e1*e1+e2*e2,axis=1),1.e-30)
        self.bad_count=int(np.sum(quality<1.e-5))
        bad=vf[f[quality<1.e-5]].reshape(-1,3)
        self.badlo=wp.vec3(bad.min(0)-self.pad) if len(bad) else wp.vec3(1.e25)
        self.badhi=wp.vec3(bad.max(0)+self.pad) if len(bad) else wp.vec3(1.e25)

        self.n=len(self.faces)
        self.dist=wp.empty(self.n,dtype=wp.float32,device=DEVICE)
        self.pa=wp.empty(self.n,dtype=wp.vec3,device=DEVICE)
        self.pb=wp.empty(self.n,dtype=wp.vec3,device=DEVICE)
        self.hit=wp.empty(self.n,dtype=wp.int32,device=DEVICE)
        self.reduced=wp.empty(256,dtype=wp.float32,device=DEVICE)
        self.rindex=wp.empty(256,dtype=wp.int32,device=DEVICE)
        self.output=wp.empty(7,dtype=wp.float32,device=DEVICE)
        self.seeds=wp.empty(min(64,max(2,len(self.vertices))),dtype=wp.float32,device=DEVICE)
class CollisionObject:
    def __init__(self,model,pose=None):
        global _NEXT_ID
        self.uid=_NEXT_ID;_NEXT_ID+=1
        self.model=model;self.pose=pose or Transform()
    def setTransform(self,pose):self.pose=pose
class DistanceRequest:
    def __init__(self,**kwargs):pass
class DistanceResult:
    def __init__(self):self.nearest_points=[np.zeros(3),np.zeros(3)]
class CollisionRequest:
    def __init__(self,**kwargs):pass
class CollisionResult:
    def __init__(self):self.is_collision=False

def _inputs(left,right):
    swapped=(left.model.n<right.model.n) if min(left.model.n,right.model.n)<=128 else (left.model.n>right.model.n)
    if bool(left.model.bad_count) != bool(right.model.bad_count):
        swapped=not bool(left.model.bad_count)
    a,b=(right,left) if swapped else (left,right)
    rot=b.pose.rotation.T@a.pose.rotation
    tr=b.pose.rotation.T@(a.pose.translation-b.pose.translation)
    return a,b,wp.mat33(rot),wp.vec3(tr),swapped

def distance(left,right,request,result):
    start=time.perf_counter()
    key=(left.uid,right.uid)
    signature=(left.pose.rotation.tobytes(),left.pose.translation.tobytes(),right.pose.rotation.tobytes(),right.pose.translation.tobytes())
    cached=_CACHE.get(key)
    if cached and cached[0]==signature:
        result.nearest_points=[x.copy() for x in cached[2]]
        STATS['cache_hits']+=1
        return cached[1]
    a,b,rot,tr,swapped=_inputs(left,right);m=a.model;n=b.model
    m.seeds.fill_(1.e30)
    wp.launch(seed_distance,dim=len(m.seeds),inputs=[m.v,n.mesh.id,n.v,n.idx,rot,tr,m.seeds],device=DEVICE)
    pad=n.pad+m.pad
    wp.launch(full_distance,dim=m.n,inputs=[m.v,m.idx,n.mesh.id,n.v,n.idx,rot,tr,m.seeds,pad,n.badlo,n.badhi,m.dist,m.pa,m.pb,m.hit],device=DEVICE)
    wp.launch(reduce_chunks,dim=256,inputs=[m.dist,m.reduced,m.rindex],device=DEVICE)
    wp.launch(gather_min,dim=1,inputs=[m.reduced,m.rindex,m.pa,m.pb,m.output],device=DEVICE)
    packed=m.output.numpy()
    STATS['readbacks']+=1
    value=float(np.sqrt(max(0.,float(packed[0]))))
    if not np.isfinite(value) or value>=1.e14:raise ValueError('Warp exhaustive query found no candidate')
    if value>0:
        pa=b.pose.rotation@packed[1:4].astype(float)+b.pose.translation
        pb=b.pose.rotation@packed[4:7].astype(float)+b.pose.translation
        result.nearest_points=[pb,pa] if swapped else [pa,pb]
    elapsed=time.perf_counter()-start
    if elapsed>.1: print('WARP_SLOW_QUERY',m.n,n.n,'distance',value,'seconds',round(elapsed,3),flush=True)
    STATS['calls']+=1;STATS['seconds']+=time.perf_counter()-start;STATS['triangles']+=m.n
    _CACHE[key]=(signature,value,[x.copy() for x in result.nearest_points])
    return value

def collide(left,right,request,result):
    key=(left.uid,right.uid)
    signature=(left.pose.rotation.tobytes(),left.pose.translation.tobytes(),right.pose.rotation.tobytes(),right.pose.translation.tobytes())
    cached=_COLLISION_CACHE.get(key)
    if cached and cached[0]==signature:
        result.is_collision=cached[1]
        return int(result.is_collision)
    a,b,rot,tr,_=_inputs(left,right);m=a.model;n=b.model
    wp.launch(full_intersection,dim=m.n,inputs=[m.v,m.idx,n.mesh.id,n.v,n.idx,rot,tr,n.pad+m.pad,m.hit],device=DEVICE)
    result.is_collision=bool(m.hit.numpy().any())
    STATS['readbacks']+=1;STATS['collision_calls']+=1
    _COLLISION_CACHE[key]=(signature,result.is_collision)
    return int(result.is_collision)


def batch_queries(jobs):
    """Distance/collision jobs share one CUDA launch sequence and one readback.

    Jobs are (left, right, collision_only). Preserve pose-exact scalar caches,
    including results needed by compatibility callers after a frame batch.
    """
    from kitchen_safety.warp_batch import RUNNER
    output=[None]*len(jobs);pending=[];indices=[]
    for i,(left,right,collision) in enumerate(jobs):
        key=(left.uid,right.uid)
        signature=(left.pose.rotation.tobytes(),left.pose.translation.tobytes(),
                   right.pose.rotation.tobytes(),right.pose.translation.tobytes())
        cache=(_COLLISION_CACHE if collision else _CACHE).get(key)
        if cache and cache[0]==signature:
            output[i]=cache[1] if collision else (cache[1],[p.copy() for p in cache[2]])
            STATS['cache_hits']+=1
        else:
            pending.append((left,right,collision));indices.append((i,key,signature))
    if pending:
        results=RUNNER.run(pending)
        for (i,key,signature),value,(_,_,collision) in zip(indices,results,pending):
            output[i]=value
            if collision:_COLLISION_CACHE[key]=(signature,value)
            else:
                _CACHE[key]=(signature,value[0],[p.copy() for p in value[1]])
                # A zero squared primitive distance is also an intersection
                # under the scalar collision routine's 1e-10 contact tolerance.
                if value[0]==0:_COLLISION_CACHE[key]=(signature,True)
    return output

# Legacy frozen archives retain their original backend identity and kernels.
BACKEND_ID='warp'
METRIC='warp_authored_collision_surface_distance'
DEFAULT_FRONTIER=8
def batch_stats():
    from kitchen_safety.warp_batch import RUNNER
    return dict(RUNNER.stats)
