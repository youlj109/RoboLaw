"""gDist CUDA authored triangle distances, with lazy GPU construction.

Value-class names preserve the observer interface. Raw capture freezes only CPU
meshes; GPU BVHs are built on the first offline query. No Warp/FCL/SDF fallback.
Closest collider IDs are retained; closest point coordinates are explicitly unavailable.
"""
import hashlib,importlib.util,json,time
from pathlib import Path
import numpy as np
from kitchen_safety.geometry_backend import DEFAULT_OPTIONS,GIT_COMMIT

BACKEND_ID='gdist'
DEVICE='cuda:0';METRIC='gdist_authored_collision_surface_distance';DEFAULT_FRONTIER=2
ROOT=Path(__file__).resolve().parents[1]
_OPTIONS=dict(DEFAULT_OPTIONS);_NATIVE=None;_BUILD=None;_NEXT_ID=0;_CACHE={}
STATS=dict(cache_hits=0,calls=0,collision_calls=0,seconds=0.,triangles=0,readbacks=0,
    batches=0,pairs=0,graph_launches=0,backend='gdist_cuda_full_triangle_surface_v1')

def configure(options):
    if set(options)!=set(DEFAULT_OPTIONS):raise ValueError('Explicit complete gDist options required')
    for k,v in options.items():
        if isinstance(v,bool) or not isinstance(v,int) or v<0:raise ValueError('Invalid gDist option: '+k)
    if options['target']<1 or not 1<=options['maxstep']<=8 or not 1<=options['concurrency']<=128 or not 2<=options['seeds']<=128:
        raise ValueError('gDist launch settings outside qualified implementation limits')
    _OPTIONS.clear();_OPTIONS.update(options)

def build_info():
    path=ROOT/'runtime/gdist/build_info.json'
    if not path.is_file():return dict(available=False,install='python scripts/build_gdist_backend.py',source_commit=GIT_COMMIT)
    return json.loads(path.read_text())

def _native():
    global _NATIVE,_BUILD
    if _NATIVE is None:
        _BUILD=build_info();path=ROOT/'runtime/gdist/gdist_native.so'
        if not _BUILD.get('available') or not path.is_file():raise RuntimeError('gDist native backend is not installed. Run python scripts/build_gdist_backend.py inside the Arena container; no fallback is enabled.')
        if _BUILD.get('source_commit')!=GIT_COMMIT or hashlib.sha256(path.read_bytes()).hexdigest()!=_BUILD.get('binary_sha256'):
            raise RuntimeError('gDist binary/source fingerprint mismatch; rebuild the backend')
        spec=importlib.util.spec_from_file_location('gdist_native',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);_NATIVE=module
    return _NATIVE

def batch_stats():return {k:STATS[k] for k in ('batches','pairs','triangles','readbacks','seconds','graph_launches')}

def ensure_available():_native()

def runtime_info():return dict(id='gdist',metric=METRIC,options=dict(_OPTIONS),frontier=DEFAULT_FRONTIER,build=(_BUILD if _NATIVE is not None else build_info()),closest_point_coordinates='unavailable',fallback='disabled')

class BVHModel:
    metric=METRIC
    def beginModel(self,*args):self._parts=[];self._native_model=None
    def addSubModel(self,vertices,faces):self._parts.append((np.asarray(vertices,dtype=np.float64).copy(),np.asarray(faces,dtype=np.int32).copy()))
    def endModel(self):
        vv=[];ff=[];offset=0
        for v,f in self._parts:vv.append(v);ff.append(f+offset);offset+=len(v)
        self.vertices=np.ascontiguousarray(np.concatenate(vv),dtype=np.float64);self.faces=np.ascontiguousarray(np.concatenate(ff),dtype=np.int32)
        if self.vertices.ndim!=2 or self.vertices.shape[1]!=3 or self.faces.ndim!=2 or self.faces.shape[1]!=3 or not len(self.faces) or not np.isfinite(self.vertices).all() or self.faces.min()<0 or self.faces.max()>=len(self.vertices):raise ValueError('Invalid complete collision mesh')
        self.n=len(self.faces);self.degenerate_count=int(np.sum(np.linalg.norm(np.cross(self.vertices[self.faces[:,1]]-self.vertices[self.faces[:,0]],self.vertices[self.faces[:,2]]-self.vertices[self.faces[:,0]]),axis=1)<1e-18))
        self._native_model=None;del self._parts
    def gpu(self):
        if self._native_model is None:self._native_model=_native().Mesh(np.ascontiguousarray(self.vertices[self.faces],dtype=np.float32))
        return self._native_model

class Transform:
    def __init__(self,rotation=None,translation=None):
        self.rotation=np.asarray(np.eye(3) if rotation is None else rotation,dtype=float).copy()
        self.translation=np.asarray(np.zeros(3) if translation is None else translation,dtype=float).copy()
        if self.rotation.shape!=(3,3) or self.translation.shape!=(3,) or not np.isfinite(self.rotation).all() or not np.isfinite(self.translation).all():raise ValueError('Invalid rigid transform')
class CollisionObject:
    def __init__(self,model,pose=None):
        global _NEXT_ID
        self.uid=_NEXT_ID;_NEXT_ID+=1;self.model=model;self.pose=pose or Transform()
    def setTransform(self,pose):self.pose=pose
class DistanceRequest:
    def __init__(self,**kwargs):pass
class DistanceResult:
    def __init__(self):self.nearest_points=None
class CollisionRequest:
    def __init__(self,**kwargs):pass
class CollisionResult:
    def __init__(self):self.is_collision=False

def batch_queries(jobs):
    if not jobs:return []
    tick=time.perf_counter();output=[None]*len(jobs);pending=[];keys=[];indices=[]
    for i,(a,b,collision) in enumerate(jobs):
        key=(a.uid,b.uid,bool(collision));signature=(a.pose.rotation.tobytes(),a.pose.translation.tobytes(),b.pose.rotation.tobytes(),b.pose.translation.tobytes())
        cached=_CACHE.get(key)
        if cached is not None and cached[0]==signature:output[i]=cached[1];STATS['cache_hits']+=1
        else:pending.append((a,b,collision));keys.append((key,signature));indices.append(i)
    if pending:
        aa=[];bb=[];source=[];destination=[]
        for a,b,_ in pending:
            if a.model.n>b.model.n:a,b=b,a
            aa.append(a.model.gpu());bb.append(b.model.gpu());source.append(a);destination.append(b)
        ar=np.asarray([a.pose.rotation for a in source]);br=np.asarray([b.pose.rotation for b in destination]);at=np.asarray([a.pose.translation for a in source]);bt=np.asarray([b.pose.translation for b in destination])
        rr=np.einsum('nji,njk->nik',br,ar);tr=np.einsum('nji,nj->ni',br,at-bt)
        poses=np.ascontiguousarray(np.concatenate([rr.reshape(-1,9),tr],axis=1),dtype=np.float32)
        values,_,_=_native().batch_parallel(aa,bb,poses,**_OPTIONS)
        if len(values)!=len(pending) or not np.isfinite(values).all() or np.any(values<0):raise ValueError('Invalid/incomplete gDist GPU distance batch')
        for i,d,(_,_,collision),(key,signature) in zip(indices,values,pending,keys):
            value=bool(d==0.) if collision else (float(d),None);output[i]=value;_CACHE[key]=(signature,value)
        STATS['batches']+=1;STATS['pairs']+=len(pending);STATS['readbacks']+=len(pending)
        STATS['triangles']+=sum(a.model.n for a in source)
    STATS['seconds']+=time.perf_counter()-tick
    return output

def distance(left,right,request,result):
    STATS['calls']+=1;value,points=batch_queries([(left,right,False)])[0];result.nearest_points=points;return value

def collide(left,right,request,result):
    STATS['collision_calls']+=1;result.is_collision=batch_queries([(left,right,True)])[0];return int(result.is_collision)
