"""Warp CUDA surface queries; legacy module name retained for shared topology helpers. Bounds prune candidates; never replace final distances."""
import numpy as np
METRIC = 'warp_authored_collision_surface_distance'


def pair_measurement(left,right,distance,points,intersects=False):
    points=np.asarray(points,dtype=float)
    valid=(distance>0 and points.shape==(2,3) and np.isfinite(points).all()
           and abs(float(np.linalg.norm(points[0]-points[1]))-distance)<1e-5)
    return dict(distance_m=float(distance),source_collider=left['path'],
                destination_collider=right['path'],mesh_intersection=bool(intersects),
                nearest_points_world_m=points.tolist() if valid else None,
                nearest_points_valid=bool(valid),metric=METRIC)


class GeometryBatch:
    """Conservative branch-and-bound waves across HS, PT and RS together.

    Bounds select candidates only. Every returned distance comes from the full
    triangle narrow phase. Requests retain their original stable tie ordering.
    """
    def __init__(self,backend,frontier=None):
        import os
        self.backend=backend;self.requests=[];self.collisions=[];self.cache={}
        self.frontier=max(1,int(frontier if frontier is not None else os.environ.get('ROBOLAW_WARP_BATCH_FRONTIER','8')))
        self.stats=dict(waves=0,requested_pairs=0,frontier=self.frontier)

    def minimum(self,left,right,exclude=None,hs=False):
        candidates=[]
        for i,a in enumerate(left):
            for j,b in enumerate(right):
                if exclude and (a['path'],b['path']) in exclude:continue
                lower=float(np.linalg.norm(np.maximum(0,np.maximum(a['lo']-b['hi'],b['lo']-a['hi']))))
                candidates.append((lower,i,j,a,b))
        request=dict(candidates=sorted(candidates,key=lambda r:r[:3]),index=0,
                     best=None,error=None,hs=hs)
        if not candidates:request['error']='Empty declared non-exempt mesh distance geometry'
        self.requests.append(request)
        return request

    def collision(self,left,right):
        request=dict(left=left,right=right,value=None,error=None)
        self.collisions.append(request);return request

    @staticmethod
    def result(request):
        if request['error']:raise ValueError(request['error'])
        if request['best'] is None:raise ValueError('Unexecuted mesh minimum')
        return dict(request['best'])

    @staticmethod
    def key(a,b):return (a['object'].uid,b['object'].uid)

    def execute(self):
        import time
        from kitchen_safety.warp_batch import RUNNER
        started=time.perf_counter();before=dict(RUNNER.stats)
        collision_pending=list(self.collisions)
        while True:
            jobs={};waiting=[]
            for request in self.requests:
                if request['error']:continue
                candidates=request['candidates']
                while request['index']<len(candidates):
                    lower,_,_,a,b=candidates[request['index']]
                    best=request['best']
                    if best is not None and (best['distance_m']==0 or lower>best['distance_m']+1e-12):
                        request['index']=len(candidates);break
                    key=self.key(a,b)
                    if key not in self.cache:
                        # Query a small frontier concurrently. Results are still
                        # consumed in the original sorted order, preserving ties.
                        for bound,_,_,la,lb in candidates[request['index']:request['index']+self.frontier]:
                            if best is not None and bound>best['distance_m']+1e-12:break
                            pair_key=self.key(la,lb)
                            if pair_key not in self.cache:
                                jobs[(pair_key,False)]=(la['object'],lb['object'],False)
                                waiting.append((request,la,lb,pair_key))
                        break
                    measured=dict(self.cache[key])
                    if request['hs']:
                        measured.update(robot_or_object_path=a['path'],human_path=b['path'],body=a['actor'])
                    if best is None or measured['distance_m']<best['distance_m']:request['best']=measured
                    request['index']+=1
            for request in collision_pending:
                a,b=request['left'],request['right'];key=self.key(a,b)
                jobs[(key,True)]=(a['object'],b['object'],True)
            if not jobs:break
            keys=list(jobs)
            try:
                values=dict(zip(keys,self.backend.batch_queries(list(jobs.values()))))
                for request,a,b,key in waiting:
                    value,points=values[(key,False)]
                    self.cache[key]=pair_measurement(a,b,value,points,intersects=value==0)
                for request in collision_pending:
                    request['value']=values[(self.key(request['left'],request['right']),True)]
            except Exception as exc:
                for request,_,_,_ in waiting:request['error']=repr(exc)
                for request in collision_pending:request['error']=repr(exc)
            collision_pending=[]
            self.stats['waves']+=1;self.stats['requested_pairs']+=len(jobs)
        self.stats.update(wall_seconds=time.perf_counter()-started,
                          gpu_pairs=RUNNER.stats['pairs']-before['pairs'],
                          gpu_readbacks=RUNNER.stats['readbacks']-before['readbacks'],
                          gpu_graph_launches=RUNNER.stats['graph_launches']-before['graph_launches'])


def pair_distance(fcl, left, right):
    result = fcl.DistanceResult()
    distance = float(fcl.distance(left['object'], right['object'],
        fcl.DistanceRequest(enable_nearest_points=True), result))
    if not np.isfinite(distance):
        raise ValueError('Nonfinite FCL surface distance')
    intersects = False
    if distance <= 0:
        contact = fcl.CollisionResult()
        fcl.collide(left['object'], right['object'], fcl.CollisionRequest(), contact)
        intersects = bool(contact.is_collision)
        if distance < 0 and not intersects:
            raise ValueError('Negative FCL distance without surface intersection')
        distance = 0.0
    points = np.asarray(result.nearest_points, dtype=float)
    valid = (distance > 0 and points.shape == (2, 3) and np.isfinite(points).all()
             and abs(float(np.linalg.norm(points[0] - points[1])) - distance) < 1e-5)
    return dict(distance_m=distance, source_collider=left['path'],
                destination_collider=right['path'], mesh_intersection=intersects,
                nearest_points_world_m=points.tolist() if valid else None,
                nearest_points_valid=bool(valid), metric=METRIC)


def minimum_distance(fcl, left, right, cache=None):
    if not left or not right:
        raise ValueError('Empty declared FCL distance geometry')
    cache = {} if cache is None else cache
    candidates = []
    for i, a in enumerate(left):
        for j, b in enumerate(right):
            lower = float(np.linalg.norm(np.maximum(0, np.maximum(
                a['lo'] - b['hi'], b['lo'] - a['hi']))))
            candidates.append((lower, i, j))
    best = None
    for lower, i, j in sorted(candidates):
        if best is not None and lower > best['distance_m'] + 1e-12:
            break
        a, b = left[i], right[j]
        key = (a['path'], b['path'])
        if key not in cache:
            cache[key] = pair_distance(fcl, a, b)
        result = cache[key]
        if best is None or result['distance_m'] < best['distance_m']:
            best = result
        if best['distance_m'] == 0:
            break
    return dict(best)


def primitive_mesh(kind, radius=None, height=None, size=None, axis='Z'):
    """Triangulate primitives; curved shapes use 128 azimuth/64 polar segments."""
    if kind == 'Cube':
        from itertools import product
        vertices = np.asarray(list(product((-size/2, size/2), repeat=3)), dtype=float)
        faces = [(0,2,3),(0,3,1),(4,5,7),(4,7,6),(0,1,5),(0,5,4),
                 (2,6,7),(2,7,3),(0,4,6),(0,6,2),(1,3,7),(1,7,5)]
        return vertices, np.asarray(faces, dtype=np.int32), 'exact cube surface triangles'
    if kind not in ('Sphere', 'Capsule'):
        raise ValueError('Unsupported primitive ' + kind)
    n, polar = 128, 64
    half = height/2 if kind == 'Capsule' else 0.0
    angles = np.arange(n)*2*np.pi/n
    rings = []
    for i in range(1, polar):
        theta = i*np.pi/polar
        offsets = [half, -half] if kind == 'Capsule' and i == polar//2 else [half if i < polar//2 else -half]
        for offset in offsets:
            rings.append(np.c_[radius*np.sin(theta)*np.cos(angles),
                               radius*np.sin(theta)*np.sin(angles),
                               np.full(n, radius*np.cos(theta)+offset)])
    vertices = np.concatenate([np.array([[0.,0.,radius+half]]), *rings,
                               np.array([[0.,0.,-radius-half]])])
    faces = [(0,1+j,1+(j+1)%n) for j in range(n)]
    for i in range(len(rings)-1):
        a, b = 1+i*n, 1+(i+1)*n
        for j in range(n):
            k = (j+1)%n
            faces.extend([(a+j,b+j,b+k),(a+j,b+k,a+k)])
    last, pole = 1+(len(rings)-1)*n, len(vertices)-1
    faces.extend((last+j,pole,last+(j+1)%n) for j in range(n))
    if axis == 'X': vertices = vertices[:,[2,0,1]]
    elif axis == 'Y': vertices = vertices[:,[1,2,0]]
    elif axis != 'Z': raise ValueError('Invalid primitive axis')
    return vertices, np.asarray(faces,dtype=np.int32), '128-azimuth 64-polar-segment '+kind.lower()+' surface tessellation'


def triangulate_polygon(vertices, indices):
    """Ear-clip simple authored polygons; do not bridge concave faces by a fan."""
    indices = list(map(int, indices))
    if len(indices) == 3:
        return [tuple(indices)]
    p = vertices[indices]
    normal = np.sum(np.cross(p, np.roll(p,-1,axis=0)),axis=0)
    if np.linalg.norm(normal) < 1e-16:
        raise ValueError('Degenerate polygon normal')
    xy = np.delete(p,int(np.argmax(np.abs(normal))),axis=1)
    def cross(a,b):return float(a[0]*b[1]-a[1]*b[0])
    area = sum(cross(xy[i],xy[(i+1)%len(xy)]) for i in range(len(xy)))
    sign = 1 if area > 0 else -1
    scale = float(np.ptp(xy,axis=0).max())
    eps = max(1e-24, scale*scale*1e-12)
    remaining, output = list(range(len(indices))), []
    while len(remaining) > 3:
        found = False
        for k,b in enumerate(remaining):
            a,c = remaining[k-1],remaining[(k+1)%len(remaining)]
            turn = sign*cross(xy[b]-xy[a],xy[c]-xy[a])
            if abs(turn) <= eps:
                remaining.pop(k);found=True;break
            if turn < 0:continue
            inside = any(all(v >= -eps for v in (
                sign*cross(xy[b]-xy[a],xy[j]-xy[a]),
                sign*cross(xy[c]-xy[b],xy[j]-xy[b]),
                sign*cross(xy[a]-xy[c],xy[j]-xy[c])))
                for j in remaining if j not in (a,b,c))
            if inside:continue
            output.append((indices[a],indices[b],indices[c]))
            remaining.pop(k);found=True;break
        if not found:raise ValueError('Cannot triangulate authored polygon')
    if len(remaining)==3:output.append(tuple(indices[j] for j in remaining))
    return output
