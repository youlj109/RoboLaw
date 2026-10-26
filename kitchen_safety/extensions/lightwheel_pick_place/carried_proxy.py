"""Spatially distributed inscribed spheres; full source-hull FCL remains mandatory."""
import numpy as np
from scipy.spatial import ConvexHull
def distributed_interior_spheres(vertices,capacity=100,spacing=.008):
    vertices=np.asarray(vertices,dtype=float)
    if vertices.ndim!=2 or vertices.shape[1]!=3 or len(vertices)<4 or not np.isfinite(vertices).all():raise ValueError('Finite source vertices required')
    if capacity<1 or spacing<=0:raise ValueError('Positive sphere budget and spacing required')
    hull=ConvexHull(vertices);lo=vertices.min(0);hi=vertices.max(0)
    counts=np.maximum(3,np.ceil((hi-lo)/spacing).astype(int));counts += (counts%2==0)
    if np.prod(counts)>200000:raise ValueError('Unexpected carried-object grid size')
    points=np.array(np.meshgrid(*[np.linspace(a,b,int(n)+2)[1:-1] for a,b,n in zip(lo,hi,counts)],indexing='ij')).reshape(3,-1).T
    depth=-(points@hull.equations[:,:3].T+hull.equations[:,3]).max(1)
    good=depth>.0008;points=points[good];radii=np.minimum(.012,depth[good])
    if not len(points):raise ValueError('Source hull has no finite interior proxy centers')
    selected=[];remaining=np.full(len(points),np.inf);index=int(np.argmax(radii))
    for _ in range(min(capacity,len(points))):
        selected.append(index)
        remaining=np.minimum(remaining,np.linalg.norm(points-points[index],axis=1)-radii[index])
        remaining[selected]=-np.inf;index=int(np.argmax(remaining))
    centers=points[selected];radii=radii[selected]
    inscribed=bool(np.all(centers@hull.equations[:,:3].T+hull.equations[:,3]+radii[:,None]<=1e-7))
    if not inscribed:raise RuntimeError('Proxy sphere escaped source convex hull')
    gap=(np.linalg.norm(vertices[:,None]-centers[None],axis=2)-radii[None]).min(1)
    return centers,radii,{'policy':'Uniform interior grid with greedy spatial coverage; no outside padding','sphere_count':len(centers),'all_spheres_inside_original_source_convex_hull':inscribed,'full_surface_coverage':False,'maximum_source_vertex_distance_outside_proxy_m':float(gap.max()),'mean_source_vertex_distance_outside_proxy_m':float(np.maximum(gap,0).mean()),'candidate_grid_spacing_m':spacing}
