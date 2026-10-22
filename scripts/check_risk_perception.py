#!/usr/bin/env python3
"""CUDA preflight for the configured risk geometry backend; does not start Isaac Sim."""
import importlib.metadata,json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))

def main():
    import numpy as np
    from kitchen_safety.geometry_backend import configured_name,get_backend
    from kitchen_safety.fcl_distance import primitive_mesh,pair_distance
    from kitchen_safety.risk_profile_loader import load_config
    settings=load_config('configs/safety_observer_arena.json')
    backend=get_backend(configured_name(settings['config']),settings['config'].get('mesh_distance_backend',{}).get('options'))
    vertices,faces,_=primitive_mesh('Cube',size=1.)
    objects=[]
    for x in (0.,1.25):
        model=backend.BVHModel();model.beginModel();model.addSubModel(vertices,faces);model.endModel()
        objects.append(dict(path='preflight/'+str(x),object=backend.CollisionObject(model,backend.Transform(np.eye(3),[x,0,0]))))
    measured=pair_distance(backend,*objects)
    if abs(measured['distance_m']-.25)>1.e-6:
        raise RuntimeError('Configured backend analytical cube distance failed')
    print(json.dumps(dict(passed=True,scope='CUDA imports, current profile, full-triangle analytical clearance',
        backend=backend.STATS['backend'],device=backend.DEVICE,runtime=backend.runtime_info() if hasattr(backend,'runtime_info') else {'id':'warp'},
        nearest_points_valid=measured['nearest_points_valid'],
        analytical_distance_m=measured['distance_m'],adapter_version=settings['adapter_version'],
        collection_hz=settings['config']['collection_hz'],decision_hz=settings['config']['decision_hz'],
        disabled_categories=settings['config']['disabled_categories'],simulation_started=False),indent=2))
    return 0
if __name__=='__main__':raise SystemExit(main())
