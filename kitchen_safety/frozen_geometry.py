"""Rebuild HS/PT/RS geometry from per-episode frozen evidence, without USD/Kit.

Human applicability and independent robot geometry come from the binding,
not a hard-coded scenario ID. No missing geometry is replaced by a bbox.
"""
import hashlib,itertools,json
import numpy as np
from kitchen_safety import warp_mesh_backend as backend
from kitchen_safety.hs_geometry import HumanDistanceGeometry
from kitchen_safety.pt_geometry import PropertyDistanceGeometry
from kitchen_safety.rs_evidence import SelfCollisionGeometry
from kitchen_safety.fcl_distance import GeometryBatch

def load_geometry(episode,cache):
    b=json.loads((episode/'safety_observer_binding.json').read_text())['binding']
    human=bool(b['human_inventory']['observed_human_paths'])
    hs=HumanDistanceGeometry.__new__(HumanDistanceGeometry) if human else None
    robot_geometry=hs or HumanDistanceGeometry.__new__(HumanDistanceGeometry)
    robot_meta=b['hs_geometry'] if human else b.get('rs_authored_geometry')
    if robot_meta is None:
        raise ValueError('Nonhuman raw binding lacks rs_authored_geometry; recapture with the general raw workflow')
    pt=PropertyDistanceGeometry.__new__(PropertyDistanceGeometry)
    models={}
    for geometry,meta in ((robot_geometry,robot_meta),(pt,b['pt_geometry'])):
        geometry.fcl=backend;geometry.root=b['robot_prim_path'].rsplit('/',1)[0]
        geometry.errors=[];geometry.entries=[];geometry.metadata=meta
        for original in meta['geometry']:
            e=dict(original);key=e['fingerprint']
            if key not in models:
                data=np.load(cache/(key+'.npz'),allow_pickle=False)
                actual=hashlib.sha256(np.ascontiguousarray(data['vertices'],dtype=np.float64).tobytes()+np.ascontiguousarray(data['faces'],dtype=np.int32).tobytes()).hexdigest()
                if actual!=key:raise ValueError('Frozen mesh fingerprint mismatch: '+key)
                m=backend.BVHModel()
                m.beginModel();m.addSubModel(data['vertices'],data['faces']);m.endModel();models[key]=m
            m=models[key];e['model']=m;e['object']=backend.CollisionObject(m)
            lo=m.vertices.min(0);hi=m.vertices.max(0)
            e['corners']=np.array(list(itertools.product(*zip(lo,hi))))
            if geometry is pt:
                e['last_pose']=None
                e['key']=(e['actor'].rsplit('/',1)[-1] if e['group']=='robot' else e['actor'][len(geometry.root)+1:]) if e['actor'] else None
            geometry.entries.append(e)
    robot_geometry.ee_names=set(robot_meta['ee_body_names'])
    robot_geometry.fixed_body_names=set(robot_meta['fixed_body_names'])
    robot_geometry.dynamic_pairs_enabled=robot_meta.get('dynamic_pairs_enabled',False)
    robot_geometry.exempt_fixed_distance=robot_meta.get('exempt_fixed_distance',False)
    audit_keys={'matched','resolved_robot_collider','resolved_human_collider'}
    robot_geometry.excluded_pairs={(e['resolved_robot_collider'],e['resolved_human_collider']):
        {k:v for k,v in e.items() if k not in audit_keys}
        for e in robot_meta.get('excluded_pairs',[]) if e['matched']}
    inventory=b['pt_asset_inventory'];pt.assets=inventory['objects'];pt.target=inventory['target_object_ids'][0]
    rs=SelfCollisionGeometry.__new__(SelfCollisionGeometry);rs.geometry=robot_geometry
    rs.errors=[];rs.metadata=b['rs_self_geometry'];rs.allowed={tuple(p) for p in rs.metadata['allowed_pairs']}
    rs.owners={}
    for e in robot_geometry.entries:
        if e['group']=='robot':rs.owners.setdefault(e['actor'],[]).append(e)
    rs.pairs=[p for p in itertools.combinations(sorted(rs.owners),2) if p not in rs.allowed]
    return b,hs,pt,rs

def run_frame(hs,pt,rs,poses,objects):
    batch=GeometryBatch(backend)
    p=pt.prepare(poses,objects,batch)
    h=hs.prepare(poses,objects,batch) if hs is not None else None
    r=rs.prepare(poses,objects,batch)
    batch.execute()
    return h() if h else None,p(),r()
