"""Validated camera/inspection-only additions; cannot author scene geometry."""
import hashlib
import json
import math


def validate_camera_overrides(cameras):
    if not isinstance(cameras,list) or not cameras:
        raise ValueError('camera_overrides must be a nonempty list')
    result=[]
    permitted={'kind','eye','target','focal_length_mm','exposure_ev','required_entities','label'}
    for item in cameras:
        if not isinstance(item,dict) or set(item)-permitted:
            raise ValueError('Only explicit camera/inspection fields are permitted')
        if item.get('kind') not in ('overview','operation_side','operation_top'):
            raise ValueError('Unknown camera view kind')
        for name in ('eye','target'):
            if not isinstance(item.get(name),list) or len(item[name])!=3 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in item[name]):
                raise ValueError('Camera coordinates must be three finite numbers')
        if sum((a-b)**2 for a,b in zip(item['eye'],item['target']))<.01:
            raise ValueError('Camera eye and target are too close')
        focal=item.get('focal_length_mm',24.)
        ev=item.get('exposure_ev',0.)
        if not isinstance(focal,(int,float)) or not 8<=focal<=100 or not isinstance(ev,(int,float)) or not -6<=ev<=0:
            raise ValueError('Camera focal length or exposure compensation out of range')
        if not isinstance(item.get('required_entities',[]),list) or not all(isinstance(n,str) for n in item.get('required_entities',[])):
            raise ValueError('required_entities must list inspection entity names')
        result.append(dict(item,focal_length_mm=float(focal),inspection_override=True))
    return result


def override_fingerprint(row):
    values={key:row.get(key) for key in ('camera_overrides','inspection_prims','camera_overrides_source_sha256')}
    return hashlib.sha256(json.dumps(values,sort_keys=True).encode()).hexdigest() if any(values.values()) else None


def inspection_entities(stage, items):
    from pxr import Usd,UsdGeom
    cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render'])
    entities={}
    for item in items:
        path=item['prim_path']
        if not path.startswith(('/World/robot/','/World/hazard/')):
            raise ValueError('Inspection prim must belong to existing robot or hazard')
        prim=stage.GetPrimAtPath(path)
        if not prim:
            raise ValueError('Missing inspection source prim: '+path)
        box=cache.ComputeWorldBound(prim).ComputeAlignedRange()
        entities[item['name']]=dict(prefix=path,bounds=[list(box.GetMin()),list(box.GetMax())],minimum_pixels=item.get('minimum_pixels',150))
    return entities
