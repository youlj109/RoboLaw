"""Explicit idealized weighted-base mass model; source lamp geometry is untouched."""
import hashlib
import json
from pathlib import Path

import numpy as np

SOURCE_SHA256='54cd9df85575001be0591bc6d1f6350bd7119f450a050c3f84be03f25b31c045'


def weighted_lamp_mass_properties():
    # Four mass groups, including the smaller attached parts in their group.
    groups=[('base_and_collar',4.6,[0.,0.,.0175],.16,.035,'z'),
            ('upright',.7,[0.,0.,.615],.014,1.16,'z'),
            ('arm',.2,[0.,.39,1.185],.011,.8,'y'),
            ('shade_socket_bulb',.5,[0.,.78,1.04],.1,.16,'z')]
    mass=sum(g[1] for g in groups)
    center=sum(g[1]*np.asarray(g[2]) for g in groups)/mass
    tensor=np.zeros((3,3))
    for _,m,position,radius,height,axis in groups:
        axial=.5*m*radius**2
        radial=m*(3*radius**2+height**2)/12
        inertia=np.diag([radial,radial,radial])
        inertia['xyz'.index(axis),'xyz'.index(axis)]=axial
        delta=np.asarray(position)-center
        tensor+=inertia+m*(np.dot(delta,delta)*np.eye(3)-np.outer(delta,delta))
    diagonal,axes=np.linalg.eigh(tensor)
    if np.linalg.det(axes)<0:axes[:,0]*=-1
    return dict(mass_kg=mass,center_of_mass_local_m=center.tolist(),
                inertia_tensor_body_kg_m2=tensor.tolist(),diagonal_inertia_kg_m2=diagonal.tolist(),
                principal_axes_matrix=axes.tolist(),mass_groups=groups,
                model='Idealized 6 kg weighted reading lamp: four declared mass groups; head inertia approximated as a cylinder. Not manufacturer calibration.')


def balanced_reading_lamp(source,output_dir):
    from pxr import Gf,Usd,UsdPhysics
    from scipy.spatial.transform import Rotation
    source=Path(source).resolve()
    digest=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
    source_hash=digest(source)
    if source_hash!=SOURCE_SHA256:
        raise ValueError('Weighted-base recipe is bound to the reviewed lamp asset')
    recipe_hash=digest(__file__)
    directory=Path(output_dir)
    directory.mkdir(parents=True,exist_ok=True)
    output=directory/f'lamp_weighted_v1_{source_hash[:16]}_{recipe_hash[:12]}.usda'
    manifest=output.with_suffix('.json')
    if output.exists() and manifest.exists():
        info=json.loads(manifest.read_text())
        if info['overlay_sha256']!=digest(output):raise ValueError('Changed cached lamp mass overlay')
        return output,info
    original=Usd.Stage.Open(str(source))
    root_path=original.GetDefaultPrim().GetPath()
    properties=weighted_lamp_mass_properties()
    if abs(UsdPhysics.MassAPI(original.GetPrimAtPath(root_path)).GetMassAttr().Get()-properties['mass_kg'])>1e-6:
        raise ValueError('Original lamp total mass must be retained')
    stage=Usd.Stage.CreateNew(str(output))
    stage.GetRootLayer().subLayerPaths=[str(source)]
    body=stage.GetPrimAtPath(root_path)
    stage.SetDefaultPrim(body)
    mass=UsdPhysics.MassAPI.Apply(body)
    mass.CreateMassAttr(properties['mass_kg'])
    mass.CreateCenterOfMassAttr(Gf.Vec3f(*properties['center_of_mass_local_m']))
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*properties['diagonal_inertia_kg_m2']))
    q=Rotation.from_matrix(properties['principal_axes_matrix']).as_quat()
    mass.CreatePrincipalAxesAttr(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3])))
    stage.GetRootLayer().Save()
    info=dict(recipe='weighted-reading-lamp-v1',recipe_sha256=recipe_hash,source=str(source),
        source_sha256=source_hash,overlay_sha256=digest(output),**properties,
        geometry_and_collision_unchanged=True,pose_unchanged=True,kinematic_lock_added=False,
        runtime_state_setters=False)
    manifest.write_text(json.dumps(info,indent=2)+'\n')
    return output,info
