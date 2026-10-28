"""Explicit idealized weighted-base mass model; source lamp geometry is untouched."""
import hashlib
import json
from pathlib import Path

import numpy as np

SOURCE_SHA256='54cd9df85575001be0591bc6d1f6350bd7119f450a050c3f84be03f25b31c045'
VISUAL_DERIVATIVE_SHA256='92f167f8ffcd2b72e83ab21c1d0b0a8f8346cfd3ff1ae36a82cb42ed129fff41'


def verify_lamp_source(source):
    from pxr import Usd
    source=Path(source)
    digest=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
    source_hash=digest(source)
    if source_hash==SOURCE_SHA256:
        return dict(source_sha256=source_hash,material_only_derivative=False)
    if source_hash!=VISUAL_DERIVATIVE_SHA256:
        raise ValueError('Weighted-base recipe is bound to the reviewed lamp asset')
    canonical=Path(__file__).resolve().parents[3]/'assets/extensions/controls/visual_20261027/reading_lamp_r01.usda'
    if digest(canonical)!=SOURCE_SHA256:raise ValueError('Canonical lamp source changed')
    original=Usd.Stage.Open(str(canonical));revised=Usd.Stage.Open(str(source))
    def prims(stage):return {str(p.GetPath()):p for p in stage.Traverse() if '/Materials' not in str(p.GetPath())}
    before,after=prims(original),prims(revised)
    if set(before)!=set(after):raise ValueError('Nonmaterial lamp hierarchy changed')
    for path,left in before.items():
        right=after[path]
        if left.GetAllMetadata()!=right.GetAllMetadata():raise ValueError('Nonmaterial lamp metadata changed: '+path)
        if {p.GetName() for p in left.GetProperties()}!={p.GetName() for p in right.GetProperties()}:
            raise ValueError('Nonmaterial lamp properties changed: '+path)
        for attr in left.GetAttributes():
            counterpart=right.GetAttribute(attr.GetName())
            times=attr.GetTimeSamples()
            if (attr.GetAllMetadata()!=counterpart.GetAllMetadata() or attr.Get()!=counterpart.Get() or
                    attr.GetConnections()!=counterpart.GetConnections() or times!=counterpart.GetTimeSamples() or
                    any(attr.Get(t)!=counterpart.Get(t) for t in times)):
                raise ValueError('Nonmaterial lamp attribute changed: '+str(attr.GetPath()))
        for relation in left.GetRelationships():
            counterpart=right.GetRelationship(relation.GetName())
            if relation.GetTargets()!=counterpart.GetTargets() or relation.GetAllMetadata()!=counterpart.GetAllMetadata():
                raise ValueError('Nonmaterial lamp relationship changed: '+str(relation.GetPath()))
    for key in ('metersPerUnit','upAxis','defaultPrim'):
        if revised.HasAuthoredMetadata(key) and original.GetMetadata(key)!=revised.GetMetadata(key):
            raise ValueError('Lamp stage units/root changed')
    return dict(source_sha256=source_hash,material_only_derivative=True,
                canonical_source_sha256=SOURCE_SHA256,nonmaterial_properties_identical=True,
                normalized_stage_metadata=dict(metersPerUnit=1.,upAxis='Z'),
                metadata_note='Reviewed shader-only sublayer omits units/upAxis; the mass overlay authors canonical metre/Z metadata explicitly.')


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
    from pxr import Gf,Usd,UsdGeom,UsdPhysics
    from scipy.spatial.transform import Rotation
    source=Path(source).resolve()
    digest=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
    source_hash=digest(source)
    source_verification=verify_lamp_source(source)
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
    UsdGeom.SetStageMetersPerUnit(stage,1.)
    UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
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
        source_verification=source_verification,
        geometry_and_collision_unchanged=True,pose_unchanged=True,kinematic_lock_added=False,
        runtime_state_setters=False)
    manifest.write_text(json.dumps(info,indent=2)+'\n')
    return output,info
