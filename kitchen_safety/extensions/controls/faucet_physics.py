"""Traceable idealized passive balance for the background faucet cartridge."""
import hashlib
import json
import math
from pathlib import Path


def balanced_faucet(source, output_dir):
    from pxr import Gf, Usd, UsdPhysics
    source = Path(source).resolve()
    digest = lambda path: hashlib.sha256(Path(path).read_bytes()).hexdigest()
    recipe_hash = digest(__file__)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    output = output_dir / f'faucet_balanced_v1_{digest(source)[:16]}_{recipe_hash[:12]}.usda'
    manifest = output.with_suffix('.json')
    if output.exists() and manifest.exists():
        info = json.loads(manifest.read_text())
        if info['overlay_sha256'] != digest(output):
            raise ValueError('Changed cached faucet overlay')
        return output, info
    stage = Usd.Stage.CreateNew(str(output))
    stage.GetRootLayer().subLayerPaths = [str(source)]
    stage.SetDefaultPrim(stage.GetPrimAtPath('/world'))
    body = stage.GetPrimAtPath('/world/sink_left_group/Sink009_handle')
    joint = UsdPhysics.Joint(stage.GetPrimAtPath(str(body.GetPath()) + '/handle_joint'))
    if not body or not joint:
        raise ValueError('Missing original two-axis faucet handle')
    anchor = joint.GetLocalPos1Attr().Get()
    mass = UsdPhysics.MassAPI(body)
    info = {
        'recipe': 'balanced-passive-faucet-v1', 'recipe_sha256': recipe_hash,
        'source': str(source), 'source_sha256': digest(source),
        'body': str(body.GetPath()), 'joint': str(joint.GetPath()),
        'source_mass_kg': mass.GetMassAttr().Get(),
        'source_com_attribute': str(mass.GetCenterOfMassAttr().Get()),
        'balanced_com_body_local_m': list(anchor), 'drives': {},
        'model': 'Idealized mass concentrated at the two-axis cartridge pivot; '
                 'not manufacturer calibration. Geometry, mass and joint limits unchanged. '
                 'Zero positional stiffness and real viscous damping; no runtime pose/joint setters.',
    }
    UsdPhysics.MassAPI.Apply(body).CreateCenterOfMassAttr(Gf.Vec3f(*anchor))
    for axis in ('rotX', 'rotY'):
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
        info['drives'][axis] = {
            'original_max_force': drive.GetMaxForceAttr().Get(),
            'original_stiffness': drive.GetStiffnessAttr().Get(),
            'original_damping': drive.GetDampingAttr().Get(),
            'angular_damping_nm_s_per_rad': .05,
            'limit_low_degrees': joint.GetPrim().GetAttribute(f'limit:{axis}:physics:low').Get(),
            'limit_high_degrees': joint.GetPrim().GetAttribute(f'limit:{axis}:physics:high').Get(),
        }
        drive.CreateTypeAttr('force')
        drive.CreateStiffnessAttr(0.)
        drive.CreateDampingAttr(.05 * math.pi / 180.)
        drive.CreateTargetVelocityAttr(0.)
        drive.CreateMaxForceAttr(10000.)
    stage.GetRootLayer().Save()
    info['overlay_sha256'] = digest(output)
    manifest.write_text(json.dumps(info, indent=2) + '\n')
    return output, info
