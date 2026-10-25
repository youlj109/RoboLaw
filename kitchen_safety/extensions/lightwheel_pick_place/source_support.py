"""Local source-support classification calibrated from the native resting fruit.

This does not change any external collision margin. The authored raw collision
hull penetrates the rendered top by about 3.9 mm in the native PhysX rest pose;
rolling during liftoff must not be mistaken for a new forbidden table collision.
The exception ends permanently after geometric separation and loss of contact.
"""
import numpy as np

SOURCE_SURFACE='/World/envs/env_0/kitchen/counter_main_main_group/top_geometry_right'
JITTER_M=.0005
MAX_XY_M=.05
DETACH_CLEARANCE_M=.002

def support_allowed(*,phase,detached,hull_min_z,root,initial_hull_min_z,initial_root):
    root=np.asarray(root,dtype=float);initial_root=np.asarray(initial_root,dtype=float)
    return bool(phase in ('pregrasp','approach','close','lift') and not detached
        and np.linalg.norm(root[:2]-initial_root[:2])<MAX_XY_M
        and hull_min_z>=initial_hull_min_z-JITTER_M)

def update_detached(*,detached,hull_min_z,surface_top,source_contact_count):
    return bool(detached or (hull_min_z>=surface_top+DETACH_CLEARANCE_M and source_contact_count==0))


def near_support_slab(*, hull_min_z, hull_max_z, surface_bottom, surface_top, clearance_m=.002):
    """Height overlap with the finite source collider, including its margin.

    A fruit below a high shelf is not in contact with that shelf merely because
    its height is less than the shelf top. Full-world collision remains separate.
    """
    return bool(hull_min_z < surface_top + clearance_m
                and hull_max_z >= surface_bottom - clearance_m)


def native_contact_is_shallow(contact, *, tolerance_m=JITTER_M):
    """Actual exact-source cooked-contact separations, not an orientation-dependent raw vertex height."""
    count=contact.get('count',0)
    if not isinstance(count,int) or count<1:return False
    for key,width in (('points_world_m',3),('normals_world',3),('normal_forces_n',1),('separations_m',1)):
        values=np.asarray(contact.get(key,[]),dtype=float)
        if values.size!=count*width or not np.isfinite(values).all():return False
    normals=np.asarray(contact['normals_world'],float).reshape(-1,3)
    return bool(np.allclose(np.linalg.norm(normals,axis=1),1.,atol=1e-4) and np.min(contact['separations_m'])>=-tolerance_m)
