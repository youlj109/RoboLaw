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
