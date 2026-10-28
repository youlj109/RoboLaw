"""Source measured upper-wall/rim grasp and original sink-floor placement."""
import numpy as np

from .task_motion import pose_matrix
from .task_robot_spheres import open_gripper_minimum_z


def bowl_rim_geometry(geometry):
    # A central y=0 slice has separate inner/outer wall surfaces. Intersect it
    # again 2 mm below the actual highest source rim to measure wall thickness.
    sections = []
    for triangle in geometry.vertices[geometry.triangles]:
        points = []
        for a, b in ((triangle[0], triangle[1]), (triangle[1], triangle[2]), (triangle[2], triangle[0])):
            if (a[1] <= 0 < b[1]) or (b[1] <= 0 < a[1]):
                points.append(a+(b-a)*(-a[1]/(b[1]-a[1])))
        if len(points) == 2:
            sections.append(points)
    z = geometry.high[2]-.002
    intersections = []
    for a, b in sections:
        if (a[2] <= z < b[2]) or (b[2] <= z < a[2]):
            intersections.append(a+(b-a)*((z-a[2])/(b[2]-a[2])))
    negative_side = np.asarray([point for point in intersections if point[0] < 0])
    if negative_side.shape != (2, 3):
        raise ValueError("Original bowl negative-X upper rim must expose exactly inner and outer source intersections")
    low, high = negative_side.min(0), negative_side.max(0)
    width = float(high[0]-low[0])
    if not .001 < width < .01:
        raise ValueError("Source upper rim thickness outside supported narrow pinch geometry")
    return dict(center_local_m=((low+high)/2).tolist(), wall_thickness_m=width,
        inner_outer_source_intersections_local_m=negative_side.tolist(), slice_z_local_m=float(z),
        contact_region_local_m=[[float(geometry.low[0]-.001), -.03, float(geometry.high[2]-.03)],
                                [-.06, .03, float(geometry.high[2]+.001)]],
        source_sha256=geometry.provenance["source_sha256"],
        meaning="Negative-X upper sidewall/rim pinch; one inner finger enters the bowl cavity. Original 161 mm bowl cannot be gripped across its diameter by an 85 mm gripper.")


def bowl_rim_grasp(pose, geometry, world):
    source = pose_matrix(pose)
    info = bowl_rim_geometry(geometry)
    center = source[:3, :3]@info["center_local_m"]+source[:3, 3]
    jaw = source[:3, 0].copy()
    jaw[2] = 0.
    if np.linalg.norm(jaw) < .5:
        raise ValueError("Original upright bowl frame is required for source rim approach")
    jaw /= np.linalg.norm(jaw)
    down = np.array([0., 0., -1.])
    rotation = np.column_stack((down, jaw, np.cross(down, jaw)))
    sphere_min, spheres = open_gripper_minimum_z(rotation)
    local_links = world._gripper_links(dict(zip(world.GRIPPER_JOINT_NAMES, np.zeros(6))))
    points = []
    for body, _, vertices in world.gripper_models:
        transform = local_links[body]
        points.append((vertices@transform[:3, :3].T+transform[:3, 3]-[.131, 0., 0.])@rotation.T)
    source_min = float(np.concatenate(points)[:, 2].min())
    retained_min = min(sphere_min, source_min, -(.145-.131+.012))
    supports = [item for item in world.objects if item[0] == world.contact_policy.source_support]
    if len(supports) != 1:
        raise ValueError("Original bowl support must be the exact CapSurface_02 collider")
    target = np.eye(4)
    target[:3, :3], target[:3, 3] = rotation, center
    target[2, 3] = max(center[2], supports[0][3][2]+world.default_clearance+.001-retained_min)
    info.update(target_tcp_world_m=target[:3, 3].tolist(), source_rim_world_m=center.tolist(),
        source_finger_minimum_tcp_z_m=source_min, retained_minimum_tcp_z_m=retained_min,
        curobo_sphere_geometry=spheres, target_raise_m=float(target[2, 3]-center[2]),
        noncontact_clearance_m=world.default_clearance, additional_target_guard_m=.001)
    return target, info


def sink_reference_bounds(world):
    # The sink destination is a source kitchen reference, not a rigid scene
    # object root_state. Keep the source collider and all wall geometry.
    path = world.contact_policy.destination_name
    rows = [row for row in world.objects if row[0] == path or row[0].startswith(path+"/")]
    if not rows:
        raise ValueError("Missing original sink BasinBottom collision geometry")
    return np.min([row[2] for row in rows], axis=0), np.max([row[3] for row in rows], axis=0)


def sink_placement_object_pose(payload_pose, geometry, world, gap=.003, center_xy=None):
    if gap != .003:
        raise ValueError("Original source-support placement profile fixes release gap at 3 mm")
    pose = pose_matrix(payload_pose)
    low, high = sink_reference_bounds(world)
    rotated = geometry.vertices@pose[:3, :3].T
    source_center = (low[:2]+high[:2])/2
    center_xy = source_center if center_xy is None else np.asarray(center_xy, dtype=float)
    if center_xy.shape != (2,) or not np.isfinite(center_xy).all():
        raise ValueError("Sink placement centre must be a finite world XY pair")
    pose[:2, 3] = center_xy-(rotated[:, :2].min(0)+rotated[:, :2].max(0))/2
    pose[2, 3] = high[2]+gap-rotated[:, 2].min()
    vertices = rotated+pose[:3, 3]
    if np.any(vertices[:, :2].min(0) < low[:2]) or np.any(vertices[:, :2].max(0) > high[:2]):
        raise ValueError("Carried source bowl does not fit within the original sink floor XY bounds")
    return pose, dict(original_sink_reference=world.contact_policy.destination_name, source_bounds_m=[low.tolist(), high.tolist()],
        requested_source_floor_center_xy_m=center_xy.tolist(), offset_from_source_floor_center_m=(center_xy-source_center).tolist(),
        release_gap_m=gap, full_source_bowl_xy_within_floor=True,
        scope="Source-floor top candidate only; full rim/wall/robot path and original support-force predicate remain mandatory")
