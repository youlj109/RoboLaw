"""Source banana grasp/placement targets; no simulator writes or success claims."""
import numpy as np
from scipy.spatial.transform import Rotation


def pose_matrix(pose):
    pose = np.asarray(pose, float)
    if pose.shape != (7,) or not np.isfinite(pose).all() or np.linalg.norm(pose[3:]) < 1e-8:
        raise ValueError("Expected observed xyz + xyzw pose")
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(pose[3:]).as_matrix()
    result[:3, 3] = pose[:3]
    return result


def banana_midbody(geometry):
    """Intersect the source triangles with local y=0, avoiding the empty bend."""
    triangles = geometry.vertices[geometry.triangles]
    intersections = []
    for i, j in ((0, 1), (1, 2), (2, 0)):
        a, b = triangles[:, i], triangles[:, j]
        mask = ((a[:, 1] <= 0) & (b[:, 1] > 0)) | ((b[:, 1] <= 0) & (a[:, 1] > 0))
        a, b = a[mask], b[mask]
        if len(a):
            intersections.append(a + (b-a) * (-a[:, 1] / (b[:, 1]-a[:, 1]))[:, None])
    if not intersections:
        raise ValueError("Source banana has no central cross-section")
    points = np.concatenate(intersections)
    lo, hi = points.min(0), points.max(0)
    if not .01 < hi[0]-lo[0] < .085:
        raise ValueError("Measured banana central width does not fit the original gripper")
    return dict(center_local_m=((lo+hi)/2).tolist(), section_bounds_local_m=[lo.tolist(), hi.tolist()],
        contact_region_local_m=[[-.06, -.04, -.025], [0., .04, .025]],
        source_sha256=geometry.provenance["source_sha256"],
        meaning="Measured y=0 central section; bilateral physical contacts inside the 80 mm central band still required")


def top_down_grasp(pose, geometry, world):
    """Jaw axis follows source x; local TCP +x approaches vertically downward.

    The TCP is raised only as required by the actual source gripper's lowest
    open geometry and the unchanged counter clearance. The robot still has to
    establish contacts in the measured central band; this target is not proof.
    """
    source = pose_matrix(pose)
    info = banana_midbody(geometry)
    local = np.asarray(info["center_local_m"])
    center = source[:3, :3] @ local + source[:3, 3]
    jaw = source[:3, 0].copy()
    jaw[2] = 0.
    if np.linalg.norm(jaw) < .5:
        raise ValueError("Banana source x axis is not sufficiently horizontal for this grasp")
    jaw /= np.linalg.norm(jaw)
    approach = np.array([0., 0., -1.])
    rotation = np.column_stack((approach, jaw, np.cross(approach, jaw)))
    links = world._gripper_links(dict(zip(world.GRIPPER_JOINT_NAMES, np.zeros(6))))
    gripper_points = []
    for body, _, vertices in world.gripper_models:
        transform = links[body]
        points = vertices @ transform[:3, :3].T + transform[:3, 3] - [.131, 0., 0.]
        gripper_points.append(points @ rotation.T)
    source_min_relative_z = float(np.concatenate(gripper_points)[:, 2].min())
    # Preserve the baseline's conservative finger capsules as well as source
    # part hulls: their distal end x=.145, r=.012 extends 26 mm beyond TCP.
    # Do not silently delete those capsules merely because the source hull is
    # shorter. The 1 mm target guard adds clearance; it changes no safety rule.
    from .task_robot_spheres import open_gripper_minimum_z
    sphere_min_relative_z, sphere_geometry = open_gripper_minimum_z(rotation)
    planner_min_relative_z = min(source_min_relative_z, -(.145-.131+.012), sphere_min_relative_z)
    target_guard = .001
    supports = [item for item in world.objects if item[0] == world.contact_policy.source_support]
    if len(supports) != 1:
        raise ValueError("Expected exactly the original source support collider")
    support_z = supports[0][3][2]
    target = np.eye(4)
    target[:3, :3], target[:3, 3] = rotation, center
    target[2, 3] = max(center[2], support_z + world.default_clearance + target_guard - planner_min_relative_z)
    info.update(target_tcp_world_m=target[:3, 3].tolist(), source_midbody_world_m=center.tolist(),
        source_gripper_lowest_relative_tcp_z_m=source_min_relative_z,
        retained_planner_envelope_lowest_relative_tcp_z_m=float(planner_min_relative_z),
        original_curobo_gripper_sphere_geometry=sphere_geometry,
        target_guard_above_required_clearance_m=target_guard,
        source_support_top_m=float(support_z), tcp_raise_for_source_fingers_m=float(target[2, 3]-center[2]),
        orientation="TCP +X down; jaw Y parallel observed banana local X projected into horizontal plane")
    return target, info


def banana_bowl_lower_target(payload_pose, bowl_pose, payload_geometry, bowl_geometry, *, gap=.003):
    """Vertical first-support target from all source vertices and bowl triangles.

    Overhanging banana tips are permitted by the *original* center/support
    predicate. They do not need to cast onto the bowl. Every overlapping source
    vertex does participate, unlike a bottle-only lowest-z band heuristic.
    This candidate must pass full path and measured execution collision checks.
    """
    if not np.isfinite(gap) or gap != .003:
        raise ValueError("This profile fixes its source-triangle release gap at 3 mm")
    payload, bowl = pose_matrix(payload_pose), pose_matrix(bowl_pose)
    vertices = payload_geometry.vertices @ payload[:3, :3].T + payload[:3, 3]
    points = bowl_geometry.vertices @ bowl[:3, :3].T + bowl[:3, 3]
    triangles = points[bowl_geometry.triangles]
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    v, w = b[:, :2]-a[:, :2], c[:, :2]-a[:, :2]
    determinant = v[:, 0]*w[:, 1]-w[:, 0]*v[:, 1]
    xylo, xyhi = triangles[:, :, :2].min(1), triangles[:, :, :2].max(1)
    largest, critical, hits = -np.inf, None, 0
    for point in vertices:
        ids = np.flatnonzero((np.abs(determinant) > 1e-12) & np.all(point[:2] >= xylo-1e-9, axis=1)
                            & np.all(point[:2] <= xyhi+1e-9, axis=1))
        if not len(ids):
            continue
        delta = point[:2]-a[ids, :2]
        u = (delta[:, 0]*w[ids, 1]-w[ids, 0]*delta[:, 1])/determinant[ids]
        t = (v[ids, 0]*delta[:, 1]-delta[:, 0]*v[ids, 1])/determinant[ids]
        inside = (u >= -1e-8) & (t >= -1e-8) & (u+t <= 1+1e-8)
        if not inside.any():
            continue
        ids, u, t = ids[inside], u[inside], t[inside]
        highest = np.max(a[ids, 2]+u*(b[ids, 2]-a[ids, 2])+t*(c[ids, 2]-a[ids, 2]))
        correction = float(highest-point[2]+gap)
        hits += 1
        if correction > largest:
            largest, critical = correction, dict(vertex_world_m=point.tolist(), bowl_surface_z_m=float(highest))
    if not hits or not -.3 <= largest <= .002:
        raise ValueError(f"No legal downward source-support target: hits={hits}, correction={largest}")
    return float(largest), dict(vertical_delta_m=float(largest), gap_m=gap,
        overlapping_source_vertices=hits, total_source_vertices=len(vertices), critical=critical,
        original_success_semantics="Bounds center over destination + upward contact support + settled; full containment is not required",
        representation="Original enabled source collision triangles; not PhysX cooked shapes")
