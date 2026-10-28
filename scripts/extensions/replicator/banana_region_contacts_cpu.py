"""Source-mesh contact-region audit without trusting FCL's approximate points.

Each original banana triangle is clipped against the unchanged six central
region planes. Resulting inside/outside triangle surfaces partition the whole
source surface (shared plane edges retained conservatively). Collision against
both sets checks whether actual source surface pairs lie outside the region.
"""
import fcl
import numpy as np


def clip_polygon(polygon, axis, boundary, keep_below):
    result = []
    if not len(polygon):
        return result
    sign = 1. if keep_below else -1.
    for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
        da, db = sign*(a[axis]-boundary), sign*(b[axis]-boundary)
        inside_a, inside_b = da <= 0., db <= 0.
        if inside_a:
            result.append(a)
        if inside_a != inside_b:
            fraction = da/(da-db)
            point = a+fraction*(b-a)
            point[axis] = boundary
            result.append(point)
    return result


def triangulate(polygons):
    triangles = []
    for polygon in polygons:
        for index in range(1, len(polygon)-1):
            triangle = np.array([polygon[0], polygon[index], polygon[index+1]])
            if np.linalg.norm(np.cross(triangle[1]-triangle[0], triangle[2]-triangle[0])) > 1e-18:
                triangles.append(triangle)
    return np.asarray(triangles).reshape(-1, 3, 3)


def triangle_model(triangles):
    if not len(triangles):
        return None
    vertices = triangles.reshape(-1, 3)
    faces = np.arange(len(vertices), dtype=np.int32).reshape(-1, 3)
    model = fcl.BVHModel()
    model.beginModel(len(vertices), len(faces))
    model.addSubModel(vertices, faces)
    model.endModel()
    return model


class SourceRegionContacts:
    def __init__(self, geometry, region, finger_meshes):
        low, high = map(np.asarray, region)
        original = geometry.vertices[geometry.triangles]
        inside, outside = [], []
        for triangle in original:
            polygon = list(triangle)
            for axis in range(3):
                for boundary, keep_below in ((low[axis], False), (high[axis], True)):
                    rejected = clip_polygon(polygon, axis, boundary, not keep_below)
                    if len(rejected) >= 3:
                        outside.append(rejected)
                    polygon = clip_polygon(polygon, axis, boundary, keep_below)
                    if len(polygon) < 3:
                        break
                if len(polygon) < 3:
                    break
            if len(polygon) >= 3:
                inside.append(polygon)
        inside, outside = triangulate(inside), triangulate(outside)
        area = lambda triangles: float(np.linalg.norm(np.cross(triangles[:,1]-triangles[:,0],
            triangles[:,2]-triangles[:,0]), axis=1).sum()/2)
        original_area, split_area = area(original), area(inside)+area(outside)
        if abs(original_area-split_area) > max(1e-12, original_area*1e-9):
            raise ValueError("Source region clipping failed original triangle area conservation")
        self.models = dict(inside=triangle_model(inside), outside=triangle_model(outside))
        self.finger_meshes = finger_meshes
        self.provenance = dict(region_local_m=region, original_triangles=len(original),
            inside_triangles=len(inside), outside_triangles=len(outside),
            original_area_m2=original_area, partition_area_m2=split_area,
            partition_area_absolute_error_m2=abs(original_area-split_area),
            source_sha256=geometry.provenance["source_sha256"],
            method="Clip every source triangle by six unchanged region planes; use actual source triangle-pair collision, never FCL approximate contact.pos for classification",
            shared_boundary="Boundary triangles/edges retained in both halfspaces; outside-touch rejection is conservative",
            physical_contact_verified=False)

    def at(self, world, kin, q):
        source = world.payload_pose
        parts = world._gripper_links(dict(zip(world.GRIPPER_JOINT_NAMES, world.gripper_joint_positions)))
        palm = kin.links(q)[world.gripper_root]
        payloads = {name:fcl.CollisionObject(model, fcl.Transform(source[:3,:3],source[:3,3]))
                    for name,model in self.models.items() if model is not None}
        fingers = {}
        for body, meshes in self.finger_meshes.items():
            transform = palm@parts[body]
            counts = dict(inside=0, outside=0)
            for _, model in meshes:
                finger = fcl.CollisionObject(model, fcl.Transform(transform[:3,:3],transform[:3,3]))
                for name, payload in payloads.items():
                    response = fcl.CollisionResult()
                    count = fcl.collide(finger,payload,fcl.CollisionRequest(num_max_contacts=8192,enable_contact=True),response)
                    if count >= 8192 or len(response.contacts) >= 8192:
                        raise ValueError("Source region collision pair buffer saturated")
                    counts[name] += int(count)
            fingers[body.rsplit("/",1)[-1]] = dict(**counts, central_only=counts["inside"]>0 and counts["outside"]==0)
        return dict(fingers=fingers, bilateral_all_central=all(row["central_only"] for row in fingers.values()))
