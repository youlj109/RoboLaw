"""Anatomical arm posing helpers used inside Blender, with metric world points."""
import math
from mathutils import Vector, Matrix
import bpy


def bone_point(rig, name):
    return rig.matrix_world @ rig.pose.bones[name].head


def _frame(first, second):
    first = first.normalized()
    second = (second - first * first.dot(second)).normalized()
    third = first.cross(second).normalized()
    return Matrix((first, second, third)).transposed()


def _set_world_rotation(rig, name, rotation):
    bone = rig.pose.bones[name]
    world = rig.matrix_world @ bone.matrix
    transform = Matrix.LocRotScale(world.translation, rotation.to_quaternion(), world.to_scale())
    bone.matrix = rig.matrix_world.inverted() @ transform
    bpy.context.view_layer.update()


def hinge_arm(rig, side, elbow, wrist):
    """Keep all three joint points while using the source rig's elbow hinge.

    The MPFB game-engine rest elbow is already flexed by about 44–46 degrees and
    its hinge is not exactly a named bone Euler axis. Derive that axis from the
    rest upper/forearm directions, then rotate only about that anatomical axis.
    """
    upper = rig.pose.bones['upperarm_' + side]
    lower = rig.pose.bones['lowerarm_' + side]
    shoulder = bone_point(rig, upper.name)
    du = (Vector(elbow) - shoulder).normalized()
    dl = (Vector(wrist) - Vector(elbow)).normalized()
    upper_rest = upper.bone.matrix_local.to_3x3().normalized()
    lower_rest = lower.bone.matrix_local.to_3x3().normalized()
    yu = upper_rest @ Vector((0, 1, 0))
    yl = lower_rest @ Vector((0, 1, 0))
    rest_angle = yu.angle(yl)
    target_angle = du.angle(dl)
    hinge_rest = yu.cross(yl).normalized()
    hinge_lower = lower_rest.transposed() @ hinge_rest
    bend = Matrix.Rotation(target_angle - rest_angle, 3, hinge_lower)
    relative = upper_rest.transposed() @ lower_rest
    lower_in_upper = relative @ bend @ Vector((0, 1, 0))
    rotation = _frame(du, dl) @ _frame(Vector((0, 1, 0)), lower_in_upper).transposed()
    _set_world_rotation(rig, upper.name, rotation)
    lower.matrix_basis = bend.to_4x4()
    bpy.context.view_layer.update()
    error = (bone_point(rig, 'hand_' + side) - Vector(wrist)).length
    return {'elbow_flexion_deg': math.degrees(target_angle),
            'rest_elbow_flexion_deg': math.degrees(rest_angle),
            'wrist_target_error_m': error,
            'hinge_axis_lower_local': list(hinge_lower)}


def align_palm(rig, side, fingers, palm_normal):
    """Align the actual metacarpal plane, not just the short hand bone tail."""
    wrist = bone_point(rig, 'hand_' + side)
    index = bone_point(rig, 'index_01_' + side) - wrist
    pinky = bone_point(rig, 'pinky_01_' + side) - wrist
    current_forward = bone_point(rig, 'middle_01_' + side) - wrist
    current_palm = index.cross(pinky) * (1 if side == 'r' else -1)
    desired = _frame(Vector(fingers), Vector(palm_normal))
    actual = _frame(current_forward, current_palm)
    current_rotation = (rig.matrix_world @ rig.pose.bones['hand_' + side].matrix).to_3x3().normalized()
    _set_world_rotation(rig, 'hand_' + side, desired @ actual.transposed() @ current_rotation)
    return palm_metrics(rig, side)


def palm_metrics(rig, side):
    wrist = bone_point(rig, 'hand_' + side)
    index = bone_point(rig, 'index_01_' + side) - wrist
    pinky = bone_point(rig, 'pinky_01_' + side) - wrist
    middle = bone_point(rig, 'middle_01_' + side) - wrist
    normal = (index.cross(pinky) * (1 if side == 'r' else -1)).normalized()
    forearm = wrist - bone_point(rig, 'lowerarm_' + side)
    return {'palm_normal_local': list(normal),
            'palm_down_angle_deg': math.degrees(normal.angle(Vector((0, 0, -1)))),
            'wrist_direction_change_deg': math.degrees(forearm.angle(middle))}


def refine_library_arms(rig, root, pose):
    wrist = bone_point(rig, 'hand_r').copy()
    elbow = bone_point(rig, 'lowerarm_r').copy()
    report = {'right': hinge_arm(rig, 'r', elbow, wrist)}
    forearm = (wrist - elbow).normalized()
    fingers = Vector((forearm.x, forearm.y, 0)).normalized()
    elevation = math.radians(0 if pose == 'palm_down' else 10)
    fingers = fingers * math.cos(elevation) + Vector((0, 0, math.sin(elevation)))
    report['right'].update(align_palm(rig, 'r', fingers, Vector((0, 0, -1))))
    shoulder = bone_point(rig, 'upperarm_l')
    left_elbow = bone_point(rig, 'lowerarm_l')
    left_wrist = bone_point(rig, 'hand_l')
    l1 = (left_elbow - shoulder).length
    l2 = (left_wrist - left_elbow).length
    rotation = root.matrix_world.to_3x3().normalized()
    forward = rotation @ Vector((0, -1, 0))
    outward = rotation @ Vector((1, 0, 0))
    upper_direction = (Vector((0, 0, -1)) + outward * .65 + forward * .05).normalized()
    lower_direction = (upper_direction + forward * .26).normalized()
    left_elbow = shoulder + upper_direction * l1
    left_wrist = left_elbow + lower_direction * l2
    report['left'] = hinge_arm(rig, 'l', left_elbow, left_wrist)
    report['left'].update(align_palm(rig, 'l', lower_direction, -outward))
    report['right_wrist_preservation_error_m'] = (bone_point(rig, 'hand_r') - wrist).length
    return report


def correct_sportsuit_coverage(rig, meshes):
    """Hide only inner layers covered by the compact character's clothing.

    The source sportsuit body mask misses some proximal upper-arm triangles
    after posing. Its close-fitting trousers also intersect the tall shoes01
    boot upper. Extend the existing inner-layer masks instead of inflating or
    moving the garment, forearm, hand, or obstacle silhouette.
    """
    body = next(obj for obj in meshes if obj.name == 'LibraryAdult')
    masks = [modifier for modifier in body.modifiers if modifier.type == 'MASK']
    original_visibility = [modifier.show_viewport for modifier in masks]
    try:
        for modifier in masks:
            modifier.show_viewport = False
        bpy.context.view_layer.update()
        deps = bpy.context.evaluated_depsgraph_get()
        evaluated = body.evaluated_get(deps)
        points = [body.matrix_world @ vertex.co for vertex in evaluated.data.vertices]
        shoulder = bone_point(rig, 'upperarm_r')
        upper = bone_point(rig, 'lowerarm_r') - shoulder
        hidden = []
        for index, point in enumerate(points):
            fraction = (point - shoulder).dot(upper) / upper.length_squared
            radius = (point - shoulder - upper * fraction).length
            if -.12 < fraction < .44 and radius < .085:
                hidden.append(index)
        body.vertex_groups['Delete.female_sportsuit01'].add(hidden, 1., 'REPLACE')
    finally:
        for modifier, visible in zip(masks, original_visibility):
            modifier.show_viewport = visible
    shoe = next(obj for obj in meshes if obj.name.endswith('.shoes01'))
    subdivisions = [modifier for modifier in shoe.modifiers if modifier.type == 'SUBSURF']
    original_visibility = [modifier.show_viewport for modifier in subdivisions]
    try:
        for modifier in subdivisions:
            modifier.show_viewport = False
        bpy.context.view_layer.update()
        deps = bpy.context.evaluated_depsgraph_get()
        points = [shoe.matrix_world @ vertex.co for vertex in shoe.evaluated_get(deps).data.vertices]
        group = shoe.vertex_groups.new(name='VisibleBelowTrouserCuff')
        visible = [index for index, point in enumerate(points) if point.z < .11]
        group.add(visible, 1., 'REPLACE')
    finally:
        for modifier, visible in zip(subdivisions, original_visibility):
            modifier.show_viewport = visible
    mask = shoe.modifiers.new('HideBootUpperInsideTrousers', 'MASK')
    mask.vertex_group = group.name
    bpy.context.view_layer.update()
    return {'extra_hidden_body_vertices': len(hidden), 'boot_upper_hidden_above_m': .11,
            'outer_garment_geometry_unchanged': True}
