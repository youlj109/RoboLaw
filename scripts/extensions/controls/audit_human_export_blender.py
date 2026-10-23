"""Independently import a rebuilt human and compare its feet to the source."""
import argparse
import hashlib
import json
import sys
from pathlib import Path

import bpy
from mathutils.kdtree import KDTree

parser = argparse.ArgumentParser()
parser.add_argument('--asset-dir', required=True)
parser.add_argument('--report', required=True)
args = parser.parse_args(sys.argv[sys.argv.index('--') + 1:])
root = Path.cwd()
asset = root / args.asset_dir
report = Path(args.report)
if report.exists():
    raise FileExistsError(report)


def feet(objects):
    graph = bpy.context.evaluated_depsgraph_get()
    result = []
    for obj in objects:
        if obj.type != 'MESH':
            continue
        evaluated = obj.evaluated_get(graph)
        mesh = evaluated.to_mesh()
        result.extend(obj.matrix_world @ v.co for v in mesh.vertices
                      if (obj.matrix_world @ v.co).z <= .2)
        evaluated.to_mesh_clear()
    return result


def max_distance(source, target):
    if not source or not target:
        raise ValueError('Missing foot vertices')
    tree = KDTree(len(target))
    for index, point in enumerate(target):
        tree.insert(point, index)
    tree.balance()
    return max(tree.find(point)[2] for point in source)


before = set(bpy.data.objects)
original = feet(before)
bpy.ops.wm.usd_import(filepath=str(asset / 'person.usdc'))
imported = set(bpy.data.objects) - before
exported = feet(imported)
forward = max_distance(original, exported)
reverse = max_distance(exported, original)
meshes = [obj for obj in imported if obj.type == 'MESH']
pose = json.loads((asset / 'pose.json').read_text())
checks = {
    'six_exported_meshes': len(meshes) == 6,
    'feet_unchanged': max(forward, reverse) < 1e-5,
    'foot_vertex_count_unchanged': len(original) == len(exported),
    'non_arm_bones_unchanged': pose['max_unchanged_bone_error_m'] < 1e-5,
    'wrist_target_reached': pose['wrist_error_m'] < 1e-5,
}
result = {
    'passed': all(checks.values()), 'checks': checks,
    'source_vertices_below_20cm': len(original),
    'exported_vertices_below_20cm': len(exported),
    'source_to_export_max_m': forward, 'export_to_source_max_m': reverse,
    'mesh_vertex_counts': {obj.name: len(obj.data.vertices) for obj in meshes},
    'source_sha256': hashlib.sha256((root / 'assets/people/mpfb_corridor/posed.blend').read_bytes()).hexdigest(),
    'export_sha256': hashlib.sha256((asset / 'person.usdc').read_bytes()).hexdigest(),
    'scope': 'Independent Blender USD import; bidirectional world-space feet comparison. '
             'Bone checks use the separately saved pose generator report. '
             'Not byte equivalence to the unavailable upstream USDC or a clearance certificate.',
}
report.parent.mkdir(parents=True, exist_ok=True)
report.write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result, indent=2))
if not result['passed']:
    raise RuntimeError('Rebuilt human geometry audit failed')
