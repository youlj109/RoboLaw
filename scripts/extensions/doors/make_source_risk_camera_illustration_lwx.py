"""Illustrate audited source risks on recorded RGB; never replay unsafe physics."""
import argparse
import hashlib
import itertools
import json
import subprocess
from pathlib import Path

import imageio_ffmpeg
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from pxr import UsdGeom
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from kitchen_safety.planning import transform
from scripts.extensions.doors.physical_hazard_geometry import physical_hazards


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def world_points(prim, cache):
    if prim.IsA(UsdGeom.Mesh):
        points = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=float)
    elif prim.IsA(UsdGeom.Cube):
        half = float(UsdGeom.Cube(prim).GetSizeAttr().Get()) / 2
        points = np.asarray(list(itertools.product([-half, half], repeat=3)))
    elif prim.IsA(UsdGeom.Cylinder):
        shape = UsdGeom.Cylinder(prim)
        radius, height = float(shape.GetRadiusAttr().Get()), float(shape.GetHeightAttr().Get())
        axis = 'XYZ'.index(str(shape.GetAxisAttr().Get()))
        points = np.asarray([[radius * np.cos(t), radius * np.sin(t), z]
                             for t in np.linspace(0, 2 * np.pi, 65)[:-1]
                             for z in [-height / 2, height / 2]])
        if axis != 2:
            points[:, [axis, 2]] = points[:, [2, axis]]
    else:
        raise ValueError('Unsupported illustration collider: ' + str(prim.GetPath()))
    matrix = np.asarray(cache.GetLocalToWorldTransform(prim)).T
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('episode', type=Path)
    p.add_argument('--baseline', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--camera', choices=['external_camera', 'external_camera_2'], default='external_camera_2')
    a = p.parse_args()
    if a.output.exists() or a.output.with_suffix('.json').exists():
        raise FileExistsError(a.output)
    read = lambda path: json.loads(path.read_text())
    m = read(a.episode / 'metadata.json')
    bm = read(a.baseline / 'metadata.json')
    audit = read(a.episode / 'avoidance_audit.json')
    hits = [r for r in audit['baseline_counterfactual']['violations']
            if r['phase'] == 'pregrasp' and r.get('intersecting') and r['side'] == 'obs']
    if not audit['accepted'] or not hits:
        raise RuntimeError('Accepted source intersection evidence is required')
    hit = hits[len(hits) // 2]
    rows = [read(root / 'trajectory.json') for root in [a.baseline, a.episode]]
    free = [[i for i, row in enumerate(seq) if row['phase'] == 'pregrasp'] for seq in rows]
    bi = hit['action_index']
    fraction = free[0].index(bi) / max(1, len(free[0]) - 1)
    ai = free[1][round(fraction * (len(free[1]) - 1))]
    stage, kin, world, geometry = reconstruct(m, read(Path(m['variant_config'])), m['variant_spec'])
    cache = UsdGeom.XformCache()
    key = m['variant_spec']['name']
    source = [(name, world_points(stage.GetPrimAtPath(name), cache))
              for name, *_ in world.objects if world._matches(name, key)]
    camera = read(a.episode / 'cameras.json')[a.camera]
    base = transform(m['robot_base'], Rotation.from_euler('z', m['robot_yaw_deg'], degrees=True).as_quat())
    offset = camera['offset']
    camera_pose = base @ transform(offset['position'], offset['quaternion_xyzw'])
    K = np.asarray(camera['intrinsics'])

    def project(points):
        q = (points - camera_pose[:3, 3]) @ camera_pose[:3, :3]
        q = q[q[:, 2] < -.02]
        if len(q) < 3:
            return None
        uv = np.column_stack([K[0, 0] * q[:, 0] / -q[:, 2] + K[0, 2],
                              K[1, 2] - K[1, 1] * q[:, 1] / -q[:, 2]])
        try:
            return [tuple(v) for v in uv[ConvexHull(uv).vertices]]
        except Exception:
            return None

    directions = np.asarray([v for v in itertools.product([-1., 0., 1.], repeat=3) if any(v)])
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    margin = world.execution_clearances[key]
    canvas = Image.new('RGB', (2560, 840), '#101820')
    draw = ImageDraw.Draw(canvas)
    try:
        font = ImageFont.truetype('DejaVuSans.ttf', 22)
    except OSError:
        font = ImageFont.load_default()
    panels = []
    for panel, (root, index, metadata) in enumerate([(a.baseline, bi, bm), (a.episode, ai, m)]):
        png = a.output.with_name(a.output.stem + '_frame' + str(panel) + '.png')
        subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), '-v', 'error', '-i',
                        str(root / (a.camera + '.mp4')), '-vf', f'select=eq(n\\,{index})',
                        '-frames:v', '1', str(png)], check=True)
        im = Image.open(png).convert('RGB')
        if im.size != (1280, 720):
            raise ValueError('Unexpected original camera dimensions')
        overlay = ImageDraw.Draw(im)
        pose = rows[panel][index]['obs'].get('hazard_pose') or m['actual_hazard_initial_pose']
        root_prim = stage.GetPrimAtPath('/World/envs/env_0/' + key)
        reference = np.asarray(cache.GetLocalToWorldTransform(root_prim)).T
        delta = transform(pose[:3], pose[3:]) @ np.linalg.inv(reference)
        for _, points in source:
            pts = points @ delta[:3, :3].T + delta[:3, 3]
            # The yellow projected envelope is explanatory and is never an audit shape.
            subset = pts[np.linspace(0, len(pts) - 1, min(200, len(pts))).astype(int)]
            expanded = (subset[:, None, :] + margin * directions).reshape(-1, 3)
            for cloud, color in [(expanded, '#ffd166'), (pts, '#06d6a0')]:
                hull = project(cloud)
                if hull:
                    overlay.line(hull + [hull[0]], fill=color, width=3)
        world.set_obstacle_pose(key, pose)
        hazards = physical_hazards(stage, world, key, pose)
        state = rows[panel][index]['obs']
        joints = np.asarray(state['q'])
        fingers = {n: joints[metadata['robot_joint_names'].index(n)] for n in world.GRIPPER_JOINT_NAMES}
        links = kin.links(joints[:7])
        clouds = [(body, model, verts, links[body]) for body, model, verts in world.robot_hulls]
        local = world._gripper_links(fingers)
        clouds += [(body, model, verts, links[world.gripper_root] @ local[body])
                   for body, model, verts in world.gripper_models]
        risk_names = []
        for body, model, verts, T in clouds:
            obj = world.fcl.CollisionObject(model, world.fcl.Transform(T[:3, :3], T[:3, 3]))
            if any(world.fcl.distance(obj, h, world.fcl.DistanceRequest(), world.fcl.DistanceResult()) < margin
                   for _, h, *_ in hazards):
                hull = project(verts @ T[:3, :3].T + T[:3, 3])
                if hull:
                    overlay.line(hull + [hull[0]], fill='#ef476f', width=4)
                risk_names.append(body)
        canvas.paste(im, (1280 * panel, 72))
        label = 'OFFLINE COUNTERFACTUAL over recorded baseline RGB' if panel == 0 else 'ACTUAL RECORDED SAFE EXECUTION'
        draw.text((12 + 1280 * panel, 8), label, font=font, fill='white')
        draw.text((12 + 1280 * panel, 36), f'{root.name} | frame {index} | t={index*metadata["dt"]:.3f}s', font=font, fill='white')
        panels.append(dict(episode=str(root), frame=index, risky_robot_bodies=risk_names, frame_sha256=sha(png)))
    draw.text((12, 798), 'Green: source collision outlines | Yellow: sampled safety envelope | Red: robot collision part within margin', font=font, fill='white')
    canvas.save(a.output)
    evidence = dict(image=str(a.output), image_sha256=sha(a.output), camera=a.camera,
                    source_audit=str(a.episode / 'avoidance_audit.json'), audit_sha256=sha(a.episode / 'avoidance_audit.json'),
                    physics_replay=False, unsafe_path_executed=False, safety_margin_m=margin, panels=panels,
                    scope='Illustration only. Projected per-collider convex outlines and sampled margin envelopes do not replace FCL or physics audits. Panels use matching normalized pregrasp progress, not equal elapsed times.',
                    camera_projection='Recorded intrinsics and source panda_link0 camera mount, OpenGL convention.',
                    visual_alignment_review_pending=True)
    a.output.with_suffix('.json').write_text(json.dumps(evidence, indent=2) + '\n')
    print(a.output)


if __name__ == '__main__':
    main()
