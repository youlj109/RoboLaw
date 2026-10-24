"""Show recorded arm states and source colliders outside the native camera FOV.

This is an offline geometry reconstruction, never an RGB capture or physics replay.
"""
import argparse
import itertools
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from pxr import UsdGeom
from scipy.spatial import ConvexHull

from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from kitchen_safety.planning import transform
from scripts.extensions.doors.make_source_risk_camera_illustration_lwx import sha, world_points
from scripts.extensions.doors.physical_hazard_geometry import physical_hazards


def topology(prim, points):
    if not prim.IsA(UsdGeom.Mesh):
        return ConvexHull(points).simplices
    mesh = UsdGeom.Mesh(prim)
    sizes = mesh.GetFaceVertexCountsAttr().Get()
    indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=int)
    triangles, start = [], 0
    for size in sizes:
        face = indices[start:start + size]
        triangles.extend((face[0], face[j], face[j + 1]) for j in range(1, size - 1))
        start += size
    return np.asarray(triangles, dtype=int)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episode', type=Path)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists() or args.output.with_suffix('.json').exists():
        raise FileExistsError(args.output)
    read = lambda path: json.loads(path.read_text())
    metadata = read(args.episode / 'metadata.json')
    audit = read(args.episode / 'avoidance_audit.json')
    if not audit['accepted']:
        raise ValueError('Accepted independent avoidance audit is required')
    hits = [v for v in audit['baseline_counterfactual']['violations']
            if v['phase'] == 'pregrasp' and v['side'] == 'obs' and v.get('intersecting')]
    if not hits:
        raise ValueError('Actual source intersection in recorded off path is required')
    sequences = [read(p / 'trajectory.json') for p in [args.baseline, args.episode]]
    free = [[i for i, row in enumerate(seq) if row['phase'] == 'pregrasp'] for seq in sequences]
    bi = hits[len(hits) // 2]['action_index']
    progress = free[0].index(bi) / max(1, len(free[0]) - 1)
    indices = [bi, free[1][round(progress * (len(free[1]) - 1))]]
    stage, kin, world, geometry = reconstruct(metadata, read(Path(metadata['variant_config'])),
                                              metadata['variant_spec'])
    cache = UsdGeom.XformCache()
    key = metadata['variant_spec']['name']
    margin = world.execution_clearances[key]
    source = []
    for name, *_ in world.objects:
        if world._matches(name, key):
            prim = stage.GetPrimAtPath(name)
            points = world_points(prim, cache)
            source.append((name, points, topology(prim, points)))
    root = stage.GetPrimAtPath('/World/envs/env_0/' + key)
    reference = np.asarray(cache.GetLocalToWorldTransform(root)).T
    panels, meshes, risky_points = [], [], []
    for panel, (episode, index) in enumerate(zip([args.baseline, args.episode], indices)):
        state = sequences[panel][index]['obs']
        pose = state.get('hazard_pose') or metadata['actual_hazard_initial_pose']
        delta = transform(pose[:3], pose[3:]) @ np.linalg.inv(reference)
        hazards = physical_hazards(stage, world, key, pose)
        q = np.asarray(state['q'])
        names = metadata['robot_joint_names']
        fingers = {n: q[names.index(n)] for n in world.GRIPPER_JOINT_NAMES}
        links, local = kin.links(q[:7]), world._gripper_links(fingers)
        clouds = [(body, model, verts, links[body]) for body, model, verts in world.robot_hulls]
        clouds += [(body, model, verts, links[world.gripper_root] @ local[body])
                   for body, model, verts in world.gripper_models]
        robot, risks, distances = [], [], []
        for body, model, verts, T in clouds:
            obj = world.fcl.CollisionObject(model, world.fcl.Transform(T[:3, :3], T[:3, 3]))
            raw_distances, contacts = [], []
            for _, h, *_ in hazards:
                raw = world.fcl.distance(obj, h, world.fcl.DistanceRequest(), world.fcl.DistanceResult())
                intersecting = bool(world.fcl.collide(obj, h, world.fcl.CollisionRequest(),
                                                     world.fcl.CollisionResult())) if raw <= 0 else False
                if raw < 0 and not intersecting:
                    raise ValueError('Negative FCL distance without independently confirmed intersection')
                raw_distances.append(float(raw)); contacts.append(intersecting)
            # FCL may return -1 as a collision sentinel, NOT a penetration distance.
            distance = max(0., min(raw_distances))
            pts = verts @ T[:3, :3].T + T[:3, 3]
            robot.append((pts, ConvexHull(pts).simplices, distance < margin))
            distances.append(dict(body=body, source_distance_m=float(distance),
                                  raw_fcl_distance_min=min(raw_distances), intersecting=any(contacts)))
            if distance < margin:
                risks.append(body)
                if panel == 0:
                    risky_points.append(pts)
        entities = [(name, pts @ delta[:3, :3].T + delta[:3, 3], faces)
                    for name, pts, faces in source]
        meshes.append((entities, robot))
        panels.append(dict(episode=str(episode), frame=index, risky_robot_bodies=risks,
                           minimum_source_distance_m=min(d['source_distance_m'] for d in distances),
                           source_intersection=any(d['intersecting'] for d in distances),
                           source_distances=distances,
                           trajectory_sha256=sha(episode / 'trajectory.json')))
    if not panels[0]['risky_robot_bodies'] or panels[1]['risky_robot_bodies']:
        raise ValueError('Reconstructed selected frames do not reproduce audited risk/safety')
    points = np.concatenate([p for entities, robot in meshes
                             for p in [*[v[1] for v in entities], *[v[0] for v in robot]]])
    full_center = (points.min(0) + points.max(0)) / 2
    full_radius = max(np.ptp(points, axis=0)) / 2 * 1.06
    near = np.concatenate(risky_points)
    zoom_center = (near.min(0) + near.max(0)) / 2
    zoom_radius = max(max(np.ptp(near, axis=0)) / 2 + .075, .18)
    plt.rcParams.update({'font.size': 10})
    fig = plt.figure(figsize=(17, 13), facecolor='white')
    fig.suptitle('OFFLINE SOURCE-GEOMETRY RECONSTRUCTION — NOT CAMERA RGB\n'
                 'Recorded arm states; added source colliders. No simulation or unsafe replay.', fontsize=15)
    topology_counts = []
    for row in range(2):
        for panel, (entities, robot) in enumerate(meshes):
            ax = fig.add_subplot(2, 2, row * 2 + panel + 1, projection='3d')
            for name, pts, faces in entities:
                # Drawing thinning only; FCL and stored meshes use every source face.
                step = max(1, len(faces) // 30000)
                ax.add_collection3d(Poly3DCollection(pts[faces[::step]], facecolor='#44b388',
                                                    edgecolor='none', alpha=.60))
                if row == 0 and panel == 0:
                    topology_counts.append(dict(prim=name, source_triangles=len(faces),
                                                displayed_triangles=len(faces[::step])))
                lo, hi = pts.min(0) - margin, pts.max(0) + margin
                corners = np.asarray(list(itertools.product(*zip(lo, hi))))
                for i, j in itertools.combinations(range(8), 2):
                    if np.count_nonzero(corners[i] != corners[j]) == 1:
                        ax.plot(*corners[[i, j]].T, color='#c39100', linewidth=.7, linestyle='--')
            for pts, faces, risk in robot:
                color = '#e2364d' if risk else '#4f6fa1'
                ax.add_collection3d(Poly3DCollection(pts[faces], facecolor=color,
                                                    edgecolor='none', alpha=.78))
            center, radius = (full_center, full_radius) if row == 0 else (zoom_center, zoom_radius)
            ax.set_xlim(center[0] - radius, center[0] + radius)
            ax.set_ylim(center[1] - radius, center[1] + radius)
            ax.set_zlim(center[2] - radius, center[2] + radius)
            ax.set_box_aspect((1, 1, 1))
            ax.view_init(elev=24, azim=-55)
            ax.set_xlabel('World X (m)'); ax.set_ylabel('World Y (m)'); ax.set_zlabel('World Z (m)')
            detail = panels[panel]
            label = 'OFFLINE COUNTERFACTUAL' if panel == 0 else 'RECORDED SAFE STATE (reconstructed)'
            ax.set_title(f'{label} | frame {detail["frame"]}\n'
                         f'{"Overview" if row == 0 else "Same-region close-up"}; '
                         + ('source INTERSECTION (FCL); depth not computed' if detail['source_intersection'] else
                            f'min source distance = {detail["minimum_source_distance_m"] * 1000:.2f} mm'), fontsize=11)
    fig.text(.02, .025, 'Green: source mesh | Blue: robot hulls | Red: arm part within original margin\n'
             f'Yellow dashed: conservative per-collider AABB + {margin*1000:.0f} mm, illustration only. '
             'Unshown kitchen surfaces still participate in independent audits.\n'
             'Frames match normalized pregrasp progress. Raw native cameras do not show the entity clearly; '
             'this supplemental view does not replace those recordings.', fontsize=10)
    fig.subplots_adjust(top=.90, bottom=.11, left=.01, right=.99, wspace=.03, hspace=.14)
    fig.savefig(args.output, dpi=150)
    plt.close(fig)
    report = dict(image=str(args.output), image_sha256=sha(args.output),
                  rendering='Offline matplotlib source-mesh and robot-hull reconstruction, NOT recorded RGB',
                  source_audit=str(args.episode / 'avoidance_audit.json'),
                  audit_sha256=sha(args.episode / 'avoidance_audit.json'),
                  config_sha256=sha(metadata['variant_config']), metadata_sha256=sha(args.episode / 'metadata.json'),
                  variant_asset_sha256=sha(metadata['variant_spec']['asset']), implementation_sha256=sha(__file__),
                  physics_replay=False, unsafe_path_executed=False, safety_margin_m=margin,
                  panels=panels, topology=topology_counts,
                  visual_alignment_review_pending=True,
                  native_camera_limitation='Added entity outside/obscured in sampled original recordings; source-risk '
                  'annotations in both original external views are outside or at the edge of FOV.',
                  scope='Supplementary geometry visibility only. FCL uses complete source meshes; rendering '
                  'may thin displayed faces. Conservative yellow AABBs are not exact safety shells. '
                  'RGB, PhysX cooked shapes, continuous clearance and dynamics are not proved by this image.')
    args.output.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n')
    print(args.output)


if __name__ == '__main__':
    main()
