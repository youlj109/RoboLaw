"""Original-time same-camera comparison; overlays explain, never replace geometry audits."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import h5py
import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial import ConvexHull, QhullError
from scipy.spatial.transform import Rotation
from kitchen_safety.dataset import video_encoder, close_encoders, decode_video
from kitchen_safety.extensions.doors.reconstruction import reconstruct
from scripts.extensions.doors.physical_hazard_geometry import physical_hazards, physical_robot_shapes
from scripts.extensions.doors.search_avoidance_layouts import clearance


def corners(low, high):
    return np.array([[x, y, z] for x in (low[0], high[0])
                     for y in (low[1], high[1]) for z in (low[2], high[2])])


def project(points, position, quaternion, intrinsics):
    xyz = Rotation.from_quat(quaternion).inv().apply(points - position)
    xyz = xyz[xyz[:, 2] > .05]
    if len(xyz) < 3:
        return None
    uv = (xyz @ intrinsics.T)[:, :2] / xyz[:, 2:3]
    try:
        return [tuple(x) for x in (uv[ConvexHull(uv).vertices] * .5)]
    except QhullError:
        return None


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('episode', type=Path)
    p.add_argument('--baseline', type=Path, required=True)
    a = p.parse_args()
    output = a.episode / 'baseline_comparison.mp4'
    receipt = a.episode / 'comparison_audit.json'
    if output.exists() or receipt.exists():
        p.error('Refusing to overwrite comparison evidence')
    m = json.loads((a.episode / 'metadata.json').read_text())
    spec = m['variant_spec']
    stage, kin, world = reconstruct(m, spec)
    camera = 'external_camera_2'
    dt = m['dt']
    datasets = [a.baseline, a.episode]
    records = [json.loads((d / 'trajectory.json').read_text()) for d in datasets]
    metadata = [json.loads((d / 'metadata.json').read_text()) for d in datasets]
    cameras = [json.loads((d / 'cameras.json').read_text())[camera] for d in datasets]
    if cameras[0] != cameras[1] or metadata[0]['dt'] != dt:
        raise ValueError('Camera or sampling differs')
    intrinsics = np.asarray(cameras[0]['intrinsics'])
    files = [h5py.File(d / 'images.h5', 'r') for d in datasets]
    nframes = max(len(r) for r in records)
    enc = video_encoder(output, 1280, 456, dt)
    measurements = []
    try:
        for frame in range(nframes):
            canvas = Image.new('RGB', (1280, 456), (16, 20, 26))
            for side, (rows, f, meta) in enumerate(zip(records, files, metadata)):
                index = min(frame, len(rows) - 1)
                state = rows[index]['obs']
                pose = state.get('hazard_pose') or m['actual_hazard_initial_pose']
                world.set_obstacle_pose(spec['name'], pose)
                hazard = physical_hazards(stage, world, spec['name'], pose)
                q = np.asarray(state['q'])
                joints = dict(zip(meta['robot_joint_names'], q))
                shapes = physical_robot_shapes(world.shapes(q[:7], {n: joints[n] for n in world.GRIPPER_JOINT_NAMES}))
                margin = world.execution_clearances[spec['name']]
                value = clearance(shapes, hazard, margin, True)
                measurements.append({'frame': frame, 'side': side, 'source_frame': index, **value})
                image = Image.open(io.BytesIO(f[camera]['jpeg'][index].tobytes())).convert('RGB').resize((640, 360))
                draw = ImageDraw.Draw(image)
                position = f[camera]['position_world'][index]
                quaternion = f[camera]['quaternion_world_ros_xyzw'][index]
                for name, obj, low, high in hazard:
                    if name in world.meshes:
                        points = world.meshes[name][0] @ np.asarray(obj.getRotation()).T + obj.getTranslation()
                    else:
                        # Primitive bounding-box outline is explicitly illustrative.
                        points = corners(low, high)
                    polygon = project(points, position, quaternion, intrinsics)
                    if polygon:
                        draw.line(polygon + [polygon[0]], fill=(255, 190, 50), width=2)
                low = np.min([o[2] for o in hazard], axis=0) - margin
                high = np.max([o[3] for o in hazard], axis=0) + margin
                polygon = project(corners(low, high), position, quaternion, intrinsics)
                if polygon:
                    for start, stop in zip(polygon, polygon[1:] + polygon[:1]):
                        start, stop = np.asarray(start), np.asarray(stop)
                        for t in np.arange(0, 1, .1):
                            draw.line([tuple(start + t * (stop-start)), tuple(start + min(t+.05,1) * (stop-start))], fill=(255, 100, 200), width=1)
                closest = min(shapes, key=lambda shape: clearance([shape], hazard, margin, True)['distance_m'])
                polygon = project(corners(closest[2], closest[3]), position, quaternion, intrinsics)
                if polygon:
                    draw.line(polygon + [polygon[0]], fill=(100, 220, 255), width=2)
                canvas.paste(image, (side * 640, 34))
                text = ImageDraw.Draw(canvas)
                label = 'BASELINE / hazard is OFFLINE overlay' if side == 0 else 'ACTUAL / added hazard present'
                text.text((side*640+8, 8), label, fill='white')
                colour = (90, 245, 160) if value['valid'] else (255, 100, 100)
                text.text((side*640+8, 400), f"t={index*dt:.2f}s | {rows[index]['phase']} | clearance={value['distance_m']*1000:.1f}mm / {margin*1000:g}mm", fill=colour)
                if frame >= len(rows):
                    text.text((side*640+8, 417), 'END: last recorded frame held; no time rescaling', fill='white')
            text = ImageDraw.Draw(canvas)
            text.text((8, 438), '1x / 15fps | gold: source outline | cyan: nearest robot part | pink dashed: AABB + margin (illustration only)', fill='white')
            enc.stdin.write(np.asarray(canvas).tobytes())
    finally:
        for f in files:
            f.close()
        close_encoders({'comparison': enc})
    decoded = decode_video(output, nframes, expected_size=(1280, 456), expected_dt=dt)
    receipt.write_text(json.dumps({'passed': decoded == nframes, 'camera': camera,
        'frames': nframes, 'fps': 1/dt, 'speed': 1, 'source_frame_counts': [len(r) for r in records],
        'padding': 'Shorter recording holds its final image and is labeled; timestamps not rescaled.',
        'comparison_sha256': hashlib.sha256(output.read_bytes()).hexdigest(),
        'overlay_scope': 'Projected convex source collision-mesh outlines; primitives use bounds. Pink expanded axis-aligned bounds are explanatory, not an exact offset surface. Cyan bounds mark the closest robot collision part. Left hazard is offline counterfactual, never physically executed. Distances use FCL source geometry, not projected outlines.',
        'measurements': measurements}, indent=2))
    print(output, 'full_decode_passed', decoded, flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
