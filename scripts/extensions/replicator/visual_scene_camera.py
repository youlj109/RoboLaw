"""Camera-only scene inspection helpers; projection ranking is not visibility proof."""
import itertools
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]


def box_corners(bounds):
    low, high = np.asarray(bounds)
    return np.array(list(itertools.product(*zip(low, high))))


def project(points, eye, target, focal=24., width=1280, height=848):
    eye, target = np.asarray(eye), np.asarray(target)
    forward = target - eye
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0., 0., 1.])
    if np.linalg.norm(right) < 1e-6:
        right = np.cross(forward, [0., 1., 0.])
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    delta = np.asarray(points) - eye
    depth = delta @ forward
    x = width / 2 + (delta @ right) / np.maximum(depth, .001) * width * focal / 36.
    y = height / 2 - (delta @ up) / np.maximum(depth, .001) * height * focal / 24.
    return np.stack([x, y, depth], axis=1)


def segment_box_occlusions(eye, target, bounds):
    """Conservative mesh-AABB ranking only; never used as final visual evidence."""
    if not len(bounds):
        return 0
    bounds = np.asarray(bounds)
    eye, vector = np.asarray(eye), np.asarray(target) - eye
    inv = np.divide(1., vector, out=np.full(3, 1e12), where=np.abs(vector) > 1e-12)
    t0, t1 = (bounds[:, 0] - eye) * inv, (bounds[:, 1] - eye) * inv
    enter, leave = np.minimum(t0, t1).max(1), np.maximum(t0, t1).min(1)
    return int(((leave >= np.maximum(enter, .03)) & (enter < .94) & (leave > .03)).sum())


def landmarks(stage, layout):
    from pxr import Usd, UsdGeom
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render'])

    def bounds(path):
        prim = stage.GetPrimAtPath(path)
        if not prim:
            raise ValueError('Missing original scene landmark: ' + path)
        box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        result = [list(box.GetMin()), list(box.GetMax())]
        if not np.isfinite(result).all() or np.any(np.asarray(result[1]) < result[0]):
            raise ValueError('Invalid source bounds: ' + path)
        return result

    entities = {'robot': dict(prefix='/World/robot', bounds=bounds('/World/robot'), minimum_pixels=1200),
                'hazard': dict(prefix='/World/hazard', bounds=bounds('/World/hazard'),
                               minimum_pixels=1800 if layout['obstacle']['category'] == 'human' else 450)}
    for name in layout['objects']:
        path = '/World/objects/' + name
        entities['object:' + name] = dict(prefix=path, bounds=bounds(path), minimum_pixels=450)
    if layout['task_id'].endswith('bowl_sink'):
        path = '/World/kitchen/Asset/Sink/BasinBottom'
        entities['target:sink'] = dict(prefix=path, bounds=bounds(path), minimum_pixels=700)
    if layout['obstacle']['category'] == 'human':
        pose = json.loads((ROOT / 'assets/people/mpfb_corridor/pose.json').read_text())
        yaw = math.radians(layout['obstacle']['yaw_deg'])
        rotation = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        hand = np.asarray(pose['hand_tip_local']) @ rotation.T + layout['obstacle']['position']
        entities['human:hand_region'] = dict(prefix='/World/hazard', bounds=[(hand-.045).tolist(), (hand+.045).tolist()],
            minimum_pixels=100, roi_only=True, scope='Human segmentation inside projected source hand-landmark region; visual inspection still required for anatomical/occlusion certainty.')
    occluders = []
    for prefix in ('/World/kitchen', '/World/robot'):
        root = stage.GetPrimAtPath(prefix)
        for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            imageable = UsdGeom.Imageable(prim)
            if imageable.ComputeVisibility() == 'invisible' or imageable.ComputePurpose() not in ('default', 'render'):
                continue
            occluders.append(bounds(str(prim.GetPath())))
    return dict(entities=entities, floor=bounds('/World/kitchen/Asset/KitchenRoom/Floor'),
                ceiling=bounds('/World/kitchen/Asset/KitchenRoom/Ceiling'), occluders=occluders)


def camera_candidates(info, layout, width=1280, height=848, per_kind=3):
    entities = info['entities']
    category = layout['obstacle']['category']
    operation = [name for name in entities if name.startswith(('object:', 'target:'))]
    critical = operation + (['human:hand_region'] if category == 'human' else ['hazard'])
    centers = {name: np.mean(item['bounds'], axis=0) for name, item in entities.items()}
    task_center = np.mean([centers[name] for name in critical], axis=0)
    overview_center = np.mean([centers[name] for name in ('robot', 'hazard')] + [task_center], axis=0)
    floor, ceiling = np.asarray(info['floor']), np.asarray(info['ceiling'])
    maximum_z = float(ceiling[0, 2]) - .16
    yaw = math.radians(layout['robot_yaw_deg'])
    rows = []
    for kind, target, radius, z in (
            ('overview', overview_center, 1.85, min(2.28, maximum_z)),
            ('operation_side', task_center, 1.25, min(float(task_center[2])+.65, maximum_z)),
            ('operation_top', task_center, .38, min(float(task_center[2])+1.2, maximum_z))):
        candidates = []
        for angle_offset in (0, 90, -90, 45, -45, 135, -135, 180):
            angle = yaw + math.radians(angle_offset)
            eye = np.array([target[0]+math.cos(angle)*radius, target[1]+math.sin(angle)*radius, z])
            eye[:2] = np.clip(eye[:2], floor[0, :2]+.18, floor[1, :2]-.18)
            required = ['robot', 'hazard'] + operation if kind == 'overview' else critical
            best = None
            for focal in (30., 24., 20., 16., 12.):
                size_score, clipped, blocked = 0., 0, 0
                for name in required:
                    projected = project(box_corners(entities[name]['bounds']), eye, target, focal, width, height)
                    inside = ((projected[:, 0] > 15) & (projected[:, 0] < width-15)
                              & (projected[:, 1] > 15) & (projected[:, 1] < height-15) & (projected[:, 2] > .05))
                    clipped += int(not inside.all())
                    span = np.ptp(projected[:, :2], axis=0)
                    size_score += math.log1p(float(np.prod(np.minimum(span, [width, height]))))
                    if name != 'robot':
                        blocked += segment_box_occlusions(eye, centers[name], info['occluders'])
                score = size_score - clipped*35 - blocked*4
                row = dict(kind=kind, eye=eye.tolist(), target=target.tolist(), focal_length_mm=focal,
                           geometric_rank_score=score, projected_clipped_entities=clipped,
                           coarse_aabb_occlusion_count=blocked, angle_offset_deg=angle_offset)
                if best is None or score > best['geometric_rank_score']:
                    best = row
            candidates.append(best)
        rows.extend(sorted(candidates, key=lambda x: x['geometric_rank_score'], reverse=True)[:per_kind])
    return rows


def measure_visibility(segmentation, rgb, entities, camera, width, height):
    if not isinstance(segmentation, dict) or 'data' not in segmentation:
        raise ValueError('Actual instance segmentation data is missing')
    data = np.asarray(segmentation['data']).squeeze()
    if data.shape != (height, width):
        raise ValueError('Unexpected segmentation shape: ' + str(data.shape))
    labels = segmentation.get('info', {}).get('idToLabels', {})
    result = {}
    for name, entity in entities.items():
        ids = [int(value) for value, label in labels.items() if entity['prefix'] in str(label)]
        mask = np.isin(data, ids)
        projection = project(box_corners(entity['bounds']), camera['eye'], camera['target'],
                             camera['focal_length_mm'], width, height)
        if entity.get('roi_only'):
            low = np.maximum(0, np.floor(projection[:, :2].min(0))).astype(int)
            high = np.minimum([width, height], np.ceil(projection[:, :2].max(0))).astype(int)
            roi = np.zeros_like(mask)
            if np.all(high > low) and (projection[:, 2] > 0).all():
                roi[low[1]:high[1], low[0]:high[0]] = True
            mask &= roi
        y, x = np.nonzero(mask)
        pixels = len(x)
        span = [int(np.ptp(x)+1), int(np.ptp(y)+1)] if pixels else [0, 0]
        saturation = float(np.mean((rgb[:, :, :3][mask] >= 250).all(axis=1))) if pixels else None
        # Pixel area and two-dimensional extent jointly reject the old 20-pixel
        # "visible sliver" condition. Photometry is reported separately because
        # a real white ceramic/glass object need not contain dark pixels.
        passed = pixels >= entity['minimum_pixels'] and min(span) >= (9 if entity.get('roi_only') else 16)
        result[name] = dict(pixels=pixels, visible_span_px=span, minimum_pixels=entity['minimum_pixels'],
                            readable_pixels=passed, near_white_fraction=saturation,
                            projected_bounds_px=projection[:, :2].tolist(), roi_only=entity.get('roi_only', False))
    return result


def multi_view_summary(views, entities):
    passing = [view for view in views if view.get('rgb_valid')]
    by_entity = {name: [view['view_id'] for view in passing if view['visibility'][name]['readable_pixels']]
                 for name in entities}
    kinds = {view['kind'] for view in passing}
    complete = all(by_entity.values()) and {'overview', 'operation_side', 'operation_top'} <= kinds
    return dict(readability_passed=bool(complete), readable_in_views=by_entity,
                missing_entities=[name for name, rows in by_entity.items() if not rows],
                view_kinds=sorted(kinds), human_hand_visual_review_required='human:hand_region' in entities,
                scope='Actual segmentation pixel area and silhouette extent across camera-only views; not anatomical certainty, native physics, or safety acceptance.')
