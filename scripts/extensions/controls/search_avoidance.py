"""CPU-only placement search against recorded baseline, never dangerous replay.

Search candidates are diagnostics, not accepted episodes. Exact all-frame
counterfactual and physical execution remain separate acceptance requirements.
"""
import argparse
import hashlib
import itertools
import json
from pathlib import Path

import fcl
import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.controls.collision import ControlsCollisionWorld
from kitchen_safety.planning import DroidKinematics, transform

ROOT = Path(__file__).resolve().parents[3]


def fixed_shapes(pose,stand_height=.8):
    if stand_height not in (.7,.8):raise ValueError('Unsupported original controls stand height')
    source = Path.home() / f'.cache/isaaclab_arena/usd/robot_on_stand/controls_droid_original_stand_{stand_height}_1.08x0.91032.usd'
    stage = Usd.Stage.Open(str(source))
    cache = UsdGeom.XformCache()
    base = next(p for p in stage.Traverse() if p.GetName() == 'panda_link0')
    inverse = np.linalg.inv(np.array(cache.GetLocalToWorldTransform(base)).T)
    matrix = transform(pose[:3], pose[3:7])
    shapes = []
    for prim in Usd.PrimRange(base, Usd.TraverseInstanceProxies()):
        if not prim.HasAPI(UsdPhysics.CollisionAPI) or not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():
            continue
        if not prim.IsA(UsdGeom.Mesh):
            raise ValueError('Unsupported fixed collider ' + str(prim.GetPath()))
        mesh = UsdGeom.Mesh(prim)
        local = matrix @ inverse @ np.array(cache.GetLocalToWorldTransform(prim)).T
        vertices = np.array(mesh.GetPointsAttr().Get()) @ local[:3, :3].T + local[:3, 3]
        faces, offset = [], 0
        indices = np.array(mesh.GetFaceVertexIndicesAttr().Get())
        for count in mesh.GetFaceVertexCountsAttr().Get():
            polygon = indices[offset:offset + count]
            faces.extend([[polygon[0], polygon[j], polygon[j + 1]] for j in range(1, count - 1)])
            offset += count
        model = fcl.BVHModel()
        model.beginModel(len(vertices), len(faces))
        model.addSubModel(vertices, np.array(faces, dtype=np.int32))
        model.endModel()
        shapes.append((str(prim.GetPath()), fcl.CollisionObject(model), vertices.min(0), vertices.max(0)))
    return shapes


def minimum(first, second, limit=float('inf')):
    """Exact FCL narrow phase, conservative AABB rejection."""
    low = np.array([x[2] for x in first]); high = np.array([x[3] for x in first])
    low2 = np.array([x[2] for x in second]); high2 = np.array([x[3] for x in second])
    bounds = np.linalg.norm(np.maximum(np.maximum(low[:, None] - high2, low2 - high[:, None]), 0), axis=2)
    best = dict(distance_m=float('inf'), intersecting=False, pair=None)
    for flat in np.argsort(bounds, axis=None):
        i, j = np.unravel_index(flat, bounds.shape)
        if bounds[i, j] > min(best['distance_m'], limit):
            break
        raw = float(fcl.distance(first[i][1], second[j][1], fcl.DistanceRequest(), fcl.DistanceResult()))
        overlap = raw < 0 or (raw == 0 and bool(fcl.collide(first[i][1], second[j][1], fcl.CollisionRequest(), fcl.CollisionResult())))
        if max(raw, 0) < best['distance_m'] or overlap:
            best = dict(distance_m=max(raw, 0), intersecting=overlap, pair=[first[i][0], second[j][0]])
        if overlap:
            break
    return best


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline', default='datasets/avoidance_trials/controls/base_attempt01')
    parser.add_argument('--variant', choices=['human', 'property', 'robot'], required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--step', type=float, default=.06)
    parser.add_argument('--config', default='configs/extensions/controls/left_button_trial.json')
    parser.add_argument('--y-min', type=float, default=None)
    parser.add_argument('--yaw-step', type=int, default=30)
    parser.add_argument('--stage', default='reports/skill_trials/controls/inspection/stage.usda')
    parser.add_argument('--local-window', type=float)
    parser.add_argument('--yaw-window', type=int, default=30)
    parser.add_argument('--hand-between', action='store_true')
    parser.add_argument('--hand-height-max', type=float, default=1.25)
    parser.add_argument('--hand-height-min', type=float, default=1.05)
    parser.add_argument('--hand-fraction-max', type=float, default=.8)
    parser.add_argument('--hand-lateral-max', type=float, default=.09)
    parser.add_argument('--source-risk-only', action='store_true')
    parser.add_argument('--sample-stride',type=int,default=8)
    args = parser.parse_args()
    out = Path(args.output)
    if out.exists():
        raise FileExistsError(out)
    episode = Path(args.baseline)
    result = json.loads((episode / 'result.json').read_text())
    records = json.loads((episode / 'records.json').read_text())
    cfg = json.loads((ROOT / args.config).read_text())
    pose = result['initial_robot_pose']
    kin = DroidKinematics(pose[:3], Rotation.from_quat(pose[3:7]).as_euler('xyz', degrees=True)[2])
    spec = cfg['variants'][args.variant]
    name = {'human': 'person', 'property': 'glass', 'robot': 'barrier'}[args.variant]
    stage = Usd.Stage.CreateInMemory()
    prim = UsdGeom.Xform.Define(stage, '/World/envs/env_0/' + name)
    prim.GetPrim().GetReferences().AddReference(str(ROOT / spec['asset']))
    prim.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
    world = ControlsCollisionWorld(stage, kin, env_root='/World/envs/env_0')
    scene = ControlsCollisionWorld(Usd.Stage.Open(str(ROOT / args.stage)), kin, env_root='/World/envs/env_0')
    fixed = fixed_shapes(pose)
    names = result.get('robot_joint_names') or list(json.loads((ROOT/'reports/skill_trials/controls/inspection/state.json').read_text())['robot_joints'])
    ids = [names.index(f'panda_joint{i}') for i in range(1, 8)]
    selected = sorted(set([0, len(records) - 1] + list(range(14, len(records), args.sample_stride))))
    robot_shapes = {}
    for index in selected:
        q = np.array(records[index]['after']['q'])
        robot_shapes[index] = world.shapes(q[ids], dict(zip(names, q)))
    initial_q = np.array(records[0]['before']['q'])
    robot_shapes[-1] = world.shapes(initial_q[ids], dict(zip(names, initial_q)))
    endpoints = [-1, 0] + [i for i in selected if records[i]['phase'] in ('press', 'hold')]
    intermediate = [i for i in selected if records[i]['phase'] == 'prepress']
    margin = world.execution_clearances[name]
    planning_margin = world.clearances[name]
    hand_filter = None
    if args.hand_between:
        if args.variant != 'human':raise ValueError('Hand filter requires human variant')
        bones=json.loads(((ROOT/spec['asset']).parent/'pose.json').read_text())['after']
        hand_local=np.asarray(bones['hand_r']).mean(axis=0)
        button=np.asarray(cfg.get('button_center_world_m',[2.08087763,-.38513822422,1.03313965]))
        hand_filter={'button_world_m':button.tolist(),'maximum_lateral_offset_m':args.hand_lateral_max,
            'base_to_button_fraction':[.2,args.hand_fraction_max],'height_range_m':[args.hand_height_min,args.hand_height_max]}
    if args.variant == 'human':
        grid = itertools.product(np.arange(1.1, 3.11, args.step), np.arange(-1.7 if args.y_min is None else args.y_min, -.39, args.step), range(-180, 180, args.yaw_step))
    else:
        grid = itertools.product(np.arange(1.52, 2.51, args.step), np.arange(-.38 if args.y_min is None else args.y_min, .13, args.step), [0] if args.variant == 'property' else [0, 45, 90, 135, 180, 225, 270, 315])
    if args.local_window is not None:
        if args.variant!='human' or args.local_window<=0:raise ValueError('Positive human local window required')
        offsets=np.arange(-args.local_window,args.local_window+args.step*.1,args.step)
        grid=itertools.product(spec['position'][0]+offsets,spec['position'][1]+offsets,
            range(int(spec['yaw_deg'])-args.yaw_window,int(spec['yaw_deg'])+args.yaw_window+1,args.yaw_step))
    candidates = []; no_risk_layouts=[];endpoint_rejections=[]
    counts = {'tested': 0, 'endpoint_rejected': 0, 'no_risk': 0, 'fixed_rejected': 0, 'kitchen_rejected': 0}
    for x, y, yaw in grid:
        counts['tested'] += 1
        xyz = [round(float(x), 6), round(float(y), 6), 0 if args.variant == 'human' else .921]
        if hand_filter:
            hand=Rotation.from_euler('z',yaw,degrees=True).apply(hand_local)+xyz
            fraction=(hand[1]-pose[1])/(button[1]-pose[1])
            if not (abs(hand[0]-button[0])<=args.hand_lateral_max and .2<=fraction<=args.hand_fraction_max and args.hand_height_min<=hand[2]<=args.hand_height_max):
                counts['hand_region_rejected']=counts.get('hand_region_rejected',0)+1
                continue
        world.set_obstacle_pose(name, xyz + Rotation.from_euler('z', yaw, degrees=True).as_quat().tolist())
        # Live controls checks conservatively represent USD primitives by
        # world-aligned bounds. Rebuild that bound after candidate rotation;
        # merely rotating the old FCL box yields a different, tighter model.
        world.objects=[(n,obj if n in world.meshes else fcl.CollisionObject(
            fcl.Box(*np.maximum(hi-lo,1e-4)),fcl.Transform((lo+hi)/2)),lo,hi)
            for n,obj,lo,hi in world.objects]
        hazard = world.objects
        endpoint_measures=[(i,minimum(robot_shapes[i],hazard,planning_margin+.02)) for i in endpoints]
        endpoint_index,endpoint_worst=min(endpoint_measures,key=lambda row:row[1]['distance_m'])
        endpoint_min=endpoint_worst['distance_m']
        if endpoint_min < planning_margin + .005:
            if len(endpoint_rejections)<50:
                endpoint_rejections.append({'position':xyz,'yaw_deg':int(yaw),
                    'action_index':endpoint_index,**endpoint_worst})
            counts['endpoint_rejected'] += 1; continue
        measures = [(i, minimum([s for s in robot_shapes[i] if not args.source_risk_only or
            s[0]=='robot_hull' or s[0].startswith('gripper/')], hazard, margin)) for i in intermediate]
        bad = [(i, m) for i, m in measures if m['intersecting'] or m['distance_m'] < margin]
        if not bad:
            no_risk_layouts.append({'position':xyz,'yaw_deg':int(yaw),'sampled_endpoint_minimum_m':endpoint_min})
            counts['no_risk'] += 1; continue
        fixed_min = minimum(fixed, hazard, .01)
        if fixed_min['intersecting'] or fixed_min['distance_m'] < .005:
            counts['fixed_rejected'] += 1; continue
        kitchen_min = minimum(scene.objects, hazard, .0001)
        if kitchen_min['intersecting']:
            counts['kitchen_rejected'] += 1; continue
        candidate = dict(spec=dict(asset=spec['asset'], position=xyz, yaw_deg=int(yaw)),
                         baseline_violations=[dict(action_index=i, phase=records[i]['phase'], **m) for i, m in bad],
                         sampled_endpoint_minimum_m=min(minimum(robot_shapes[i], hazard)['distance_m'] for i in endpoints),
                         fixed_base=minimum(fixed,hazard), kitchen=minimum(scene.objects,hazard))
        candidates.append(candidate)
        print('CANDIDATE', xyz, yaw, 'risk', len(bad), 'minimum', min(m['distance_m'] for _, m in bad), flush=True)
        if counts['tested'] % 200 == 0:
            print('PROGRESS', counts, flush=True)
    candidates.sort(key=lambda c: (-len(c['baseline_violations']), -c['sampled_endpoint_minimum_m']))
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(dict(baseline=str(episode), baseline_records_sha256=hashlib.sha256((episode/'records.json').read_bytes()).hexdigest(),
        variant=args.variant, counts=counts, candidates=candidates,hand_region_constraint=hand_filter,
        source_robot_hulls_only_for_risk=args.source_risk_only,sample_stride=args.sample_stride,
        no_risk_layouts=no_risk_layouts,endpoint_rejection_examples=endpoint_rejections,
        scope='Subsampled CPU candidate search only; no physical success or complete counterfactual acceptance claimed.'), indent=2))
    print('SEARCH_DONE', counts, 'candidates', len(candidates), flush=True)


if __name__ == '__main__':
    main()
