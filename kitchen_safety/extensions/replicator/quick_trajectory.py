"""One-shot nominal cuRobo candidates for current rendered layouts.

This is deliberately NOT an execution/collection acceptance path. Task contact
masks, nominal grasp attachments and closed gripper angles are explicit planning
assumptions. No simulator runs, posthoc collision audits or task-success tests.
"""
from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.curobo_backend import CuroboPlanner
from kitchen_safety.extensions.replicator.scene_build_runtime import ROOT


def pose(position, yaw=0.):
    value = np.eye(4)
    value[:3, :3] = Rotation.from_euler('z', yaw, degrees=True).as_matrix()
    value[:3, 3] = position
    return value


class PlanningWorld:
    """Read enabled source colliders without constructing unused FCL models."""
    def __init__(self, stage):
        from pxr import Usd, UsdGeom, UsdPhysics
        from kitchen_safety.extensions.replicator.collision_planner import _primitive_world_bounds
        self.objects, self.meshes, self.attachment = [], {}, None
        self.gripper_joint_position = 0.
        cache = UsdGeom.XformCache()
        bounds = UsdGeom.BBoxCache(Usd.TimeCode(0), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render,
            UsdGeom.Tokens.proxy, UsdGeom.Tokens.guide], False, True)
        for prim in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
            name = str(prim.GetPath())
            if name.startswith('/World/robot/') or not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
                continue
            transform = np.asarray(cache.GetLocalToWorldTransform(prim)).T
            if prim.IsA(UsdGeom.Mesh):
                mesh = UsdGeom.Mesh(prim)
                vertices = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
                vertices = vertices @ transform[:3, :3].T + transform[:3, 3]
                indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=int)
                faces, cursor = [], 0
                for count in mesh.GetFaceVertexCountsAttr().Get():
                    faces.extend([[indices[cursor], indices[cursor+j], indices[cursor+j+1]] for j in range(1, count-1)])
                    cursor += count
                if not len(vertices) or not faces:
                    raise ValueError('Empty enabled collider: '+name)
                self.meshes[name] = vertices, np.asarray(faces, dtype=np.int32)
                low, high = vertices.min(0), vertices.max(0)
            else:
                value = _primitive_world_bounds(prim, transform)
                if value is None:
                    bbox = bounds.ComputeWorldBound(prim).ComputeAlignedRange()
                    value = np.asarray(bbox.GetMin()), np.asarray(bbox.GetMax())
                low, high = value
            if not np.isfinite([low, high]).all():
                raise ValueError('Non-finite enabled collider: '+name)
            self.objects.append((name, None, low, high))

    @staticmethod
    def _matches(name, key):
        return name == key or name.startswith(key.rstrip('/')+'/') if key.startswith('/') else key in name.split('/')

    @staticmethod
    def _margin(name, kind):
        return 0.


def authored_start(stage, kin):
    """Infer joint angles from actual source-authored transforms, not defaults."""
    from pxr import UsdGeom, UsdPhysics
    cache, values = UsdGeom.XformCache(), {}
    def local(joint, side):
        quat = getattr(joint, 'GetLocalRot'+side+'Attr')().Get()
        value = np.eye(4)
        value[:3, :3] = Rotation.from_quat([*quat.GetImaginary(), quat.GetReal()]).as_matrix()
        value[:3, 3] = getattr(joint, 'GetLocalPos'+side+'Attr')().Get()
        return value
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith('/World/robot/') or prim.GetName() not in [f'panda_joint{i}' for i in range(1, 8)]:
            continue
        joint = UsdPhysics.Joint(prim)
        a = stage.GetPrimAtPath(joint.GetBody0Rel().GetTargets()[0])
        b = stage.GetPrimAtPath(joint.GetBody1Rel().GetTargets()[0])
        value = np.linalg.inv(np.asarray(cache.GetLocalToWorldTransform(a)).T @ local(joint, '0')) @ np.asarray(cache.GetLocalToWorldTransform(b)).T @ local(joint, '1')
        rotation = Rotation.from_matrix(value[:3, :3]).as_rotvec()
        if np.linalg.norm(rotation[1:]) > 1e-4 or np.linalg.norm(value[:3, 3]) > 1e-4:
            raise ValueError('Source-authored joint transform is inconsistent: '+prim.GetName())
        values[prim.GetName()] = float(rotation[0])
    q = np.asarray([values[f'panda_joint{i}'] for i in range(1, 8)])
    if np.any(q < kin.limits[:, 0]) or np.any(q > kin.limits[:, 1]):
        raise ValueError('Source-authored initial q outside joint limits')
    return q


def task_targets(layout, world, assignment):
    from kitchen_safety.extensions.replicator.task_collision import PayloadGeometry
    from kitchen_safety.extensions.replicator.task_motion import banana_midbody
    from kitchen_safety.extensions.replicator.bowl_motion import bowl_rim_geometry
    payload = 'banana' if 'banana' in layout['objects'] else next((name for name in layout['objects'] if name.startswith('mustard')), 'bowl')
    item = layout['objects'][payload]
    geometry = PayloadGeometry(ROOT/item['usd_path'])
    initial = pose(item['position'], item.get('yaw_deg', 0.))
    if payload == 'banana':
        info = banana_midbody(geometry)
        center = np.asarray(info['center_local_m'])
        grip = .50
    elif payload == 'bowl':
        info = bowl_rim_geometry(geometry)
        center = np.asarray(info['center_local_m'])
        grip = .73
    else:
        center = (geometry.low+geometry.high)/2
        center[2] = min(.025, geometry.high[2]-.025)
        info = {'rule': 'upright mustard body pinch, top-down jaw along source X', 'center_local_m': center.tolist()}
        grip = .38
    down, jaw = np.array([0., 0., -1.]), initial[:3, 0].copy()
    jaw[2] = 0.; jaw /= np.linalg.norm(jaw)
    grasp = np.eye(4)
    grasp[:3, :3] = np.column_stack((down, jaw, np.cross(down, jaw)))
    grasp[:3, 3] = initial[:3, :3] @ center + initial[:3, 3]
    # The source render is not physics-settled. Keep original object poses and
    # choose a nominal finger-height guard, without editing the scene.
    grasp[2, 3] = max(grasp[2, 3], initial[2, 3]+float(geometry.low[2])+.035)
    attachment = np.linalg.inv(grasp) @ initial
    pregrasp, lift = grasp.copy(), grasp.copy()
    pregrasp[2, 3] += .125
    lift[2, 3] += .18
    source_bottom = initial[:3, 3] + np.min(geometry.vertices @ initial[:3, :3].T, axis=0)
    supports = [row for row in world.objects if row[0].startswith('/World/kitchen/')
        and np.all(source_bottom[:2] >= row[2][:2]-.08) and np.all(source_bottom[:2] <= row[3][:2]+.08)
        and abs(row[3][2]-source_bottom[2]) < .035]
    source_support = min(supports, key=lambda row: abs(row[3][2]-source_bottom[2]))[0] if supports else None
    destination = initial.copy()
    if payload == 'bowl':
        reference = next(row for row in assignment['object_references'] if row['id'] == 'sink')
        destination_mask = '/World/kitchen/Asset/'+reference['prim_path']
        rows = [row for row in world.objects if world._matches(row[0], destination_mask)]
        if not rows:
            raise ValueError('Missing original source sink collision region '+destination_mask)
        low, high = np.min([row[2] for row in rows], axis=0), np.max([row[3] for row in rows], axis=0)
        rotated = geometry.vertices @ initial[:3, :3].T
        destination[:2, 3] = (low[:2]+high[:2])/2-(rotated.min(0)[:2]+rotated.max(0)[:2])/2
        destination[2, 3] = high[2]+.006-rotated[:, 2].min()
        rim = max(row[3][2] for row in world.objects if '/Sink/' in row[0])
    else:
        bowl = layout['objects']['bowl']
        bowl_geometry = PayloadGeometry(ROOT/bowl['usd_path'])
        bowl_pose = pose(bowl['position'], bowl.get('yaw_deg', 0.))
        destination_mask = '/World/objects/bowl'
        bowl_vertices = bowl_geometry.vertices @ bowl_pose[:3, :3].T + bowl_pose[:3, 3]
        destination[:2, 3] = bowl_pose[:2, 3]
        rim = float(bowl_vertices[:, 2].max())
        # Nominal placement above the bowl interior; physics settlement and
        # retention are explicitly not evaluated by this quick plan.
        destination[2, 3] = bowl_pose[2, 3]+.005-float((geometry.vertices @ initial[:3, :3].T)[:, 2].min())
    lower = destination @ np.linalg.inv(attachment)
    transfer = lower.copy()
    transfer[2, 3] = max(lift[2, 3], lower[2, 3]+.15, rim+.13)
    lift[2, 3] = max(lift[2, 3], transfer[2, 3])
    retreat = lower.copy(); retreat[2, 3] += .15
    return dict(payload_name=payload, geometry=geometry, nominal_gripper_joint_rad=grip,
        initial_payload_pose=initial, attachment=attachment, grasp_info=info,
        source_support=source_support, destination_mask=destination_mask,
        targets={key: value for key, value in zip(('pregrasp','approach','lift','transfer','lower','retreat'),
            (pregrasp,grasp,lift,transfer,lower,retreat))})


class QuickPlanner(CuroboPlanner):
    """Reuse public source geometry construction; one MotionGen call per phase."""
    def _sync_gripper_and_attachment(self, grip_binary, object_padding_m=0.):
        grip = self.world.gripper_joint_position
        if self._last_grip is None or abs(grip-self._last_grip) > .002:
            locked = dict(self.robot_dict['kinematics']['lock_joints'])
            for name, sign in zip(('finger_joint','right_outer_knuckle_joint','left_inner_finger_joint',
                'right_inner_finger_joint','left_inner_finger_knuckle_joint','right_inner_finger_knuckle_joint'), (1,1,-1,1,-1,-1)):
                locked[name] = sign*grip
            self.motion_gen.update_locked_joints(locked, deepcopy(self.robot_dict))
            self._last_grip = grip
        if self.world.attachment is None:
            self.motion_gen.detach_spheres_from_robot()
            return
        if not hasattr(self, '_cover'):
            self._cover = self.world.payload_geometry.sphere_cover(100)
        centers, radii, _ = self._cover
        transform = self.T_ee_tcp @ self.world.attachment
        spheres = np.zeros((100,4), np.float32); spheres[:,3] = -100.
        spheres[:len(centers),:3] = centers @ transform[:3,:3].T + transform[:3,3]
        spheres[:len(centers),3] = radii
        self.motion_gen.attach_spheres_to_robot(sphere_tensor=self.args.to_device(spheres))

    def quick_plan(self, q, target, phase, masks, timeout=6.):
        from curobo.types.math import Pose
        from curobo.types.robot import JointState
        from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig
        for name in self.obstacle_names:
            self.motion_gen.world_coll_checker.enable_obstacle(name, enable=not any(self.world._matches(name, mask) for mask in masks))
        self._sync_gripper_and_attachment(self.world.attachment is not None)
        self.motion_gen.reset(reset_seed=True)
        start = JointState.from_position(self.args.to_device(np.asarray(q)[None]), joint_names=[f'panda_joint{i}' for i in range(1,8)])
        goal = Pose.from_list(self._pose_list(self.T_B_W @ target @ np.linalg.inv(self.T_ee_tcp)), self.args)
        started = time.monotonic()
        result = self.motion_gen.plan_single(start, goal, MotionGenPlanConfig(max_attempts=1,
            timeout=timeout, enable_graph=True, enable_graph_attempt=1, enable_finetune_trajopt=True,
            finetune_attempts=1, ik_fail_return=1))
        self.torch.cuda.synchronize()
        info = dict(phase=phase, status='generated' if bool(result.success.all()) else 'failed',
            solver_success=bool(result.success.all()),
            solver_status='SUCCESS' if result.status is None else str(result.status),
            valid_query=bool(result.valid_query) if result.valid_query is not None else None,
            planning_seconds=time.monotonic()-started, contact_masks=masks,
            nominal_payload_attached=self.world.attachment is not None)
        if not bool(result.success.all()):
            return None, info
        positions = result.get_interpolated_plan().position.detach().cpu().numpy()
        if positions.ndim != 2 or positions.shape[1] != 7 or not np.isfinite(positions).all():
            raise ValueError('Invalid cuRobo trajectory output')
        if np.max(np.abs(positions[0]-q)) > 1e-4:
            raise ValueError('cuRobo trajectory start mismatch')
        info.update(samples=len(positions), interpolation_dt=float(result.interpolation_dt))
        return positions, info
