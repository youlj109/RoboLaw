"""All-collider world for control manipulation, without the original kitchen crop.

Constructor is adapted locally from kitchen_safety.collision_planner so the
shared benchmark remains unchanged. Robot geometry/math remain inherited.
"""
import numpy as np
from kitchen_safety.collision_planner import CollisionWorld, _primitive_world_bounds


def fixture_cylinder_bounds(prim, matrix, count):
    """Union of conservative slice AABBs, not an inscribed cylinder mesh."""
    from pxr import UsdGeom
    if isinstance(count, bool) or not isinstance(count, int) or not 2 <= count <= 64:
        raise ValueError('Fixture cylinder subdivision must be an integer from 2 to 64')
    cylinder = UsdGeom.Cylinder(prim)
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('Fixture transform must be a finite 4x4 matrix')
    axis = {'X': 0, 'Y': 1, 'Z': 2}[str(cylinder.GetAxisAttr().Get())]
    linear = np.asarray(matrix)[:3, :3]
    scale = np.linalg.norm(linear, axis=0)
    if np.any(scale <= 0):
        raise ValueError('Degenerate fixture scale is unsupported')
    rotation = linear / scale
    radial = [i for i in range(3) if i != axis]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7) or
            not np.isclose(scale[radial[0]], scale[radial[1]], rtol=1e-7, atol=1e-8)):
        raise ValueError('Sheared or elliptical fixture cylinder is unsupported')
    direction = rotation[:, axis]
    radius = float(cylinder.GetRadiusAttr().Get()) * scale[radial[0]]
    length = float(cylinder.GetHeightAttr().Get()) * scale[axis]
    if not np.isfinite([radius, length]).all() or min(radius, length) <= 0:
        raise ValueError('Fixture cylinder dimensions must be finite and positive')
    half = radius * np.sqrt(np.maximum(0., 1 - direction**2)) + length / (2*count) * abs(direction)
    center = np.asarray(matrix)[:3, 3]
    return [(c-half, c+half) for c in (
        center + direction * length * ((i+.5)/count-.5) for i in range(count))]


def held_gripper_pose(joint_names, positions, velocities, expected_opening):
    """Capture a settled commanded opening; never changes physical joint states."""
    if not np.isfinite(expected_opening) or not 0<=expected_opening<=np.pi/4:
        raise ValueError('Declared opening must be within the source gripper stroke')
    names=CollisionWorld.GRIPPER_JOINT_NAMES
    indices=[joint_names.index(name) for name in names]
    measured=np.asarray(positions,dtype=float)[indices]
    speeds=np.asarray(velocities,dtype=float)[indices]
    expected=expected_opening*np.array([1,1,-1,1,-1,-1])
    if (not np.isfinite(measured).all() or not np.isfinite(speeds).all() or
            np.max(abs(speeds))>.01 or np.max(abs(measured-expected))>.001):
        raise ValueError('Gripper is not settled at its declared held opening')
    return dict(zip(names,measured.tolist()))


class ControlsCollisionWorld(CollisionWorld):
    allow_button_contact = False
    planning_gripper_joint_positions = None

    def _robot_hulls(self):
        # The baseline includes an open-gripper static hull. Fine button contact
        # uses the measured articulated finger hulls already supplied by shapes.
        return [x for x in super()._robot_hulls() if '/Gripper/' not in x[0]]

    def shapes(self, q, gripper_q=None, object_pose=None, *, _cache_robot=False):
        if gripper_q is None and np.asarray(q).size == 7 and self.planning_gripper_joint_positions is not None:
            gripper_q = self.planning_gripper_joint_positions
        shapes = super().shapes(q, gripper_q=gripper_q, object_pose=object_pose, _cache_robot=_cache_robot)
        # Fixed open-finger capsules are redundant with measured finger hulls.
        return [x for x in shapes if x[0] != 'finger']

    def _pairs(self, *args, **kwargs):
        pairs=super()._pairs(*args, **kwargs)
        if self.allow_button_contact:
            pairs=[p for p in pairs if not (
                p[1] in ('gripper/left_inner_finger','gripper/right_inner_finger')
                and self.contact_body_fragment in p[2])]
        return pairs

    def __init__(self, stage, kin, clearances=None, execution_clearances=None,
                 default_clearance=.002, *, env_root=None, env_origin=None,
                 contact_body_fragment='/Toaster015_sideL_button_cancel/',
                 fixture_cylinder_segments=1):
        import fcl
        from pxr import Usd, UsdGeom, UsdPhysics
        self.fcl=fcl;self.kin=kin;self.objects=[];self.attachment=None;self.meshes={}
        if not contact_body_fragment or not contact_body_fragment.startswith('/'):
            raise ValueError('Contact exemption requires an explicit target body path')
        self.contact_body_fragment=contact_body_fragment
        self._root_reference_transforms={};self._object_reference=[]
        self._last_obstacle_pose_call=None
        self._pair_selection_key=None;self._pair_selection_cache={}
        self._last_robot_shapes=None
        if (isinstance(fixture_cylinder_segments, bool) or not isinstance(fixture_cylinder_segments, int)
                or not 1 <= fixture_cylinder_segments <= 64):
            raise ValueError('Fixture cylinder segments must be an integer from 1 to 64')
        self.fixture_geometry_refinement=[]
        self.clearances={'person':.03,'glass':.06,'barrier':.02}
        self.execution_clearances={'person':.02,'glass':.05,'barrier':.01}
        if clearances is not None:self.clearances.update(clearances)
        if execution_clearances is not None:self.execution_clearances.update(execution_clearances)
        self.default_clearance=float(default_clearance)
        # Vectorized scenes place each kitchen at an environment origin. Keep
        # planning in the original kitchen coordinates and never include a
        # neighbouring clone in this world's collision geometry.
        origin = np.zeros(3) if env_origin is None else np.asarray(env_origin, dtype=float)
        if origin.shape != (3,) or not np.isfinite(origin).all():
            raise ValueError('Environment origin must be a finite xyz vector')
        root = stage.GetPseudoRoot() if env_root is None else stage.GetPrimAtPath(env_root)
        if not root:
            raise ValueError(f'Collision environment root does not exist: {env_root}')
        self.env_root = env_root
        self.env_origin = origin.copy()
        if min([self.default_clearance,*self.clearances.values(),*self.execution_clearances.values()])<0:
            raise ValueError('Clearances must be non-negative')
        cache=UsdGeom.XformCache()
        # Physics colliders can be hidden or authored with purpose="guide".
        # Rendering visibility/purpose must not remove them from collision checks.
        bounds=UsdGeom.BBoxCache(Usd.TimeCode(0),
            [UsdGeom.Tokens.default_,UsdGeom.Tokens.render,UsdGeom.Tokens.proxy,UsdGeom.Tokens.guide],
            False,True)
        for p in Usd.PrimRange(root,Usd.TraverseInstanceProxies()):
            name=str(p.GetPath())
            if '/Robot' in name or not p.HasAPI(UsdPhysics.CollisionAPI): continue
            if not UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get(): continue
            if p.IsA(UsdGeom.Mesh):
                m=UsdGeom.Mesh(p);verts=np.array(m.GetPointsAttr().Get(),dtype=float)
                if verts.ndim!=2 or verts.shape[1]!=3 or not len(verts):continue
                mat=np.array(cache.GetLocalToWorldTransform(p)).T
                mat[:3,3] -= origin
                verts=verts@mat[:3,:3].T+mat[:3,3]
                # Use collision vertices directly, independent of render bounds
                # and any stale authored extent on an invisible mesh.
                lo,hi=verts.min(0),verts.max(0)
            else:
                mat=np.array(cache.GetLocalToWorldTransform(p)).T
                mat[:3,3] -= origin
                primitive_bounds=_primitive_world_bounds(p,mat)
                if primitive_bounds is None:
                    bb=bounds.ComputeWorldBound(p).ComputeAlignedRange()
                    lo,hi=np.array(bb.GetMin())-origin,np.array(bb.GetMax())-origin
                else:
                    lo,hi=primitive_bounds
            if (not np.all(np.isfinite([lo,hi])) or np.any(hi<lo)
                    ):continue
            if p.IsA(UsdGeom.Mesh):
                indices=np.array(m.GetFaceVertexIndicesAttr().Get());faces=[];start=0
                for count in m.GetFaceVertexCountsAttr().Get():
                    face=indices[start:start+count];start+=count
                    faces.extend([[face[0],face[j],face[j+1]] for j in range(1,count-1)])
                self.meshes[name]=(verts.copy(),np.asarray(faces,dtype=np.int32))
                model=fcl.BVHModel();model.beginModel(len(verts),len(faces));model.addSubModel(verts,np.array(faces,dtype=np.int32));model.endModel()
                obj=fcl.CollisionObject(model)
            else:
                # Conservative world AABB for primitive USD colliders.
                obj=fcl.CollisionObject(fcl.Box(*(np.maximum(hi-lo,1e-4))),fcl.Transform((lo+hi)/2))
            components=[(name,obj,lo,hi)]
            if (fixture_cylinder_segments > 1 and '/barrier/' in name and p.IsA(UsdGeom.Cylinder)
                    and UsdGeom.Cylinder(p).GetHeightAttr().Get() > 4*UsdGeom.Cylinder(p).GetRadiusAttr().Get()):
                pieces=fixture_cylinder_bounds(p,mat,fixture_cylinder_segments)
                components=[(name+f'/__conservative_segment_{i:02d}',
                    fcl.CollisionObject(fcl.Box(*(high-low)),fcl.Transform((low+high)/2)),low,high)
                    for i,(low,high) in enumerate(pieces)]
                self.fixture_geometry_refinement.append(dict(source_collider=name,
                    segments=fixture_cylinder_segments,model='Union of circumscribing slice AABBs; no gaps or omitted source volume',
                    original_margins_retained=True,physics_collider_modified=False))
            # Retain the authored world geometry once. A moving rigid obstacle
            # can then be measured at its observed pose without rebuilding BVHs
            # or accumulating floating-point drift from incremental transforms.
            for label,component,low,high in components:
                self.objects.append((label,component,low,high))
                self._object_reference.append((np.array(component.getRotation()),np.array(component.getTranslation()),
                                               low.copy(),high.copy(),self.meshes.get(label)))
            ancestor=p
            while ancestor and not ancestor.IsPseudoRoot():
                ancestor_path=str(ancestor.GetPath())
                if ancestor_path not in self._root_reference_transforms:
                    reference=np.array(cache.GetLocalToWorldTransform(ancestor)).T
                    reference[:3,3] -= origin
                    self._root_reference_transforms[ancestor_path]=reference
                ancestor=ancestor.GetParent()
        self.low=np.array([x[2] for x in self.objects]).reshape(-1,3);self.high=np.array([x[3] for x in self.objects]).reshape(-1,3)
        self.robot_hulls=self._robot_hulls()
        self.gripper_models,self.gripper_sweep=self._gripper_geometry()
        self.last_collision=None
        self.last_clearance=None
        print('PLANNER_WORLD',len(self.objects),'collision shapes',flush=True)
