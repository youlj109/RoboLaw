"""CPU collision-aware joint planning for Arena; no InternData code is copied.

World shapes are read from the live USD stage. Arm capsules and mesh envelopes
are retained, and the gripper covers its full stroke or measured finger poses.
Mesh obstacles such as the person retain their triangles. Configurable surface
separation margins distinguish planned paths from measured execution.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def _primitive_world_bounds(prim, world_transform):
    """Bounds from USD shape parameters, never an authored ``extent``.

    The support of each primitive is evaluated along the three world axes.
    This also handles inherited rotation, nonuniform scale, and shear. USD
    capsule ``height`` is the cylindrical segment, excluding both end caps.
    Return None for schemas handled by the caller's general BBox fallback.
    """
    from pxr import UsdGeom
    transform=np.asarray(world_transform,dtype=float)
    linear,center=transform[:3,:3],transform[:3,3]
    if prim.IsA(UsdGeom.Cube):
        half=abs(float(UsdGeom.Cube(prim).GetSizeAttr().Get()))/2
        extent=np.abs(linear).sum(axis=1)*half
    elif prim.IsA(UsdGeom.Sphere):
        radius=abs(float(UsdGeom.Sphere(prim).GetRadiusAttr().Get()))
        extent=np.linalg.norm(linear,axis=1)*radius
    else:
        schema=next((cls(prim) for cls in
            (UsdGeom.Cylinder,UsdGeom.Cone,UsdGeom.Capsule) if prim.IsA(cls)),None)
        if schema is None:return None
        axis={'X':0,'Y':1,'Z':2}[str(schema.GetAxisAttr().Get()).upper()]
        radius=abs(float(schema.GetRadiusAttr().Get()))
        half=abs(float(schema.GetHeightAttr().Get()))/2
        axial=linear[:,axis]*half
        radial=np.linalg.norm(np.delete(linear,axis,axis=1),axis=1)*radius
        if prim.IsA(UsdGeom.Cone):
            # The base disk is at -height/2, the tip at +height/2.
            return center+np.minimum(-axial-radial,axial),center+np.maximum(-axial+radial,axial)
        if prim.IsA(UsdGeom.Capsule):
            extent=np.abs(axial)+np.linalg.norm(linear,axis=1)*radius
        else:
            extent=np.abs(axial)+radial
    return center-extent,center+extent


class CollisionWorld:
    GRIPPER_JOINT_NAMES = ('finger_joint', 'right_outer_knuckle_joint',
                           'left_inner_finger_joint', 'right_inner_finger_joint',
                           'left_inner_finger_knuckle_joint', 'right_inner_finger_knuckle_joint')

    def __init__(self, stage, kin, clearances=None, execution_clearances=None,
                 default_clearance=.002, *, env_root=None, env_origin=None,
                 pick_object='lime', destination_object='wooden_bowl',
                 source_surface_suffix='/counter_main_main_group/top_geometry_right',
                 movable_objects=None):
        import fcl
        from pxr import Usd, UsdGeom, UsdPhysics
        self.object_size=np.array([.075506996,.061337937,.060035701]);self.object_center=np.array([-.000032013,.002372919,.029799371]);self.fcl=fcl;self.kin=kin;self.objects=[];self.attachment=None;self.meshes={}
        self._root_reference_transforms={};self._object_reference=[]
        self._last_obstacle_pose_call=None
        self._pair_selection_key=None;self._pair_selection_cache={}
        self._last_robot_shapes=None
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
        self.pick_object = pick_object
        self.destination_object = destination_object
        self.source_surface_suffix = source_surface_suffix
        if movable_objects is None:
            movable_objects = (destination_object, 'broccoli', 'sweet_potato', 'red_bell_pepper', 'glass')
        self.movable_objects = tuple(movable_objects)
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
            self.objects.append((name,obj,lo,hi))
            # Retain the authored world geometry once. A moving rigid obstacle
            # can then be measured at its observed pose without rebuilding BVHs
            # or accumulating floating-point drift from incremental transforms.
            self._object_reference.append((np.array(obj.getRotation()),np.array(obj.getTranslation()),
                                           lo.copy(),hi.copy(),self.meshes.get(name)))
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
        # The round fruit's rotated bounding box can cut deeply through its
        # support and neighbours. Retain the source collider vertices and use
        # their conservative convex hull, transformed by the measured root.
        pick_root=(env_root or '/World/envs/env_0')+'/'+self.pick_object
        if pick_root not in self._root_reference_transforms:
            raise RuntimeError(f'Pick object root missing from collision world: {pick_root}')
        reference=self._root_reference_transforms[pick_root]
        inverse=np.linalg.inv(reference)
        parts=[v@inverse[:3,:3].T+inverse[:3,3] for n,(v,f) in self.meshes.items()
               if self._matches(n,self.pick_object)]
        if not parts:raise RuntimeError(f'Original {self.pick_object} collision vertices are required')
        self.lime_model,self.lime_vertices=self._convex(np.concatenate(parts))
        # Geometry center/size from actual pick-object hull (not lime-only constants).
        lo,hi=self.lime_vertices.min(0),self.lime_vertices.max(0)
        self.object_center=(lo+hi)/2
        self.object_size=hi-lo
        self.allowed_phase='initial'
        # Spawn scale is already baked into world vertices. PhysX root poses
        # contain rigid transforms only (sweet_potato uses source scale 1.5).
        # Preserve that geometry, and remove scale only from the reference
        # transform used to compute measured rigid pose deltas.
        self.baked_root_scales={}
        for object_name in self.movable_objects:
            key=(env_root or '/World/envs/env_0')+'/'+object_name
            if key not in self._root_reference_transforms:continue
            reference=self._root_reference_transforms[key].copy()
            u,scale,vh=np.linalg.svd(reference[:3,:3]);reference[:3,:3]=u@vh
            if np.linalg.det(reference[:3,:3])<0:raise ValueError('Reflected source root is unsupported')
            self.baked_root_scales[object_name]=scale.tolist()
            self._root_reference_transforms[key]=reference
        self.last_collision=None
        self.last_clearance=None
        print('PLANNER_WORLD',len(self.objects),'collision shapes',flush=True)

    def bind_pick(self, pick_object):
        """Rebind convex pick hull to another rigid (B03 banana_1 -> banana_2)."""
        import numpy as np
        self.pick_object = pick_object
        pick_root = (self.env_root or '/World/envs/env_0') + '/' + self.pick_object
        if pick_root not in self._root_reference_transforms:
            raise RuntimeError(f'Pick object root missing from collision world: {pick_root}')
        reference = self._root_reference_transforms[pick_root]
        inverse = np.linalg.inv(reference)
        parts = [v @ inverse[:3, :3].T + inverse[:3, 3] for n, (v, f) in self.meshes.items()
                 if self._matches(n, self.pick_object)]
        if not parts:
            raise RuntimeError(f'Original {self.pick_object} collision vertices are required')
        self.lime_model, self.lime_vertices = self._convex(np.concatenate(parts))
        lo, hi = self.lime_vertices.min(0), self.lime_vertices.max(0)
        self.object_center = (lo + hi) / 2
        self.object_size = hi - lo
        self.attachment = None

    def set_obstacle_pose(self, key, pose):
        """Set one rigid obstacle root to an observed world xyz + xyzw pose.

        ``key`` is an exact USD root path or one ancestor name, such as ``glass``.
        Every matching collider is transformed relative to its construction-time
        USD root pose. Meshes, FCL transforms, and broad-phase bounds all stay in
        sync. This does not update an already-created cuRobo world; that planner
        must separately refresh its world before planning against a moved prop.
        """
        pose=np.asarray(pose,dtype=float)
        if pose.shape!=(7,) or not np.isfinite(pose).all() or np.linalg.norm(pose[3:])<1e-12:
            raise ValueError('Obstacle pose must contain finite xyz + nonzero xyzw quaternion')
        previous=self._last_obstacle_pose_call
        if previous is not None and key==previous[0] and np.array_equal(pose,previous[1]):
            return previous[2]
        # Cache only the immediately preceding successful call. Updating another
        # key (including a parent root or alias) invalidates the old geometry.
        self._last_obstacle_pose_call=None
        target=np.eye(4);target[:3,:3]=Rotation.from_quat(pose[3:]).as_matrix();target[:3,3]=pose[:3]
        indices=[i for i,(name,*_) in enumerate(self.objects) if self._matches(name,key)]
        if not indices:raise ValueError(f'No collision geometry matches obstacle {key!r}')
        roots=[]
        for i in indices:
            name=self.objects[i][0]
            if key.startswith('/'):
                # Preserve the USD path's case for reference-transform lookup.
                root=name[:len(key.rstrip('/'))]
            else:
                parts=name.split('/');end=next(j for j,part in enumerate(parts) if part.lower()==key.lower())
                root='/'.join(parts[:end+1])
            roots.append(root)
        if len(set(roots))!=1:
            raise ValueError(f'Obstacle key {key!r} is ambiguous; use an exact USD root path')
        reference=self._root_reference_transforms[roots[0]]
        delta=target@np.linalg.inv(reference)
        rotation,translation=delta[:3,:3],delta[:3,3]
        if not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-6) or np.linalg.det(rotation)<0:
            raise ValueError('Moving obstacle root must have a rigid, unscaled reference transform')
        for i in indices:
            name,obj,_,_=self.objects[i]
            initial_rotation,initial_translation,lo,hi,mesh=self._object_reference[i]
            obj.setTransform(self.fcl.Transform(rotation@initial_rotation,rotation@initial_translation+translation))
            if mesh is not None:
                vertices,faces=mesh
                current=vertices@rotation.T+translation
                self.meshes[name]=(current,faces)
                new_lo,new_hi=current.min(0),current.max(0)
            else:
                center=rotation@((lo+hi)/2)+translation
                radius=np.abs(rotation)@((hi-lo)/2)
                new_lo,new_hi=center-radius,center+radius
            self.objects[i]=(name,obj,new_lo,new_hi)
            self.low[i]=new_lo;self.high[i]=new_hi
        self._last_obstacle_pose_call=(key,pose.copy(),len(indices))
        return len(indices)

    def _convex(self, vertices):
        """A solid convex shape, including containment (a triangle shell alone does not)."""
        from scipy.spatial import ConvexHull
        vertices=np.asarray(vertices,dtype=float)
        hull=ConvexHull(vertices)
        # FCL requires outward, counter-clockwise face winding.
        faces=hull.simplices.copy()
        normals=np.cross(vertices[faces[:,1]]-vertices[faces[:,0]],vertices[faces[:,2]]-vertices[faces[:,0]])
        reverse=np.einsum('ij,ij->i',normals,hull.equations[:,:3])<0
        faces[reverse]=faces[reverse][:,[0,2,1]]
        # Interior source vertices do not change the hull and slow every GJK support query.
        remap=np.full(len(vertices),-1,dtype=np.int32)
        remap[hull.vertices]=np.arange(len(hull.vertices),dtype=np.int32)
        faces=remap[faces];vertices=vertices[hull.vertices]
        packed=np.column_stack([np.full(len(faces),3,dtype=np.int32),faces]).ravel()
        return self.fcl.Convex(vertices,len(faces),packed),vertices

    def _gripper_geometry(self):
        """Articulated finger hulls plus a conservative envelope of their full stroke."""
        from pxr import Usd,UsdGeom,UsdPhysics
        from kitchen_safety.planning import ROBOT_USD,transform
        stage=Usd.Stage.Open(str(ROBOT_USD));cache=UsdGeom.XformCache()
        self.gripper_root='/panda/Gripper/Robotiq_2F_85/base_link'
        self.gripper_joints=[];points={}
        def joint_frame(j,side):
            quat=getattr(j,'GetLocalRot'+side+'Attr')().Get()
            return transform(getattr(j,'GetLocalPos'+side+'Attr')().Get(),[*quat.GetImaginary(),quat.GetReal()])
        for prim in Usd.PrimRange(stage.GetPrimAtPath('/panda/Gripper'),Usd.TraverseInstanceProxies()):
            if prim.IsA(UsdPhysics.Joint):
                j=UsdPhysics.Joint(prim)
                axis=UsdPhysics.RevoluteJoint(prim).GetAxisAttr().Get().lower() if prim.IsA(UsdPhysics.RevoluteJoint) else None
                self.gripper_joints.append((prim.GetName(),str(j.GetBody0Rel().GetTargets()[0]),str(j.GetBody1Rel().GetTargets()[0]),joint_frame(j,'0'),np.linalg.inv(joint_frame(j,'1')),axis))
            if not prim.IsA(UsdGeom.Mesh):continue
            body=prim.GetParent()
            while body and not body.HasAPI(UsdPhysics.RigidBodyAPI):body=body.GetParent()
            if not body:continue
            local=np.linalg.inv(np.array(cache.GetLocalToWorldTransform(body)).T)@np.array(cache.GetLocalToWorldTransform(prim)).T
            xyz=np.array(UsdGeom.Mesh(prim).GetPointsAttr().Get(),dtype=float)
            points.setdefault(str(body.GetPath()),[]).append(xyz@local[:3,:3].T+local[:3,3])
        models=[]
        for body,parts in points.items():
            model,verts=self._convex(np.concatenate(parts));models.append((body,model,verts))
        sweep=[]
        # Mimic gearing in the source USD: +,+,-,+,-,- for the six named joints.
        # Sampling every 1.4 degrees has <0.02 mm chord error for this gripper;
        # a 1 mm axis-aligned Minkowski padding also covers that sampling error.
        signs=np.array([1,1,-1,1,-1,-1])
        for angle in np.linspace(0,np.pi/4,33):
            links=self._gripper_links(dict(zip(self.GRIPPER_JOINT_NAMES,signs*angle)))
            for body,_,verts in models:
                t=links[body];sweep.append(verts@t[:3,:3].T+t[:3,3])
        _,surface=self._convex(np.concatenate(sweep))
        offsets=np.array([[x,y,z] for x in [-.001,.001] for y in [-.001,.001] for z in [-.001,.001]])
        envelope,verts=self._convex((surface[:,None,:]+offsets[None,:,:]).reshape(-1,3))
        return models,(envelope,verts)

    def _gripper_links(self, angles):
        links={self.gripper_root:np.eye(4)};pending=self.gripper_joints.copy()
        while pending:
            count=len(pending)
            for j in pending.copy():
                name,parent,child,a,b,axis=j
                if parent not in links:continue
                r=np.eye(4)
                if axis:r[:3,:3]=Rotation.from_euler(axis,angles[name]).as_matrix()
                links[child]=links[parent]@a@r@b;pending.remove(j)
            if len(pending)==count:raise RuntimeError('Disconnected gripper joint tree')
        return links

    def _robot_hulls(self):
        from pxr import Usd,UsdGeom
        from scipy.spatial import ConvexHull
        from kitchen_safety.planning import ROBOT_USD
        stage=Usd.Stage.Open(str(ROBOT_USD));cache=UsdGeom.XformCache();models=[]
        bodies=[(f'/panda/panda_link{i}',f'/panda/panda_link{i}') for i in range(9)]
        bodies.append(('/panda/Gripper/Robotiq_2F_85/base_link','/panda/Gripper'))
        for body_path,geometry_path in bodies:
            body=stage.GetPrimAtPath(body_path)
            inv=np.linalg.inv(np.array(cache.GetLocalToWorldTransform(body)).T)
            points=[]
            for prim in Usd.PrimRange(stage.GetPrimAtPath(geometry_path),Usd.TraverseInstanceProxies()):
                if not prim.IsA(UsdGeom.Mesh):continue
                xyz=np.array(UsdGeom.Mesh(prim).GetPointsAttr().Get(),dtype=float)
                if len(xyz)<4:continue
                local=inv@np.array(cache.GetLocalToWorldTransform(prim)).T
                points.append(xyz@local[:3,:3].T+local[:3,3])
            if not points:continue
            verts=np.concatenate(points)
            hull=ConvexHull(verts)
            # A closed convex envelope accounts for protrusions missed by joint-axis capsules.
            model=self.fcl.BVHModel();model.beginModel(len(verts),len(hull.simplices))
            model.addSubModel(verts,hull.simplices.astype(np.int32));model.endModel()
            models.append((body_path,model,verts[hull.vertices]))
        return models

    def shapes(self,q,gripper_q=None,object_pose=None,*,_cache_robot=False):
        q=np.asarray(q,dtype=float)
        if gripper_q is None and len(q)>7:gripper_q=q[7:13]
        q=q[:7]
        if _cache_robot:
            # Only immutable input values are cached. In-place changes to actual
            # joints, measured fingers or the robot base must invalidate reuse.
            fingers=(None if gripper_q is None else
                     tuple(gripper_q[name] for name in self.GRIPPER_JOINT_NAMES)
                     if isinstance(gripper_q,dict) else tuple(gripper_q))
            key=(tuple(q),fingers,id(self.kin),tuple(self.kin.base.flat))
            if self._last_robot_shapes is not None and key==self._last_robot_shapes[0]:
                shapes=list(self._last_robot_shapes[1])
                self._append_bottle_shape(shapes,q,object_pose)
                return shapes
        fcl=self.fcl
        links=self.kin.links(q)
        ts=list(links.values());pos=[t[:3,3] for t in ts]
        shapes=[]
        def capsule(a,b,r,label):
            d=b-a;length=np.linalg.norm(d)
            if length<1e-5: geom=fcl.Sphere(r);rot=np.eye(3)
            else:
                geom=fcl.Capsule(r,length)
                z=d/length
                # Both reference axes are basis vectors; preserve the same
                # cross products without NumPy's generic dimension dispatch.
                x=np.array([0.,-z[2],z[1]]) if abs(z[0])<.9 else np.array([z[2],0.,-z[0]])
                x/=np.linalg.norm(x)
                y=np.array([z[1]*x[2]-z[2]*x[1],z[2]*x[0]-z[0]*x[2],z[0]*x[1]-z[1]*x[0]])
                rot=np.column_stack([x,y,z])
            shapes.append((label,fcl.CollisionObject(geom,fcl.Transform(rot,(a+b)/2)),np.minimum(a,b)-r,np.maximum(a,b)+r))
        for a,b,r in [(0,1,.075),(1,3,.065),(3,4,.065),(4,5,.065),(5,7,.065),(7,8,.055)]:
            capsule(pos[a],pos[b],r,'arm')
        base=ts[-1]
        def local(x,y=0):return base[:3,3]+base[:3,:3]@np.array([x,y,0])
        capsule(local(0),local(.065),.043,'gripper')
        for y in [-.039,.039]:capsule(local(.085,y),local(.145,y),.012,'finger')
        for body,model,verts in self.robot_hulls:
            t=links[body];world_verts=verts@t[:3,:3].T+t[:3,3]
            shapes.append(('robot_hull',fcl.CollisionObject(model,fcl.Transform(t[:3,:3],t[:3,3])),world_verts.min(axis=0),world_verts.max(axis=0)))
        base=links[self.gripper_root]
        if gripper_q is None:
            model,verts=self.gripper_sweep
            world_verts=verts@base[:3,:3].T+base[:3,3]
            shapes.append(('gripper_sweep',fcl.CollisionObject(model,fcl.Transform(base[:3,:3],base[:3,3])),world_verts.min(0),world_verts.max(0)))
        else:
            angles=gripper_q if isinstance(gripper_q,dict) else dict(zip(self.GRIPPER_JOINT_NAMES,gripper_q))
            local_links=self._gripper_links(angles)
            for body,model,verts in self.gripper_models:
                t=base@local_links[body];world_verts=verts@t[:3,:3].T+t[:3,3]
                shapes.append(('gripper/'+body.rsplit('/',1)[-1],fcl.CollisionObject(model,fcl.Transform(t[:3,:3],t[:3,3])),world_verts.min(0),world_verts.max(0)))
        if _cache_robot:self._last_robot_shapes=(key,list(shapes))
        self._append_bottle_shape(shapes,q,object_pose)
        return shapes

    def _append_bottle_shape(self,shapes,q,object_pose):
        # Bottle pose/attachment is intentionally outside the robot cache:
        # successive free/attached checks can have identical joints and a
        # different observed bottle pose, even within one physics update.
        t=self._object_pose(q,object_pose)
        if t is not None:
            vertices=self.lime_vertices@t[:3,:3].T+t[:3,3]
            shapes.append(('carried_lime',self.fcl.CollisionObject(self.lime_model,
                self.fcl.Transform(t[:3,:3],t[:3,3])),vertices.min(0),vertices.max(0)))

    def _object_pose(self,q,object_pose):
        if object_pose is None:
            return self.kin.fk(np.asarray(q)[:7])@self.attachment if self.attachment is not None else None
        pose=np.asarray(object_pose,dtype=float)
        if pose.shape==(4,4):return pose
        if pose.shape==(7,):
            from kitchen_safety.planning import transform
            return transform(pose[:3],pose[3:])
        raise ValueError('object_pose must be a 4x4 transform or xyz+xyzw pose')

    @staticmethod
    def _matches(name,key):
        name=name.lower();key=key.lower()
        return (name==key or name.startswith(key.rstrip('/')+'/')) if key.startswith('/') else key in name.split('/')

    def _margin(self,name,mode):
        if mode not in ('planning','execution'):raise ValueError('mode must be planning or execution')
        rules=self.clearances if mode=='planning' else self.execution_clearances
        matched=[value for key,value in rules.items() if self._matches(name,key)]
        return float(max(matched)) if matched else self.default_clearance

    def _pairs(self,q,ignore,mode,gripper_q,object_pose,obstacle_filter,max_slack=float('inf')):
        """Candidates with a conservative AABB lower bound on required clearance."""
        # The target container remains an obstacle for every robot link.
        # Only the fruit/container support pair is intended during placement.
        ignore=tuple(x for x in ignore if x!=self.destination_object)
        pairs=[]
        # Placements change low/high and FCL transforms, never this selection.
        # Include live rule values and ordered names so mutable margins, masks,
        # or topology replacement cannot reuse stale indices or thresholds.
        rules=self.clearances if mode=='planning' else self.execution_clearances
        selection_key=(tuple(entry[0] for entry in self.objects),mode,
                       tuple(rules.items()),self.default_clearance)
        if selection_key!=self._pair_selection_key:
            self._pair_selection_key=selection_key;self._pair_selection_cache={}
        query=(tuple(ignore),tuple(obstacle_filter) if obstacle_filter else ())
        cached=self._pair_selection_cache.get(query)
        if cached is None:
            indices=np.array([i for i,(name,*_) in enumerate(self.objects)
                              if (not obstacle_filter or any(self._matches(name,key) for key in obstacle_filter))
                              and not any(self._matches(name,key) for key in ignore)],dtype=int)
            margins=np.array([self._margin(self.objects[i][0],mode) for i in indices])
            cached=(indices,margins);self._pair_selection_cache[query]=cached
        indices,margins=cached
        if not len(indices):return pairs
        lows=self.low[indices];highs=self.high[indices]
        shapes=self.shapes(q,gripper_q,object_pose,_cache_robot=True)
        # Bounds and candidate lookup traverse this collection more than once.
        # Materialize one-shot iterables; the ordinary list path needs no copy.
        if not isinstance(shapes,list):shapes=list(shapes)
        shape_lows=np.array([entry[2] for entry in shapes])
        shape_highs=np.array([entry[3] for entry in shapes])
        if np.isfinite(max_slack):
            # Every arm/gripper/bottle AABB is contained in this union, so
            # its distance to a world AABB cannot exceed any per-shape bound.
            # Reject far world columns before allocating the shape x world
            # array. Keep the same subtraction/comparison and index order;
            # unrestricted clearance queries retain their original path.
            union_low=shape_lows.min(axis=0);union_high=shape_highs.max(axis=0)
            union_bounds=np.linalg.norm(np.maximum(np.maximum(lows-union_high,
                                                               union_low-highs),0),axis=1)
            keep=union_bounds-margins<=max_slack
            if not np.any(keep):return pairs
            if not np.all(keep):
                indices=indices[keep];margins=margins[keep]
                lows=lows[keep];highs=highs[keep]
        bounds=np.linalg.norm(np.maximum(np.maximum(lows[None]-shape_highs[:,None],
                                                    shape_lows[:,None]-highs[None]),0),axis=2)
        # np.nonzero retains the original shape-major order, including equal
        # lower-bound ties, while batching the unchanged AABB lower bound.
        for shape_index,k in zip(*np.nonzero(bounds-margins[None]<=max_slack)):
            label,shape,lo,hi=shapes[shape_index]
            name,obj,ob_lo,ob_hi=self.objects[indices[k]]
            if label=='carried_lime':
                if self._matches(name,self.destination_object) and self.allowed_phase in ('lower','release','retreat','settle_final'):continue
                source_surface=name.endswith(self.source_surface_suffix)
                if source_surface and getattr(self,'source_support_mode',None)=='native_rest_until_detached':
                    from .source_support import support_allowed
                    actual_root=self._object_pose(q,object_pose)[:3,3]
                    if support_allowed(phase=self.allowed_phase,detached=self.source_detached,
                        hull_min_z=lo[2],root=actual_root,initial_hull_min_z=self.source_resting_min_z,
                        initial_root=self.source_resting_root):continue
                    # This model has one bounded exception. Do not fall through
                    # to the historical closed-grasp/monotonic alternatives.
                    margin=float(margins[k]);bound=float(bounds[shape_index,k])
                    pairs.append((bound-margin,label,name,shape,obj,margin));continue
                if (source_surface and self.allowed_phase in ('pregrasp','approach','close')
                    and hasattr(self,'source_resting_min_z') and object_pose is not None):
                    actual_root=self._object_pose(q,object_pose)[:3,3]
                    if (np.linalg.norm(actual_root[:2]-self.source_resting_xy)<.05
                        and lo[2]>=self.source_resting_min_z-.0005):continue
                # Only this source support: calibrate from the physically
                # observed, already-safe closed grasp. The collector increases
                # the lower bound with measured upward progress, permitting at
                # most 0.5 mm jitter and no lateral dragging beyond 50 mm.
                if self.allowed_phase=='lift' and source_surface:
                    floor=getattr(self,'source_lift_floor_z',ob_hi[2]-.003)
                    root=self._object_pose(q,object_pose)[:3,3]
                    xy_ok=(not hasattr(self,'source_resting_xy') or
                        np.linalg.norm(root[:2]-self.source_resting_xy)<.05)
                    if xy_ok and lo[2]>=floor:continue
            margin=float(margins[k]);bound=float(bounds[shape_index,k])
            pairs.append((bound-margin,label,name,shape,obj,margin))
        return sorted(pairs,key=lambda pair:pair[0])

    def _measurement(self,pair):
        _,label,name,shape,obj,margin=pair
        raw=float(self.fcl.distance(shape,obj,self.fcl.DistanceRequest(),self.fcl.DistanceResult()))
        intersecting=raw<0
        if raw==0:
            intersecting=bool(self.fcl.collide(shape,obj,self.fcl.CollisionRequest(),self.fcl.CollisionResult()))
        # FCL's negative mesh distance is a collision sentinel, not penetration depth.
        distance=max(0.,raw)
        return dict(valid=bool(not intersecting and distance>=margin),distance_m=distance,
                    required_m=margin,slack_m=distance-margin,robot_part=label,obstacle=name,
                    intersecting=intersecting)

    def valid(self,q,ignore=(),*,mode='planning',gripper_q=None,object_pose=None,obstacle_filter=None):
        """Fast clearance test. Execution callers should supply actual finger joint angles.

        A 13-element q uses GRIPPER_JOINT_NAMES for its trailing six entries;
        gripper_q={joint_name: angle} avoids dependence on articulation joint ordering.
        """
        self.last_collision=None;self.last_clearance=None
        for pair in self._pairs(q,ignore,mode,gripper_q,object_pose,obstacle_filter,max_slack=0.):
            if pair[0]>0:break
            result=self._measurement(pair)
            if not result['valid']:
                self.last_clearance=result;self.last_collision=(result['robot_part'],result['obstacle']);return False
        return True

    def clearance(self,q,ignore=(),*,mode='execution',gripper_q=None,object_pose=None,obstacle_filter=None):
        """Return the pair with the least clearance slack, optionally restricted to named obstacles.

        distance_m is the measured surface separation, not the margin. No matching
        obstacle returns infinity and obstacle=None. Intersections report zero;
        negative FCL sentinels are never presented as penetration depths.
        """
        best=dict(valid=True,distance_m=float('inf'),required_m=0.,slack_m=float('inf'),
                  robot_part=None,obstacle=None,intersecting=False)
        for pair in self._pairs(q,ignore,mode,gripper_q,object_pose,obstacle_filter):
            if pair[0]>best['slack_m']:break
            result=self._measurement(pair)
            if result['slack_m']<best['slack_m'] or (result['slack_m']==best['slack_m'] and result['intersecting'] and not best['intersecting']):best=result
        self.last_clearance=best
        return best

    def edge(self,a,b,ignore=(),resolution=.02):
        steps=max(2,int(np.ceil(np.max(np.abs(b-a))/resolution))+1)
        return all(self.valid(a+(b-a)*u,ignore) for u in np.linspace(0,1,steps))

    def plan(self,start,goal,ignore=(),seed=0,max_iterations=2500):
        if not self.valid(start,ignore):raise RuntimeError(f'Planning start in collision: {self.last_collision}')
        if not self.valid(goal,ignore):raise RuntimeError(f'Planning goal in collision: {self.last_collision}')
        if self.edge(start,goal,ignore):return [start,goal]
        rng=np.random.default_rng(seed);trees=[([start.copy()],[-1]),([goal.copy()],[-1])]
        def extend(tree,target):
            nodes,parents=tree;idx=int(np.argmin(np.linalg.norm(np.array(nodes)-target,axis=1)))
            d=target-nodes[idx];norm=np.linalg.norm(d)
            new=nodes[idx]+d*min(1,.18/max(norm,1e-9))
            if not self.edge(nodes[idx],new,ignore):return None
            nodes.append(new);parents.append(idx);return len(nodes)-1
        def chain(tree,i):
            path=[]
            while i>=0:path.append(tree[0][i]);i=tree[1][i]
            return path[::-1]
        for iteration in range(max_iterations):
            side=iteration%2;a,b=trees[side],trees[1-side]
            sample=b[0][0] if rng.random()<.2 else rng.uniform(self.kin.limits[:,0],self.kin.limits[:,1])
            ia=extend(a,sample)
            if ia is None:continue
            for _ in range(80):
                ib=extend(b,a[0][ia])
                if ib is None:break
                if np.linalg.norm(b[0][ib]-a[0][ia])<1e-6:
                    first,second=chain(a,ia),chain(b,ib)
                    path=first+second[-2::-1]
                    if side:path=path[::-1]
                    for _ in range(100):
                        i,j=sorted(rng.integers(0,len(path),2))
                        if j>i+1 and self.edge(path[i],path[j],ignore):path=path[:i+1]+path[j:]
                    return path
        raise RuntimeError('RRT-Connect exhausted its planning budget')
