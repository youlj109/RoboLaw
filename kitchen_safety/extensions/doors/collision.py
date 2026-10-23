"""Task-local full collision ingestion; no legacy kitchen-coordinate crop."""
import numpy as np
from kitchen_safety.collision_planner import CollisionWorld, _primitive_world_bounds
class DoorCollisionWorld(CollisionWorld):
    def __init__(self, stage, kin, clearances=None, execution_clearances=None,
                 default_clearance=.002, *, env_root=None, env_origin=None):
        import fcl
        from pxr import Usd, UsdGeom, UsdPhysics
        self.fcl=fcl;self.kin=kin;self.objects=[];self.attachment=None;self.meshes={}
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
            # The fixed stand is real external geometry although authored below
            # Robot/panda_link0. Keep it while excluding moving robot colliders.
            if ('/Robot' in name and '/stand_instanceable/' not in name) or not p.HasAPI(UsdPhysics.CollisionAPI): continue
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
        self.last_collision=None
        self.last_clearance=None
        print('PLANNER_WORLD',len(self.objects),'collision shapes',flush=True)


    def shapes(self,*args,**kwargs):
        shapes=super().shapes(*args,**kwargs)
        if shapes[0][0]!='arm':raise RuntimeError('Source proximal envelope ordering changed')
        shapes[0]=('arm_mount_envelope',*shapes[0][1:])
        return shapes

    def _pairs(self,*args,**kwargs):
        pairs=super()._pairs(*args,**kwargs)
        # The source proximal capsule extends 75 mm below the fixed base origin
        # into its own mounting stand. It is an envelope, not a physical link.
        # Keep every source robot hull/mesh against the stand at the original
        # clearance, and keep this envelope against all foreign scene geometry.
        pairs=[p for p in pairs if not (p[1]=='arm_mount_envelope' and '/stand_instanceable/' in p[2])]
        if not getattr(self,'allow_handle_contact',False):return pairs
        return [p for p in pairs if not ('/fridge_door_handle' in p[2] and p[1] in ('finger','gripper/left_inner_finger','gripper/right_inner_finger'))]

    def move_body(self,key,delta):
        """Apply measured rigid-body delta to initially baked, possibly scaled USD shapes."""
        for i,(name,obj,lo,hi) in enumerate(self.objects):
            if not self._matches(name,key):continue
            rot,pos,reflo,refhi,mesh=self._object_reference[i]
            obj.setRotation(delta[:3,:3]@rot);obj.setTranslation(delta[:3,:3]@pos+delta[:3,3])
            corners=np.array([[x,y,z] for x in (reflo[0],refhi[0]) for y in (reflo[1],refhi[1]) for z in (reflo[2],refhi[2])])
            pts=corners@delta[:3,:3].T+delta[:3,3]
            self.objects[i]=(name,obj,pts.min(0),pts.max(0))
        self.low=np.array([x[2] for x in self.objects]);self.high=np.array([x[3] for x in self.objects])

    def _robot_hulls(self):
        # The baseline's whole-gripper convex envelope fills the open jaw gap.
        # Keep all arm hulls; the inherited articulated per-body gripper hulls
        # still cover every finger and the gripper base at measured angles.
        return [item for item in super()._robot_hulls() if item[0]!='/panda/Gripper/Robotiq_2F_85/base_link']
