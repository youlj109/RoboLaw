"""cuRobo adapter with geometry from the actual lime asset."""
import numpy as np
from kitchen_safety.curobo_backend import CuroboPlanner
class LimePlanner(CuroboPlanner):
    def _prepare_path_query(self, ignore_names, grip_binary, phase):
        self.active_ignore_names=tuple(ignore_names)
        self.active_phase=phase
        return super()._prepare_path_query(ignore_names,grip_binary,phase)

    def plan_pose(self,q_arm7,target,ignore_names=(),grip_binary=0,phase=None):
        self.query_start_q=np.asarray(q_arm7).copy()
        if phase!='pregrasp':return super().plan_pose(q_arm7,target,ignore_names,grip_binary,phase)
        # A collision-free pregrasp endpoint alone does not guarantee the
        # contact column is feasible. Keep the original TCP pose/orientation;
        # search only redundant arm IK postures when a hazard blocks the column.
        contact_target=np.asarray(target).copy();contact_target[2,3]-=.125
        attempts=[];nominal=None
        def column(q_goal):
            self._prepare_path_query(ignore_names,grip_binary,'approach')
            local=CuroboPlanner._cartesian_vertical(self,q_goal,contact_target,'approach')
            checked=self.validate_path(local['positions'],ignore_names,grip_binary,phase='approach')
            if not checked['valid']:raise RuntimeError('Full pregrasp contact column rejected: '+str(checked))
            return checked
        try:
            nominal=super().plan_pose(q_arm7,target,ignore_names,grip_binary,phase)
            lookahead=column(nominal['positions'][-1])
            nominal['metadata']['contact_column_validation']=lookahead
            self._prepare_path_query(ignore_names,grip_binary,'pregrasp')
            self.query_start_q=np.asarray(q_arm7).copy()
            return nominal
        except RuntimeError as exc:attempts.append({'candidate':'nominal','error':str(exc)})
        seed=(nominal['positions'][-1] if nominal is not None else self.kin.ik(target,q_arm7,attempts=12))
        rng=np.random.default_rng(self.seed)
        for i,scale in enumerate([.2]*16+[.4]*16+[.7]*16+[1.]*16):
            try:
                candidate=self.kin.ik(target,seed+rng.normal(0,scale,7),attempts=1)
                self._prepare_path_query(ignore_names,grip_binary,'pregrasp')
                if not bool(self.motion_gen.ik_solver.check_valid(self.args.to_device(candidate[None])).all()):
                    raise RuntimeError('Pregrasp IK violates cuRobo world/self/limits')
                if not self.world.valid(candidate,ignore_names):raise RuntimeError('Pregrasp IK violates full FCL world')
                lookahead=column(candidate)
                path=self.plan_joint_goal(q_arm7,candidate,ignore_names,grip_binary,phase='pregrasp')
                checked=self.validate_path(path['positions'],ignore_names,grip_binary,phase='pregrasp')
                if not checked['valid']:raise RuntimeError('Pregrasp joint path rejected: '+str(checked))
                path['metadata']['pregrasp_redundancy_search']={'same_tcp_goal':np.asarray(target).tolist(),
                    'selected_q_goal':candidate.tolist(),'failed_candidates':attempts,'contact_column_validation':lookahead,
                    'policy':'Obstacle-aware redundant IK only; original source object, TCP grasp point and orientation unchanged.'}
                self.query_start_q=np.asarray(q_arm7).copy()
                return path
            except RuntimeError as exc:attempts.append({'candidate':i,'seed_scale':scale,'error':str(exc)})
        self.pregrasp_redundancy_failures=attempts
        self._prepare_path_query(ignore_names,grip_binary,'pregrasp')
        raise RuntimeError('No collision-free pregrasp/contact column at unchanged TCP target: '+str(attempts[-3:]))

    def _cartesian_vertical(self,q_arm7,target,phase):
        try:
            return super()._cartesian_vertical(q_arm7,target,phase)
        except RuntimeError as exc:
            if 'collision/joint constraints' not in str(exc):raise
            initial=self.kin.fk(q_arm7);goal=np.array(target,copy=True);goal[:3,:3]=initial[:3,:3]
            path=[np.asarray(q_arm7).copy()]
            for fraction in np.linspace(0,1,max(2,int(np.ceil(np.linalg.norm(goal[:3,3]-initial[:3,3])/.002))+1))[1:]:
                pose=initial.copy();pose[:3,3]=initial[:3,3]+fraction*(goal[:3,3]-initial[:3,3])
                path.append(self.kin.ik(pose,path[-1],attempts=1))
            states=self.args.to_device(np.asarray(path))
            valid=self.motion_gen.ik_solver.check_valid(states).detach().cpu().numpy().reshape(-1)
            failures=np.flatnonzero(~valid);detail={'phase':phase,'invalid_waypoints':failures.tolist(),'samples':len(path),'colliders':[]}
            checker=self.motion_gen.world_coll_checker
            enabled=[name for name in self.obstacle_names if not self._ignored(name,self.active_ignore_names)]
            def test(names):
                chosen=set(names)
                for name in self.obstacle_names:checker.enable_obstacle(name,enable=name in chosen)
                return bool(self.motion_gen.ik_solver.check_valid(states).all())
            def locate(names):
                if not names or test(names):return
                if len(names)==1:detail['colliders'].extend(names);return
                mid=len(names)//2;locate(names[:mid]);locate(names[mid:])
            try:
                detail['self_and_limits_valid']=test([])
                if detail['self_and_limits_valid']:locate(enabled)
            finally:
                for name in self.obstacle_names:checker.enable_obstacle(name,enable=name in enabled)
            detail['fcl_failures']=[]
            for i in failures:
                if not self.world.valid(path[i],self.active_ignore_names):
                    detail['fcl_failures'].append({'waypoint':int(i),'measurement':self.world.last_clearance})
            self.cartesian_diagnostics=getattr(self,'cartesian_diagnostics',[])+[detail]
            raise RuntimeError('Cartesian constraints: '+str(detail)) from exc

    def _sync_gripper_and_attachment(self, grip_binary, object_padding_m=0.):
        attachment=self.world.attachment
        self.world.attachment=None
        try: super()._sync_gripper_and_attachment(grip_binary, object_padding_m)
        finally: self.world.attachment=attachment
        if attachment is None: return
        # MotionGen uses a compact *interior* proxy for search guidance. An
        # external box cover wrongly intersects the source support and plant.
        # This proxy is explicitly not a full surface certificate: the source
        # collision-vertex convex hull is checked by FCL at every dense path
        # sample before execution and at each measured physics update.
        from scipy.spatial import ConvexHull
        vertices=self.world.lime_vertices
        if not hasattr(self,'lime_proxy'):
            hull=ConvexHull(vertices);lo,hi=vertices.min(0),vertices.max(0)
            centers=np.array(np.meshgrid(*[np.linspace(a,b,8)[1:-1] for a,b in zip(lo,hi)],indexing='ij')).reshape(3,-1).T
            depth=-(centers@hull.equations[:,:3].T+hull.equations[:,3]).max(1)
            chosen=np.flatnonzero(depth>.003);chosen=chosen[np.argsort(-depth[chosen])[:100]]
            centers=centers[chosen];radii=np.minimum(.012,depth[chosen])
            gap=(np.linalg.norm(vertices[:,None]-centers[None],axis=2)-radii[None]).min(1)
            self.lime_proxy=(centers,radii)
            self.collision_geometry['carried_lime_proxy']={
                'policy':'interior spheres for MotionGen search; full source convex hull checked separately by dense and physics-rate FCL',
                'sphere_count':len(centers),'full_surface_coverage':False,
                'maximum_source_vertex_distance_outside_proxy_m':float(gap.max()),
                'source_support_contact_policy':'Full source hull uses the episode source_support_contact_model (native rest hull bound, exact source surface/XY scope and permanent detach for avoidance trials); this proxy alone is not a surface certificate'}
        centers,radii=self.lime_proxy
        radii=radii.copy()
        if self.active_phase=='lift':
            source_plane=max(hi[2] for name,obj,lo,hi in self.world.objects
                if name.endswith('/counter_main_main_group/top_geometry_right'))
            root_world=self.kin.fk(self.query_start_q)@attachment
            centers_world=centers@root_world[:3,:3].T+root_world[:3,3]
            before=float(np.min(centers_world[:,2]-radii))
            radii=np.minimum(radii,centers_world[:,2]-source_plane-.0005)
            radii=np.where(radii>0,radii,-100.)
            self.collision_geometry['carried_lime_proxy']['lift_support_clip']={
                'surface_z_m':float(source_plane),'original_proxy_min_z_m':before,
                'minimum_proxy_surface_z_m':float(np.min((centers_world[:,2]-radii)[radii>0])),
                'reason':'Search-only interior proxy clipped at the intended source support. Full source convex hull remains checked by dense and measured-physics FCL with the documented episode-specific support bound.'}
        transform=self.T_ee_tcp@attachment
        centers=centers@transform[:3,:3].T+transform[:3,3]
        spheres=np.zeros((100,4),np.float32);spheres[:,3]=-100
        spheres[:len(centers),:3]=centers
        spheres[:len(centers),3]=radii+object_padding_m
        self.motion_gen.attach_spheres_to_robot(sphere_tensor=self.args.to_device(spheres))
