"""Contact-driven valve candidate; no native joint/rigid-state writes.

Every robot trajectory candidate is checked with cuRobo and the source FCL
world. Only the two inner fingertips may contact the finite valve grip, with
aggregate gripper envelopes refined by their original per-part hulls.
"""
from dataclasses import dataclass
from copy import deepcopy
import json
import math
import numpy as np

from kitchen_safety.extensions.replicator.bowl_collision import BowlContactPolicy
from kitchen_safety.extensions.replicator.bowl_target_support import SupportedDestinationWorld
from kitchen_safety.extensions.replicator.task_curobo import PayloadCuroboPlanner
from .water_mechanism import BODY, OUTLET, BASIN_TOP, PIVOT, INITIAL_DEG, STOP_DEG, CLOSE_FEEDBACK_DEG, CLOSE_TARGET_DEG

VALVE_PHASES=frozenset(("valve_pregrasp","valve_approach","valve_grasp","valve_turn",
    "valve_stop_hold","valve_release","valve_retreat","valve_closed_settle"))
CONTACT_PHASES=VALVE_PHASES-{"valve_pregrasp","valve_closed_settle"}
GRIP=BODY+"/Grip"
VALVE_GRASP_PROFILE=dict(horizontal_direction_from_radial_deg=-150.,downward_approach_angle_deg=45.,
    radius_from_pivot_m=.055,retract_along_tcp_x_m=.030,centering_along_tcp_y_m=-.01130083175669125,
    cpu_expected_first_inner_finger_touch_rad=.4125588,
    cpu_pinch_stroke_audit_max_rad=.4225588,
    selection="CPU original-part and flow-volume search; actual allowed one-finger contact or bilateral pinch can actuate the valve")
VALVE_STOP_PROFILE=dict(press_hold_seconds=.5,
    release_target_rate_rad_s=.05,
    flow_off_angle_deg=STOP_DEG,controller_stop_angle_deg=CLOSE_FEEDBACK_DEG,
    controller_target_angle_deg=CLOSE_TARGET_DEG,
    raw_speed_diagnostic_only=True,
    purpose="Brief contact hold then gradual actuator-target release with passive joint damping. Actual q/flow determines shutoff; no velocity success threshold or valve state writes")


def centered_grasp_pose(angle_deg,height=0.):
    """Planning-only top pinch that keeps the hand away from the stream."""
    theta=math.radians(180.+angle_deg)
    radial=np.array([math.cos(theta),math.sin(theta),0.])
    tangent=np.array([-math.sin(theta),math.cos(theta),0.])
    phi,beta=math.radians(-150.),math.radians(45.)
    horizontal=math.cos(phi)*radial+math.sin(phi)*tangent
    up=np.array([0.,0.,1.])
    x=math.cos(beta)*horizontal-math.sin(beta)*up
    y=math.sin(beta)*horizontal+math.cos(beta)*up
    target=np.eye(4)
    target[:3,:3]=np.column_stack((x,y,np.cross(x,y)))
    target[:3,3]=np.array(PIVOT)+.055*radial-(.03+height)*x-.01130083175669125*y
    return target


@dataclass(frozen=True)
class WaterContactPolicy(BowlContactPolicy):
    def validate_phase(self,phase):
        if phase not in VALVE_PHASES:
            super().validate_phase(phase)

    def cuda_candidate_masks(self,phase,attached):
        if phase in VALVE_PHASES:
            return (GRIP,) if phase in CONTACT_PHASES else ()
        return super().cuda_candidate_masks(phase,attached)

    def report(self):
        result=super().report()
        result["valve_contact_policy"]=dict(grip=GRIP, phases=sorted(CONTACT_PHASES),
            fingertip_parts=["left_inner_finger","right_inner_finger"],
            other_parts_and_hub_retained=True,physical_collision_filter_changes=False,
            candidate_cuda_mask_scope="Grip only; independent FCL checks retain all other gripper/Grip pairs")
        return result


class WaterWorld(SupportedDestinationWorld):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.contact_policy=WaterContactPolicy()
        self.flowing=True
        bottom=BASIN_TOP+.003
        self.flow_shape=self.fcl.CollisionObject(self.fcl.Cylinder(.0035,OUTLET[2]-bottom),
            self.fcl.Transform(np.eye(3),[OUTLET[0],OUTLET[1],(OUTLET[2]+bottom)/2]))
        self.last_flow_clearance=None
        # The generic world uses world-axis AABBs for primitives. A 90 mm
        # rotated grip's AABB fills the empty space between the fingers and
        # falsely hits knuckles. Keep its actual native analytic shape in FCL.
        # CUDA gets the same source dimensions as an oriented surface mesh:
        # exact cuboids and a circumscribed cylinder (never an inscribed one).
        # The native collider and all planning padding remain unchanged.
        from pxr import UsdGeom
        from scipy.spatial import ConvexHull
        stage=args[0]
        cache=UsdGeom.XformCache()
        self.valve_geometry_refinement=[]
        for index,(name,obj,lo,hi) in enumerate(self.objects):
            if "/water_valve/" not in name:
                continue
            prim=stage.GetPrimAtPath(name)
            matrix=np.asarray(cache.GetLocalToWorldTransform(prim)).T
            scale=np.linalg.norm(matrix[:3,:3],axis=0)
            rotation=matrix[:3,:3]/scale
            if not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-7):
                raise ValueError("Valve analytic collider has shear")
            if prim.IsA(UsdGeom.Cube):
                dimensions=scale*float(UsdGeom.Cube(prim).GetSizeAttr().Get())
                shape=self.fcl.Box(*dimensions)
                vertices=np.array([[x,y,z] for x in (-.5,.5) for y in (-.5,.5)
                    for z in (-.5,.5)])*dimensions
                excess=0.
            elif prim.IsA(UsdGeom.Cylinder):
                cylinder=UsdGeom.Cylinder(prim)
                if cylinder.GetAxisAttr().Get()!="Z" or abs(scale[0]-scale[1])>1e-7:
                    raise ValueError("Unexpected valve cylinder scaling/axis")
                radius=float(cylinder.GetRadiusAttr().Get())*scale[0]
                height=float(cylinder.GetHeightAttr().Get())*scale[2]
                shape=self.fcl.Cylinder(radius,height)
                facets=96
                outer_radius=radius/math.cos(math.pi/facets)
                angles=np.arange(facets)*2*math.pi/facets
                vertices=np.array([[outer_radius*math.cos(a),outer_radius*math.sin(a),z]
                    for z in (-height/2,height/2) for a in angles])
                excess=outer_radius-radius
            else:
                raise ValueError("Unexpected valve collider type")
            exact=self.fcl.CollisionObject(shape,self.fcl.Transform(rotation,matrix[:3,3]))
            hull=ConvexHull(vertices)
            faces=hull.simplices.copy()
            normals=np.cross(vertices[faces[:,1]]-vertices[faces[:,0]],vertices[faces[:,2]]-vertices[faces[:,0]])
            reverse=np.einsum('ij,ij->i',normals,hull.equations[:,:3])<0
            faces[reverse]=faces[reverse][:,[0,2,1]]
            vertices=vertices@rotation.T+matrix[:3,3]
            original_bounds=[lo.tolist(),hi.tolist()]
            lo,hi=vertices.min(0),vertices.max(0)
            mesh=(vertices.copy(),np.asarray(faces,dtype=np.int32))
            self.meshes[name]=mesh
            self.objects[index]=(name,exact,lo,hi)
            self.low[index],self.high[index]=lo,hi
            self._object_reference[index]=(rotation.copy(),matrix[:3,3].copy(),lo.copy(),hi.copy(),mesh)
            self.valve_geometry_refinement.append(dict(path=name,source_schema=prim.GetTypeName(),
                representation="Exact source analytic FCL collider; source-oriented CUDA mesh",
                cuda_mesh_vertices=len(vertices),cuda_mesh_triangles=len(faces),
                cylinder_radial_conservative_excess_m=excess,
                original_world_aabb_m=original_bounds,
                no_collision_masks_or_padding_changed=True))

    def flow_clearance(self,q,gripper_q=None):
        """Source-part geometry versus the declared visible stream volume.

        This risk volume is not a physical collision object or liquid model.
        All true robot/gripper parts remain; duplicate whole-hand proxies are
        omitted because they fill empty space between the actual fingers.
        """
        if not self.flowing:
            return dict(active=False,valid=True,flowing=False)
        minimum=dict(distance_m=float("inf"),robot_part=None,intersecting=False)
        count=0
        for name,obj,*_ in self.shapes(q,gripper_q=gripper_q):
            if not (name.startswith("gripper/") or name.startswith("robot_hull/")) or name=="robot_hull/whole_gripper":
                continue
            count+=1
            distance=float(self.fcl.distance(obj,self.flow_shape,self.fcl.DistanceRequest(),self.fcl.DistanceResult()))
            hit=bool(self.fcl.collide(obj,self.flow_shape,self.fcl.CollisionRequest(),self.fcl.CollisionResult()))
            if not np.isfinite(distance):raise RuntimeError("Nonfinite flow-volume clearance")
            if hit:distance=min(0.,distance)
            if distance<minimum["distance_m"]:
                minimum=dict(distance_m=distance,robot_part=name,intersecting=hit or distance<0.)
        expected=len(self.gripper_models)+sum(body!=self.gripper_root for body,*_ in self.robot_hulls)
        if count!=expected:raise RuntimeError("Incomplete source robot part coverage for flow volume")
        return dict(minimum,active=True,flowing=True,parts_checked=count,required_m=.002,
            valid=not minimum["intersecting"] and minimum["distance_m"]>=.002,
            obstacle="water_visual/FlowVisual/Stream",scope="Configured 3.5mm-radius visual cylinder versus source-derived robot part hulls; not fluid or electrical simulation")

    def valid(self,q,ignore=(),**kwargs):
        self.last_flow_clearance=self.flow_clearance(q,kwargs.get("gripper_q"))
        if not self.last_flow_clearance["valid"]:
            self.last_clearance=self.last_flow_clearance.copy()
            self.last_collision=(self.last_clearance["robot_part"],self.last_clearance["obstacle"])
            return False
        return super().valid(q,ignore,**kwargs)

    def _pairs(self,*args,**kwargs):
        pairs=super()._pairs(*args,**kwargs)
        if self.phase not in CONTACT_PHASES:
            return pairs
        # CPU exact-pair inventory (water_audit) confirms the coarse wrist
        # capsule fills empty space near the grip. Retain all native part
        # hulls, including palm/base/knuckles; refine this aggregate only.
        permitted={"gripper/left_inner_finger","gripper/right_inner_finger",
                   "finger","gripper","robot_hull/whole_gripper"}
        return [p for p in pairs if not (p[2]==GRIP and p[1] in permitted)]


class ValvePlanner(PayloadCuroboPlanner):
    """Reuse one context, updating every observed rigid world shape each query."""
    @staticmethod
    def _ignored(name,keys):
        # The base adapter's basename matcher does not recognize absolute USD
        # paths. A valve mask must select this exact Grip, never every fixture.
        return any(name==key or name.startswith(key.rstrip('/')+'/') if key.startswith('/')
            else key in name.split('/') for key in keys)

    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self._source_fcl_transforms={}
        for name,obj,*_ in self.world.objects:
            value=np.eye(4)
            value[:3,:3],value[:3,3]=obj.getRotation(),obj.getTranslation()
            self._source_fcl_transforms[name]=value
        self.observed_world_refreshes=[]

    def _sync_gripper_and_attachment(self,grip_binary,object_padding_m=0.):
        # PhysX mimic joints can differ slightly under load. Preserve all six
        # measured values rather than regenerating five from finger_joint.
        super()._sync_gripper_and_attachment(grip_binary,object_padding_m)
        values=np.asarray(self.world.gripper_joint_positions,dtype=float)
        if values.shape!=(6,) or not np.isfinite(values).all():
            raise ValueError("Valve planning requires all six finite native finger joints")
        locked=dict(self.robot_dict["kinematics"]["lock_joints"])
        locked.update(zip(self.world.GRIPPER_JOINT_NAMES,values.tolist()))
        self.motion_gen.update_locked_joints(locked,deepcopy(self.robot_dict))
        self.actual_finger_joint_values=values.tolist()

    def refresh_observed_world(self):
        from curobo.types.math import Pose
        names={name for name,*_ in self.world.objects}
        if names!=set(self.obstacle_names):
            raise RuntimeError("Valve planning world topology changed")
        checker=self.motion_gen.world_coll_checker
        updates=[]
        for name,obj,low,high in self.world.objects:
            if name in self.world.meshes:
                current=np.eye(4)
                current[:3,:3],current[:3,3]=obj.getRotation(),obj.getTranslation()
                delta=current@np.linalg.inv(self._source_fcl_transforms[name])
                pose=self._pose_list(self.T_B_W@delta@self.kin.base)
                checker.update_mesh_pose(name=name,w_obj_pose=Pose.from_list(pose,self.args))
                cpu=checker.world_model.get_obstacle(name)
                if cpu is not None:cpu.pose=pose
            else:
                # Preserve the same conservative primitive box representation
                # and exact current FCL bounds used by a fresh constructor.
                pose,dims=self._planning_box(low,high,0.)
                checker.update_obb_pose(name=name,w_obj_pose=Pose.from_list(pose,self.args))
                checker.update_obb_dims(self.args.to_device(dims),name=name)
                cpu=checker.world_model.get_obstacle(name)
                if cpu is not None:cpu.pose,cpu.dims=pose,dims
            updates.append(name)
        self._context_reused=True
        self._query_signature=None
        self._last_grip=None
        self._reset_all_seeds()
        self.observed_world_refreshes.append(dict(colliders=len(updates),all_world_shapes_updated=True,
            source="Current observed body poses in independent FCL world; rigid topology unchanged"))


def close_valve(env,robot,kin,world,water,recorder,arm_ids,step,hold,sync_world,output,seed=42):
    """Top pinch and incremental physical circular turn; reject absent contact.

    The desired angle defines TCP targets only. Actual valve q is always the
    passive articulation readback. No measured state can be replaced by a plan.
    """
    from kitchen_safety.extensions.replicator.task_motion import pose_matrix
    from scipy.spatial.transform import Rotation
    plans=[]
    planner=None
    def save():
        def convert(v):
            if hasattr(v,"tolist"):return v.tolist()
            raise TypeError(type(v).__name__)
        (output/"valve_plans.json").write_text(json.dumps(plans,indent=2,default=convert)+"\n")

    def move(target,gripper,phase,cartesian=False):
        nonlocal planner
        recorder.phase=phase
        current=recorder.rows[-1]
        sync_world(current)
        if planner is None:
            planner=ValvePlanner(world,kin,dt=env.step_dt,seed=seed,max_joint_speed=.25)
        else:
            planner.refresh_observed_world()
        entry=dict(phase=phase,start_physics_step=current["physics_step"],target_tcp=target,status="requested",
            valve_geometry_refinement=world.valve_geometry_refinement,
            grasp_profile=VALVE_GRASP_PROFILE,
            context_reused=bool(planner.observed_world_refreshes))
        plans.append(entry)
        try:
            if cartesian:
                # Small measured-state Cartesian segments keep the grip on the
                # physical arc. Each candidate retains full cuRobo/FCL checks.
                start=pose_matrix(current["tcp_pose_w"])
                delta=Rotation.from_matrix(target[:3,:3]@start[:3,:3].T).as_rotvec()
                count=max(3,int(math.ceil(np.linalg.norm(target[:3,3]-start[:3,3])/.002))+1,
                          int(math.ceil(np.linalg.norm(delta)/math.radians(2)))+1)
                q=current["q"][arm_ids].copy()
                positions=[q.copy()]
                for f in np.linspace(0.,1.,count)[1:]:
                    pose=start.copy()
                    pose[:3,3]=start[:3,3]+f*(target[:3,3]-start[:3,3])
                    pose[:3,:3]=Rotation.from_rotvec(delta*f).as_matrix()@start[:3,:3]
                    goal=kin.ik(pose,q,attempts=2)
                    n=max(1,int(math.ceil(np.max(np.abs(goal-q))/(.20*env.step_dt))))
                    positions.extend(q+(goal-q)*t for t in np.linspace(0.,1.,n+1)[1:])
                    q=goal
                positions=np.asarray(positions)
                entry["method"]="measured_cartesian_ik_candidate"
            else:
                plan=planner.plan_pose(current["q"][arm_ids],target,grip_binary=gripper,phase=phase)
                positions=np.asarray(plan["positions"])
                entry["metadata"]=plan["metadata"]
            np.savez_compressed(output/f"valve_plan_{len(plans):03d}_{phase}.npz",positions=positions,target=target)
            validation=planner.validate_path(positions,grip_binary=gripper,phase=phase,resolution=.02)
            entry["validation"]=validation
            if not validation["valid"]:
                raise RuntimeError("Valve candidate rejected by cuRobo/source FCL")
            entry["status"]="validated"
            for command in positions[1:]:
                step(command,gripper,phase)
            hold(positions[-1],gripper,phase,.2)
            entry["status"]="executed"
            return positions[-1].copy()
        except Exception as error:
            entry.update(status="failed",error=repr(error))
            raise
        finally:
            save()

    if abs(water.rows[-1]["angle_deg"]-INITIAL_DEG)>1.:
        raise RuntimeError("Valve moved before contact; matched neutral stability is required")
    angle=water.rows[-1]["angle_deg"]
    q=move(centered_grasp_pose(angle,.10),0.,"valve_pregrasp")
    q=move(centered_grasp_pose(angle),0.,"valve_approach",True)
    region=water.config["contact_region_lever_local_m"]
    grip_term=env.action_manager.get_term("gripper_action")
    grip_term.configure_valve(True,rate_rad_s=.10)
    held=False
    for _ in range(int(math.ceil(8./env.step_dt))):
        step(q,1.,"valve_grasp")
        observed=water.contacts.summarize(*region,phase="valve_grasp",last_steps=10)
        if observed["each_step_any_loaded_near_in_region"]:
            grip_term.hold_verified_grasp()
            held=True
            break
    if held:
        hold(q,1.,"valve_grasp",.2)
    contact=water.contacts.summarize(*region,phase="valve_grasp",last_steps=10)
    contact.update(rate_limited_target_rad_s=.10,verified_target_held=held,
        held_processed_target_rad=water.rows[-1]["gripper_processed_target_rad"],
        contact_mode="bilateral_pinch" if contact["both_fingers_loaded"] and contact["both_fingers_near"] else "single_finger_contact_push")
    (output/"valve_initial_pinch.json").write_text(json.dumps(contact,indent=2)+"\n")
    if not (held and contact["each_step_any_loaded_near_in_region"]):
        raise RuntimeError("No actual allowed loaded near valve contact")
    # A measured push reference specifies TCP candidates only. It neither
    # assumes a rigid attachment nor creates a physical constraint.
    tcp=pose_matrix(recorder.rows[-1]["tcp_pose_w"])
    lever=pose_matrix(water.rows[-1]["lever_pose"])
    push_reference=np.linalg.inv(lever)@tcp
    stalled=0
    for index in range(40):
        observed=water.rows[-1]["angle_deg"]
        if observed<=CLOSE_FEEDBACK_DEG:
            break
        goal=max(CLOSE_TARGET_DEG,observed-2.)
        measured_lever=pose_matrix(water.rows[-1]["lever_pose"])
        desired=measured_lever.copy()
        desired[:3,:3]=Rotation.from_euler("z",goal-observed,degrees=True).as_matrix()@measured_lever[:3,:3]
        q=move(desired@push_reference,1.,"valve_turn",True)
        after=water.rows[-1]["angle_deg"]
        support=water.contacts.summarize(*region,phase="valve_turn",last_steps=10)
        plans[-1]["post_segment_contacts"]=support
        save()
        if after>CLOSE_FEEDBACK_DEG and not support["each_step_any_loaded_near_in_region"]:
            raise RuntimeError("Valve turn lost allowed loaded near contact")
        stalled=stalled+1 if observed-after<.15 else 0
        if stalled>=3:
            raise RuntimeError("Valve contact turn stalled; no joint setter fallback")
        if after>observed+1.:
            raise RuntimeError("Observed passive valve moved opposite to closing direction")
    # The user judges actual shutoff, not numerical velocity. Keep dq as a
    # diagnostic; only genuine reopening may block the bowl continuation.
    hold(q,1.,'valve_stop_hold',VALVE_STOP_PROFILE['press_hold_seconds'])
    latest=water.rows[-1]
    preparation=dict(profile=VALVE_STOP_PROFILE,readback=dict(flowing=latest['flowing'],
        angle_deg=latest['angle_deg'],speed_deg_s_diagnostic=latest['speed_deg_s'],
        physics_step=latest['physics_step']),
        starting_processed_target_rad=water.rows[-1]['gripper_processed_target_rad'],
        source='Every native physical substep; held arm actuator command only')
    (output/'valve_release_preparation.json').write_text(json.dumps(preparation,indent=2)+'\n')
    if latest['flowing']:
        raise RuntimeError('Valve is still flowing before release')
    grip_term.configure_release(rate_rad_s=VALVE_STOP_PROFILE['release_target_rate_rad_s'])
    opening_duration=float(np.max(water.rows[-1]['gripper_processed_target_rad']))/VALVE_STOP_PROFILE['release_target_rate_rad_s']+.3
    hold(q,0.,"valve_release",opening_duration)
    retreat=pose_matrix(recorder.rows[-1]["tcp_pose_w"])
    retreat[:3,3]-=.10*retreat[:3,0]
    q=move(retreat,0.,"valve_retreat",True)
    hold(q,0.,"valve_closed_settle",1.2)
    grip_term.configure_valve(False)
    result=water.closure()
    result.update(initial_pinch_contacts=contact,plans_file="valve_plans.json",segments=len(plans),
        contact_mode=contact["contact_mode"],push_reference_tcp_in_lever=push_reference.tolist(),
        release_preparation=preparation,
        bilateral_contact_required=False,physics_joint_state_writes=False,physical_weld=False,external_torque=False)
    return result
