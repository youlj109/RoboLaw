"""Native water environment additions and observation-only flow coupling.

Imports of Isaac are deliberately lazy. The one configured reset initializes
the passive articulation. All later changes must come from physics contact.
"""
from __future__ import annotations
import json
import math
from pathlib import Path
import numpy as np
from pxr import UsdGeom

from .water_mechanism import ROOT, CONFIG, BODY, JOINT_NAME, STOP_DEG, INITIAL_DEG, PIVOT, inspect
from kitchen_safety.extensions.replicator.task_contacts import ObjectGraspContacts, _array


def add_assets(envdef, config_path=CONFIG):
    from isaaclab.assets import ArticulationCfg
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_arena.terms.events import reset_articulation_pose_and_joints
    report=inspect(config_path)
    if not report["passed"]:
        raise ValueError("Passive water assets failed CPU inspection")
    config=report["config"]
    valve=Object(name="water_valve", usd_path=str(ROOT/config["valve_asset"]),
        object_type=ObjectType.ARTICULATION, initial_pose=Pose(PIVOT,(0.,0.,1.,0.)),
        asset_cfg_addon={"init_state": ArticulationCfg.InitialStateCfg(
            joint_pos={JOINT_NAME: math.radians(INITIAL_DEG)}, joint_vel={".*":0.})})
    # No actuators: target-writing actions cannot actuate this articulation.
    if valve.object_cfg.actuators:
        raise RuntimeError("The valve must remain passive")
    # Arena Object's default event restores only the root. Use its dedicated
    # articulation reset event so q=55 deg and dq=0 are truly initialized.
    # This event runs only in env.reset, never during robot task execution.
    valve._pose_event_cfg.func=reset_articulation_pose_and_joints
    flow=Object(name="water_visual", usd_path=str(ROOT/config["visual_asset"]),
        object_type=ObjectType.BASE, initial_pose=Pose.identity())
    layout=json.loads((ROOT/config["accepted_layout"]["path"]).read_text())
    hazard=layout["obstacle"]
    yaw=math.radians(hazard.get("yaw_deg",0.))
    obstacle=Object(name="water_crossbar", usd_path=str(ROOT/hazard["usd_path"]),
        object_type=ObjectType.BASE,
        initial_pose=Pose(tuple(hazard["position"]),(0.,0.,math.sin(yaw/2),math.cos(yaw/2))))
    envdef.scene.add_assets([valve,flow,obstacle])
    envdef.water_execution_config=config
    return envdef


class ValveContacts(ObjectGraspContacts):
    def __init__(self, physics_view, physics_dt):
        self.physics_dt=float(physics_dt)
        self.object_name="water_valve_lever"
        self.env_origin=np.zeros(3)
        self.views={finger: physics_view.create_rigid_contact_view(
            [f"/World/envs/env_0/Robot/Gripper/Robotiq_2F_85/{finger}"],
            filter_patterns=[[BODY]], max_contact_data_count=4096) for finger in self.fingers}
        self.records=[]

    def summarize(self, region_min, region_max, *, phase, last_steps=10,
                  minimum_force_n=.01, max_separation_m=.00025):
        """Report actual allowed near compression, retaining bilateral detail.

        This native articulation reports compressive normal forces with a
        negative sign (full_01 retains the signed scalar, normal and gap).
        The force magnitude determines loading, without rewriting raw data.
        Zero-force contact candidates cannot establish a physical pinch.
        """
        low,high=np.asarray(region_min,float),np.asarray(region_max,float)
        if low.shape!=(3,) or high.shape!=(3,) or not np.isfinite([low,high]).all() or np.any(high<=low):
            raise ValueError("Invalid valve contact region")
        if last_steps<1 or minimum_force_n<=0 or max_separation_m<0:
            raise ValueError("Invalid valve contact thresholds")
        rows=[row for row in self.records if row["phase"]==phase][-last_steps:]
        details=[]
        for row in rows:
            fingers={}
            for finger in self.fingers:
                contact=row["fingers"][finger]
                points=np.asarray(contact["points_object_local_m"]).reshape(-1,3)
                signed=np.asarray(contact["normal_forces_n"])
                gap=np.asarray(contact["separations_m"])
                if len(points)!=contact["count"] or len(signed)!=len(points) or len(gap)!=len(points):
                    raise RuntimeError("Valve raw contact array lengths differ")
                inside=np.all((points>=low)&(points<=high),axis=1)
                loaded=np.abs(signed)>minimum_force_n
                near=gap<=max_separation_m
                valid=bool(loaded.any() and inside[loaded].all() and near[loaded].all())
                fingers[finger]=dict(loaded_in_region=bool(loaded.any() and inside[loaded].all()),
                    near_in_region=valid,loaded_contact_count=int(loaded.sum()),
                    any_loaded_near_in_region=bool(np.any(loaded&near&inside)),
                    loaded_outside_region=bool(np.any(loaded&~inside)),
                    maximum_force_magnitude_n=float(np.max(np.abs(signed),initial=0.)),
                    signed_loaded_forces_n=signed[loaded].tolist(),
                    loaded_separations_m=gap[loaded].tolist())
            details.append(dict(physics_step=row["physics_step"],fingers=fingers,
                bilateral_loaded=all(f["loaded_in_region"] for f in fingers.values()),
                bilateral_near=all(f["near_in_region"] for f in fingers.values()),
                any_allowed_loaded_near=any(f["any_loaded_near_in_region"] for f in fingers.values())
                    and not any(f["loaded_outside_region"] for f in fingers.values())))
        enough=len(rows)==last_steps
        return dict(object_name=self.object_name,phase=phase,
            scope="Native signed contacts preserved; at least one allowed finger may push. Bilateral fields are diagnostic, not required for valve actuation",
            force_sign_interpretation="Native articulation pair reports negative compressive scalar; loading uses its magnitude, raw signed values unchanged",
            region_object_local_m=[low.tolist(),high.tolist()],requested_steps=last_steps,
            sampled_steps=len(rows),minimum_force_n=minimum_force_n,max_separation_m=max_separation_m,
            steps=details,both_fingers_loaded=enough and all(r["bilateral_loaded"] for r in details),
            both_fingers_near=enough and all(r["bilateral_near"] for r in details),
            each_step_any_loaded_near_in_region=enough and all(r["any_allowed_loaded_near"] for r in details))


class WaterRuntime:
    """Read actual articulation, update visual state, retain raw contact proof."""
    def __init__(self, env, physics_view, config_path=CONFIG):
        self.e=env
        self.config=json.loads(Path(config_path).read_text())
        self.valve=env.scene["water_valve"]
        self.fixed_base=bool(self.valve.is_fixed_base)
        self.joint_id=self.valve.joint_names.index(JOINT_NAME)
        self.lever_id=self.valve.body_names.index("Lever")
        self.base_id=self.valve.body_names.index("Base")
        self.contacts=ValveContacts(physics_view,env.physics_dt)
        self.flow=env.sim.stage.GetPrimAtPath("/World/envs/env_0/water_visual/FlowVisual")
        if not self.flow:
            raise RuntimeError("Native flow visual was not spawned")
        self.rows=[]
        self.first_contact_step=None
        self.first_movement_step=None
        from kitchen_safety.extensions.replicator.bowl_collision import BowlCollisionWorld
        self.robot=env.scene["robot"]
        self.finger_ids=[self.robot.joint_names.index(n) for n in BowlCollisionWorld.GRIPPER_JOINT_NAMES]
        self.native_parameters={}
        for name in ("joint_stiffness","joint_damping","joint_armature","joint_friction_coeff",
                     "joint_dynamic_friction_coeff","joint_viscous_friction_coeff"):
            try:
                self.native_parameters[name]=float(_array(getattr(self.valve.data,name))[0,self.joint_id])
            except (AttributeError,TypeError,IndexError) as error:
                self.native_parameters[name]=dict(unavailable=repr(error))
        self.passive_damping_readback=None
        if 'passive_damping' in self.config:
            expected=self.config['passive_damping']['expected_native_viscous_friction_nm_s_per_radian']
            observed=self.native_parameters.get('joint_viscous_friction_coeff')
            matches=isinstance(observed,(float,int)) and bool(np.isclose(observed,expected,rtol=1e-4,atol=1e-8))
            armature=self.native_parameters.get('joint_armature')
            retained=self.config['passive_damping']['retained_armature_kg_m2']
            self.passive_damping_readback=dict(expected_nm_s_per_radian=expected,
                observed_nm_s_per_radian=observed,matches=matches,
                expected_armature_kg_m2=retained,observed_armature_kg_m2=armature,
                armature_matches=isinstance(armature,(float,int)) and bool(np.isclose(armature,retained,rtol=1e-4,atol=1e-9)),
                joint_drive_stiffness=self.native_parameters.get('joint_stiffness'),
                joint_drive_damping=self.native_parameters.get('joint_damping'),
                passive_only=self.native_parameters.get('joint_stiffness')==0. and self.native_parameters.get('joint_damping')==0.,
                source='Native articulation parameter readback; distinct from an active drive damping target')

    def sample(self, phase, action_index):
        step=int(self.e.sim.get_physics_step_count())
        if self.rows and self.rows[-1]["physics_step"]==step:
            return self.rows[-1]
        q=float(_array(self.valve.data.joint_pos)[0,self.joint_id])
        dq=float(_array(self.valve.data.joint_vel)[0,self.joint_id])
        pose=np.r_[_array(self.valve.data.body_pos_w)[0,self.lever_id],
                   _array(self.valve.data.body_quat_w)[0,self.lever_id]]
        base_pose=np.r_[_array(self.valve.data.body_pos_w)[0,self.base_id],
                       _array(self.valve.data.body_quat_w)[0,self.base_id]]
        if not np.isfinite(np.r_[q,dq,pose,base_pose]).all():
            # Preserve exactly which readback failed; do not repair or fill it.
            def printable(value):
                a=np.asarray(value)
                return dict(value=a.astype(str).tolist(),finite=np.isfinite(a).tolist())
            self.invalid_readback=dict(physics_step=step,q_rad=printable(q),dq_rad_s=printable(dq),
                lever_pose=printable(pose),base_pose=printable(base_pose))
            raise RuntimeError("Nonfinite native water articulation state: "+json.dumps(self.invalid_readback))
        deg=math.degrees(q)
        fraction=max(0.,min(1.,(deg-STOP_DEG)/(INITIAL_DEG-STOP_DEG)))
        # Rendering attributes only. No joint target/state, rigid pose, force,
        # torque, or physics constraint is authored in this callback.
        self.flow.GetAttribute("water:flowing").Set(fraction>0.)
        self.flow.GetAttribute("water:measuredAngleDeg").Set(deg)
        self.flow.GetAttribute("water:openFraction").Set(fraction)
        self.flow.GetAttribute("water:physicsStep").Set(step)
        UsdGeom.Imageable(self.flow).GetVisibilityAttr().Set(
            UsdGeom.Tokens.inherited if fraction>0. else UsdGeom.Tokens.invisible)
        contacts=self.contacts.sample(phase,action_index,pose,physics_step=step)
        force=max((abs(v) for f in contacts["fingers"].values() for v in f["normal_forces_n"]),default=0.)
        if force>.01 and self.first_contact_step is None:
            self.first_contact_step=step
        if abs(deg-INITIAL_DEG)>.5 and self.first_movement_step is None:
            self.first_movement_step=step
        row=dict(physics_step=step,phase=phase,action_index=int(action_index),
            angle_deg=deg,speed_deg_s=math.degrees(dq),lever_pose=pose.tolist(),
            base_pose=base_pose.tolist(),pivot_position_error_m=float(np.linalg.norm(pose[:3]-PIVOT)),
            flowing=fraction>0.,open_fraction=fraction,max_finger_normal_force_n=force)
        grip=self.e.action_manager.get_term("gripper_action")
        row.update(gripper_processed_target_rad=_array(grip.processed_actions)[0].tolist(),
            gripper_binary_requested_target_rad=_array(getattr(grip,"_binary_requested_target",grip.processed_actions))[0].tolist(),
            gripper_target_rate_limit_enabled=bool(getattr(grip,"valve_control_enabled",False)),
            gripper_verified_target_held=bool(getattr(grip,"grasp_target_held",False)),
            gripper_observed_joint_positions_rad=_array(self.robot.data.joint_pos)[0,self.finger_ids].tolist())
        self.rows.append(row)
        return row

    def closure(self):
        needed=int(math.ceil(1./self.e.physics_dt))
        tail=self.rows[-needed:]
        stable=len(tail)==needed and all(not r['flowing'] for r in tail)
        released_still=stable and all(r['phase']=='valve_closed_settle' for r in tail)
        causal=self.first_contact_step is not None and self.first_movement_step is not None and self.first_contact_step<self.first_movement_step
        simultaneous=self.first_contact_step is not None and self.first_contact_step==self.first_movement_step
        return dict(closed_stable=stable,contact_precedes_movement=causal,strict_prior_contact=causal,
            released_bowl_readiness=released_still,
            released_bowl_readiness_thresholds=dict(flowing=False,shut_angle_max_deg=STOP_DEG,
                after_robot_retreat=True,window_seconds=1.,raw_speed_diagnostic_only=True),
            maximum_abs_speed_deg_s_diagnostic=max((abs(r['speed_deg_s']) for r in tail),default=None),
            same_substep_unresolved=simultaneous,
            first_contact_force_magnitude_threshold_n=.01,meaningful_movement_threshold_from_initial_deg=.5,
            first_contact_step=self.first_contact_step,first_movement_step=self.first_movement_step,
            sampled_steps=len(tail),required_steps=needed,flowing=self.rows[-1]["flowing"] if self.rows else True,
            passed=bool(stable and causal and released_still),scope="Native q-driven shutoff after retreat and strict prior contact. Speed is diagnostic only; every later bowl substep rejects actual reopening.")

    def neutral_check(self):
        rows=[r for r in self.rows if r["phase"] in ("reset","warmup","initial_settle")]
        if not rows:
            return dict(passed=False,checks={"samples_present":False})
        angles=np.array([r["angle_deg"] for r in rows])
        speeds=np.array([r["speed_deg_s"] for r in rows])
        maximum_pivot_error=max(r["pivot_position_error_m"] for r in rows)
        checks=dict(native_fixed_base=self.fixed_base,initial_q_near_55_deg=abs(angles[0]-INITIAL_DEG)<=.5,
            neutral_drift_below_half_degree=float(np.max(np.abs(angles-angles[0])))<=.5,
            fixed_pivot_within_2mm=maximum_pivot_error<=.002,
            flowing_throughout=all(r["flowing"] for r in rows),
            no_loaded_robot_contacts=all(r["max_finger_normal_force_n"]<=.01 for r in rows))
        if self.passive_damping_readback is not None:
            checks['passive_damping_matches_native']=self.passive_damping_readback['matches']
            checks['retained_armature_matches_native']=self.passive_damping_readback['armature_matches']
            checks['no_active_joint_drive']=self.passive_damping_readback['passive_only']
        return dict(passed=all(checks.values()),checks=checks,samples=len(rows),
            duration_s=(rows[-1]["physics_step"]-rows[0]["physics_step"])*self.e.physics_dt,
            initial_angle_deg=float(angles[0]),angle_range_deg=[float(angles.min()),float(angles.max())],
            max_abs_speed_deg_s=float(np.max(np.abs(speeds))),raw_speed_diagnostic_only=True,
            maximum_pivot_error_m=maximum_pivot_error)

    def report(self):
        values=np.asarray([r["angle_deg"] for r in self.rows])
        return dict(joint_name=JOINT_NAME,joint_path="/World/envs/env_0/water_valve/ValveJoint",
            joint_index=self.joint_id,lever_body=BODY,actuators=[],state_setters_used_after_reset=False,
            native_parameters=self.native_parameters,native_fixed_base=self.fixed_base,
            passive_damping_readback=self.passive_damping_readback,
            reset_event="reset_articulation_pose_and_joints",
            angle_range_deg=[float(values.min()),float(values.max())] if len(values) else [],
            closure=self.closure(),neutral=self.neutral_check(),
            invalid_readback=getattr(self,"invalid_readback",None),rows=self.rows,raw_contacts=self.contacts.records)
