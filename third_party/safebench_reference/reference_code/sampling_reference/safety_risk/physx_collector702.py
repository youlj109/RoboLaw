"""PhysX runtime data collector for Sim_Raw_GT fields.

Collects per-step physics data from Isaac Sim's PhysX engine:
- Joint torques, link poses, link velocities
- Contact forces, collision pairs, penetration depth
- Object-link distances

Usage:
    In plan_with_render() loop, after world.step():
        collector.collect_step(task, step_id)
    After episode ends:
        collector.save(output_dir)
"""

from __future__ import annotations

import json
from copy import deepcopy
import logging
import math
import os
import pickle
import re
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
from .contact_vectors702 import contact_vec3_702
from .pick_target_binding import resolve_pick_target_id, robot_distance_root
from .multi_env_risk636 import scoped_stage636, contact_report_belongs636

RISK_ISOLATION636_HOOKS = True

logger = logging.getLogger(__name__)


class PhysXDataCollector:
    """Collects per-step PhysX runtime data for safety risk evaluation.

    Call collect_step() after each world.step() in the simulation loop.
    Call save() after the episode to persist data alongside LMDB.
    """

    def __init__(self):
        self._data: Dict[str, List] = {
            # S-ROBOT-001/002: live articulation state for every robot DOF.
            # Rows are flattened across robots in deterministic task order;
            # joint_state_metadata records the robot/local-index mapping.
            "joint_position_q_gt": [],
            "joint_velocity_dq_gt": [],
            "joint_state_metadata": None,
            # S-ROBOT-004: joint torques
            "joint_torque_gt": [],
            # S-ROBOT-005: all link poses (all links, per step)
            "link_pose_gt": [],
            # S-ROBOT-006: link velocities
            "link_velocity_gt": [],
            # S-HUM-001: per-step world poses for every human-surrogate body
            "human_body_pose_gt": [],
            # S-COLL-001: collision pairs
            "collision_pair_gt": [],
            "contact_coverage_per_frame": [],
            # S-COLL-002: collision locations
            "collision_location_gt": [],
            # S-COLL-003: penetration depths
            "penetration_depth_gt": [],
            # S-COLL-004: contact forces (per contact view, per step)
            "contact_force_gt": [],
            # S-COLL-005: contact impulses (derived from forces)
            "contact_impulse_gt": [],
            # S-COLL-006: contact duration tracking
            "contact_events": [],  # raw events for duration computation
            # S-GRASP-001: gripper-object contact force
            "gripper_object_contact_force_gt": [],
            # S-DIST-001..003: collider-surface human distances
            "robot_human_distance_matrix_gt": [],
            "ee_human_distance_gt": [],
            "object_human_distance_gt": [],
            "target_object_linear_velocity_gt": [],
            # S-DIST-004: object-env distances
            "object_env_distance_gt": [],
            # S-DIST-005: link-env distances
            "link_env_distance_gt": [],
            "link_target_mesh_aabb_distance_gt": [],
            "link_target_projected_clearance_gt": [],
            "pt_environment_projected_clearance_gt": [],
            # S-DIST-006: self distances
            "self_distance_gt": [],
            # S-PLAN-001: planned trajectory (captured per plan event)
            "planned_trajectory": [],
            # S-PLAN-003: safety gate status per step
            "safety_gate_status": [],
            # S-PLAN-004: controller command-plan activity per step
            "low_level_command_sent": [],
            # Online intervention supervision.  The observed risk level is
            # evaluated after the current physics step; the action fields
            # describe the constraint applied to that step's command.
            "intervention_level": [],
            "intervention_action": [],
            "intervention_command_scale": [],
            "intervention_reasons": [],
            "intervention_observation": [],
            "control_training_eligible": [],
            "perception_training_eligible": [],
            "intervention_level_before_action": [],
            "intervention_stop_state": [],
            "intervention_transition": [],
            "intervention_policy_config": None,
            # Safety gate / stop tracking (episode-level results)
            "stop_success": None,
            "stop_margin_s": None,
            "t_stop_s": None,
            # Metadata
            "step_ids": [],
            "object_world_pose_per_frame": [],
        }
        self._link_paths: Dict[str, List[str]] = {}  # robot_name -> [link_paths]
        self._joint_layout: Optional[Dict[str, Any]] = None
        self._initialized = False
        self._contact_state: Dict[tuple, dict] = {}  # (bodyA, bodyB) -> active contact state
        self._physics_dt_s = 0.033
        self._pending_contact_reports = []
        self._contact_report_sub = None
        self._data["contact_coverage_status"] = {
            "configured": False,
            "successful_steps": 0,
            "failed_steps": 0,
            "last_error": None,
        }
        self._data["contact_report_status"] = {
            "subscribed": False,
            "steps_with_reports": 0,
            "total_report_headers": 0,
        }
        self._data["runtime_physics"] = {
            "physics_dt_s": None,
            "gravity_direction": None,
            "gravity_magnitude_mps2": None,
        }
        self._data["distance_coverage_status"] = {
            "method": "collider_world_aabb_surface_clearance",
            "robot_body_count": 0,
            "human_body_count": 0,
            "object_count": 0,
            "environment_body_count": 0,
            "mano_proxy_body_count": 0,
            "robot_human_complete": False,
            "robot_visual_rigid_bodies_without_collider": [],
        }
        try:
            from omni.physx import get_physx_simulation_interface
            from omni.physx.scripts import physicsUtils

            def _decode_path(encoded):
                try:
                    return str(physicsUtils.PhysicsSchemaTools.intToSdfPath(encoded))
                except Exception:
                    return ""

            def _on_contact_report(contact_headers, contact_data):
                pending = []
                for header in contact_headers:
                    record = {
                        "actor0": _decode_path(header.actor0),
                        "actor1": _decode_path(header.actor1),
                        "collider0": _decode_path(header.collider0),
                        "collider1": _decode_path(header.collider1),
                    }
                    if not contact_report_belongs636(self, record):
                        continue
                    start = int(header.contact_data_offset)
                    count = int(header.num_contact_data)
                    if count <= 0:
                        continue
                    trace_context = getattr(self, "_contact_trace_context661", None)
                    if trace_context is not None:
                        from .contact_trace661 import native_contact_trace661
                        self._data.setdefault("contact_source_trace661", []).append(
                            native_contact_trace661(trace_context, record, count))
                    impulses = []
                    positions = []
                    separations = []
                    normals = []
                    for index in range(start, start + count):
                        datum = contact_data[index]
                        impulses.append(contact_vec3_702(datum.impulse))
                        positions.append(contact_vec3_702(datum.position))
                        separations.append(float(datum.separation))
                        normals.append(contact_vec3_702(datum.normal))
                    record.update({
                        "impulses_ns": impulses,
                        "positions_m": positions,
                        "separations_m": separations,
                        "normals": normals,
                        "source": "PhysX contact-report callback",
                    })
                    pending.append(record)
                self._pending_contact_reports.extend(pending)

            self._contact_report_sub = (
                get_physx_simulation_interface().subscribe_contact_report_events(
                    _on_contact_report
                )
            )
            self._data["contact_report_status"]["subscribed"] = True
        except Exception as exc:
            logger.warning("PhysX contact-report subscription unavailable: %s", exc)

        # Safety gate runtime state
        self._stop_triggered = False
        self._stop_step = None
        self._detect_step = None
        self._detect_distance_m = None
        self._prev_ee_obstacle_dist_m = None
        self._safety_stop_active = False
        self._safety_gate_enabled = True
        self._intervention_policy = None
        self._prev_intervention_distance_m = None
        self._target_object_names: List[str] = []
        self._human_surrogate_names: List[str] = []
        self._entity_roles_explicit = False
        self._online_iso = None

    def begin_recording(self):
        """Separate setup callbacks from the first recorded physics interval.

        Keep all warmup evidence under metadata, without attributing the sum of
        several setup steps to frame zero's single-step force/impulse.
        """
        if self._data["step_ids"]:
            raise RuntimeError("cannot restart recording after physics samples")
        if "pre_recording_contact_reports" in self._data:
            raise RuntimeError("recording boundary already established")
        self._data["pre_recording_contact_reports"] = list(self._pending_contact_reports)
        self._pending_contact_reports.clear()

    def configure_safety_context(self, config: Optional[dict]) -> None:
        """Provide task entity roles without coupling the collector to one task class."""
        entities = (config or {}).get("entities", {}) or {}
        self._entity_roles_explicit = "human_surrogates" in entities
        self._closed_world_human_inventory = (config or {}).get("closed_world_human_inventory") is True
        self._target_objects_declared = "target_objects" in entities
        self._target_object_names = [str(v) for v in entities.get("target_objects", []) or []]
        self._human_surrogate_names = [str(v) for v in entities.get("human_surrogates", []) or []]

    def configure_online_iso(self, config):
        if config and config.get("enabled", False):
            from safety_risk.online_iso import OnlineISOEvaluator, ObservationISOEvaluator
            cls = ObservationISOEvaluator if config.get("decision_hz") is not None else OnlineISOEvaluator
            self._online_iso = cls(dict(config))
            # Bind every explicitly declared environmental protected object.
            # Task targets retain their existing grasp-aware distance channel.
            pairs = self._online_iso.evaluator.profile.get("pt_hs_inspired", {}).get("pairs", [])
            target_ids = set(getattr(self, "_target_object_names", []) or [])
            declared_target = resolve_pick_target_id(
                getattr(self, "_target_object_names", []) if getattr(self, "_target_objects_declared", False) else None,
                config.get("runtime_metadata") or {})
            if declared_target is not None:
                target_ids.add(declared_target)
            self._data["pt_environment_distance_pairs"] = [
                {"actor": p["actor"], "object_id": p["object_id"]} for p in pairs
                if isinstance(p.get("object_id"), str) and p["object_id"]
                and p["object_id"] not in target_ids
                and p.get("actor") in {"robot", "held_object"}]
            self._observation_cadence = isinstance(self._online_iso, ObservationISOEvaluator)
            if self._observation_cadence:
                self._physics_dt_s = float(config["physics_dt_s"])
                self._data["runtime_physics"]["physics_dt_s"] = self._physics_dt_s
                self._policy_cameras = tuple(config.get("policy_cameras", ["franka_head", "franka_hand"]))
                if not self._policy_cameras:
                    raise ValueError("observation cadence requires named policy cameras")
                self._previous_policy_views = {}
                self._observation_state = None
                self._prev_intervention_source_step = None

    def configure_intervention_policy(self, config: Optional[dict]) -> None:
        """Configure the per-frame L0-L3 command intervention policy."""
        from safety_risk.intervention_factory import make_intervention_policy

        self._intervention_policy = make_intervention_policy(config or {})
        from safety_risk.controlled_ir_hold import ControlledInputHold
        self._controlled_input_hold = ControlledInputHold()
        from safety_risk.gripper_force_response import GripperForceResponse
        self._gripper_force_response = GripperForceResponse(
            (config or {}).get('gripper_force_response', {}))
        if (self._gripper_force_response.enabled
                and (config or {}).get('control_baseline') in {'fixed_distance', 'ordinary_evidence_ready'}):
            raise ValueError('gripper force response requires attachment evaluator control')
        online = getattr(self, "_online_iso", None)
        if online is not None and online.mode == "control" and not self._intervention_policy.enabled:
            raise ValueError("online ISO control requires intervention_policy.enabled=true")
        self._data["intervention_policy_config"] = (
            self._intervention_policy.config_snapshot()
        )
        self._data['intervention_policy_config']['gripper_force_response'] = dict(
            self._gripper_force_response.config)
        if online is not None and online.mode == "control":
            self._data["intervention_policy_config"]["source"] = (
                {"fixed_distance": "fixed_distance_baseline",
                 "ordinary_evidence_ready": "evidence_ready_ordinary_baseline"}.get(
                    (config or {}).get("control_baseline"), "attachment_four_category_evaluator"))
        self._stop_identification = None
        diagnostic = (config or {}).get("stop_identification", {})
        if diagnostic.get("enabled", False):
            if not self._intervention_policy.enabled or online is None or online.mode != "shadow":
                raise ValueError("stop identification requires enabled intervention and online ISO shadow")
            if not self._entity_roles_explicit or self._human_surrogate_names:
                raise ValueError("initial stop identification requires an explicitly human-free scene")
            from safety_risk.stop_identification import StopIdentificationTrigger
            self._stop_identification = StopIdentificationTrigger(dict(diagnostic))
            self._data["intervention_policy_config"]["stop_identification"] = dict(diagnostic)
            self._data["intervention_policy_config"]["source"] = "stop_identification_experiment"
        logger.info(
            "[intervention] configured: enabled=%s, L1 scale=%.3f, "
            "L2 brake=%d steps",
            self._intervention_policy.enabled,
            self._intervention_policy.l1_speed_scale,
            self._intervention_policy.l2_brake_steps,
        )

    def update_intervention_stop_state(self, task, decision=None, observation=None) -> Optional[Dict[str, Any]]:
        """Feed measured articulation speed back into stop verification."""
        if self._intervention_policy is None:
            return None
        from safety_risk.joint_motion import joint_speed_summary
        velocities_by_robot = {}
        for name, robot in getattr(task, "robots", {}).items():
            try:
                row = np.asarray(robot.get_joints_state().velocities, dtype=float).reshape(-1)
                velocities_by_robot[str(name)] = row.tolist()
            except Exception:
                velocities_by_robot[str(name)] = None
        motion = joint_speed_summary(velocities_by_robot, self.get_joint_layout(task).get("channels", []))
        state = self._intervention_policy.update_stop_confirmation(
            motion["max_angular_speed_rad_s"], motion["max_linear_speed_m_s"])
        state["joint_motion_evidence"] = motion
        state["stop_episode_id"] = self._intervention_policy._stop_episode_id
        state["stop_timeout_ever"] = self._intervention_policy._stop_timeout_ever
        self._data["intervention_stop_state"].append(dict(state))
        # Termination must use measurements AFTER this applied hold, not a
        # previously confirmed frame that may have been disturbed meanwhile.
        if decision and decision.get("level_before_action") in {"L2", "L3"}:
            policy = self._intervention_policy
            is_l3 = decision["level_before_action"] == "L3"
            elapsed = policy._l3_action_step if is_l3 else policy._hold_step
            limit = policy.l3_hold_steps if is_l3 else policy.l2_hold_steps
            decision["stop_verified"] = state["stop_confirmed"]
            decision["stop_timeout"] = elapsed >= limit and not state["stop_confirmed"]
            window = policy.update_post_stop_window(state["stop_confirmed"], decision["stop_timeout"])
            window["stop_episode_id"] = policy._stop_episode_id
            window["stop_timeout_ever"] = policy._stop_timeout_ever
            window["current_stop_timeout_seen"] = policy._stop_timeout_seen
            state.update(window)
            self._data["intervention_stop_state"][-1].update(window)
            decision.update(window)
            force_unresolved = decision.get('gripper_force_response', {}).get('unresolved') is True
            recoverable_l2 = (not is_l3 and policy.l2_recovery_mode == "risk_following"
                              and not force_unresolved)
            decision["terminate_after_step"] = (
                window["stop_observation_termination_ready"] and not recoverable_l2)
            if force_unresolved:
                decision['termination_reason'] = 'gripper_force_limit_unresolved'
                decision['control_training_eligible'] = False
                state['gripper_force_limit_unresolved'] = True
                self._data['intervention_stop_state'][-1]['gripper_force_limit_unresolved'] = True
            from safety_risk.intervention_policy import finalize_stop_evidence
            finalize_stop_evidence(decision, state)
        from safety_risk.intervention_policy import intervention_transition
        transition = intervention_transition(decision, observation, state)
        self._data["intervention_transition"].append(transition)
        if self._data["control_training_eligible"]:
            self._data["control_training_eligible"][-1] = transition["control_training_eligible"]
        return state

    def get_intervention_decision(self, step_id: int) -> Optional[Dict[str, Any]]:
        """Return the action constraint derived from the previous frame."""
        if self._intervention_policy is None:
            return None
        diagnostic = getattr(self, "_stop_identification", None)
        status = None
        if diagnostic is not None:
            states = self._data.get("intervention_stop_state", [])
            motion = states[-1].get("joint_motion_evidence", {}) if states else {}
            load = None
            if diagnostic.held_object is not None:
                from safety_risk.stop_identification import held_load_evidence
                cfg = diagnostic.held_object
                poses = self._data.get("object_world_pose_per_frame", [])
                contacts = self._data.get("collision_pair_gt", [])
                steps = self._data.get("step_ids", [])
                if steps and len(steps) == len(poses) == len(contacts) and steps[-1] == step_id - 1:
                    load = held_load_evidence(cfg["name"], cfg["robot_name"],
                        diagnostic.reference_positions.get(cfg["name"]),
                        poses[-1].get(cfg["name"]), contacts[-1], steps[-1])
            status = diagnostic.evaluate(step_id, motion, load)
            self._intervention_policy.observe_stop_identification(status["stop_requested"])
            self._data["stop_identification"] = diagnostic.export()
        decision = self._intervention_policy.command_decision(step_id)
        if self._intervention_policy.enabled:
            decision = self._controlled_input_hold.constrain(decision, self._intervention_policy)
        force_response = getattr(self, '_gripper_force_response', None)
        if force_response is not None:
            decision['gripper_force_response'] = force_response.snapshot()
            decision['gripper_force_hold'] = decision['gripper_force_response'].get('hold_task', False)
        if getattr(self, "_observation_cadence", False):
            decision["risk_observation_frame_id"] = len(self._online_iso.records)-1
            decision["risk_classification_fresh"] = step_id % self._online_iso.stride == 0
        if status is not None:
            decision["stop_identification"] = status
        return decision

    def capture_initial_pre_action_state(self, task, sim_time_s, world_step_index):
        """Read settled geometry/joints without consuming contact callbacks or physics."""
        if not getattr(self, "_observation_cadence", False):
            return None
        if self._data.get("step_ids") or "initial_pre_action_state" in self._data:
            raise RuntimeError("initial state must be captured once before recording")
        live_data = self._data
        distance_fields = ("robot_human_distance_matrix_gt", "ee_human_distance_gt",
            "object_human_distance_gt", "target_object_linear_velocity_gt", "object_env_distance_gt", "link_env_distance_gt", "link_target_mesh_aabb_distance_gt", "link_target_projected_clearance_gt", "pt_environment_projected_clearance_gt", "self_distance_gt")
        joint_fields = ("joint_position_q_gt", "joint_velocity_dq_gt", "joint_torque_gt")
        sample = {key: [] for key in distance_fields + joint_fields}
        sample.update(step_ids=[0], runtime_physics=deepcopy(live_data["runtime_physics"]),
            pt_environment_distance_pairs=deepcopy(live_data.get("pt_environment_distance_pairs", [])),
            joint_state_metadata=None, collision_pair_gt=[None], contact_force_gt=[None],
            contact_impulse_gt=[None], contact_coverage_per_frame=[False], low_level_command_sent=[None])
        errors = []
        try:
            self._data = sample
            for name in ("_collect_joint_states", "_collect_joint_torques", "_collect_distances"):
                try:
                    getattr(self, name)(task)
                except Exception as exc:
                    errors.append(name + ": " + type(exc).__name__ + ": " + str(exc))
            for key in distance_fields + joint_fields:
                if len(sample[key]) != 1:
                    errors.append("missing_or_invalid_initial_sample:" + key)
                    sample[key] = [None]
        finally:
            self._data = live_data
        record = {"schema_version":"initial-pre-action-state-1",
            "state_phase":"before_first_recorded_action", "sim_time_s":float(sim_time_s),
            "world_step_index":int(world_step_index), "sample":deepcopy(sample),
            "runtime_metadata":deepcopy(self._online_iso.metadata), "collection_errors":errors,
            "projection_axis":"one separate sample; step_ids=[0] is not a recorded physical execution",
            "limitations":["contact callbacks remain in separate pre-recording evidence; initial contact unknown",
                           "initial derivatives and unexecuted command properties remain unknown",
                           "collider world AABB geometry remains an experimental proxy"]}
        live_data["initial_pre_action_state"] = record
        return deepcopy(record)

    def prepare_observation_decision(self, step_id, observations, sim_time_s):
        """Classify once per observation slot before submitting any action."""
        if not getattr(self, "_observation_cadence", False):
            return None
        online = self._online_iso
        if step_id % online.stride:
            return None
        event = observations.get('_controlled_ir_evidence') if isinstance(observations, dict) else None
        if event is not None:
            event = deepcopy(event)
            event['ir_scope'] = 'controlled_injections_only'
            previous = self._data.get('controlled_ir_observations', [])
            prior = previous[-1] if previous else None
            # Only execution already observed while this same anomaly was
            # active counts; no future action is invented at onset.
            decisions = self._data.get('applied_intervention_decisions', [])
            start = prior['frame'] if prior else step_id
            relevant = decisions[start:step_id]
            from safety_risk.controlled_ir_hold import continued_nominal_execution
            if event.get('kind') == 'blackout':
                event['continued_execution'] = continued_nominal_execution(event, relevant, start, step_id)
                event['continued_execution_source'] = 'completed_physics_nominal_arm_execution_v1'
            else:
                # Other existing injection kinds retain their legacy semantics;
                # 550 must not silently mark their execution stopped.
                same = prior is not None and prior.get('active') and event.get('active')
                event['continued_execution'] = (any(d.get('trajectory_advanced') is True for d in relevant)
                                                if same and relevant else False)
                event['continued_execution_source'] = 'legacy_trajectory_advanced_not_actual_execution'
            event['planner_input_limitation'] = 'GT planner does not consume camera pixels; no unsafe-command claim'
            self._data.setdefault('controlled_ir_observations', []).append(event)
            online.controlled_ir_evidence = event
        # Only fresh decision observations can establish validity or recovery.
        self._controlled_input_hold.observe(event)
        from safety_risk.camera_timing import recording_timing
        cameras = observations.get("cameras", {}) if isinstance(observations, dict) else {}
        views = {}
        for name in self._policy_cameras:
            camera = cameras.get(name) or {}
            row = recording_timing(camera.get("capture_metadata"), step_id, sim_time_s, "before_action")
            identity = (row.get("acquisition_epoch"), row.get("acquisition_sequence"))
            age = row.get("age_at_read_s")
            valid = (row.get("capture_clock_consistent") is True and type(identity[1]) is int
                     and isinstance(age, (int, float)) and math.isfinite(age)
                     and -1e-6 <= age <= 1./online.decision_hz+1e-6)
            status = ("new" if valid and identity != self._previous_policy_views.get(name)
                      else "stale" if valid else "missing_or_invalid")
            views[name] = {"status":status, "acquisition_epoch":identity[0],
                           "acquisition_sequence":identity[1], "age_s":age,
                           "capture_sim_time_s":row.get("capture_sim_time_s")}
            self._previous_policy_views[name] = identity
        self._observation_state = self.observe_intervention_after_step(
            step_id-1, None, _pre_action_context=(step_id, views, sim_time_s))
        return deepcopy(self._observation_state)

    def set_stop_identification_reference(self, positions):
        diagnostic = getattr(self, "_stop_identification", None)
        if diagnostic is not None:
            diagnostic.set_reference(positions)

    def prime_intervention_state(self, task) -> Optional[Dict[str, Any]]:
        """Classify the initial geometry before any policy command is sent.

        The temporary distance sample initializes the stateful intervention
        policy but is removed from the report arrays, so all persisted raw-GT
        streams remain aligned with actual physics frames.
        """
        if self._intervention_policy is None or getattr(self, "_observation_cadence", False):
            return None
        distance_fields = (
            "robot_human_distance_matrix_gt",
            "ee_human_distance_gt",
            "object_human_distance_gt",
            "target_object_linear_velocity_gt",
            "object_env_distance_gt",
            "link_env_distance_gt",
            "link_target_mesh_aabb_distance_gt",
            "link_target_projected_clearance_gt",
            "pt_environment_projected_clearance_gt",
            "self_distance_gt",
        )
        intervention_fields = (
            "intervention_level",
            "intervention_action",
            "intervention_command_scale",
            "intervention_reasons",
            "intervention_observation",
            "control_training_eligible",
            "perception_training_eligible",
            "intervention_level_before_action",
        )
        fields = distance_fields + intervention_fields
        lengths = {name: len(self._data[name]) for name in fields}
        try:
            self._collect_distances(task)
            return self.observe_intervention_after_step(-1, None)
        finally:
            for name, length in lengths.items():
                del self._data[name][length:]

    @staticmethod
    def _finite_values(value: Any):
        if isinstance(value, dict):
            for child in value.values():
                yield from PhysXDataCollector._finite_values(child)
        elif isinstance(value, (list, tuple)):
            for child in value:
                yield from PhysXDataCollector._finite_values(child)
        elif isinstance(value, (int, float)) and not isinstance(value, bool):
            number = float(value)
            if math.isfinite(number):
                yield number

    @staticmethod
    def _iter_contact_records(value: Any):
        if isinstance(value, dict):
            if "bodyA" in value or "bodyB" in value:
                yield value
            else:
                for child in value.values():
                    yield from PhysXDataCollector._iter_contact_records(child)
        elif isinstance(value, (list, tuple)):
            for child in value:
                yield from PhysXDataCollector._iter_contact_records(child)

    @staticmethod
    def _record_is_human_contact(record: Dict[str, Any]) -> bool:
        text = f"{record.get('bodyA', '')} {record.get('bodyB', '')}".lower()
        return any(token in text for token in ("obstacle", "human", "mano"))

    def observe_intervention_after_step(
        self,
        step_id: int,
        applied_decision: Optional[Dict[str, Any]],
        _pre_action_context=None,
    ) -> Optional[Dict[str, Any]]:
        """Evaluate current HS signals and align them with the applied action."""
        if self._intervention_policy is None:
            return None

        sparse = getattr(self, "_observation_cadence", False)
        if sparse and _pre_action_context is None:
            if self._observation_state is None:
                raise RuntimeError("missing pre-action observation decision")
            observation = deepcopy(self._observation_state)
            observation["execution_reference"] = self._online_iso.record_execution(step_id)
            # Full-rate event evidence does not run another risk classification.
            records = self._data.get("collision_pair_gt", [])
            observation["interval_human_contact_seen"] = bool(records and any(
                self._record_is_human_contact(r) for r in self._iter_contact_records(records[-1])))
            return self._append_intervention_observation(observation, applied_decision)

        distance_data = self._data
        if sparse and step_id == -1 and _pre_action_context is not None:
            initial = self._data.get("initial_pre_action_state", {})
            if initial:
                if abs(float(initial["sim_time_s"]) - float(_pre_action_context[2])) > 1e-7:
                    raise ValueError("initial proxy distance time differs from decision time")
                distance_data = initial["sample"]

        distance_values = []
        if distance_data["robot_human_distance_matrix_gt"]:
            distance_values.extend(self._finite_values(
                distance_data["robot_human_distance_matrix_gt"][-1]
            ))
        if distance_data["ee_human_distance_gt"]:
            distance_values.extend(self._finite_values(
                distance_data["ee_human_distance_gt"][-1]
            ))
        if distance_data["object_human_distance_gt"]:
            distance_values.extend(self._finite_values(
                distance_data["object_human_distance_gt"][-1]
            ))
        distance_m = min(distance_values) if distance_values else None

        records = []
        if self._data["collision_pair_gt"]:
            records.extend(self._iter_contact_records(
                self._data["collision_pair_gt"][-1]
            ))
        if self._data["contact_force_gt"]:
            records.extend(self._iter_contact_records(
                self._data["contact_force_gt"][-1]
            ))
        human_records = [r for r in records if self._record_is_human_contact(r)]
        human_contact = bool(human_records)
        forces = []
        for record in human_records:
            value = record.get("force_n", record.get("force_magnitude_n"))
            try:
                value = abs(float(value))
            except (TypeError, ValueError):
                continue
            if math.isfinite(value):
                forces.append(value)
        contact_force_n = max(forces) if forces else (0.0 if human_contact else None)
        if human_contact:
            distance_m = 0.0

        elapsed_s = float(self._physics_dt_s)
        if sparse:
            previous = getattr(self, "_prev_intervention_source_step", None)
            if previous is not None:
                elapsed_s *= step_id - previous
            self._prev_intervention_source_step = step_id
        closing_speed_mps = 0.0
        if distance_m is not None and self._prev_intervention_distance_m is not None:
            closing_speed_mps = max(
                0.0,
                (self._prev_intervention_distance_m - distance_m)
                / max(elapsed_s, 1.0e-6),
            )
        if distance_m is not None:
            self._prev_intervention_distance_m = distance_m
        ttc_s = (
            distance_m / closing_speed_mps
            if distance_m is not None and closing_speed_mps > 0.01
            else None
        )

        online = getattr(self, "_online_iso", None)
        diagnostic = getattr(self, "_stop_identification", None)
        if diagnostic is not None:
            observation = self._intervention_policy.observe_stop_identification(diagnostic.trigger_frame is not None)
        elif online is not None and online.mode == "control":
            observation = {}
        else:
            observation = self._intervention_policy.observe(
                distance_m=distance_m, closing_speed_mps=closing_speed_mps,
                ttc_s=ttc_s, human_contact=human_contact, contact_force_n=contact_force_n,
                human_present=(bool(self._human_surrogate_names)
                    if getattr(self, "_entity_roles_explicit", False) else None))
        online = getattr(self, "_online_iso", None)
        if online is not None:
            if step_id >= 0 or sparse:
                report = (online.evaluate_before_action(self._data, *_pre_action_context)
                          if sparse else online.evaluate(self._data))
                from safety_risk.pt_runtime_evidence import record_clearance
                record_clearance(self._data, online.records[-1])
                if online.mode == "control":
                    force_response = getattr(self, '_gripper_force_response', None)
                    if force_response is not None and force_response.enabled:
                        context = force_response.observe(online.records[-1])
                        observation = self._intervention_policy.observe_evaluation(report, control_context=context)
                    else:
                        observation = self._intervention_policy.observe_evaluation(report)
                elif report.get("risk_levels", {}).get("overall") == "L3":
                    # Shadow labels never change the baseline action, but a
                    # known hazardous frame is not a positive safe demonstration.
                    observation["attachment_hazard_seen"] = True
            elif online.mode == "control":
                observation = self._intervention_policy.observe_evaluation({})

        if sparse:
            observation["risk_observation_frame_id"] = len(online.records)-1
            observation["risk_source_physics_step_index"] = step_id if step_id >= 0 else None
            observation["risk_classification_phase"] = "before_action_observation_slot"
            return observation
        return self._append_intervention_observation(observation, applied_decision)

    def _append_intervention_observation(self, observation, applied_decision):
        decision = applied_decision or {
            "level_before_action": "L0",
            "action": "none",
            "command_scale": 1.0,
            "control_training_eligible": True,
            "perception_training_eligible": True,
            "reasons": [],
        }
        observed_level = observation["level"]
        self._data.setdefault('applied_intervention_decisions', []).append(deepcopy(decision))
        control_eligible = (
            bool(decision.get("control_training_eligible", True))
            and observed_level != "L3"
            and not observation.get("attachment_hazard_seen", False)
            and not observation.get("interval_human_contact_seen", False)
        )
        self._data["intervention_level"].append(observed_level)
        self._data["intervention_action"].append(decision.get("action", "none"))
        self._data["intervention_command_scale"].append(
            float(decision.get("command_scale", 1.0))
        )
        self._data["intervention_reasons"].append(list(observation.get("reasons", [])))
        self._data["intervention_observation"].append(dict(observation))
        self._data["control_training_eligible"].append(control_eligible)
        self._data["perception_training_eligible"].append(True)
        self._data["intervention_level_before_action"].append(
            decision.get("level_before_action", "L0")
        )
        return observation

    def collect_step(self, task, step_id: int, actuator_feedforward=None) -> None:
        """Collect PhysX data for one simulation step.

        Call this AFTER world.step() returns.

        Parameters
        ----------
        task : BananaBaseTask
            The task object with robots, objects, and contact views.
        step_id : int
            Current step index.
        """
        self._data["step_ids"].append(step_id)
        # Separate applicability evidence from frozen numeric distance features.
        try:
            import omni.usd
            from safety_risk.human_presence import scene_inventory
            inventory = scene_inventory(
                task, omni.usd.get_context().get_stage(),
                self._human_surrogate_names, self._entity_roles_explicit, step_id,
                getattr(self, "_closed_world_human_inventory", False))
        except Exception as exc:
            inventory = {"frame_index": step_id, "complete": False,
                         "error": type(exc).__name__ + ": " + str(exc)}
        self._data.setdefault("human_inventory_per_frame", []).append(inventory)
        self._data.setdefault("actuator_feedforward", []).append(
            {"frame_index":step_id,"command":deepcopy(actuator_feedforward)})

        # Capture object state at exactly the same post-physics sampling point
        # as joints and contacts, rather than taking pre-action LMDB poses.
        from safety_risk.object_sampling import sample_object
        poses = {str(name): sample_object(obj)
                 for name, obj in getattr(task, "objects", {}).items()}
        self._data["object_world_pose_per_frame"].append(poses)

        try:
            self._collect_joint_states(task)
        except Exception as e:
            logger.warning("Failed to collect joint state: %s", e)
            self._data["joint_position_q_gt"].append(None)
            self._data["joint_velocity_dq_gt"].append(None)

        try:
            self._collect_joint_torques(task)
        except Exception as e:
            self._data["joint_torque_gt"].append(None)

        try:
            self._collect_all_link_poses(task)
        except Exception as e:
            self._data["link_pose_gt"].append(None)

        try:
            self._collect_human_body_poses(task)
        except Exception as e:
            logger.warning("Failed to collect human body poses: %s", e)
            self._data["human_body_pose_gt"].append(None)

        successes_before = self._data["contact_coverage_status"]["successful_steps"]
        try:
            self._collect_contact_data(task, step_id)
        except Exception as e:
            self._data["collision_pair_gt"].append(None)
            self._data["contact_force_gt"].append(None)
            self._data["contact_impulse_gt"].append(None)

        self._data["contact_coverage_per_frame"].append(
            self._data["contact_coverage_status"]["successful_steps"] == successes_before + 1)

        try:
            self._collect_distances(task)
        except Exception as e:
            logger.warning("Failed to collect collider surface distances: %s", e)
            self._data["robot_human_distance_matrix_gt"].append(None)
            self._data["ee_human_distance_gt"].append(None)
            self._data["object_human_distance_gt"].append(None)
            self._data["target_object_linear_velocity_gt"].append(None)
            self._data["object_env_distance_gt"].append(None)
            self._data["link_env_distance_gt"].append(None)
            self._data["link_target_mesh_aabb_distance_gt"].append(None)
            self._data["link_target_projected_clearance_gt"].append(None)
            self._data["pt_environment_projected_clearance_gt"].append(None)
            self._data["self_distance_gt"].append(None)

        # Planner data (from controllers)
        try:
            self._collect_planner_step(task, step_id)
        except Exception:
            self._data["safety_gate_status"].append(None)
            self._data["low_level_command_sent"].append(None)

        # Causal fault evidence for the next observation, never an episode-wide
        # outcome copied backwards into earlier risk snapshots.
        if getattr(self, "_online_iso", None) is not None:
            from safety_risk.live_fault_monitor import LiveFaultMonitor
            from safety_risk.online_iso import current_snapshot
            if not hasattr(self, "_live_fault_monitor"):
                self._live_fault_monitor = LiveFaultMonitor()
            try:
                snapshot = current_snapshot(self._data, self._online_iso.metadata)
                rs = snapshot['features']['rs']
                jm = self._data.get('joint_state_metadata', {})
                indices = jm.get('joint_limit_metric_indices', [])
                dq = self._data['joint_velocity_dq_gt'][-1]
                arm = [dq[i] for i in indices] if indices and isinstance(dq, list) else None
                fault = self._live_fault_monitor.observe(dt=self._physics_dt_s,
                    torque_ratio=rs['joint_torque_ratio_gt'], joint_margin=rs['joint_limit_margin_gt_rad'],
                    pairs=self._data['collision_pair_gt'][-1],
                    contact_covered=self._data['contact_coverage_per_frame'][-1], arm_velocity=arm)
            except Exception as exc:
                self._live_fault_monitor.coverage_complete = False
                fault = {'motion_after_fault_gt': None, 'error': str(exc)}
            self._data.setdefault('motion_after_fault_gt', []).append(fault['motion_after_fault_gt'])
            self._data.setdefault('fault_monitor_evidence', []).append(fault)
            from safety_risk.live_drop_monitor import LiveDropMonitor, bound_contact_grasp_evidence
            from safety_risk.iso_input import _kind
            if not hasattr(self, '_live_drop_monitors'):
                self._live_drop_monitors = {}
            outcomes = self._data.setdefault('live_outcome_gt', {'drop_event_gt': {}, 'drop_height_gt': {}})
            evidence = {}
            for target in getattr(self, '_target_object_names', []):
                monitor = self._live_drop_monitors.setdefault(target, LiveDropMonitor(
                    self._online_iso.metadata.get('drop_history_starts_before_first_grasp', False)))
                arm = 'right' if 'right' in target.lower() else 'left'
                channels = self._data.get('joint_state_metadata', {}).get('channels', [])
                fingers = [c['compact_index'] for c in channels
                           if (c['role'] == arm+'_gripper' or
                               (arm=='left' and c['dof_name']=='panda_finger_joint2'
                                and any(x['robot_name']==c['robot_name'] and x['role']=='left_gripper'
                                        and x['dof_name']=='panda_finger_joint1' for x in channels)))
                           and c['position_unit'] == 'm'
                           and c.get('joint_type_verified')]
                q = self._data['joint_position_q_gt'][-1]
                width = sum(q[i] for i in fingers) if len(fingers)==2 and isinstance(q,list) else None
                max_width = self._online_iso.metadata.get('physics_config', {}).get('gripper_max_width_m_by_arm', {}).get(arm)
                translation = (poses.get(target) or {}).get('translation')
                z = translation[2] if translation is not None else None
                pairs = self._data['collision_pair_gt'][-1]
                robot_contact = environment_contact = False
                for pair in pairs if isinstance(pairs,list) else []:
                    a,b = str(pair.get('bodyA','')),str(pair.get('bodyB',''))
                    other = b if a.endswith('/'+target) else a if b.endswith('/'+target) else None
                    if other is not None:
                        robot_contact |= _kind(other)=='robot'
                        environment_contact |= _kind(other)=='environment'
                # Angular or single-active-joint grippers do not expose a
                # verified metric width. Use the existing exact opposing-body
                # contact binding instead; never treat radians as metres.
                # Keep Franka's established two-linear-finger route unchanged.
                linear_width_valid=(isinstance(width,(int,float)) and not isinstance(width,bool)
                    and np.isfinite(width) and isinstance(max_width,(int,float))
                    and not isinstance(max_width,bool) and np.isfinite(max_width) and max_width>0)
                grasp_evidence=None if linear_width_valid else bound_contact_grasp_evidence(
                    self._online_iso.metadata,pairs,target,self._data['contact_coverage_per_frame'][-1])
                value = monitor.observe(z=z, contact=robot_contact, width=width, max_width=max_width,
                    environment_contact=environment_contact, covered=self._data['contact_coverage_per_frame'][-1],
                    grasp_evidence=grasp_evidence)
                outcomes['drop_event_gt'].setdefault(target, []).append(value['drop_flag_gt'])
                outcomes['drop_height_gt'].setdefault(target, []).append(value['h_drop_gt_m'])
                evidence[target] = value
            self._data.setdefault('drop_monitor_evidence', []).append(evidence)

    # ── Safety Gate (stop_success / stop_margin) ───────────────────────────

    # Default values (overridable via configure_safety_gate)
    SAFETY_DISTANCE_M = 0.30
    STOP_VELOCITY_THRESHOLD = 0.01
    STOP_VERIFY_STEPS = 5

    def configure_safety_gate(self, config: dict) -> None:
        """Configure safety gate from YAML config.

        Parameters
        ----------
        config : dict
            safety_gate section from task YAML, e.g.:
                enabled: true
                distance_threshold_m: 0.30
                stop_verify_steps: 5
        """
        if not config:
            return
        self._safety_gate_enabled = config.get("enabled", True)
        if "distance_threshold_m" in config:
            self.SAFETY_DISTANCE_M = config["distance_threshold_m"]
        if "stop_verify_steps" in config:
            self.STOP_VERIFY_STEPS = config["stop_verify_steps"]
        logger.info(
            f"[safety_gate] configured: enabled={self._safety_gate_enabled}, "
            f"threshold={self.SAFETY_DISTANCE_M}m, verify_steps={self.STOP_VERIFY_STEPS}"
        )

    def check_safety_gate(self, task, step_id: int, dt: float = 0.033) -> bool:
        """Check EE-to-obstacle distance and manage protective stop.

        Call AFTER collect_step(). Returns True if the safety gate is actively
        blocking actions (robot should hold position).

        Parameters
        ----------
        task : BananaBaseTask
            The task with robots and objects.
        step_id : int
            Current simulation step.
        dt : float
            Simulation timestep in seconds.

        Returns
        -------
        bool
            True if the safety stop is active and actions should be suppressed.
        """
        if not self._safety_gate_enabled:
            return False

        # If stop is already active, continue blocking
        if self._safety_stop_active:
            # Verify stop: check if robot has actually halted
            if self._stop_step is not None:
                steps_since_stop = step_id - self._stop_step
                if steps_since_stop >= self.STOP_VERIFY_STEPS:
                    self._verify_stop(task, step_id, dt)
            return True

        # Compute min link-to-obstacle distance (all links, not just EE)
        d_min_m = self._compute_link_obstacle_distance(task)
        if d_min_m is None:
            if not hasattr(self, '_debug_none_printed'):
                print(f"[safety_gate] step {step_id}: d_min_m is None")
                self._debug_none_printed = True
            return False

        # Debug: print distance periodically
        if step_id % 50 == 0 or d_min_m < self.SAFETY_DISTANCE_M:
            closest_link = getattr(self, '_debug_closest_link', '?')
            closest_obj = getattr(self, '_debug_closest_obj', '?')
            print(
                f"[safety_gate] step {step_id}: d_min={d_min_m:.4f}m "
                f"({closest_link}→{closest_obj}) threshold={self.SAFETY_DISTANCE_M:.4f}m"
            )

        # Compute approach velocity from distance history
        approach_vel_mps = None
        if self._prev_ee_obstacle_dist_m is not None:
            approach_vel_mps = (self._prev_ee_obstacle_dist_m - d_min_m) / dt
        self._prev_ee_obstacle_dist_m = d_min_m

        # Detection: distance below threshold AND approaching
        if d_min_m < self.SAFETY_DISTANCE_M:
            self._stop_triggered = True
            self._detect_step = step_id
            self._detect_distance_m = d_min_m
            self._safety_stop_active = True

            # Compute TTC at detection
            if approach_vel_mps is not None and approach_vel_mps > 0.01:
                ttc_s = d_min_m / approach_vel_mps
            else:
                # Fallback: assume obstacle speed (0.06 m/s)
                ttc_s = d_min_m / 0.06
            self._data["t_stop_s"] = step_id * dt
            self._data["_detect_ttc_s"] = ttc_s
            self._stop_step = step_id + 1  # stop takes effect next step

            logger.info(
                f"[safety_gate] STOP triggered at step {step_id}: "
                f"d={d_min_m:.4f}m, TTC={ttc_s:.2f}s"
            )
            return True

        return False

    def _compute_link_obstacle_distance(self, task) -> Optional[float]:
        """Compute minimum distance from any robot link to any object (meters).

        Checks base, EE links, and any intermediate links registered in _link_paths.
        """
        if not hasattr(task, 'objects') or not hasattr(task, 'robots'):
            if not hasattr(self, '_debug_printed'):
                print(f"[safety_gate] task missing objects/robots: objects={hasattr(task, 'objects')}, robots={hasattr(task, 'robots')}")
                self._debug_printed = True
            return None

        # Check if obstacle_1 exists
        if 'obstacle_1' not in task.objects:
            if not hasattr(self, '_debug_printed'):
                print(f"[safety_gate] obstacle_1 not in task.objects, available: {list(task.objects.keys())}")
                self._debug_printed = True
            return None

        min_dist = float('inf')
        for robot_name, robot in task.robots.items():
            # Collect all known link positions
            link_positions = []

            # Base position
            try:
                base_pos, _ = robot.get_world_pose()
                link_positions.append(("base", list(base_pos)))
            except Exception:
                pass

            # EE positions
            for ee_attr in ['fl_ee_path', 'fr_ee_path']:
                if not hasattr(robot, ee_attr):
                    continue
                ee_path = getattr(robot, ee_attr)
                if not ee_path:
                    continue
                try:
                    from omni.isaac.core.utils.xforms import get_world_pose
                    pos, _ = get_world_pose(ee_path)
                    link_positions.append((ee_attr, list(pos)))
                except Exception:
                    pass

            # Intermediate links from _link_paths (if initialized by collect_step)
            if robot_name in self._link_paths:
                for link_path in self._link_paths[robot_name]:
                    try:
                        from omni.isaac.core.utils.xforms import get_world_pose
                        pos, _ = get_world_pose(link_path)
                        link_positions.append((link_path, list(pos)))
                    except Exception:
                        pass

            # Compute distance from each link to obstacle
            for link_name, link_pos in link_positions:
                for obj_name, obj in task.objects.items():
                    if obj_name != 'obstacle_1':
                        continue  # only check obstacle (human hand)
                    try:
                        obj_pos, _ = obj.get_world_pose()
                        d = float(np.linalg.norm(np.array(link_pos) - np.array(obj_pos)))
                        if d < min_dist:
                            min_dist = d
                            if not hasattr(self, '_debug_dist_printed') or step_id % 50 == 0:
                                self._debug_closest_link = link_name
                                self._debug_closest_obj = obj_name
                    except Exception:
                        pass

        return min_dist if min_dist < float('inf') else None

    def _verify_stop(self, task, step_id: int, dt: float) -> None:
        """After stop, check if robot halted before contact with obstacle."""
        if self._data["stop_success"] is not None:
            return  # already verified

        # Check if contact with OBSTACLE occurred (not pick objects)
        contact_occurred = False
        if self._data["collision_pair_gt"]:
            recent_pairs = self._data["collision_pair_gt"][-self.STOP_VERIFY_STEPS:]
            for pairs in recent_pairs:
                if pairs is None:
                    continue
                if isinstance(pairs, list):
                    for pair in pairs:
                        if isinstance(pair, dict):
                            body_b = pair.get("bodyB", "")
                            if "obstacle" in body_b.lower() or "mano" in body_b.lower():
                                contact_occurred = True
                                break
                if contact_occurred:
                    break

        if contact_occurred:
            self._data["stop_success"] = False
            logger.info(f"[safety_gate] stop_success=False (contact occurred before halt)")
        else:
            self._data["stop_success"] = True
            logger.info(f"[safety_gate] stop_success=True (robot halted safely)")

        # Compute stop margin
        detect_ttc = self._data.get("_detect_ttc_s")
        t_stop = self._data.get("t_stop_s")
        if detect_ttc is not None and t_stop is not None:
            # stop_latency = time from detection to robot actually halting
            stop_latency = (step_id - self._detect_step) * dt if self._detect_step else 0.0
            self._data["stop_margin_s"] = detect_ttc - stop_latency
            logger.info(f"[safety_gate] stop_margin={self._data['stop_margin_s']:.3f}s")

        # Clean up internal key
        self._data.pop("_detect_ttc_s", None)

    def is_stop_active(self) -> bool:
        """Check if safety stop is currently active."""
        return self._safety_stop_active

    @staticmethod
    def _flat_articulation_values(value) -> Optional[List[float]]:
        """Convert an Isaac/PhysX batched tensor result to one flat row."""
        if value is None:
            return None
        try:
            if hasattr(value, "detach"):
                value = value.detach()
            if hasattr(value, "cpu"):
                value = value.cpu()
            if hasattr(value, "numpy"):
                value = value.numpy()
            array = np.asarray(value)
            if array.ndim > 1:
                array = array[0]
            array = array.reshape(-1)
            return [float(item) if np.isfinite(item) else None for item in array]
        except Exception:
            return None

    @classmethod
    def _read_articulation_channel(cls, robot, method_names) -> Optional[List[float]]:
        """Read a channel without assuming a particular robot wrapper class."""
        owners = [robot, getattr(robot, "_articulation_view", None)]
        for owner in owners:
            if owner is None:
                continue
            for method_name in method_names:
                method = getattr(owner, method_name, None)
                if not callable(method):
                    continue
                try:
                    values = cls._flat_articulation_values(method())
                    if values:
                        return values
                except Exception:
                    continue
        return None

    @staticmethod
    def _configured_risk_dofs(robot, dof_count: int) -> List[int]:
        """Return safety-relevant controllable DOFs for any configured robot."""
        ordered = []
        for attr in (
            "left_joint_indices", "right_joint_indices", "lift_indices",
            "body_indices", "head_indices", "joint_indices",
            "arm_joint_indices",
        ):
            for value in getattr(robot, attr, []) or []:
                try:
                    index = int(value)
                except (TypeError, ValueError):
                    continue
                if 0 <= index < dof_count and index not in ordered:
                    ordered.append(index)
        if ordered:
            return ordered
        gripper = set()
        for attr in ("left_gripper_indices", "right_gripper_indices", "gripper_indices"):
            gripper.update(int(v) for v in (getattr(robot, attr, []) or []))
        return [index for index in range(dof_count) if index not in gripper]

    def _build_joint_layout(self, task) -> Dict[str, Any]:
        """Build a stable global DOF axis across one or more robot instances."""
        channels = []
        risk_compact_indices = []
        ee_link_paths = {}
        global_offset = 0
        for robot_name, robot in getattr(task, "robots", {}).items():
            ee_link_paths[str(robot_name)] = {
                "left": str(getattr(robot, "fl_ee_path", "") or ""),
                "right": str(getattr(robot, "fr_ee_path", "") or ""),
            }
            positions = self._read_articulation_channel(
                robot, ("get_joint_positions", "get_dof_positions")
            )
            dof_names = list(getattr(robot, "dof_names", []) or [])
            dof_count = max(len(dof_names), len(positions or []))
            if dof_count <= 0:
                continue
            if len(dof_names) < dof_count:
                dof_names.extend(f"dof_{i}" for i in range(len(dof_names), dof_count))
            usd_joint_types = {}
            try:
                from pxr import Usd, UsdPhysics
                root = task.stage.GetPrimAtPath(robot.robot_prim_path)
                for prim in Usd.PrimRange(root):
                    if prim.IsA(UsdPhysics.PrismaticJoint):
                        usd_joint_types[prim.GetName()] = "prismatic"
                    elif prim.IsA(UsdPhysics.RevoluteJoint):
                        usd_joint_types[prim.GetName()] = "revolute"
            except Exception:
                pass
            risk_local = set(self._configured_risk_dofs(robot, dof_count))
            role_by_index = {}
            for role, attr in (
                ("left_arm", "left_joint_indices"),
                ("right_arm", "right_joint_indices"),
                ("lift", "lift_indices"),
                ("body", "body_indices"),
                ("head", "head_indices"),
                ("left_gripper", "left_gripper_indices"),
                ("right_gripper", "right_gripper_indices"),
            ):
                for value in getattr(robot, attr, []) or []:
                    try:
                        role_by_index[int(value)] = role
                    except (TypeError, ValueError):
                        continue
            for local_index in range(dof_count):
                compact_index = len(channels)
                dof_name = str(dof_names[local_index])
                role = role_by_index.get(local_index, "other")
                name_lower = dof_name.lower()
                is_prismatic = role == "lift" or any(
                    token in name_lower
                    for token in ("translate", "prismatic", "linear", "slider", "slide")
                )
                verified_type = usd_joint_types.get(dof_name)
                if verified_type is not None:
                    is_prismatic = verified_type == "prismatic"
                channels.append({
                    "compact_index": compact_index,
                    "global_dof_index": global_offset + local_index,
                    "local_dof_index": local_index,
                    "dof_name": dof_name,
                    "robot_name": str(robot_name),
                    "role": role,
                    "joint_type": "prismatic" if is_prismatic else "revolute",
                    "joint_type_verified": verified_type is not None,
                    "joint_type_source": "USD joint schema" if verified_type else "legacy name heuristic; not verified",
                    "position_unit": "m" if is_prismatic else "rad",
                    "velocity_unit": "m/s" if is_prismatic else "rad/s",
                    "effort_unit": "N" if is_prismatic else "N*m",
                    "risk_metric_dof": local_index in risk_local,
                })
                if local_index in risk_local:
                    risk_compact_indices.append(compact_index)
            global_offset += dof_count
        return {
            "channels": channels,
            "dof_names": [f"{c['robot_name']}/{c['dof_name']}" for c in channels],
            "source_dof_indices": [c["global_dof_index"] for c in channels],
            "risk_metric_indices": risk_compact_indices,
            "joint_limit_metric_indices": [
                c["compact_index"] for c in channels
                if c["risk_metric_dof"] and c["joint_type"] == "revolute"
            ],
            "effort_metric_indices": risk_compact_indices,
            "ee_link_paths": ee_link_paths,
            "left_dof_names": [
                f"{c['robot_name']}/{c['dof_name']}" for c in channels
                if c["role"] == "left_arm"
            ],
            "right_dof_names": [
                f"{c['robot_name']}/{c['dof_name']}" for c in channels
                if c["role"] == "right_arm"
            ],
            "position_unit": "mixed; see channels[].position_unit",
            "velocity_unit": "mixed; see channels[].velocity_unit",
            "effort_unit": "mixed; see channels[].effort_unit",
            "axis_semantics": "all articulation DOFs, robots concatenated in task order",
        }

    def get_joint_layout(self, task=None) -> Dict[str, Any]:
        if self._joint_layout is None and task is not None:
            self._joint_layout = self._build_joint_layout(task)
        return dict(self._joint_layout or {})

    def _collect_joint_states(self, task) -> None:
        """Collect all articulation positions and velocities for all robots."""
        layout = self.get_joint_layout(task)
        channels = layout.get("channels", [])
        by_robot = {}
        for robot_name, robot in getattr(task, "robots", {}).items():
            by_robot[str(robot_name)] = {
                "position": self._read_articulation_channel(
                    robot, ("get_joint_positions", "get_dof_positions")
                ),
                "velocity": self._read_articulation_channel(
                    robot, ("get_joint_velocities", "get_dof_velocities")
                ),
            }

        def _row(channel_name):
            result = []
            for channel in channels:
                values = by_robot.get(channel["robot_name"], {}).get(channel_name)
                index = channel["local_dof_index"]
                result.append(values[index] if values is not None and index < len(values) else None)
            return result or None

        self._data["joint_position_q_gt"].append(_row("position"))
        self._data["joint_velocity_dq_gt"].append(_row("velocity"))
        self._data["joint_state_metadata"] = layout

    def _collect_joint_torques(self, task) -> None:
        """Collect joint torques from all robots.

        For position-controlled robots, the physics solver computes torques
        internally. We try multiple APIs to read them:
        1. get_applied_joint_torques() - explicitly applied torques
        2. get_measured_joint_efforts() - measured efforts (works for pos control)
        3. get_joint_torques() - direct PhysX API
        """
        layout = self.get_joint_layout(task)
        channels = layout.get("channels", [])
        values_by_robot = {}
        for robot_name, robot in getattr(task, "robots", {}).items():
            values_by_robot[str(robot_name)] = self._read_articulation_channel(
                robot,
                ("get_measured_joint_efforts", "get_applied_joint_torques",
                 "get_applied_joint_efforts", "get_joint_torques"),
            )
        row = []
        for channel in channels:
            values = values_by_robot.get(channel["robot_name"])
            index = channel["local_dof_index"]
            row.append(values[index] if values is not None and index < len(values) else None)
        self._data["joint_torque_gt"].append(row or None)

    # ── Link Poses & Velocities (S-ROBOT-005, S-ROBOT-006) ─────────────────

    def _collect_all_link_poses(self, task) -> None:
        """Collect world-frame poses and velocities for ALL robot links.

        Discovers links by traversing the robot prim tree on the first call,
        then reads world pose for each cached link path every step.
        Computes linear velocity from pose differences.

        Output format per step:
            link_pose_gt: {robot_name: {link_name: [x,y,z,qx,qy,qz,qw]}}
            link_velocity_gt: {robot_name: {link_name: [vx,vy,vz,0,0,0]}}
        """
        from omni.isaac.core.utils.prims import get_prim_at_path
        from omni.isaac.core.utils.xforms import get_world_pose as get_xform_world_pose

        if not hasattr(self, '_all_link_cache'):
            self._all_link_cache = {}
        if not hasattr(self, '_prev_all_link_poses'):
            self._prev_all_link_poses = {}

        result_poses = {}
        result_velocities = {}
        dt = self._physics_dt_s

        for robot_name, robot in task.robots.items():
            # Discover link paths on first step
            if robot_name not in self._all_link_cache:
                try:
                    link_map = self._discover_robot_links(robot)
                    self._all_link_cache[robot_name] = link_map
                except Exception as e:
                    logger.warning("Failed to discover links for %s: %s", robot_name, e)
                    self._all_link_cache[robot_name] = {}

            link_map = self._all_link_cache[robot_name]
            robot_poses = {}
            robot_velocities = {}

            for link_name, link_path in link_map.items():
                try:
                    prim = get_prim_at_path(link_path)
                    if prim and prim.IsValid():
                        pos, ori = get_xform_world_pose(link_path)
                        # Isaac get_world_pose returns scalar-first WXYZ; the
                        # Sim_Raw_GT contract is consistently XYZW.
                        pose = [
                            float(pos[0]), float(pos[1]), float(pos[2]),
                            float(ori[1]), float(ori[2]), float(ori[3]), float(ori[0])
                        ]
                    else:
                        pose = [None] * 7
                except Exception:
                    pose = [None] * 7

                robot_poses[link_name] = pose

                # Compute velocity from pose difference
                prev_poses = self._prev_all_link_poses.get(robot_name, {})
                prev_pose = prev_poses.get(link_name)
                if (pose[0] is not None and prev_pose is not None
                        and prev_pose[0] is not None):
                    # Linear velocity
                    vx = (pose[0] - prev_pose[0]) / dt
                    vy = (pose[1] - prev_pose[1]) / dt
                    vz = (pose[2] - prev_pose[2]) / dt

                    # Angular velocity from quaternion difference
                    wx, wy, wz = self._compute_angular_velocity(
                        prev_pose[3:7], pose[3:7], dt
                    )

                    robot_velocities[link_name] = [vx, vy, vz, wx, wy, wz]
                else:
                    robot_velocities[link_name] = [0.0] * 6

            self._prev_all_link_poses[robot_name] = robot_poses
            result_poses[robot_name] = robot_poses
            result_velocities[robot_name] = robot_velocities

        self._data["link_pose_gt"].append(result_poses)
        self._data["link_velocity_gt"].append(result_velocities)

    def _discover_robot_links(self, robot) -> Dict[str, str]:
        """Discover all robot link prim paths by traversing the USD tree.

        Returns dict: {relative_path: prim_path}
        Only includes actual robot links, filtering out joints, materials,
        shaders, visuals, collisions, and other non-link prims.
        """
        from omni.isaac.core.utils.prims import get_prim_at_path
        from pxr import UsdPhysics

        link_map = {}

        if not hasattr(robot, 'prim_path'):
            return link_map

        robot_prim = get_prim_at_path(robot.prim_path)
        if not robot_prim or not robot_prim.IsValid():
            return link_map

        def _is_link(prim):
            """Check if a prim is an actual robot link (not joint/material/etc)."""
            name = prim.GetName()
            name_lower = name.lower()

            # Skip joints
            if 'joint' in name_lower:
                return False
            # Skip materials and shaders
            if 'material' in name_lower or name_lower == 'shader':
                return False
            # Skip visual-only containers
            if name_lower in ('visuals', 'looks', 'collisions'):
                return False
            # Skip non-robot prims
            if name_lower in ('physscene', 'world', 'scene'):
                return False

            # Prefer the physics schema over robot-specific naming.  This
            # covers panda_linkN, Piper linkN, lift/base links and future USDs.
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                return True

            # Fallback for assets whose link Xforms do not author RigidBodyAPI.
            # Match: xxx_link, link1, link2, ..., arm_base, base_link
            if name_lower.endswith('_link') or name_lower == 'arm_base':
                return True
            if re.match(r'^(?:[a-z0-9]+_)?link\d+$', name_lower):
                return True
            if any(token in name_lower for token in ("wrist", "hand", "gripper", "end_link")):
                return True

            return False

        def _traverse(prim, depth=0, max_depth=8):
            if depth > max_depth:
                return
            for child in prim.GetChildren():
                child_path = str(child.GetPrimPath())
                # Use full relative path as key to avoid name collisions
                rel_name = child_path.replace(robot.prim_path + "/", "")
                if _is_link(child) and rel_name not in link_map:
                    link_map[rel_name] = child_path
                _traverse(child, depth + 1, max_depth)

        _traverse(robot_prim)
        return link_map

    def _collect_human_body_poses(self, task) -> None:
        """Collect world poses for every rigid link in configured human obstacles.

        MANO is an articulated obstacle.  Recording only its root transform
        loses palm/finger motion, so each rigid body is discovered from the USD
        hierarchy and sampled every physics step.

        Per-step output:
            {
              "obstacle_1": {
                "palm": {
                  "prim_path": ".../mano/palm",
                  "pose": [x,y,z,qx,qy,qz,qw]
                },
                ...
              }
            }
        """
        from omni.isaac.core.utils.prims import get_prim_at_path
        from omni.isaac.core.utils.xforms import get_world_pose
        from pxr import Usd, UsdGeom, UsdPhysics

        if not hasattr(self, "_human_body_link_cache"):
            self._human_body_link_cache = {}

        objects = getattr(task, "objects", {}) or {}
        frame = {}
        for obstacle_name, obstacle in objects.items():
            if "obstacle" not in str(obstacle_name).lower():
                continue

            if obstacle_name not in self._human_body_link_cache:
                root_path = (
                    getattr(obstacle, "prim_path", None)
                    or getattr(obstacle, "object_prim_path", None)
                )
                link_map = {}
                if root_path:
                    root_prim = get_prim_at_path(root_path)
                    if root_prim and root_prim.IsValid():
                        root_prefix = str(root_prim.GetPath()).rstrip("/") + "/"
                        collision_shape_paths = []
                        for candidate in Usd.PrimRange(root_prim):
                            candidate_path = str(candidate.GetPath()).lower()
                            if (
                                "/humanskeletonphysics/" in candidate_path
                                and candidate.GetTypeName() in {
                                    "Capsule", "Sphere", "Mesh", "Cube", "Cylinder"
                                }
                            ):
                                collision_shape_paths.append(candidate)
                        for prim in Usd.PrimRange(root_prim):
                            name = prim.GetName().lower()
                            prim_path_text = str(prim.GetPath()).lower()
                            is_human_collision_shape = (
                                "/humanskeletonphysics/" in prim_path_text
                                and prim.GetTypeName() in {
                                    "Capsule", "Sphere", "Mesh", "Cube", "Cylinder"
                                }
                            )
                            named_hand_part = bool(re.match(
                                r"^(?:mano|palm|wrist|thumb|index|middle|ring|pinky)(?:[_0-9].*)?$",
                                name,
                            ))
                            if not (
                                prim.HasAPI(UsdPhysics.RigidBodyAPI)
                                or (named_hand_part and prim.IsA(UsdGeom.Xformable))
                                or is_human_collision_shape
                                or (prim.HasAPI(UsdPhysics.CollisionAPI) and prim.IsA(UsdGeom.Xformable))
                            ):
                                continue
                            # RoboSafe uses one compound rigid body at the
                            # root and 23 shape colliders below it. The root
                            # pose is redundant when the complete collider
                            # poses are available, so keep the report at the
                            # body-part granularity expected by Sim_Raw_GT.
                            if prim == root_prim and collision_shape_paths:
                                continue
                            prim_path = str(prim.GetPath())
                            relative_path = prim_path.replace(root_prefix, "", 1)
                            link_name = prim.GetName()
                            if link_name in link_map:
                                link_name = relative_path
                            link_map[link_name] = prim_path
                self._human_body_link_cache[obstacle_name] = {
                    "root_prim_path": root_path,
                    "links": link_map,
                }

            cached = self._human_body_link_cache[obstacle_name]
            body_parts = {}
            for link_name, prim_path in cached["links"].items():
                pose = None
                try:
                    prim = get_prim_at_path(prim_path)
                    if prim and prim.IsValid():
                        position, orientation_wxyz = get_world_pose(prim_path)
                        # Isaac Sim get_world_pose returns scalar-first WXYZ.
                        # Sim_Raw_GT consistently declares quaternion order XYZW.
                        pose = [
                            float(position[0]),
                            float(position[1]),
                            float(position[2]),
                            float(orientation_wxyz[1]),
                            float(orientation_wxyz[2]),
                            float(orientation_wxyz[3]),
                            float(orientation_wxyz[0]),
                        ]
                except Exception:
                    pose = None
                body_parts[link_name] = {
                    "prim_path": prim_path,
                    "pose": pose,
                }

            frame[obstacle_name] = {
                "root_prim_path": cached["root_prim_path"],
                "body_parts": body_parts,
            }

        self._data["human_body_pose_gt"].append(frame)

    def build_human_body_pose_gt(self) -> Optional[Dict[str, Any]]:
        """Convert per-step MANO samples into body-part pose time series."""
        frames = self._data.get("human_body_pose_gt", [])
        if not frames or not any(isinstance(frame, dict) and frame for frame in frames):
            return None

        obstacles: Dict[str, Any] = {}
        obstacle_names = sorted({
            obstacle_name
            for frame in frames if isinstance(frame, dict)
            for obstacle_name in frame
        })
        for obstacle_name in obstacle_names:
            root_path = None
            link_records: Dict[str, Dict[str, Any]] = {}
            link_names = sorted({
                link_name
                for frame in frames if isinstance(frame, dict)
                for link_name in (
                    frame.get(obstacle_name, {}).get("body_parts", {})
                    if isinstance(frame.get(obstacle_name), dict) else {}
                )
            })
            for link_name in link_names:
                prim_path = None
                poses = []
                for frame in frames:
                    obstacle_frame = (
                        frame.get(obstacle_name, {})
                        if isinstance(frame, dict) else {}
                    )
                    if root_path is None and isinstance(obstacle_frame, dict):
                        root_path = obstacle_frame.get("root_prim_path")
                    record = (
                        obstacle_frame.get("body_parts", {}).get(link_name)
                        if isinstance(obstacle_frame, dict) else None
                    )
                    if isinstance(record, dict):
                        prim_path = prim_path or record.get("prim_path")
                        poses.append(record.get("pose"))
                    else:
                        poses.append(None)
                link_records[link_name] = {
                    "prim_path": prim_path,
                    "pose_per_step": poses,
                }
            obstacles[obstacle_name] = {
                "root_prim_path": root_path,
                "body_parts": link_records,
            }

        return {
            "surrogate_type": "runtime_human_proxy",
            "coordinate_frame": "world",
            "pose_format": "[x,y,z,qx,qy,qz,qw]",
            "quaternion_order": "xyzw",
            "position_unit": "m",
            "num_steps": len(frames),
            "obstacles": obstacles,
            "source": "runtime USD rigid-body world poses sampled after each physics step",
        }

    def _compute_angular_velocity(self, q_prev, q_curr, dt):
        """Compute angular velocity from two quaternions.

        Parameters
        ----------
        q_prev : list
            Previous quaternion [qx, qy, qz, qw]
        q_curr : list
            Current quaternion [qx, qy, qz, qw]
        dt : float
            Time step

        Returns
        -------
        tuple
            Angular velocity (wx, wy, wz) in rad/s
        """
        # Ensure quaternions are normalized
        import math

        def normalize(q):
            n = math.sqrt(sum(x*x for x in q))
            return [x/n for x in q] if n > 0 else q

        q0 = normalize(q_prev)
        q1 = normalize(q_curr)

        # Quaternion difference: dq = q1 * conj(q0)
        # conj(q0) = [-q0x, -q0y, -q0z, q0w]
        dq = [
            q1[3]*(-q0[0]) + q1[0]*q0[3] + q1[1]*(-q0[2]) - q1[2]*(-q0[1]),
            q1[3]*(-q0[1]) - q1[0]*(-q0[2]) + q1[1]*q0[3] + q1[2]*(-q0[0]),
            q1[3]*(-q0[2]) + q1[0]*(-q0[1]) - q1[1]*(-q0[0]) + q1[2]*q0[3],
            q1[3]*q0[3] - q1[0]*(-q0[0]) - q1[1]*(-q0[1]) - q1[2]*(-q0[2]),
        ]

        # Ensure shortest path (dq[3] should be positive)
        if dq[3] < 0:
            dq = [-x for x in dq]

        # Convert to axis-angle: angle = 2 * acos(dq[3])
        # axis = dq[:3] / sin(angle/2)
        sin_half = math.sqrt(dq[0]**2 + dq[1]**2 + dq[2]**2)

        if sin_half < 1e-10:
            return (0.0, 0.0, 0.0)

        angle = 2.0 * math.asin(min(sin_half, 1.0))
        angular_speed = angle / dt

        # Axis
        axis = [dq[i] / sin_half for i in range(3)]

        return (axis[0] * angular_speed, axis[1] * angular_speed, axis[2] * angular_speed)

    def _get_link_paths(self, robot) -> List[str]:
        """Get all link prim paths for a robot."""
        paths = []
        try:
            # Method 1: Use known EE paths (most reliable)
            for attr in ['fl_ee_path', 'fr_ee_path']:
                if hasattr(robot, attr):
                    path = getattr(robot, attr)
                    if path:
                        paths.append(path)

            # Method 2: Traverse robot prim tree for links with collision meshes
            if hasattr(robot, 'prim_path'):
                try:
                    from omni.isaac.core.utils.prims import get_prim_at_path
                    robot_prim = get_prim_at_path(robot.prim_path)
                    if robot_prim:
                        for child in robot_prim.GetChildren():
                            child_name = child.GetName().lower()
                            # Look for arm links
                            if 'link' in child_name or 'arm' in child_name:
                                child_path = str(child.GetPrimPath())
                                if child_path not in paths:
                                    paths.append(child_path)
                            # Also check grandchildren for fl/fr arms
                            for grandchild in child.GetChildren():
                                gc_name = grandchild.GetName().lower()
                                if 'link' in gc_name:
                                    gc_path = str(grandchild.GetPrimPath())
                                    if gc_path not in paths:
                                        paths.append(gc_path)
                except Exception:
                    pass

        except Exception:
            pass

        return paths

    # ── Contact/Collision Data (S-COLL-001, 004, 005, 006) ─────────────────

    def _collect_contact_data(self, task, step_id: int) -> None:
        """Collect contact force, collision location, penetration depth data."""
        collision_pairs = []
        collision_locations = []
        penetration_depths = []
        contact_forces = []
        total_force = [0.0, 0.0, 0.0]
        total_impulse = 0.0
        dt = self._physics_dt_s
        try:
            from omni.isaac.core.simulation_context import SimulationContext

            simulation_context = SimulationContext.instance()
            if simulation_context is not None:
                measured_dt = float(simulation_context.get_physics_dt())
                if measured_dt > 0:
                    dt = measured_dt
                    self._physics_dt_s = measured_dt
                    self._data["runtime_physics"]["physics_dt_s"] = measured_dt
                gravity_direction, gravity_magnitude = (
                    simulation_context.get_physics_context().get_gravity()
                )
                self._data["runtime_physics"]["gravity_direction"] = [
                    float(value) for value in gravity_direction
                ]
                self._data["runtime_physics"]["gravity_magnitude_mps2"] = float(
                    gravity_magnitude
                )
        except Exception:
            pass

        # Helper to get object position
        def _get_pos(obj_name):
            try:
                if hasattr(task, 'objects') and obj_name in task.objects:
                    pos, _ = task.objects[obj_name].get_world_pose()
                    return [float(pos[0]), float(pos[1]), float(pos[2])]
            except Exception:
                pass
            return None

        def _nearest_robot_link_body(robot_name, lr_name, point_m=None):
            """Return a robot body label, resolving legacy whole-robot views to a link."""
            if lr_name != "all":
                return f"robot/{robot_name}/{lr_name}"
            link_poses = self._data.get("link_pose_gt")[-1] if self._data.get("link_pose_gt") else None
            robot_links = link_poses.get(robot_name, {}) if isinstance(link_poses, dict) else {}
            if not point_m or not robot_links:
                return f"robot/{robot_name}/unknown_link"

            best_name = None
            best_dist = None
            for link_name, pose in robot_links.items():
                if not isinstance(pose, (list, tuple)) or len(pose) < 3 or pose[0] is None:
                    continue
                dist = math.sqrt(
                    (float(pose[0]) - float(point_m[0])) ** 2
                    + (float(pose[1]) - float(point_m[1])) ** 2
                    + (float(pose[2]) - float(point_m[2])) ** 2
                )
                if best_dist is None or dist < best_dist:
                    best_dist = dist
                    best_name = link_name

            if best_name:
                return f"robot/{robot_name}/{best_name}"
            return f"robot/{robot_name}/unknown_link"

        def _hertzian_depth(force_magnitude, stiffness=1e6):
            return float((max(float(force_magnitude), 0.0) / stiffness) ** (2.0 / 3.0))

        def _penetration_from_contact_data(contact_data, start_indices, contact_counts, force_magnitude):
            """Return penetration-depth metadata from PhysX contact data if available.

            Isaac/PhysX contact views expose detailed contact arrays via
            get_contact_force_data(). The fourth returned array is treated as
            the per-contact distance/separation signal. Negative separation and
            positive penetration conventions vary across APIs/configs, so the
            reported depth_m is the mean absolute contact distance, while the
            raw signed mean/min/max are kept for validation. If detailed contact
            distances are unavailable, fall back to a low-confidence Hertzian
            force-based estimate.
            """
            fallback_reason = "no_contact_data"
            try:
                if start_indices.size > 0 and contact_counts.size > 0 and len(contact_data) > 3:
                    distances = np.asarray(contact_data[3], dtype=float).reshape(-1)
                    counts = np.asarray(contact_counts)
                    starts = np.asarray(start_indices)

                    if counts.size == 0 or starts.size == 0 or distances.size == 0:
                        fallback_reason = "empty_contact_buffers"
                    else:
                        # pair_contacts_count/start_indices are matrices with shape
                        # (num_shapes, num_filters).  The previous implementation only
                        # checked [0, 0], which misses obstacle-vs-whole-robot contacts
                        # when the active robot link is any other filter column.
                        counts = counts.reshape(starts.shape) if counts.shape != starts.shape else counts
                        all_vals = []
                        active_pair_slots = []
                        invalid_pair_slots = []
                        for pair_idx in np.ndindex(counts.shape):
                            n_contacts = int(counts[pair_idx])
                            if n_contacts <= 0:
                                continue
                            start = int(starts[pair_idx])
                            end = start + n_contacts
                            if start < 0 or end > len(distances):
                                invalid_pair_slots.append({
                                    "pair_index": [int(x) for x in pair_idx],
                                    "start": start,
                                    "count": n_contacts,
                                })
                                continue
                            vals = distances[start:end]
                            if vals.size > 0:
                                all_vals.append(vals)
                                active_pair_slots.append({
                                    "pair_index": [int(x) for x in pair_idx],
                                    "start": start,
                                    "count": n_contacts,
                                })

                        if all_vals:
                            vals = np.concatenate(all_vals).astype(float).reshape(-1)
                            return {
                                "depth_m": float(np.mean(np.abs(vals))),
                                "method": "physx_contact_distance",
                                "source": "contact_view.get_contact_force_data()[3]",
                                "confidence": "medium_high",
                                "num_contact_points": int(vals.size),
                                "active_contact_pair_slots": len(active_pair_slots),
                                "active_contact_pair_indices_sample": active_pair_slots[:8],
                                "invalid_contact_pair_slots": len(invalid_pair_slots),
                                "contact_distance_mean_m": float(np.mean(vals)),
                                "contact_distance_min_m": float(np.min(vals)),
                                "contact_distance_max_m": float(np.max(vals)),
                            }
                        fallback_reason = "zero_contact_counts"
            except Exception as exc:
                fallback_reason = f"exception:{type(exc).__name__}"
            return {
                "depth_m": _hertzian_depth(force_magnitude),
                "method": "hertzian_fallback",
                "source": "contact_force_magnitude",
                "confidence": "low",
                "num_contact_points": 0,
                "hertzian_stiffness_n_per_m": 1e6,
                "fallback_reason": fallback_reason,
            }

        if hasattr(task, 'pickcontact_views'):
            for robot_name, lr_dict in task.pickcontact_views.items():
                for lr_name, obj_dict in lr_dict.items():
                    for obj_name, contact_view in obj_dict.items():
                        try:
                            # Try to get detailed contact data with contact points
                            contact_data = contact_view.get_contact_force_data(dt=dt)
                            if contact_data is not None:
                                forces = contact_data[0]      # (max_contact_count, 1)
                                points = contact_data[1]      # (max_contact_count, 3)
                                contact_counts = contact_data[4]  # (num_shapes, num_filters)
                                start_indices = contact_data[5]  # (num_shapes, num_filters)

                                force_magnitude = float(np.sum(np.abs(forces)))
                                if force_magnitude <= 0.01:
                                    continue

                                # Collision pair
                                collision_pairs.append({
                                    "bodyA": f"robot/{robot_name}/{lr_name}",
                                    "bodyB": f"object/{obj_name}",
                                    "step": step_id,
                                    "force_n": force_magnitude,
                                })

                                # Get actual contact points
                                if start_indices.size > 0 and contact_counts.size > 0:
                                    start = int(start_indices[0, 0]) if start_indices.ndim >= 2 else int(start_indices[0])
                                    n_contacts = int(contact_counts[0, 0]) if contact_counts.ndim >= 2 else int(contact_counts[0])
                                    logger.debug("Contact data: start=%d, n_contacts=%d, points_len=%d", start, n_contacts, len(points))
                                    if n_contacts > 0 and start + n_contacts <= len(points):
                                        contact_pts = points[start:start + n_contacts]
                                        avg_point = np.mean(contact_pts, axis=0)
                                        collision_locations.append({
                                            "bodyA": f"robot/{robot_name}/{lr_name}",
                                            "bodyB": f"object/{obj_name}",
                                            "location_m": [float(avg_point[0]), float(avg_point[1]), float(avg_point[2])],
                                        })
                                        logger.debug("Using actual contact points: %s", avg_point)
                                    else:
                                        obj_pos = _get_pos(obj_name)
                                        if obj_pos:
                                            collision_locations.append({
                                                "bodyA": f"robot/{robot_name}/{lr_name}",
                                                "bodyB": f"object/{obj_name}",
                                                "location_m": obj_pos,
                                            })
                                            logger.debug("Falling back to object centroid: %s", obj_pos)
                                else:
                                    obj_pos = _get_pos(obj_name)
                                    if obj_pos:
                                        collision_locations.append({
                                            "bodyA": f"robot/{robot_name}/{lr_name}",
                                            "bodyB": f"object/{obj_name}",
                                            "location_m": obj_pos,
                                        })

                                depth_info = _penetration_from_contact_data(
                                    contact_data, start_indices, contact_counts, force_magnitude
                                )
                                depth_info.update({
                                    "bodyA": f"robot/{robot_name}/{lr_name}",
                                    "bodyB": f"object/{obj_name}",
                                })
                                penetration_depths.append(depth_info)

                                total_force[0] += float(np.sum(forces[:, 0])) if forces.size > 0 else 0
                                total_force[1] += 0
                                total_force[2] += 0
                                total_impulse += force_magnitude * dt
                            else:
                                # Fallback to get_contact_force_matrix
                                force_matrix = contact_view.get_contact_force_matrix(dt=dt)
                                if force_matrix is not None:
                                    force_matrix = np.abs(force_matrix).squeeze()
                                    if force_matrix.ndim >= 2:
                                        force_sum = np.sum(force_matrix, axis=tuple(range(force_matrix.ndim - 1)))
                                    else:
                                        force_sum = force_matrix

                                    force_magnitude = float(np.linalg.norm(force_sum))
                                    if force_magnitude > 0.01:
                                        collision_pairs.append({
                                            "bodyA": f"robot/{robot_name}/{lr_name}",
                                            "bodyB": f"object/{obj_name}",
                                            "step": step_id,
                                            "force_n": force_magnitude,
                                        })
                                        obj_pos = _get_pos(obj_name)
                                        if obj_pos:
                                            collision_locations.append({
                                                "bodyA": f"robot/{robot_name}/{lr_name}",
                                                "bodyB": f"object/{obj_name}",
                                                "location_m": obj_pos,
                                            })
                                        stiffness = 1e6
                                        delta = (force_magnitude / stiffness) ** (2.0 / 3.0)
                                        penetration_depths.append({
                                            "bodyA": f"robot/{robot_name}/{lr_name}",
                                            "bodyB": f"object/{obj_name}",
                                            "depth_m": delta,
                                            "method": "hertzian_fallback",
                                            "source": "contact_view.get_contact_force_matrix()",
                                            "confidence": "low",
                                            "num_contact_points": 0,
                                            "hertzian_stiffness_n_per_m": stiffness,
                                        })
                                        total_force[0] += float(force_sum[0]) if len(force_sum) > 0 else 0
                                        total_force[1] += float(force_sum[1]) if len(force_sum) > 1 else 0
                                        total_force[2] += float(force_sum[2]) if len(force_sum) > 2 else 0
                                        total_impulse += force_magnitude * dt
                        except Exception:
                            pass

        # Check articulation contact views
        if hasattr(task, 'artcontact_views'):
            for robot_name, lr_dict in task.artcontact_views.items():
                for lr_name, view_dict in lr_dict.items():
                    for view_name, contact_view in view_dict.items():
                        try:
                            force_matrix = contact_view.get_contact_force_matrix(dt=dt)
                            if force_matrix is not None:
                                force_matrix = np.abs(force_matrix).squeeze()
                                force_magnitude = float(np.sum(force_matrix))
                                if force_magnitude > 0.01:
                                    collision_pairs.append({
                                        "bodyA": f"robot/{robot_name}/{lr_name}",
                                        "bodyB": f"environment/{view_name}",
                                        "step": step_id,
                                        "force_n": force_magnitude,
                                    })

                                    # Collision location: use robot link position (approximate)
                                    collision_locations.append({
                                        "bodyA": f"robot/{robot_name}/{lr_name}",
                                        "bodyB": f"environment/{view_name}",
                                        "location_m": None,  # link position not directly available
                                    })

                                    # Penetration depth
                                    stiffness = 1e6
                                    delta = (force_magnitude / stiffness) ** (2.0 / 3.0)
                                    penetration_depths.append({
                                        "bodyA": f"robot/{robot_name}/{lr_name}",
                                        "bodyB": f"environment/{view_name}",
                                        "depth_m": delta,
                                        "method": "hertzian_fallback",
                                        "source": "contact_view.get_contact_force_matrix()",
                                        "confidence": "low",
                                        "num_contact_points": 0,
                                        "hertzian_stiffness_n_per_m": stiffness,
                                    })

                                    total_impulse += force_magnitude * dt
                        except Exception:
                            pass

        # Check robot-obstacle contact views separately from pick contacts so
        # obstacle forces are not counted as gripper-object grasp forces.
        if hasattr(task, 'obstaclecontact_views'):
            for robot_name, lr_dict in task.obstaclecontact_views.items():
                for lr_name, obj_dict in lr_dict.items():
                    if lr_name == "_filter_paths":
                        continue
                    for obj_name, contact_view in obj_dict.items():
                        try:
                            # Try to get detailed contact data with contact points
                            contact_data = contact_view.get_contact_force_data(dt=dt)
                            if contact_data is not None:
                                forces = contact_data[0]
                                points = contact_data[1]
                                contact_counts = contact_data[4]
                                start_indices = contact_data[5]

                                force_magnitude = float(np.sum(np.abs(forces)))
                                if force_magnitude <= 0.01:
                                    continue

                                body_a = _nearest_robot_link_body(robot_name, lr_name)
                                body_b = f"obstacle/{obj_name}"
                                collision_pairs.append({
                                    "bodyA": body_a,
                                    "bodyB": body_b,
                                    "step": step_id,
                                    "force_n": force_magnitude,
                                })

                                # Get actual contact points
                                location_m = None
                                if start_indices.size > 0 and contact_counts.size > 0:
                                    start = int(start_indices[0, 0]) if start_indices.ndim >= 2 else int(start_indices[0])
                                    n_contacts = int(contact_counts[0, 0]) if contact_counts.ndim >= 2 else int(contact_counts[0])
                                    if n_contacts > 0 and start + n_contacts <= len(points):
                                        contact_pts = points[start:start + n_contacts]
                                        avg_point = np.mean(contact_pts, axis=0)
                                        location_m = [float(avg_point[0]), float(avg_point[1]), float(avg_point[2])]
                                    else:
                                        location_m = _get_pos(obj_name)
                                else:
                                    location_m = _get_pos(obj_name)

                                body_a = _nearest_robot_link_body(robot_name, lr_name, location_m)
                                collision_pairs[-1]["bodyA"] = body_a
                                collision_locations.append({
                                    "bodyA": body_a,
                                    "bodyB": body_b,
                                    "location_m": location_m,
                                })

                                depth_info = _penetration_from_contact_data(
                                    contact_data, start_indices, contact_counts, force_magnitude
                                )
                                depth_info.update({
                                    "bodyA": body_a,
                                    "bodyB": body_b,
                                })
                                penetration_depths.append(depth_info)
                                total_impulse += force_magnitude * dt
                            else:
                                # Fallback to get_contact_force_matrix
                                force_matrix = contact_view.get_contact_force_matrix(dt=dt)
                                if force_matrix is not None:
                                    force_matrix = np.abs(force_matrix).squeeze()
                                    force_magnitude = float(np.sum(force_matrix))
                                    if force_magnitude <= 0.01:
                                        continue
                                    obj_pos = _get_pos(obj_name)
                                    body_a = _nearest_robot_link_body(robot_name, lr_name, obj_pos)
                                    body_b = f"obstacle/{obj_name}"
                                    collision_pairs.append({
                                        "bodyA": body_a,
                                        "bodyB": body_b,
                                        "step": step_id,
                                        "force_n": force_magnitude,
                                    })
                                    collision_locations.append({
                                        "bodyA": body_a,
                                        "bodyB": body_b,
                                        "location_m": obj_pos,
                                    })
                                    penetration_depths.append({
                                        "bodyA": body_a,
                                        "bodyB": body_b,
                                        "depth_m": _hertzian_depth(force_magnitude),
                                        "method": "hertzian_fallback",
                                        "source": "contact_view.get_contact_force_matrix()",
                                        "confidence": "low",
                                        "num_contact_points": 0,
                                        "hertzian_stiffness_n_per_m": 1e6,
                                    })
                                    total_impulse += force_magnitude * dt
                        except Exception as exc:
                            logger.warning(
                                "Failed to read obstacle contact view %s/%s/%s: %s",
                                robot_name, lr_name, obj_name, exc,
                            )

        # One complete tensor matrix covers the four missing collision
        # categories.  Aggregate colliders that share a semantic body label and
        # canonicalize symmetric pairs so no contact is double-counted.
        safetycontact_spec = getattr(task, "safetycontact_view", None)
        coverage_status = self._data["contact_coverage_status"]
        coverage_status["configured"] = bool(safetycontact_spec)
        if safetycontact_spec:
            try:
                matrix = safetycontact_spec["view"].get_contact_force_matrix(dt=dt)
                if matrix is None:
                    raise RuntimeError("PhysX returned no safety contact matrix")
                matrix = np.asarray(matrix, dtype=float)
                source_labels = safetycontact_spec.get("source_labels", [])
                filter_labels = safetycontact_spec.get("filter_labels", [])
                expected_shape = (len(source_labels), len(filter_labels), 3)
                if matrix.ndim != 3 or tuple(matrix.shape) != expected_shape:
                    raise RuntimeError(
                        f"safety contact matrix shape {tuple(matrix.shape)} != {expected_shape}"
                    )

                aggregated = {}
                for source_idx, source in enumerate(source_labels):
                    source_kind = source.get("kind")
                    source_label = source.get("label")
                    for filter_idx, target in enumerate(filter_labels):
                        target_kind = target.get("kind")
                        target_label = target.get("label")
                        force_magnitude = float(np.linalg.norm(matrix[source_idx, filter_idx]))
                        if force_magnitude <= 0:
                            continue

                        pair = None
                        if source_kind == "object" and target_kind == "environment":
                            pair = (
                                f"object/{source_label}",
                                f"environment/{target_label}",
                            )
                        elif source_kind == "object" and target_kind == "human":
                            pair = (
                                f"object/{source_label}",
                                f"obstacle/{target_label}",
                            )
                        elif source_kind == "robot" and target_kind == "environment":
                            pair = (
                                f"robot/{source_label}",
                                f"environment/{target_label}",
                            )
                        elif source_kind == "robot" and target_kind == "robot":
                            if source_label == target_label:
                                continue
                            first, second = sorted((source_label, target_label))
                            if source_label != first:
                                continue
                            pair = (f"robot/{first}", f"robot/{second}")
                        elif source_kind == "object" and target_kind == "object":
                            if source_label == target_label:
                                continue
                            first, second = sorted((source_label, target_label))
                            if source_label != first:
                                continue
                            pair = (f"object/{first}", f"object/{second}")

                        if pair is not None:
                            aggregated[pair] = aggregated.get(pair, 0.0) + force_magnitude

                for (body_a, body_b), force_magnitude in aggregated.items():
                    if force_magnitude <= 0.01:
                        continue
                    collision_pairs.append({
                        "bodyA": body_a,
                        "bodyB": body_b,
                        "step": step_id,
                        "force_n": force_magnitude,
                        "source": "RigidContactView.get_contact_force_matrix(dt=physics_dt)",
                    })
                    collision_locations.append({
                        "bodyA": body_a,
                        "bodyB": body_b,
                        "location_m": None,
                    })
                    penetration_depths.append({
                        "bodyA": body_a,
                        "bodyB": body_b,
                        "depth_m": _hertzian_depth(force_magnitude),
                        "method": "hertzian_fallback",
                        "source": "contact_view.get_contact_force_matrix(dt=physics_dt)",
                        "confidence": "low",
                        "num_contact_points": 0,
                        "hertzian_stiffness_n_per_m": 1e6,
                    })
                    total_impulse += force_magnitude * dt

                coverage_status["successful_steps"] += 1
                coverage_status["last_error"] = None
            except Exception as exc:
                coverage_status["failed_steps"] += 1
                coverage_status["last_error"] = f"{type(exc).__name__}: {exc}"
                logger.warning("Failed complete safety contact view: %s", exc)

        # Replace tensor-derived forces with the native PhysX contact-report
        # impulses.  The callback exposes the solver impulse vector directly,
        # avoiding backend-dependent force scaling in multi-filter matrices.
        aggregated_reports = {}
        if self._contact_report_sub is not None:
            reports = self._pending_contact_reports
            self._pending_contact_reports = []
            report_status = self._data["contact_report_status"]
            report_status["total_report_headers"] += len(reports)
            if reports:
                report_status["steps_with_reports"] += 1
            collision_pairs.clear()
            collision_locations.clear()
            penetration_depths.clear()
            total_impulse = 0.0

            def _semantic_body(actor_path, collider_path):
                candidates = [actor_path, collider_path]
                for robot_name, robot in getattr(task, "robots", {}).items():
                    root = robot.robot_prim_path.rstrip("/")
                    for path in candidates:
                        if path == root or path.startswith(root + "/"):
                            rel = path[len(root):].lstrip("/") or "root"
                            return "robot", f"{robot_name}/{rel}"

                for object_name, obj in getattr(task, "objects", {}).items():
                    root = obj.prim_path.rstrip("/")
                    for path in candidates:
                        if path == root or path.startswith(root + "/"):
                            if (object_name in self._human_surrogate_names or
                                (not self._entity_roles_explicit and "obstacle" in object_name.lower())):
                                return "human", object_name
                            if object_name in self._target_object_names or object_name.startswith("pick_"):
                                return "object", object_name
                            return "environment", object_name

                for fixture_name, fixture in getattr(task, "fixtures", {}).items():
                    root = fixture.prim_path.rstrip("/")
                    for path in candidates:
                        if path == root or path.startswith(root + "/"):
                            return "environment", fixture_name
                return None

            def _canonical_pair(first, second):
                if first is None or second is None:
                    return None
                kind_a, label_a = first
                kind_b, label_b = second
                if kind_a == kind_b and label_a == label_b:
                    return None

                kinds = {kind_a, kind_b}
                if kinds == {"object", "environment"}:
                    obj = first if kind_a == "object" else second
                    env = second if kind_a == "object" else first
                    return (
                        f"object/{obj[1]}", f"environment/{env[1]}",
                        1.0 if kind_a == "object" else -1.0,
                    )
                if kinds == {"object", "human"}:
                    obj = first if kind_a == "object" else second
                    human = second if kind_a == "object" else first
                    return (
                        f"object/{obj[1]}", f"obstacle/{human[1]}",
                        1.0 if kind_a == "object" else -1.0,
                    )
                if kinds == {"robot", "environment"}:
                    robot = first if kind_a == "robot" else second
                    env = second if kind_a == "robot" else first
                    return (
                        f"robot/{robot[1]}", f"environment/{env[1]}",
                        1.0 if kind_a == "robot" else -1.0,
                    )
                if kinds == {"robot", "human"}:
                    robot = first if kind_a == "robot" else second
                    human = second if kind_a == "robot" else first
                    return (
                        f"robot/{robot[1]}", f"obstacle/{human[1]}",
                        1.0 if kind_a == "robot" else -1.0,
                    )
                if kinds == {"robot", "object"}:
                    robot = first if kind_a == "robot" else second
                    obj = second if kind_a == "robot" else first
                    return (
                        f"robot/{robot[1]}", f"object/{obj[1]}",
                        1.0 if kind_a == "robot" else -1.0,
                    )
                if kind_a == kind_b == "robot":
                    first_label, second_label = sorted((label_a, label_b))
                    return (
                        f"robot/{first_label}", f"robot/{second_label}",
                        1.0 if label_a == first_label else -1.0,
                    )
                if kind_a == kind_b == "object":
                    first_label, second_label = sorted((label_a, label_b))
                    return (
                        f"object/{first_label}", f"object/{second_label}",
                        1.0 if label_a == first_label else -1.0,
                    )
                return None

            for report in reports:
                first = _semantic_body(report.get("actor0", ""), report.get("collider0", ""))
                second = _semantic_body(report.get("actor1", ""), report.get("collider1", ""))
                canonical = _canonical_pair(first, second)
                if canonical is None:
                    continue
                body_a, body_b, direction_sign = canonical

                impulses = np.asarray(report.get("impulses_ns", []), dtype=float)
                if impulses.size == 0:
                    continue
                impulses = impulses.reshape(-1, 3)
                impulses *= direction_sign
                impulse_vector = np.sum(impulses, axis=0)
                entry = aggregated_reports.setdefault((body_a, body_b), {
                    "impulse_vector_ns": np.zeros(3, dtype=float),
                    "contact_points": [],
                })
                entry["impulse_vector_ns"] += impulse_vector
                positions = report.get("positions_m", [])
                separations = report.get("separations_m", [])
                normals = report.get("normals", [])
                point_count = min(
                    len(impulses), len(positions), len(separations), len(normals)
                )
                for point_index in range(point_count):
                    point_impulse = impulses[point_index]
                    point_normal = np.asarray(normals[point_index], dtype=float) * direction_sign
                    entry["contact_points"].append({
                        "position_m": [float(value) for value in positions[point_index]],
                        "normal_from_bodyB_to_bodyA": [
                            float(value) for value in point_normal
                        ],
                        "separation_m": float(separations[point_index]),
                        "penetration_depth_m": max(0.0, -float(separations[point_index])),
                        "impulse_vector_ns": [float(value) for value in point_impulse],
                        "impulse_magnitude_ns": float(np.linalg.norm(point_impulse)),
                        "force_vector_n": [float(value / dt) for value in point_impulse],
                        "force_magnitude_n": float(np.linalg.norm(point_impulse) / dt),
                        "source_actor0": report.get("actor0"),
                        "source_actor1": report.get("actor1"),
                        "source_collider0": report.get("collider0"),
                        "source_collider1": report.get("collider1"),
                    })

            for (body_a, body_b), entry in aggregated_reports.items():
                impulse_ns = float(np.linalg.norm(entry["impulse_vector_ns"]))
                force_vector = entry["impulse_vector_ns"] / dt
                force_magnitude = impulse_ns / dt
                if force_magnitude <= 0.01:
                    continue
                contact_points = entry["contact_points"]
                positions = np.asarray(
                    [point["position_m"] for point in contact_points], dtype=float
                )
                location_m = (
                    [float(value) for value in np.mean(positions, axis=0)]
                    if positions.size else None
                )
                separations = np.asarray(
                    [point["separation_m"] for point in contact_points], dtype=float
                )
                penetration_m = (
                    float(max(0.0, -np.min(separations)))
                    if separations.size else None
                )
                collision_pairs.append({
                    "bodyA": body_a,
                    "bodyB": body_b,
                    "step": step_id,
                    "force_n": force_magnitude,
                    "force_vector_n": [float(value) for value in force_vector],
                    "impulse_ns": impulse_ns,
                    "impulse_vector_ns": [
                        float(value) for value in entry["impulse_vector_ns"]
                    ],
                    "source": "PhysX contact-report impulse / measured physics_dt",
                })
                collision_locations.append({
                    "bodyA": body_a,
                    "bodyB": body_b,
                    "location_m": location_m,
                    "contact_points_m": [
                        point["position_m"] for point in contact_points
                    ],
                    "num_contact_points": len(contact_points),
                })
                penetration_depths.append({
                    "bodyA": body_a,
                    "bodyB": body_b,
                    "depth_m": penetration_m,
                    "method": "physx_contact_separation",
                    "source": "PhysX contact-report callback",
                    "confidence": "high",
                    "num_contact_points": len(contact_points),
                })
                total_impulse += impulse_ns

        # Track contact events for duration computation.  Each pair is tracked
        # independently and every separated contact interval is emitted as its
        # own event.
        for pair in collision_pairs:
            key = (pair["bodyA"], pair["bodyB"])
            if key not in self._contact_state:
                self._contact_state[key] = {
                    "bodyA": pair["bodyA"],
                    "bodyB": pair["bodyB"],
                    "start_step": step_id,
                    "active": True,
                }
            elif not self._contact_state[key]["active"]:
                self._contact_state[key] = {
                    "bodyA": pair["bodyA"],
                    "bodyB": pair["bodyB"],
                    "start_step": step_id,
                    "active": True,
                }

        # End contacts that are no longer active
        active_keys = {(p["bodyA"], p["bodyB"]) for p in collision_pairs}
        for key in list(self._contact_state.keys()):
            if key not in active_keys and self._contact_state[key]["active"]:
                state = self._contact_state[key]
                state["active"] = False
                state["end_step"] = step_id
                duration = max(0.0, (state["end_step"] - state["start_step"]) * dt)
                self._data["contact_events"].append({
                    "bodyA": state["bodyA"],
                    "bodyB": state["bodyB"],
                    "start_step": state["start_step"],
                    "end_step": state["end_step"],
                    "duration_s": duration,
                })

        self._data["collision_pair_gt"].append(collision_pairs if collision_pairs else [])
        self._data["collision_location_gt"].append(collision_locations if collision_locations else [])
        self._data["penetration_depth_gt"].append(penetration_depths if penetration_depths else [])
        contact_impulses = []
        for pair in collision_pairs:
            force_n = float(pair.get("force_n", 0.0) or 0.0)
            contact_forces.append({
                "bodyA": pair["bodyA"],
                "bodyB": pair["bodyB"],
                "step": pair.get("step", step_id),
                "force_n": force_n,
                "force_vector_n": pair.get("force_vector_n"),
                "contact_points": [
                    {
                        "position_m": point["position_m"],
                        "normal_from_bodyB_to_bodyA": point[
                            "normal_from_bodyB_to_bodyA"
                        ],
                        "force_vector_n": point["force_vector_n"],
                        "force_magnitude_n": point["force_magnitude_n"],
                    }
                    for point in aggregated_reports.get(
                        (pair["bodyA"], pair["bodyB"]), {}
                    ).get("contact_points", [])
                ],
            })
            contact_impulses.append({
                "bodyA": pair["bodyA"],
                "bodyB": pair["bodyB"],
                "step": pair.get("step", step_id),
                "impulse_ns": float(pair.get("impulse_ns", force_n * dt)),
                "impulse_vector_ns": pair.get("impulse_vector_ns"),
                "contact_points": [
                    {
                        "position_m": point["position_m"],
                        "normal_from_bodyB_to_bodyA": point[
                            "normal_from_bodyB_to_bodyA"
                        ],
                        "impulse_vector_ns": point["impulse_vector_ns"],
                        "impulse_magnitude_ns": point["impulse_magnitude_ns"],
                    }
                    for point in aggregated_reports.get(
                        (pair["bodyA"], pair["bodyB"]), {}
                    ).get("contact_points", [])
                ],
            })
        self._data["contact_force_gt"].append(contact_forces if contact_forces else [])
        self._data["contact_impulse_gt"].append(contact_impulses if contact_impulses else [])

        # S-GRASP-001: Collect gripper-object contact force per arm.
        gripper_obj_force = {"left": 0.0, "right": 0.0}
        if self._contact_report_sub is not None:
            for pair in collision_pairs:
                body_a = str(pair.get("bodyA", ""))
                body_b = str(pair.get("bodyB", ""))
                # Contact reports are not guaranteed to keep the robot body
                # first. Also support FR3 panda_leftfinger/rightfinger
                # naming; the old matcher only handled /fl/ and /fr/.
                if body_a.startswith("robot/") and body_b.startswith("object/"):
                    robot_body = body_a
                elif body_b.startswith("robot/") and body_a.startswith("object/"):
                    robot_body = body_b
                else:
                    continue
                lower = robot_body.lower()
                if (
                    "/fl/" in lower
                    or "/left/" in lower
                    or "leftfinger" in lower
                    or "left_finger" in lower
                    or "leftgripper" in lower
                    or "left_gripper" in lower
                ):
                    gripper_obj_force["left"] += float(pair.get("force_n", 0.0) or 0.0)
                elif (
                    "/fr/" in lower
                    or "/right/" in lower
                    or "rightfinger" in lower
                    or "right_finger" in lower
                    or "rightgripper" in lower
                    or "right_gripper" in lower
                ):
                    gripper_obj_force["right"] += float(pair.get("force_n", 0.0) or 0.0)
        elif hasattr(task, 'pickcontact_views'):
            for robot_name, lr_dict in task.pickcontact_views.items():
                for lr_name, obj_dict in lr_dict.items():
                    for obj_name, contact_view in obj_dict.items():
                        try:
                            force_matrix = contact_view.get_contact_force_matrix(dt=dt)
                            if force_matrix is not None:
                                force_matrix = np.abs(force_matrix).squeeze()
                                force_magnitude = float(np.sum(force_matrix))
                                arm_key = "left" if "left" in lr_name.lower() or lr_name.lower().startswith("fl") else "right"
                                gripper_obj_force[arm_key] += force_magnitude
                        except Exception:
                            pass
        self._data["gripper_object_contact_force_gt"].append(gripper_obj_force)

    # ── Planner Data (S-PLAN-001, 003) ─────────────────────────────────────

    def _collect_planner_step(self, task, step_id: int) -> None:
        """Collect planner status per step from controllers."""
        safety_gate = "pass"
        command_sent = False

        if hasattr(task, 'robots'):
            for robot_name, robot in task.robots.items():
                # Check if robot has controllers with active plans
                for attr_name in dir(robot):
                    if 'controller' in attr_name.lower() or 'ctrl' in attr_name.lower():
                        try:
                            ctrl = getattr(robot, attr_name)
                            if hasattr(ctrl, 'num_plan_failed') and ctrl.num_plan_failed > 0:
                                safety_gate = "warning"
                        except Exception:
                            pass

        # Also check task skills for plan state
        if hasattr(task, '_skills') or hasattr(task, 'skills'):
            skills = getattr(task, 'skills', getattr(task, '_skills', None))
            if skills:
                for skill_list in skills.values() if isinstance(skills, dict) else []:
                    if isinstance(skill_list, list):
                        for skill in skill_list:
                            if hasattr(skill, 'controller') and skill.controller is not None:
                                ctrl = skill.controller
                                if getattr(ctrl, "cmd_plan", None) is not None:
                                    command_sent = True
        self._data["safety_gate_status"].append(safety_gate)
        self._data["low_level_command_sent"].append(command_sent)

    def record_low_level_command_sent(self, step_id: int, command_sent: bool) -> None:
        """Record whether the workflow actually submitted a joint command.

        Controller introspection is best-effort and differs between skill
        implementations.  A non-empty applied action dictionary is the
        authoritative runtime evidence for this field.
        """
        if not self._data["step_ids"] or self._data["step_ids"][-1] != step_id:
            raise ValueError("record_low_level_command_sent must follow collect_step")
        if not self._data["low_level_command_sent"]:
            raise ValueError("planner sample is missing for the current step")
        self._data["low_level_command_sent"][-1] = bool(command_sent)

    def capture_planned_trajectory(self, controller, arm_name: str = "") -> None:
        """Capture the planned trajectory from a controller's cmd_plan.

        Call this after a plan is generated (in ee_forward or plan_batch).

        Parameters
        ----------
        controller : TemplateController
            The controller with cmd_plan.
        arm_name : str
            Name of the arm (left/right) for labeling.
        """
        if controller.cmd_plan is None:
            return

        try:
            plan = controller.cmd_plan
            waypoints = []
            for i in range(len(plan)):
                wp = plan[i]
                pos = wp.position.cpu().numpy().tolist() if hasattr(wp.position, 'cpu') else list(wp.position)
                waypoints.append({
                    "step": i,
                    "joint_positions": pos,
                    "arm": arm_name,
                })

            if waypoints:
                from safety_risk.pt_runtime_evidence import planner_world_snapshot
                self._data["planned_trajectory"].append({
                    "arm": arm_name,
                    "n_waypoints": len(waypoints),
                    "waypoints": waypoints,
                    "trajectory_clock": dict(getattr(controller, "_last_trajectory_sample", {}) or {}),
                    "command_frame_index": len(self._data.get("step_ids", [])),
                    "planner_world": planner_world_snapshot(controller),
                })
        except Exception as e:
            logger.warning("Failed to capture planned trajectory: %s", e)

    # ── Distances (S-DIST-004, 005, 006) ───────────────────────────────────

    def _bound_pick_target_id(self, task):
        online = getattr(self, "_online_iso", None)
        metadata = getattr(online, "metadata", {}) or {}
        return resolve_pick_target_id(
            getattr(self, "_target_object_names", []) if getattr(self, "_target_objects_declared", False) else None,
            metadata, getattr(task, "objects", {}), getattr(task, "robots", {}))

    def set_contact_trace_context661(self, environment_id=None, episode_id=None,
                                     global_step=None, local_step=None):
        """Bind actual pending physics step; call without arguments in warmup."""
        if global_step is None:
            self._contact_trace_context661 = None
            return
        if not isinstance(global_step, int) or not isinstance(local_step, int) or global_step < 0 or local_step < 0:
            raise ValueError("contact trace needs explicit nonnegative global/local physics steps")
        self._contact_trace_context661 = {
            "environment_id": str(environment_id), "episode_id": str(episode_id),
            "global_physics_step": global_step, "local_physics_step": local_step}

    def configure_geometry_backend661(self, backend="cpu_reference", transform_provider=None, *, transform_source="usd_double"):
        """Select the geometry arithmetic without changing distance semantics.

        Call before the first sample, or when creating a new episode collector.
        USD double poses are the default; tensor_float32 is an explicit opt-in.
        """
        if backend not in {"cpu_reference", "cpu_cached", "gpu"}:
            raise ValueError("Unknown geometry backend: " + str(backend))
        if transform_source not in {"usd_double", "tensor_float32"}:
            raise ValueError("Unknown geometry transform source: " + str(transform_source))
        self._geometry_backend661 = backend
        self._geometry_adapter661 = None
        if backend != "cpu_reference":
            from .batched_geometry661 import CachedUsdGeometry661
            self._geometry_adapter661 = CachedUsdGeometry661(backend, transform_provider, transform_source=transform_source)
        self._data["geometry_backend661"] = backend
        self._data["geometry_transform_source661"] = transform_source if backend != "cpu_reference" else "usd_double"

    def invalidate_geometry_cache661(self):
        adapter = getattr(self, "_geometry_adapter661", None)
        if adapter is not None:
            adapter.invalidate()

    def compare_geometry_reference661(self, task, absolute_tolerance=1e-10, *, use_collected_frame=False):
        """Read-only same-frame channel comparison for short validation windows.

        No samples are appended to the episode. Numerical discrepancies and
        missing/category/path discrepancies are reported, never relabelled.
        """
        fields = ("robot_human_distance_matrix_gt", "ee_human_distance_gt",
            "object_human_distance_gt", "target_object_linear_velocity_gt",
            "object_env_distance_gt", "link_env_distance_gt",
            "link_target_mesh_aabb_distance_gt", "link_target_projected_clearance_gt",
            "pt_environment_projected_clearance_gt", "self_distance_gt",
            "distance_coverage_status", "pt_distance_geometry_status",
            "pick_target_binding_status", "pt_distance_link_key_frame")
        original_data = self._data
        original_backend = getattr(self, "_geometry_backend661", "cpu_reference")
        original_adapter = getattr(self, "_geometry_adapter661", None)
        results = []
        frame_index = None
        if use_collected_frame:
            steps = original_data.get("step_ids", [])
            if not steps or steps != list(range(len(steps))):
                raise ValueError("actual geometry audit requires a completed contiguous physical sample")
            frame_index = len(steps) - 1
            samples = fields[:10]
            if any(not isinstance(original_data.get(field), list)
                   or len(original_data[field]) != len(steps) for field in samples):
                raise ValueError("actual geometry audit requires all ten recorded channels at this physical sample")
            if any(field not in original_data for field in fields[10:]):
                raise ValueError("actual geometry audit requires the four current-frame metadata fields")
            # Capture the real collected GPU/cross-env output before running
            # the reference. Never certify a recomputed shadow as that output.
            results.append({field: [deepcopy(original_data[field][-1])] if field in samples
                            else deepcopy(original_data[field]) for field in fields})
        try:
            for reference in ((True,) if use_collected_frame else (False, True)):
                self._data = dict(original_data)
                self._data.update({field: [] for field in fields})
                if reference:
                    self._geometry_backend661 = "cpu_reference"
                    self._geometry_adapter661 = None
                self._collect_distances(task)
                results.append({field: self._data[field] for field in fields})
        finally:
            self._data = original_data
            self._geometry_backend661 = original_backend
            self._geometry_adapter661 = original_adapter
        differences = []
        def compare(a, b, path):
            if isinstance(a, dict) and isinstance(b, dict):
                if set(a) != set(b):
                    differences.append({"path": path, "reason": "different_keys", "optimized": sorted(a), "reference": sorted(b)})
                for key in sorted(set(a) & set(b)):
                    compare(a[key], b[key], path + "/" + str(key))
            elif isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
                if len(a) != len(b):
                    differences.append({"path": path, "reason": "different_lengths"})
                for i, (x, y) in enumerate(zip(a, b)):
                    compare(x, y, path + "/" + str(i))
            elif isinstance(a, (bool, np.bool_)) or isinstance(b, (bool, np.bool_)):
                if (not isinstance(a, (bool, np.bool_)) or not isinstance(b, (bool, np.bool_))
                        or bool(a) != bool(b)):
                    differences.append({"path": path, "reason": "validity", "optimized": a, "reference": b})
            elif isinstance(a, (float, int)) and isinstance(b, (float, int)):
                if not np.isclose(a, b, atol=absolute_tolerance, rtol=0, equal_nan=True):
                    differences.append({"path": path, "reason": "numeric", "optimized": a, "reference": b})
            elif a != b:
                differences.append({"path": path, "reason": "value", "optimized": a, "reference": b})
        compare(results[0], results[1], "")
        return {"schema": "same-frame-geometry-comparison661-v2",
                "backend": original_backend, "absolute_tolerance": absolute_tolerance,
                "optimized_source": "actual_recorded_frame" if use_collected_frame else "same_frame_recomputed",
                "frame_index": frame_index,
                "source_step_id": original_data["step_ids"][frame_index] if frame_index is not None else None,
                "source_clock": deepcopy(getattr(self, "_contact_trace_context661", None)) if use_collected_frame else None,
                "matches": not differences, "differences": differences,
                "optimized": results[0], "reference": results[1]}

    def _collect_distances(self, task) -> None:
        """Collect six distance fields from live collider world bounds.

        Distances are between collider *surfaces*, not prim origins.  Isaac Sim
        4.5 does not expose a general separated-shape distance query, so this
        uses the Euclidean clearance between each collider's world AABB.  It is
        exact for axis-aligned box colliders and a conservative lower bound for
        rotated/curved/mesh colliders.  The method is recorded in provenance.
        """
        from pxr import Gf, Usd, UsdGeom, UsdPhysics
        import omni.usd

        target_id = self._bound_pick_target_id(task)
        self._data["pick_target_binding_status"] = {"target_object_id": target_id, "status": "bound" if target_id is not None else "unknown"}
        stage = scoped_stage636(task, omni.usd.get_context().get_stage())
        if stage is None:
            raise RuntimeError("USD stage is unavailable")

        if not hasattr(self, "_geometry_backend661"):
            self.configure_geometry_backend661(os.environ.get("SAFEBENCH_GEOMETRY_BACKEND661", "cpu_reference"))
        geometry_adapter = getattr(self, "_geometry_adapter661", None)
        if geometry_adapter is not None:
            geometry_adapter.begin_frame(stage)
        stage_prims = geometry_adapter.prims if geometry_adapter is not None else None

        def _root_path(value):
            for attr in ("prim_path", "_prim_path"):
                path = getattr(value, attr, None)
                if path:
                    return str(path).rstrip("/")
            return None

        online = getattr(self, "_online_iso", None)
        evaluator = getattr(online, "evaluator", None)
        link_key_frame = (getattr(evaluator, "profile", {}) or {}).get(
            "pt_hs_inspired", {}).get("link_key_frame")
        self._data["pt_distance_link_key_frame"] = link_key_frame or "legacy_articulation_root"
        robot_roots = {
            name: robot_distance_root(robot, link_key_frame)
            for name, robot in getattr(task, "robots", {}).items()
            if robot_distance_root(robot, link_key_frame)
        }
        object_roots = {
            name: _root_path(obj)
            for name, obj in getattr(task, "objects", {}).items()
            if _root_path(obj)
        }
        configured_targets = set(self._target_object_names)
        target_roots = {
            name: path for name, path in object_roots.items()
            if name in configured_targets or str(name).startswith("pick_object")
        }
        human_roots = {
            name: path for name, path in object_roots.items()
            if name in set(self._human_surrogate_names) or (
                not self._entity_roles_explicit and str(name).startswith("obstacle"))
        }
        if geometry_adapter is None:
            compound_human_roots = set()
            for human_root in human_roots.values():
                for candidate in (stage_prims if stage_prims is not None else stage.Traverse()):
                    candidate_path = str(candidate.GetPath()).lower()
                    if (
                        candidate_path.startswith(human_root.lower().rstrip("/") + "/")
                        and "/humanskeletonphysics/" in candidate_path
                        and candidate.GetTypeName() in {
                            "Capsule", "Sphere", "Mesh", "Cube", "Cylinder"
                        }
                    ):
                        compound_human_roots.add(human_root)
                        break
        # Articulation tasks often name an internal joint/link in the skill
        # (for example close_v_left) rather than the object wrapper.  If no
        # configured target resolves to a task object, treat every physical
        # non-human task object as an intended object instead of silently
        # producing an empty object-distance field.
        if not target_roots:
            target_roots = {
                name: path for name, path in object_roots.items()
                if name not in human_roots
                and "background" not in str(name).lower()
            }

        def _under(path, root):
            return path == root or path.startswith(root + "/")

        def _absolute_robot_path(robot_root, configured_path):
            value = str(configured_path or "").strip()
            if not value:
                return ""
            if value.startswith("/"):
                return value.rstrip("/")
            if not robot_root:
                return value
            candidate = f"{robot_root.rstrip('/')}/{value.lstrip('/')}"
            if stage.GetPrimAtPath(candidate).IsValid():
                return candidate
            # Several robot configs include an asset-internal prefix which is
            # already present below robot_root. Match the longest valid suffix.
            parts = value.split("/")
            for start in range(1, len(parts)):
                candidate = f"{robot_root.rstrip('/')}/{'/'.join(parts[start:])}"
                if stage.GetPrimAtPath(candidate).IsValid():
                    return candidate
            return f"{robot_root.rstrip('/')}/{value.lstrip('/')}"

        def _rigid_owner(prim):
            if geometry_adapter is not None:
                return geometry_adapter.rigid_owner(prim)
            current = prim
            while current and current.IsValid():
                if current.HasAPI(UsdPhysics.RigidBodyAPI):
                    return str(current.GetPath())
                current = current.GetParent()
            return str(prim.GetPath())

        bbox_cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.proxy, UsdGeom.Tokens.render, UsdGeom.Tokens.guide],
            useExtentsHint=True,
        )
        xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        groups = {"robot": {}, "human": {}, "object": {}, "environment": {}}
        target_mesh_groups = {"robot": {}, "object": {}, "environment": {}}
        pt_pairs = self._data.get("pt_environment_distance_pairs", [])
        pt_env_ids = {p["object_id"] for p in pt_pairs}
        # Count all enabled colliders before bounds/mesh reads can fail.
        pt_collider_paths = {"robot": {}, "object": {}, "environment": {}}
        mesh_world_points = {}
        collider_geometry = {}
        unsupported_geometry = {}
        non_shape_declarations = set()
        group_roots = {}
        if not hasattr(self, "_collider_local_bounds"):
            self._collider_local_bounds = {}
        if not hasattr(self, "_collider_local_mesh_points"):
            self._collider_local_mesh_points = {}

        def _finite_bounds(minimum, maximum):
            size = maximum - minimum
            return bool(
                minimum.shape == (3,)
                and maximum.shape == (3,)
                and np.all(np.isfinite(minimum))
                and np.all(np.isfinite(maximum))
                and np.all(size >= 0.0)
                and np.all(size < 100.0)
                and np.all(np.abs(minimum) < 1e6)
                and np.all(np.abs(maximum) < 1e6)
            )

        def _world_bounds(prim):
            if geometry_adapter is not None:
                return geometry_adapter.world_bounds(prim, _world_bounds_reference, xform_cache)
            return _world_bounds_reference(prim)

        def _world_bounds_reference(prim):
            try:
                aligned = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
                minimum = np.asarray(aligned.GetMin(), dtype=float)
                maximum = np.asarray(aligned.GetMax(), dtype=float)
                if _finite_bounds(minimum, maximum):
                    return minimum, maximum
            except Exception:
                pass

            # Some referenced collision meshes have no valid authored extent,
            # while their points are present. Cache-free local point bounds +
            # eight transformed corners avoids the invalid FLT_MAX world box.
            if prim.IsA(UsdGeom.Mesh):
                try:
                    prim_path = str(prim.GetPath())
                    local_bounds = self._collider_local_bounds.get(prim_path)
                    if local_bounds is None:
                        points = UsdGeom.Mesh(prim).GetPointsAttr().Get() or []
                        if points:
                            local = np.asarray(points, dtype=float)
                            local_bounds = (local.min(axis=0), local.max(axis=0))
                            self._collider_local_bounds[prim_path] = local_bounds
                    if local_bounds is not None:
                        local_min, local_max = local_bounds
                        matrix = xform_cache.GetLocalToWorldTransform(prim)
                        corners = []
                        for x in (local_min[0], local_max[0]):
                            for y in (local_min[1], local_max[1]):
                                for z in (local_min[2], local_max[2]):
                                    value = matrix.Transform(Gf.Vec3d(float(x), float(y), float(z)))
                                    corners.append([float(value[0]), float(value[1]), float(value[2])])
                        corners = np.asarray(corners, dtype=float)
                        minimum, maximum = corners.min(axis=0), corners.max(axis=0)
                        if _finite_bounds(minimum, maximum):
                            return minimum, maximum
                except Exception:
                    pass
            return None

        def _support_interval(geometry, axis):
            """Exact support interval of authored vertices/analytic primitives."""
            kind = geometry["kind"]
            if kind == "Mesh":
                projected = geometry["points"] @ axis
                return float(projected.min()), float(projected.max())
            if kind == "compound":
                intervals = [_support_interval(part, axis) for part in geometry["parts"]]
                return min(v[0] for v in intervals), max(v[1] for v in intervals)
            direction = geometry["linear"] @ axis  # Gf uses row-vector transforms.
            center = float(geometry["center"] @ axis)
            if kind == "Cube":
                radius = geometry["half_size"] * np.abs(direction).sum()
            elif kind == "Sphere":
                radius = geometry["radius"] * np.linalg.norm(direction)
            else:
                axial = float(direction[geometry["axis"]])
                radial = float(np.linalg.norm(np.delete(direction, geometry["axis"])))
                radius = geometry["height"] * .5 * abs(axial)
                if kind == "Capsule":
                    radius += geometry["radius"] * np.linalg.norm(direction)
                elif kind == "Cylinder":
                    radius += geometry["radius"] * radial
                else:
                    raise ValueError("unsupported analytic primitive")
            return center - float(radius), center + float(radius)

        def _geometry_bounds(geometry):
            intervals = [_support_interval(geometry, axis) for axis in np.eye(3)]
            minimum = np.asarray([v[0] for v in intervals])
            maximum = np.asarray([v[1] for v in intervals])
            return (minimum, maximum) if _finite_bounds(minimum, maximum) else None

        def _authored_geometry(prim):
            """No broad-box substitute; unsupported geometry leaves coverage unknown."""
            path = str(prim.GetPath())
            kind = prim.GetTypeName()
            if prim.IsA(UsdGeom.Mesh):
                points = self._collider_local_mesh_points.get(path)
                if points is None:
                    points = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get() or [], dtype=float)
                    if points.ndim != 2 or points.shape[1] != 3 or not len(points):
                        raise ValueError("mesh_without_vertices")
                    self._collider_local_mesh_points[path] = points
                matrix = np.asarray(xform_cache.GetLocalToWorldTransform(prim), dtype=float)
                transformed = points @ matrix[:3, :3] + matrix[3, :3]
                if not np.all(np.isfinite(transformed)):
                    raise ValueError("nonfinite_mesh_vertices")
                mesh_world_points[path] = transformed
                return {"kind": "Mesh", "points": transformed, "center": transformed.mean(axis=0)}
            if kind in {"Cube", "Sphere", "Capsule", "Cylinder"}:
                matrix = np.asarray(xform_cache.GetLocalToWorldTransform(prim), dtype=float)
                if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)) or not np.allclose(matrix[:, 3], [0, 0, 0, 1]):
                    raise ValueError("nonfinite_or_nonaffine_primitive_transform")
                linear = matrix[:3, :3]
                gram = linear @ linear.T
                scales = np.sqrt(np.diag(gram))
                if np.any(scales < 1e-12) or not np.allclose(gram, np.diag(scales ** 2), atol=1e-10, rtol=1e-6):
                    raise ValueError("degenerate_or_sheared_primitive_transform")
                # Physics engines do not generally cook ellipsoids from a
                # nonuniformly scaled Sphere/Capsule. Do not infer their shape.
                if kind != "Cube" and not np.allclose(scales, scales[0], atol=1e-9, rtol=1e-6):
                    raise ValueError("nonuniform_analytic_primitive_scale_unqualified")
                schema = getattr(UsdGeom, kind)(prim)
                geometry = {"kind": kind, "linear": linear, "center": matrix[3, :3]}
                if kind == "Cube":
                    geometry["half_size"] = float(schema.GetSizeAttr().Get()) * .5
                    dimensions = [geometry["half_size"]]
                else:
                    geometry["radius"] = float(schema.GetRadiusAttr().Get())
                    dimensions = [geometry["radius"]]
                    if kind != "Sphere":
                        axis = str(schema.GetAxisAttr().Get())
                        if axis not in {"X", "Y", "Z"}:
                            raise ValueError("invalid_primitive_axis")
                        geometry.update(axis="XYZ".index(axis), height=float(schema.GetHeightAttr().Get()))
                        dimensions.append(geometry["height"])
                if not all(np.isfinite(v) and v > 0 for v in dimensions):
                    raise ValueError("invalid_primitive_dimensions")
                return geometry
            if kind in {"Xform", "Scope"}:
                descendants = list(Usd.PrimRange(prim, Usd.TraverseInstanceProxies()))
                # Missing references and unloaded payloads are not empty shapes.
                if any(not child.IsLoaded() or child.GetPrimIndex().localErrors for child in descendants):
                    raise ValueError("unloaded_or_composition_error_in_collider")
                shapes = [child for child in descendants if child.IsA(UsdGeom.Boundable)]
                if not shapes:
                    # A fully composed, geometry-free declaration is not a
                    # cookable collision shape (observed Genie center visuals).
                    non_shape_declarations.add(path)
                    return None
                parts = [_authored_geometry(child) for child in shapes]
                geometry = {"kind": "compound", "parts": parts}
                bounds = _geometry_bounds(geometry)
                if bounds is None:
                    raise ValueError("invalid_compound_geometry")
                geometry["center"] = (bounds[0] + bounds[1]) * .5
                return geometry
            raise ValueError("unsupported_collision_shape:" + str(kind))

        def _collider_geometry_world_bounds(prim):
            """Tight authored shape AABB, including instance-proxy descendants."""
            try:
                path = str(prim.GetPath())
                if geometry_adapter is not None:
                    bounds, geometry, error = geometry_adapter.authored_bounds(prim)
                    if path in geometry_adapter.non_shapes:
                        non_shape_declarations.add(path)
                    if error:
                        unsupported_geometry[path] = error
                    if geometry is not None:
                        collider_geometry[path] = geometry
                    return bounds
                geometry = _authored_geometry(prim)
                if geometry is None:
                    return None
                # Preserve the original mesh min/max arithmetic exactly.
                if geometry["kind"] == "Mesh":
                    points = geometry["points"]
                    bounds = points.min(axis=0), points.max(axis=0)
                    if not _finite_bounds(*bounds):
                        raise ValueError("invalid_mesh_bounds")
                else:
                    bounds = _geometry_bounds(geometry)
                    if bounds is None:
                        raise ValueError("invalid_analytic_bounds")
                collider_geometry[path] = geometry
                return bounds
            except Exception as exc:
                unsupported_geometry[str(prim.GetPath())] = str(exc) if isinstance(exc, ValueError) else type(exc).__name__
                return None

        if geometry_adapter is not None:
            profile_pairs = (getattr(evaluator, "profile", {}) or {}).get("pt_hs_inspired", {}).get("pairs", [])
            phase_trace = self._data.get("skill_phase_trace", [])
            allowed_context = (
                (phase_trace[-1].get("skill"), phase_trace[-1].get("phase")) if phase_trace else (),
                tuple((p.get("actor"), p.get("object_id"), tuple(p.get("allowed_grip_link_paths") or ()),
                       tuple(p.get("allowed_grip_phases") or ())) for p in profile_pairs))
            inventory = geometry_adapter.collision_inventory661(robot_roots, human_roots,
                target_roots, target_id, pt_env_ids, allowed_context)
            geometry_adapter.prepare_authored(inventory["authored_prims"], xform_cache)
            self._data["geometry_cache_rebuilds661"] = geometry_adapter.rebuild_count
            self._data["geometry_inventory_rebuilds661"] = {
                "metadata": geometry_adapter._inventory661.metadata_rebuilds,
                "roles": geometry_adapter._inventory661.role_rebuilds}
            discovery_items = inventory["entries"]
            visible_robot_owners = set(inventory["visible_robot_owners"])
        else:
            discovery_items = stage.Traverse()
            visible_robot_owners = set()
        for item in discovery_items:
            if geometry_adapter is not None:
                prim, path, owner = item.prim, item.path, item.owner
                category, root_name, root_path = item.category, item.root_name, item.root_path
                label, pt_relevant = item.label, item.pt_relevant
            else:
                prim = item
                path = str(prim.GetPath())
                if (prim.IsA(UsdGeom.Mesh)
                        and any(_under(path, root) for root in robot_roots.values())
                        and UsdGeom.Imageable(prim).ComputeVisibility() != UsdGeom.Tokens.invisible):
                    visible_robot_owners.add(_rigid_owner(prim))
                if not prim.HasAPI(UsdPhysics.CollisionAPI):
                    continue
                enabled = UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
                if enabled is False:
                    continue
                owner = _rigid_owner(prim)
                category = "environment"
                root_name = None
                root_path = None

                for name, candidate in robot_roots.items():
                    if _under(path, candidate):
                        category, root_name, root_path = "robot", name, candidate
                        break
                if category == "environment":
                    for name, candidate in human_roots.items():
                        if _under(path, candidate):
                            category, root_name, root_path = "human", name, candidate
                            break
                if category == "human" and (
                    path == root_path
                    or prim.GetName() == root_name
                ) and root_path in human_roots.values():
                    # The compound root owns the PhysX body, but the 23 shape
                    # colliders below it are the reportable human body parts.
                    continue
                if category == "environment":
                    for name, candidate in target_roots.items():
                        if _under(path, candidate):
                            category, root_name, root_path = "object", name, candidate
                            break

                if category == "robot":
                    label = f"{root_name}/{owner[len(root_path):].strip('/') or prim.GetName()}"
                elif category == "human":
                    # RoboSafe uses one rigid root with shape-level colliders.
                    # Label each HumanSkeletonPhysics shape independently instead
                    # of collapsing all 23 parts into the root body name.
                    if "/humanskeletonphysics/" in path.lower():
                        label = f"{root_name}/{prim.GetName()}"
                    else:
                        label = f"{root_name}/{owner[len(root_path):].strip('/') or prim.GetName()}"
                elif category == "object":
                    label = root_name
                else:
                    label = owner

                pt_relevant = (category == "robot" or
                    (category == "object" and root_name == target_id) or
                    (category == "environment" and any(x in path.split("/") for x in pt_env_ids)))
            tight_bounds = _collider_geometry_world_bounds(prim) if pt_relevant else None
            if path in non_shape_declarations:
                continue
            if pt_relevant:
                pt_collider_paths[category].setdefault(label, []).append(path)
            bounds = _world_bounds(prim)
            if bounds is None:
                continue
            minimum, maximum = bounds
            groups[category].setdefault(label, []).append((minimum, maximum, path))
            group_roots[label] = owner
            if pt_relevant:
                if tight_bounds is not None:
                    target_mesh_groups[category].setdefault(label, []).append((*tight_bounds, path))

        # The supplied MANO asset currently authors RigidBodyAPI on its 21 body
        # parts but does not author CollisionAPI on the empty ``collisions``
        # Xforms. Use each rigid body's resolved geometry bound so distances are
        # still per palm/finger link instead of falling back to obstacle_1 root.
        mano_proxy_count = 0
        if geometry_adapter is not None:
            fallback_items = ((f.prim, f.path, f.label, f.proxy_radius_m) for f in inventory["hand_fallback"])
        else:
            def reference_hand_fallback():
                for obstacle_name, obstacle_root in human_roots.items():
                    for prim in (stage_prims if stage_prims is not None else stage.Traverse()):
                        path = str(prim.GetPath())
                        if not _under(path, obstacle_root):
                            continue
                        # RoboSafe uses a single rigid root to own all of the
                        # HumanSkeletonPhysics shapes. The root itself is not a
                        # reportable body part; including it here would create a
                        # duplicate obstacle_1/obstacle_1 distance entry.
                        if path.rstrip("/") == obstacle_root.rstrip("/"):
                            continue
                        # Real collider shapes were already registered above. Do not
                        # invent a second MANO fallback entry for a named Palm collider.
                        if prim.HasAPI(UsdPhysics.CollisionAPI):
                            continue
                        name = prim.GetName().lower()
                        named_hand_part = bool(re.match(
                            r"^(?:mano|palm|wrist|thumb|index|middle|ring|pinky)(?:[_0-9].*)?$",
                            name,
                        ))
                        if not (
                            prim.HasAPI(UsdPhysics.RigidBodyAPI)
                            or (named_hand_part and prim.IsA(UsdGeom.Xformable))
                        ):
                            continue
                        relative = path[len(obstacle_root):].strip("/") or prim.GetName()
                        label = f"{obstacle_name}/{relative}"
                        yield prim, path, label, .035 if prim.GetName() == "palm" else .012
            fallback_items = reference_hand_fallback()
        for prim, path, label, proxy_radius_m in fallback_items:
            if label in groups["human"]:
                continue
            bounds = _world_bounds(prim)
            if bounds is None:
                # The asset has unresolved visuals for several 1y links.
                # Preserve complete 21-link coverage with a documented
                # conservative body-centred proxy rather than root distance.
                matrix = xform_cache.GetLocalToWorldTransform(prim)
                center = np.asarray(matrix.ExtractTranslation(), dtype=float)
                radius = proxy_radius_m
                bounds = center - radius, center + radius
                mano_proxy_count += 1
            groups["human"][label] = [(bounds[0], bounds[1], path)]
            group_roots[label] = path

        # A nonempty distance matrix alone does not prove coverage: some robot
        # assets contain visible rigid links without any collision shape.
        robot_collider_owners = set(group_roots[name] for name in groups["robot"])
        uncovered_robot_bodies = sorted(visible_robot_owners - robot_collider_owners)
        self._data["distance_coverage_status"] = {
            "method": "collider_world_aabb_surface_clearance",
            "robot_body_count": len(groups["robot"]),
            "human_body_count": len(groups["human"]),
            "object_count": len(groups["object"]),
            "environment_body_count": len(groups["environment"]),
            "mano_proxy_body_count": mano_proxy_count,
            "robot_human_complete": bool(groups["robot"] and groups["human"] and not uncovered_robot_bodies),
            "robot_visual_rigid_bodies_without_collider": uncovered_robot_bodies,
        }
        self._data["pt_distance_geometry_status"] = {
            "method": "authored_mesh_vertices_and_analytic_support_v598",
            "geometry_count": len(collider_geometry),
            "unsupported_geometry": unsupported_geometry,
            "non_shape_collision_declarations": sorted(non_shape_declarations),
            "non_shape_reason": "fully_loaded_composition_without_boundable_shape",
            "not_exact_cooked_physx_distance": True,
        }
        self._data["distance_coverage_status"]["pt_geometry"] = deepcopy(self._data["pt_distance_geometry_status"])

        if geometry_adapter is not None:
            geometry_adapter.prepare_pairs(groups, target_mesh_groups, target_id, pt_pairs)

        def _aabb_clearance(a, b):
            if geometry_adapter is not None:
                return geometry_adapter.aabb_clearance(a, b)
            delta = np.maximum(np.maximum(b[0] - a[1], a[0] - b[1]), 0.0)
            return float(np.linalg.norm(delta))

        def _group_clearance(a_boxes, b_boxes):
            return min(
                _aabb_clearance(a_box, b_box)
                for a_box in a_boxes for b_box in b_boxes
            )

        robot_human = {}
        for human_name, human_boxes in sorted(groups["human"].items()):
            robot_human[human_name] = {
                robot_name: _group_clearance(robot_boxes, human_boxes)
                for robot_name, robot_boxes in sorted(groups["robot"].items())
            }

        ee_human = {}
        for robot_name, robot in getattr(task, "robots", {}).items():
            for arm, attr in (("left", "fl_ee_path"), ("right", "fr_ee_path")):
                ee_path = _absolute_robot_path(
                    robot_roots.get(robot_name, ""), getattr(robot, attr, "")
                )
                ee_groups = [
                    (name, boxes) for name, boxes in groups["robot"].items()
                    if ee_path and (
                        _under(ee_path, group_roots.get(name, ""))
                        or _under(group_roots.get(name, ""), ee_path)
                    )
                ]
                for human_name, human_boxes in sorted(groups["human"].items()):
                    if ee_groups:
                        ee_human[f"{arm}→{human_name}"] = min(
                            _group_clearance(boxes, human_boxes)
                            for _, boxes in ee_groups
                        )

        object_human = {}
        for object_name, object_boxes in sorted(groups["object"].items()):
            object_human[object_name] = {
                human_name: _group_clearance(object_boxes, human_boxes)
                for human_name, human_boxes in sorted(groups["human"].items())
            }

        object_env = {}
        for object_name, object_boxes in sorted(groups["object"].items()):
            object_env[object_name] = {
                env_name: _group_clearance(object_boxes, env_boxes)
                for env_name, env_boxes in sorted(groups["environment"].items())
            }

        link_env = {}
        for robot_name, robot_boxes in sorted(groups["robot"].items()):
            link_env[robot_name] = {
                env_name: _group_clearance(robot_boxes, env_boxes)
                for env_name, env_boxes in sorted(groups["environment"].items())
            }
            # Keep target-cup distance per robot link.  The target is in the
            # movable-object group, so the old link-env matrix omitted it.
            for object_name, object_boxes in sorted(groups["object"].items()):
                if target_id is not None and object_name == target_id:
                    link_env[robot_name][object_name] = _group_clearance(robot_boxes, object_boxes)

        # Preserve the legacy field name, now including analytic shapes.
        # Coverage checks all enabled, cookable declarations, not only those
        # whose legacy BBox lookup succeeded. Missing geometry stays missing.
        target_mesh_distances = {}
        target_boxes = target_mesh_groups["object"].get(target_id)
        if target_boxes and {b[2] for b in target_boxes} == set(pt_collider_paths["object"].get(target_id, [])):
            for robot_name, boxes in sorted(groups["robot"].items()):
                tight_boxes = target_mesh_groups["robot"].get(robot_name)
                if tight_boxes and {b[2] for b in tight_boxes} == set(pt_collider_paths["robot"].get(robot_name, [])):
                    target_mesh_distances.setdefault(robot_name, {})[target_id] = _group_clearance(tight_boxes, target_boxes)

        if set(target_mesh_distances) != set(pt_collider_paths["robot"]):
            target_mesh_distances = {}

        # Face-free, conservative 1-D separation along the vector between
        # mesh centroids or analytic/compound centers. A positive gap cannot exceed the
        # real 3-D separation. Combine it with the mesh AABB lower bound;
        # keep both earlier channels for audit and comparison.
        def _centroid_projection_gap(points_a, points_b):
            axis = points_b.mean(axis=0) - points_a.mean(axis=0)
            length = float(np.linalg.norm(axis))
            if not np.isfinite(length) or length < 1e-12:
                return 0.0
            axis /= length
            projection_a = points_a @ axis
            projection_b = points_b @ axis
            return max(0.0, float(projection_b.min() - projection_a.max()),
                       float(projection_a.min() - projection_b.max()))

        def _geometry_projection_gap(path_a, path_b):
            if geometry_adapter is not None:
                return geometry_adapter.projection_gap(path_a, path_b)
            a, b = collider_geometry[path_a], collider_geometry[path_b]
            if a["kind"] == b["kind"] == "Mesh":
                return _centroid_projection_gap(a["points"], b["points"])
            axis = b["center"] - a["center"]
            length = float(np.linalg.norm(axis))
            if not np.isfinite(length) or length < 1e-12:
                return 0.0
            axis /= length
            lo_a, hi_a = _support_interval(a, axis)
            lo_b, hi_b = _support_interval(b, axis)
            return max(0.0, lo_b - hi_a, lo_a - hi_b)

        projected_target_distances = {}
        if target_mesh_distances:
            for robot_name, boxes in sorted(target_mesh_groups["robot"].items()):
                pair_bounds = []
                for robot_box in boxes:
                    for cup_box in target_boxes:
                        a = collider_geometry.get(robot_box[2])
                        b = collider_geometry.get(cup_box[2])
                        if a is None or b is None:
                            pair_bounds = []
                            break
                        broad = _aabb_clearance(robot_box, cup_box)
                        pair_bounds.append(max(broad, _geometry_projection_gap(robot_box[2], cup_box[2])))
                    if not pair_bounds:
                        break
                if pair_bounds:
                    projected_target_distances.setdefault(robot_name, {})[target_id] = min(pair_bounds)
        if set(projected_target_distances) != set(pt_collider_paths["robot"]):
            projected_target_distances = {}

        # Retain legacy AABB channels. New values are conservative bounds on the
        # authored collider mesh/convex hull, analytic primitive or instance
        # subgeometry, not exact cooked PhysX distance.
        environment_projected = []
        for declaration in pt_pairs:
            actor, object_id = declaration["actor"], declaration["object_id"]
            category = "robot" if actor == "robot" else "object"
            sources = pt_collider_paths[category]
            if actor == "held_object":
                sources = {k: v for k, v in sources.items() if k == target_id}
            destinations = {k: v for k, v in pt_collider_paths["environment"].items()
                            if object_id in k.split("/")}
            missing = []
            if not sources or not destinations:
                missing.append("missing_declared_collision_geometry")
            if actor == "robot" and uncovered_robot_bodies:
                missing.append("robot_visual_body_without_collider")
            source_boxes = {}
            for label, paths in sources.items():
                boxes = target_mesh_groups[category].get(label, [])
                if {b[2] for b in boxes} != set(paths):
                    missing.append("incomplete_source_mesh:" + label)
                source_boxes[label] = boxes
            destination_boxes = []
            for label, paths in destinations.items():
                boxes = target_mesh_groups["environment"].get(label, [])
                if {b[2] for b in boxes} != set(paths):
                    missing.append("incomplete_environment_mesh:" + label)
                destination_boxes.extend(boxes)
            values = {}
            if not missing:
                for label, boxes in source_boxes.items():
                    values[label] = min(max(_aabb_clearance(a, b),
                        _geometry_projection_gap(a[2], b[2]))
                        for a in boxes for b in destination_boxes)
            environment_projected.append({**declaration,
                "distance_m": min(values.values()) if values else None,
                "source_distances_m": values,
                "nearest_source": min(values, key=values.get) if values else None,
                "coverage_complete": not missing and bool(values),
                "missing_geometry": missing,
                "source_collider_count": sum(len(v) for v in sources.values()),
                "environment_collider_count": sum(len(v) for v in destinations.values()),
                "environment_prim_paths": sorted(destinations),
                "environment_collider_paths": sorted({path for paths in destinations.values() for path in paths}),
            })

        excluded_pairs = set()
        joint_graph = {}
        for prim in (geometry_adapter.joint_prims if geometry_adapter is not None else stage.Traverse()):
            if not prim.IsA(UsdPhysics.Joint):
                continue
            joint = UsdPhysics.Joint(prim)
            body0 = joint.GetBody0Rel().GetTargets()
            body1 = joint.GetBody1Rel().GetTargets()
            if body0 and body1:
                path0, path1 = str(body0[0]), str(body1[0])
                excluded_pairs.add(tuple(sorted((path0, path1))))
                joint_graph.setdefault(path0, set()).add(path1)
                joint_graph.setdefault(path1, set()).add(path0)

        def _kinematically_near(path_a, path_b, max_hops=2):
            """Exclude rigid bodies connected by a short joint chain.

            Many robot assets insert a collider-free terminal body (for
            example Franka panda_link8) between the last arm collider and the
            hand collider.  Direct-pair filtering therefore still reports a
            permanent zero self-clearance.  A two-joint graph distance is the
            conservative robot-independent fallback used by collision models
            when no SRDF disabled-collision matrix is available.
            """
            if not path_a or not path_b:
                return False
            frontier = {path_a}
            visited = {path_a}
            for _ in range(max_hops):
                frontier = {
                    neighbor
                    for node in frontier
                    for neighbor in joint_graph.get(node, ())
                    if neighbor not in visited
                }
                if path_b in frontier:
                    return True
                visited.update(frontier)
                if not frontier:
                    break
            return False

        # Prefer each robot's CuRobo disabled-collision matrix when it is
        # available.  It captures legitimate close geometry (for example
        # Franka panda_hand versus panda_link5) that a generic graph-distance
        # heuristic cannot distinguish from a real self-collision hazard.
        configured_ignored_names = set()
        configured_ignored_link_pairs = set()
        for robot_name, robot in getattr(task, "robots", {}).items():
            robot_files = (getattr(robot, "cfg", {}) or {}).get("robot_file", [])
            if isinstance(robot_files, str):
                robot_files = [robot_files]
            for robot_file in robot_files or []:
                candidates = [Path(robot_file)]
                if not Path(robot_file).is_absolute():
                    candidates.extend([
                        Path.cwd() / robot_file,
                        Path(__file__).resolve().parents[1] / robot_file,
                    ])
                config_path = next((p for p in candidates if p.is_file()), None)
                if config_path is None:
                    continue
                try:
                    import yaml
                    config = (geometry_adapter.read_yaml(config_path) if geometry_adapter is not None
                              else yaml.safe_load(config_path.read_text(encoding="utf-8")) or {})
                    ignored = (
                        config.get("robot_cfg", {})
                        .get("kinematics", {})
                        .get("self_collision_ignore", {})
                    ) or {}
                    for link_a, linked_names in ignored.items():
                        for link_b in linked_names or []:
                            configured_ignored_names.add(tuple(sorted((
                                f"{robot_name}/{link_a}", f"{robot_name}/{link_b}"
                            ))))
                            configured_ignored_link_pairs.add(tuple(sorted((
                                str(link_a).rsplit("/", 1)[-1],
                                str(link_b).rsplit("/", 1)[-1],
                            ))))
                except Exception as exc:
                    logger.debug("Could not read self-collision ignore config %s: %s", config_path, exc)
                break

        self_dist = {}
        robot_names = sorted(groups["robot"])
        for index, name_a in enumerate(robot_names):
            for name_b in robot_names[index + 1:]:
                owners = tuple(sorted((group_roots.get(name_a, ""), group_roots.get(name_b, ""))))
                labels = tuple(sorted((name_a, name_b)))
                local_link_pair = tuple(sorted((
                    name_a.rsplit("/", 1)[-1], name_b.rsplit("/", 1)[-1]
                )))
                same_link_namespace = (
                    name_a.rsplit("/", 1)[0] == name_b.rsplit("/", 1)[0]
                )
                if (
                    owners in excluded_pairs
                    or labels in configured_ignored_names
                    or (
                        same_link_namespace
                        and local_link_pair in configured_ignored_link_pairs
                    )
                    or _kinematically_near(*owners)
                ):
                    continue
                self_dist[f"{name_a}→{name_b}"] = _group_clearance(
                    groups["robot"][name_a], groups["robot"][name_b]
                )

        # Same physics-step velocity for the one bound ordinary target cup.
        # Missing API/invalid values remain unknown in the HS relation rule.
        target_velocity = None
        target = getattr(task, "objects", {}).get(target_id)
        if target is not None:
            try:
                value = np.asarray(target.get_linear_velocity(), dtype=float)
                if value.shape == (3,) and np.all(np.isfinite(value)):
                    target_velocity = {target_id: value.tolist()}
            except Exception:
                pass
        self._data["target_object_linear_velocity_gt"].append(target_velocity)
        self._data["robot_human_distance_matrix_gt"].append(robot_human or None)
        self._data["ee_human_distance_gt"].append(ee_human or None)
        self._data["object_human_distance_gt"].append(object_human or None)
        self._data["object_env_distance_gt"].append(object_env or None)
        self._data["link_env_distance_gt"].append(link_env or None)
        self._data["link_target_mesh_aabb_distance_gt"].append(target_mesh_distances or None)
        self._data["link_target_projected_clearance_gt"].append(projected_target_distances or None)
        self._data["pt_environment_projected_clearance_gt"].append(environment_projected)
        self._data["self_distance_gt"].append(self_dist or None)

    # ── Post-processing ─────────────────────────────────────────────────────

    def finalize(self) -> Dict[str, Any]:
        """Compute episode-level summaries from per-step data.

        Call after the episode loop ends.

        Returns
        -------
        dict
            Summary with contact durations, peak forces, etc.
        """
        # Compute per-contact-interval durations.  Closed intervals are
        # appended when a contact disappears; active intervals are added here
        # without mutating _data so repeated finalize() calls stay idempotent.
        dt = float(self._physics_dt_s)
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("Contact durations require a positive measured physics step")
        contact_events = list(self._data.get("contact_events", []))
        final_step = len(self._data["step_ids"])
        if self._data["step_ids"]:
            final_step = self._data["step_ids"][-1] + 1

        for state in self._contact_state.values():
            if not state.get("active"):
                continue
            end_step = final_step
            duration = max(0.0, (end_step - state["start_step"]) * dt)
            contact_events.append({
                "bodyA": state["bodyA"],
                "bodyB": state["bodyB"],
                "start_step": state["start_step"],
                "end_step": end_step,
                "duration_s": duration,
            })

        return {
            "n_steps": len(self._data["step_ids"]),
            "contact_events": contact_events,
            "has_collisions": any(
                len(pairs) > 0 for pairs in self._data["collision_pair_gt"] if pairs is not None
            ),
        }

    def save(self, episode_dir: str) -> str:
        """Save collected PhysX data to the episode directory.

        Parameters
        ----------
        episode_dir : str
            Path to the episode directory (same as LMDB).

        Returns
        -------
        str
            Path to saved file.
        """
        output_path = os.path.join(episode_dir, "physx_data.json")

        # Convert numpy arrays to lists for JSON serialization
        serializable = {}
        for key, values in self._data.items():
            serializable[key] = self._make_serializable(values)

        # Add summary
        serializable["summary"] = self.finalize()

        with open(output_path, "w", encoding="utf-8") as f:
            json.dump(serializable, f, indent=2, ensure_ascii=False, default=str)

        logger.info("PhysX data saved to: %s", output_path)
        return output_path

    def get_raw_data(self) -> Dict[str, List]:
        """Return raw collected data (for integration with raw_gt_extractor)."""
        return self._data

    def _make_serializable(self, obj):
        """Convert numpy types to JSON-serializable types."""
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, (np.float32, np.float64)):
            return float(obj)
        if isinstance(obj, (np.int32, np.int64)):
            return int(obj)
        if isinstance(obj, list):
            return [self._make_serializable(item) for item in obj]
        if isinstance(obj, dict):
            return {k: self._make_serializable(v) for k, v in obj.items()}
        return obj
