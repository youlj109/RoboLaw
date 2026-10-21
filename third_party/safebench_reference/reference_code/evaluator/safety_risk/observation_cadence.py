"""Observation-rate oracle labels linked to full-rate physical evidence.

This module builds dataset indices. It does not claim the online controller has
already migrated, or expose simulator-only ground truth to a visual policy.
"""
import math
from .iso_input import raw_frame, _pair


def physics_stride(physics_dt_s, decision_hz):
    if any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) or x<=0
           for x in (physics_dt_s,decision_hz)):
        raise ValueError("finite positive physical dt and decision rate required")
    ratio=1./(physics_dt_s*decision_hz)
    stride=round(ratio)
    if stride<1 or abs(ratio-stride)>1e-5:
        raise ValueError("decision rate must divide the physics rate for this fixed-step protocol")
    return stride


def observation_indices(num_physics_steps, physics_dt_s, decision_hz=30.):
    if type(num_physics_steps) is not int or num_physics_steps<1:
        raise ValueError("positive physical frame count required")
    return list(range(0,num_physics_steps,physics_stride(physics_dt_s,decision_hz)))


def build_index(raw, evaluator, decision_hz=30., policy_cameras=("franka_head","franka_hand")):
    n=raw["metadata"]["num_steps"]
    dt=raw["episode_meta"]["physics_config"]["physics_dt_s"]
    selected=observation_indices(n,dt,decision_hz)
    clocks=raw["metadata"].get("physics_frame_clocks",[])
    if len(clocks)!=n:raise ValueError("complete physical clocks required")
    if not policy_cameras:raise ValueError("at least one declared policy camera required")
    cameras=raw.get("camera_timeline",{})
    for name in policy_cameras:
        if len(cameras.get(name,[]))!=n:raise ValueError("camera clock coverage mismatch: "+name)
    contacts=raw.get("collision_gt",{}).get("collision_pair_gt")
    if not isinstance(contacts,list) or len(contacts)!=n:raise ValueError("complete contact frame axis required")
    coverage=raw.get("collision_gt",{}).get("contact_coverage_per_frame",[])
    samples=[];previous_sources={}
    for slot,start in enumerate(selected):
        end=selected[slot+1] if slot+1<len(selected) else n
        source=start-1 if start else None
        if source is None:
            input_frame={"frame_index":None,"time_s":0.,"features":{},
                "metadata":{"source":"missing_initial_pre_action_physical_snapshot",
                            "limitations":["no future first-step state substituted for initial observation"]}}
            if raw.get("initial_pre_action_state") is not None:
                from .online_iso import initial_snapshot
                input_frame = initial_snapshot(raw["initial_pre_action_state"], clocks[start]["before_sim_time_s"])
        else:
            if abs(clocks[source]["after_sim_time_s"]-clocks[start]["before_sim_time_s"])>1e-6:
                raise ValueError("pre-action physical source is not adjacent")
            input_frame=raw_frame(raw,source)
        report=evaluator.evaluate(input_frame)
        views={}
        for name in policy_cameras:
            row=cameras[name][start]
            seq=row.get("acquisition_sequence")
            identity=(row.get("acquisition_epoch"),seq)
            age=row.get("age_at_read_s")
            valid=(row.get("capture_clock_consistent") is True and type(seq) is int
                   and isinstance(age,(int,float)) and math.isfinite(age)
                   and -1e-6<=age<=1./decision_hz+1e-6)
            status="new" if valid and identity!=previous_sources.get(name) else "stale" if valid else "missing_or_invalid"
            views[name]={"status":status,"acquisition_epoch":identity[0],"acquisition_sequence":seq,
                "capture_sim_time_s":row.get("capture_sim_time_s"),"age_s":age,
                "lmdb_image_key":f"images.rgb.{name.replace('franka_','')}/{start:04d}",
                "physical_record_index":start}
            previous_sources[name]=identity
        known_contact_frames=[];human_contact_frames=[];unknown_contact_frames=[]
        for frame in range(start,end):
            records=contacts[frame]
            if not isinstance(records,list):
                unknown_contact_frames.append(frame);continue
            if not isinstance(coverage,list) or frame>=len(coverage) or coverage[frame] is not True:
                unknown_contact_frames.append(frame)
            if records:known_contact_frames.append(frame)
            if any(isinstance(x,dict) and _pair(x,"human") for x in records):human_contact_frames.append(frame)
        samples.append({"observation_frame_id":slot,"decision_physics_step_index":start,
            "decision_sim_time_s":clocks[start]["before_sim_time_s"],
            "risk_source_physics_step_index":source,"risk_input":input_frame,"risk_report":report,
            "policy_views":views,"visual_inference_required":all(v["status"]=="new" for v in views.values()),
            "action_physics_step_indices":list(range(start,end)),
            "interval_event_evidence":{"physics_step_indices":list(range(start,end)),
                "contact_frames":known_contact_frames,"human_contact_frames":human_contact_frames,
                "unknown_contact_frames":unknown_contact_frames,
                "source_fields":["collision_gt.collision_pair_gt","collision_gt.contact_force_gt","collision_gt.contact_impulse_gt"],
                "role":"post-decision consequences; excluded from the current risk input"}})
    return {"schema_version":"observation-rate-index-1","decision_hz":decision_hz,
        "physics_dt_s":dt,"physics_steps":n,"observation_frames":len(samples),
        "profile_sha256":evaluator.profile_hash,"samples":samples,
        "policy_input_cameras":list(policy_cameras),"annotation_source":"simulator oracle pre-action state",
        "production_training_ready":False,
        "limitations":["online cadence migration is separate and not implied by this offline index",
            "initial pre-action state is unknown until explicitly captured",
            "camera exposure/mount alignment remains unverified; ground truth is a label, not visual policy input",
            "event indices retain full-rate evidence; no window maximum is substituted for a frozen feature"]}
