"""Camera provenance diagnostics; never infer freshness from image appearance."""
import math
import numbers


def reference_time(value):
    """Read the ReferenceTime rational, not an assumed integer frame counter."""
    if not isinstance(value,dict):return None
    pair=[]
    for key in ['referenceTimeNumerator','referenceTimeDenominator']:
        x=value.get(key)
        if hasattr(x,'item'):
            try:x=x.item()
            except (ValueError,TypeError):return None
        if isinstance(x,bool) or not isinstance(x,numbers.Integral):return None
        pair.append(int(x))
    if pair[1]<=0 or pair[0]<0:return None
    return {'numerator':pair[0],'denominator':pair[1]}


def observation_timing(before,after,actual_frequency,configured_frequency,pose_before,pose_after):
    before=before or {};after=after or {}
    ref=before.get('reference_time');time=before.get('capture_sim_time_s')
    finite=lambda x:isinstance(x,numbers.Real) and not isinstance(x,bool) and math.isfinite(x)
    consistent=(ref is not None and ref==after.get('reference_time')
                and finite(time) and time==after.get('capture_sim_time_s'))
    return {'schema_version':'camera-reference-clock-1','source':'Isaac Camera ReferenceTime annotator',
            'read_before':before,'read_after':after,'reference_time':ref if consistent else None,
            'capture_sim_time_s':float(time) if consistent else None,
            'capture_clock_consistent':consistent,
            'actual_sensor_frequency_hz':float(actual_frequency) if finite(actual_frequency) else None,
            'configured_frequency_hz':configured_frequency,
            'camera2env_before_read':pose_before,'camera2env_after_mount_update':pose_after,
            'pose_at_capture_verified':False,
            'scope':'render-product clock bracket; pixel/pose alignment requires runtime verification'}


def recording_timing(capture,frame,sim_time,phase,previous=None):
    capture=dict(capture) if isinstance(capture,dict) else {}
    ref=capture.get('reference_time');time=capture.get('capture_sim_time_s')
    valid=capture.get('capture_clock_consistent') is True and ref is not None and time is not None
    sequence=capture.get('acquisition_sequence')
    has_sequence=type(sequence) is int and sequence>0
    ambiguous=valid and not has_sequence and previous and ref==previous.get('reference_time') and time!=previous.get('capture_sim_time_s')
    fresh=(sequence!=(previous or {}).get('acquisition_sequence')) if has_sequence else (ref!=(previous or {}).get('reference_time'))
    return dict(capture,physics_action_frame_index=int(frame),read_sim_time_s=float(sim_time),
                observation_phase=phase,
                source_is_new_since_last_recording=fresh if valid and not ambiguous else None,
                freshness_unknown_reason='reference_time_reused_with_different_sim_time' if ambiguous else None,
                age_at_read_s=float(sim_time)-time if valid else None,
                unknown_reason=None if valid else 'missing_or_inconsistent_capture_clock')
