"""Freeze the source identity before asynchronous image writing or episode reuse."""
import json
from pathlib import Path


def append_camera_provenance(output,binding,action_index,capture_ns,receive_ns,*,mode):
    identity=dict(binding['identity'])
    dt=binding['physics_dt_s'];action_dt=binding['action_dt_s']
    source_step=action_index*round(action_dt/dt)-1
    with (Path(output)/'safety_camera_provenance.jsonl').open('a') as f:
        for camera in binding['binding']['camera_names']:
            row=dict(**identity,camera_id=camera,source_action_index=action_index,
                source_physics_step=source_step,acquisition_sequence=action_index,
                capture_timestamp_ns=capture_ns,receive_timestamp_ns=receive_ns,
                source_sim_time_s=action_index*action_dt,age_sim_s=0.,status='available',stale=False,
                capture_mode=mode,clock='wall_unix_ns; source simulation clock stored separately',
                receive_semantics='CPU camera buffer acquired; background JPEG completion covered by writer.close',
                policy_input=False)
            f.write(json.dumps(row,separators=(',',':'))+'\n')
