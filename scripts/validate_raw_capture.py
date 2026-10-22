"""Validate frozen meshes and causal raw evidence, without computing risk grades."""
import argparse,hashlib,json,math
from pathlib import Path
import numpy as np
import h5py

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--episode',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();ep=a.episode
    binding=json.loads((ep/'safety_observer_binding.json').read_text());summary=json.loads((ep/'raw_capture_summary.json').read_text())
    schema=json.loads((ep/'raw_capture_schema.json').read_text());errors=[]
    def require(condition,message):
        if not condition:errors.append(message)
    require(binding['evaluation_status']=='pending_offline','Wrong label provenance')
    require(summary['online_geometry_scalar_calls']==summary['online_geometry_batch_calls']==summary['online_risk_evaluations']==0,'Online risk computation was not disabled')
    fingerprints=set()
    for key in ('pt_geometry','hs_geometry','rs_authored_geometry'):
        fingerprints.update(e['fingerprint'] for e in binding['binding'].get(key,{}).get('geometry',[]))
    for fingerprint in fingerprints:
        with np.load(ep/'raw_meshes'/(fingerprint+'.npz')) as d:
            actual=hashlib.sha256(d['vertices'].tobytes()+d['faces'].tobytes()).hexdigest()
        require(actual==fingerprint,'Mesh fingerprint mismatch')
    physics=0;observations=[];initial=0;contact_reports=0;contact_points=0;commands=0;digest=hashlib.sha256()
    body_count=len(binding['binding']['body_names']);joint_count=len(binding['binding']['joint_names'])
    dt=binding['physics_dt_s'];stride=binding['decision_stride']
    with (ep/'raw_evidence.jsonl').open('rb') as f:
        for line in f:
            digest.update(line);r=json.loads(line)
            if r['kind']=='initial_state':
                initial+=1;require(r['physics_step']==-1,'Bad initial source');continue
            if r['kind']=='observation':
                index=len(observations)
                require(r['action_index']==index and r['physics_step']==index*stride and r['source_physics_step']==index*stride-1,'Observation/action chronology mismatch')
                require(math.isclose(r['time_s'],index*binding['action_dt_s'],abs_tol=1e-8),'Observation time mismatch')
                observation=r['observation']
                require(np.shape(observation['kitchen_link_pose'])==(schema['frozen_kitchen_metadata']['link_count'],7),'Kitchen pose inventory mismatch')
                observations.append(r);continue
            if r['kind']=='fault_event':continue
            require(r['kind']=='physics','Unknown packet type')
            require(r['physics_step']==physics and r['action_index']==physics//stride,'Physical stream gap')
            require(math.isclose(r['end_time_s'],(physics+1)*dt,abs_tol=1e-8),'Physical time mismatch')
            require(np.shape(r['state']['body_pose'])==(body_count,7),'Robot body pose coverage mismatch')
            required_actors=schema.get('rigid_actor_paths_120hz')
            if required_actors is not None:
                actors=r['state'].get('rigid_actor_states_world',{})
                require(set(actors)==set(required_actors),'Rigid actor pose channel inventory mismatch')
                for state in actors.values():
                    astate=np.asarray(state)
                    require(astate.shape==(13,) and np.isfinite(astate).all() and np.linalg.norm(astate[3:7])>1.e-8,'Invalid rigid actor state')
            require(len(r['state']['q'])==len(r['state']['dq'])==joint_count,'Joint state coverage mismatch')
            require(r['contact_coverage'] and not r['contact_errors'],'Contact reporting incomplete')
            require(not r['errors'],'Capture errors present')
            command=r['commands'];require(command is not None and command['physics_step']==physics,'Command source mismatch')
            require(command['nominal_action']==command['submitted_action'],'High-level action changed')
            for call in command['targets']['writer_calls']:
                require(call['nominal']==call['submitted'] and call['unchanged'],'Submitted PhysX command changed');commands+=1
            for contact in r['contacts']:
                contact_reports+=1
                require(all(contact[k] for k in ('actor0','actor1','collider0','collider1')),'Missing contact identity')
                for point in contact['points']:
                    contact_points+=1
                    require(all(len(point[k])==3 for k in ('position_world_m','normal','impulse_vector_ns')),'Incomplete raw contact point')
            physics+=1
    require(initial==1 and physics==len(observations)*stride==summary['physics_steps'],'Packet count mismatch')
    require(digest.hexdigest()==summary['raw_writer']['sha256'],'Raw file checksum mismatch')
    require(summary['raw_writer']['fsync_completed'],'Raw writer not flushed to storage')
    with h5py.File(ep/'trajectory.h5') as f:
        require(len(f['time'])==len(observations),'Trajectory observation count mismatch')
        for i,r in enumerate(observations):
            require(np.array_equal(np.asarray(r['nominal_action'],dtype=np.float32),f['action'][i]),'Action differs from trajectory')
            for key in ('q','dq','mustard','bowl','robot_body_pose_w','kitchen_link_pose','kitchen_joint_pos','kitchen_joint_vel'):
                require(np.array_equal(np.asarray(r['observation'][key]),f['obs/'+key][i]),'Observation differs from trajectory: '+key)
    result=dict(passed=not errors,errors=errors,physics_steps=physics,action_observations=len(observations),
        frozen_meshes=len(fingerprints),contact_reports=contact_reports,contact_points=contact_points,
        actual_command_calls=commands,raw_file_bytes=(ep/'raw_evidence.jsonl').stat().st_size,
        online_geometry_calls=0,online_risk_evaluations=0,evaluation_status='pending_offline',
        scope='Raw completeness, contact identities, actual commands, mesh hashes and exact HDF5 alignment; does not claim evaluated risk levels')
    a.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(result,ensure_ascii=False))
    return int(bool(errors))

if __name__=='__main__':raise SystemExit(main())
