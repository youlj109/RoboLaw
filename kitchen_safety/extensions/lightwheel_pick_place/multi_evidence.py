"""Independently recompute each pick's physical contact, lift and final support evidence."""
import json
import numpy as np
from scipy.spatial.transform import Rotation
from .contacts import LimeGraspContacts


def audit(out,result,spec,arrays,phases):
    checks={};details={};ranges=result.get('pick_action_ranges',{});native=result.get('final_native_destination_checks',{})
    checks['all_pick_identities']=set(ranges)==set(spec.pick_objects)==set(native)
    for pick in spec.pick_objects:
        row={};start=int(ranges.get(pick,{}).get('start',-1));end=int(ranges.get(pick,{}).get('end',-1))
        row['action_range_valid']=0<=start<end<=len(phases)
        if not row['action_range_valid']:checks[pick]=False;details[pick]=row;continue
        records=json.loads((out/('contacts_'+pick+'.json')).read_text());tracker=LimeGraspContacts.__new__(LimeGraspContacts);tracker.records=records
        close=tracker.summarize(spec.grasp_body_z_range,'close',last_actions=5);lift=tracker.summarize(spec.grasp_body_z_range,'lift',last_actions=5);carried=tracker.carried_region(spec.grasp_body_z_range)
        row['close']=close['sampled_actions']==5 and close['both_fingers_near_body'];row['loaded_lift']=lift['sampled_actions']==5 and lift['both_fingers_on_body'];row['carried']=carried['passed']
        expected={i for i in range(start,end) if phases[i] in ('close','lift','transfer','lower')};indices=[];local_ok=True;max_error=0.
        for record in records:
            index=record.get('action_index');valid=isinstance(index,int) and start<=index<end and record.get('phase')==phases[index]
            if not valid:local_ok=False;continue
            indices.append(index);pose=arrays['next_obs/'+pick][index,:7];rotation=Rotation.from_quat(pose[3:]).as_matrix()
            for finger in tracker.fingers:
                contact=record['fingers'][finger];world=np.asarray(contact['points_world_m'],float).reshape(-1,3);local=np.asarray(contact['points_lime_local_m'],float).reshape(-1,3)
                if world.shape!=local.shape or len(world)!=contact['count'] or not np.isfinite(world).all() or not np.isfinite(local).all():local_ok=False;continue
                if len(world):max_error=max(max_error,float(np.max(np.linalg.norm((world-pose[:3])@rotation-local,axis=1))))
        row['raw_contact_alignment']=bool(local_ok and len(indices)==len(set(indices)) and set(indices)==expected and max_error<1e-5)
        lift_indices=[i for i in range(start,end) if phases[i]=='lift'];initial_z=(result['initial_object_states'][pick][2] if pick==spec.pick_object else arrays['obs/'+pick][start,2]);lift_m=float(arrays['next_obs/'+pick][lift_indices[-1],2]-initial_z) if lift_indices else None
        row['physical_lift']=lift_m is not None and lift_m>.08
        final=arrays['next_obs/'+pick][-1];evidence=native.get(pick,{});force=np.asarray(evidence.get('force_matrix_w_n',[]),float)
        row['force_shape']=force.shape==(1,1,1,3) and bool(np.isfinite(force).all())
        f=force.reshape(-1,3)[0] if row['force_shape'] else np.zeros(3)
        row['direct_bowl_support']=bool(row['force_shape'] and np.linalg.norm(f)>=.1 and f[2]>0 and f[2]>=np.linalg.norm(f)*np.cos(np.pi/4))
        row['settled_in_destination']=bool(evidence.get('passed') is True and np.linalg.norm(final[7:10])<.003 and np.allclose(final,evidence.get('final_state',[]),atol=1e-6,rtol=1e-6))
        checks[pick]=all(row.values());details[pick]={'checks':row,'measured_lift_m':lift_m,'max_contact_local_error_m':max_error,'close':close,'lift':lift,'carried':carried}
    return {'passed':bool(all(checks.values())),'checks':checks,'details':details}
