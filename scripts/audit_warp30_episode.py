"""Audit complete 30 Hz trajectory/labels/render alignment and Warp coverage."""
import argparse,json,math
from collections import Counter,defaultdict
from pathlib import Path
import h5py,numpy as np

def main():
 p=argparse.ArgumentParser();p.add_argument('--episode',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();ep=a.episode
 errors=[]
 def require(value,message):
  if not value:errors.append(message)
 b=json.loads((ep/'safety_observer_binding.json').read_text());cfg=b['resolved_config']
 require(abs(b['physics_dt_s']-1/120)<1.e-12,'physics is not 120 Hz')
 require(abs(b['action_dt_s']-1/30)<1.e-12,'action sampling is not 30 Hz')
 require(b['decision_stride']==4 and cfg['decision_hz']==30,'labels are not 30 Hz')
 require(cfg['disabled_categories']==['IR'],'IR must be disabled')
 require(b['resolved_profile']['pt_hs_inspired']['distance_metric']=='warp_authored_collision_surface_distance','profile not Warp')
 grade=defaultdict(Counter);count=0;pair_count=None;wall_start=None;wall_end=None
 with h5py.File(ep/'trajectory.h5','r') as f:times=np.asarray(f['time']).reshape(-1)
 require(np.allclose(times,np.arange(len(times))/30,rtol=0,atol=1.e-7),'trajectory timestamps not a uniform 30 Hz sequence')
 with (ep/'safety_decisions.jsonl').open() as stream:
  for line in stream:
   r=json.loads(line);i=count;features=r['input']['features'];rep=r['report']
   require(r['observation_id']==i and r['action_index']==i,'label/action index mismatch '+str(i))
   require(abs(r['decision_time_s']-i/30)<1.e-8,'label timestamp mismatch '+str(i))
   require(r['source_physics_step']==4*i-1,'noncausal source step '+str(i))
   require(r['error'] is None,'evaluation error '+str(i))
   require('ir' not in features and rep['categories']['IR']['status']=='disabled','IR unexpectedly enabled '+str(i))
   require(not any(x['category']=='IR' for x in rep['rules']),'IR rule emitted '+str(i))
   require(features['hs']['distance_evidence']['complete'] is True,'incomplete HS geometry '+str(i))
   for field in ('d_robot_h_min_gt_m','d_ee_h_min_gt_m','d_obj_h_min_gt_m'):
    require(isinstance(features['hs'][field],(int,float)) and math.isfinite(features['hs'][field]),'invalid HS distance '+str(i))
   pairs=features['pt']['protected_pairs'];pair_count=len(pairs)
   require(all(x['distance_coverage_complete'] is True and 'warp' in x['distance_source'] for x in pairs),'incomplete/non-Warp PT geometry '+str(i))
   require(features['rs']['self_collision_evidence']['coverage_complete'] is True,'incomplete self geometry '+str(i))
   require('Warp' in features['rs']['self_collision_evidence']['source'],'RS backend not Warp '+str(i))
   for c in ('HS','PT','RS'):grade[c][rep['categories'][c]['level'] or 'unknown']+=1
   wall_start=r['decision_wall_time_ns'] if wall_start is None else wall_start;wall_end=r['evaluation_complete_time_ns'];count+=1
 require(count==len(times),'trajectory and label counts differ')
 report=json.loads((ep/'rendered/render_report.json').read_text());frames=report['frames']
 require(len(frames)==count,'RGB/label counts differ')
 require(all(f['source_obs_index']==i and abs(f['source_time_seconds']-times[i])<1.e-7 for i,f in enumerate(frames)),'RGB source alignment mismatch')
 fresh=0;physical=0
 with (ep/'safety_physics.jsonl').open() as stream:
  for line in stream:
   r=json.loads(line);d=r['distances'];step=r['local_physics_step']
   require(step==physical,'physical stream gap '+str(step))
   if d.get('geometry_fresh'):
    fresh+=1;require(d['geometry_sample_physics_step']==step and (step+1)%4==0,'geometry cadence mismatch '+str(step))
   physical+=1
 require(physical==4*count,'physical/label count mismatch')
 task=json.loads((ep/'result.json').read_text())
 out=dict(passed=not errors,errors=errors,episode=str(ep),physics_hz=120,collection_hz=30,label_hz=30,video_fps=30,labels=count,frames=len(frames),physics_steps=physical,geometry_updates_after_initial=fresh,pt_pairs=pair_count,grades={c:dict(v) for c,v in grade.items()},task_success=task.get('success'),simulation_duration_s=count/30,decision_span_wall_s=(wall_end-wall_start)/1e9 if count else None,scope='full recorded episode; frame identity, causal timestamps, backend and geometry coverage; does not certify damage/fault coverage')
 a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n');print(json.dumps(out,ensure_ascii=False,indent=2));return int(bool(errors))
if __name__=='__main__':raise SystemExit(main())
