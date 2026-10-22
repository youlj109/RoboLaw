"""Compare actual offline risk results across explicit mesh-backend replays."""
import argparse,itertools,json
from collections import Counter
from pathlib import Path

def distances(features):
 h=features['hs'];p=features['pt'];out={}
 for k in ('d_robot_h_min_gt_m','d_ee_h_min_gt_m','d_obj_h_min_gt_m','d_fixed_h_min_gt_m','d_moving_h_min_gt_m'):out['HS/'+k]=h.get(k)
 for q in h.get('dynamic_distance',{}).get('pairs',[]):
  for k in ('distance_m','protective_distance_m','margin_m','robot_surface_speed_mps'):out['HS/'+q['relation_id']+'/'+k]=q.get(k)
 for q in p['protected_pairs']:
  key='PT/'+q['actor']+'/'+q['object_id'];out[key]=q.get('distance_m')
  for k,v in q.get('link_distances_m',{}).items():out[key+'/'+k]=v
 return out

def rules(record):
 out={}
 for r in record['report']['rules']:
  e=r.get('evidence',{});key=(r['category'],r['rule_id'],e.get('actor'),e.get('object_id'))
  if key in out:raise ValueError('Ambiguous rule inventory: '+str(key))
  out[key]={k:r.get(k) for k in ('level','complete','counted_in_grade','evaluation_status','missing')}
 return out

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--candidate',type=Path,required=True);p.add_argument('--reference',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 count=0;levels=Counter();max_difference=0.;numbers=0;errors=[]
 with a.candidate.open() as left,a.reference.open() as right:
  for x,y in itertools.zip_longest(left,right):
   if x is None or y is None:raise ValueError('Observation count differs')
   c,r=json.loads(x),json.loads(y);index=c['action_index'];count+=1
   if (index,c['source_physics_step'])!=(r['action_index'],r['source_physics_step']):raise ValueError('Source physical time differs')
   for cat in ('HS','PT','RS'):
    if c['report']['risk_levels'][cat]!=r['report']['risk_levels'][cat]:errors.append(dict(frame=index,category=cat,candidate=c['report']['risk_levels'][cat],reference=r['report']['risk_levels'][cat]))
    levels[cat+'/'+str(c['report']['risk_levels'][cat])]+=1
    # Coverage must also match: an L0 with a missing distance is not an equivalent result.
    cc,rr=c['report']['categories'][cat],r['report']['categories'][cat]
    if any(cc.get(k)!=rr.get(k) for k in ('data_complete','missing','known_rule_count','excluded_rule_count')):errors.append(dict(frame=index,category=cat,error='category coverage differs'))
   if rules(c)!=rules(r):errors.append(dict(frame=index,error='rule inventory/severity/completeness differs'))
   cd,rd=distances(c['input']['features']),distances(r['input']['features'])
   if cd.keys()!=rd.keys():raise ValueError('Distance inventory differs')
   for k,v in cd.items():
    w=rd[k]
    if v is None or w is None:
     if v is not w:errors.append(dict(frame=index,key=k,error='distance validity differs'))
    else:
     delta=abs(v-w);numbers+=1;max_difference=max(max_difference,delta)
     if delta>1e-5:errors.append(dict(frame=index,key=k,difference_m=delta))
 result=dict(passed=not errors,observations=count,numeric_values_compared=numbers,max_numeric_difference=max_difference,
  absolute_distance_tolerance_m=1e-5,risk_counts=dict(levels),errors=errors,candidate=str(a.candidate),reference=str(a.reference),
  scope='Same immutable physical packets; compare all HS/PT/RS rule inventories, grades and coverage, measured distances, dynamic boundaries/margins and surface speeds. Metadata backend identities and nearest-point availability intentionally differ.')
 a.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k not in ('candidate','reference','scope')},ensure_ascii=False))
 if errors:raise SystemExit(1)
if __name__=='__main__':main()
