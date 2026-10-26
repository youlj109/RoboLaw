"""Eligibility of genuine baseline grasps under strictly stronger failure exclusions."""
import math
def compare(baseline,augmented):
    before=baseline.get('grasp_search_policy',{});after=augmented.get('grasp_search_policy',{})
    if not isinstance(before,dict) or not isinstance(after,dict):return {'compatible':False,'reason':'Missing search policy'}
    if {k:v for k,v in before.items() if k!='physical_exclusions'}!={k:v for k,v in after.items() if k!='physical_exclusions'}:
        return {'compatible':False,'reason':'Search inputs other than physical exclusions differ'}
    def exclusions(policy):
        result={}
        for pick,rows in policy.get('physical_exclusions',{}).items():
            if not isinstance(pick,str) or not isinstance(rows,list):raise ValueError()
            entries=set()
            for row in rows:
                if not isinstance(row,list) or len(row)!=4 or not all(isinstance(x,(int,float)) and math.isfinite(x) for x in row):raise ValueError()
                entries.add(tuple(row))
            result[pick]=entries
        return result
    try:old=exclusions(before);new=exclusions(after)
    except (ValueError,AttributeError,TypeError):return {'compatible':False,'reason':'Malformed exclusion policy'}
    if any(not rows.issubset(new.get(pick,set())) for pick,rows in old.items()):
        return {'compatible':False,'reason':'An existing failure exclusion was removed'}
    for metadata in (baseline,augmented):
        solutions=metadata.get('grasp_solutions',{})
        picks=metadata.get('pick_objects',[])
        if not picks or set(solutions)!=set(picks):return {'compatible':False,'reason':'Missing per-pick selected grasp'}
        for pick,solution in solutions.items():
            candidate=tuple(solution.get(k) for k in ('tilt','yaw','pinch_z_offset_m','waist_y_shift_m'))
            if candidate in new.get(pick,set()):
                return {'compatible':False,'reason':'A recorded selected grasp is now excluded','pick':pick}
    strengthened=old!=new
    return {'compatible':True,'reason':'Only failure exclusions strengthened; every actual baseline and augmented grasp remains eligible' if strengthened else 'Identical search policy and eligible selected grasps',
            'strengthened_exclusions':strengthened,'additional_exclusions':{pick:[list(x) for x in sorted(rows-old.get(pick,set()))] for pick,rows in new.items() if rows-old.get(pick,set())}}
