import copy
from kitchen_safety.extensions.lightwheel_pick_place.grasp_policy_compatibility import compare
def metadata():
    return {'pick_objects':['banana_1','banana_2'],'grasp_search_policy':{'name':'b03-padcenter-v5','candidates':[[0,180,.002,0],[0,0,.002,.03]],'physical_exclusions':{'banana_1':[[15,180,0,0]]}},'grasp_solutions':{p:{'tilt':0,'yaw':180,'pinch_z_offset_m':.002,'waist_y_shift_m':0} for p in ['banana_1','banana_2']}}
def strengthened():
    a=metadata();b=copy.deepcopy(a);b['grasp_search_policy']['physical_exclusions']['banana_2']=[[0,0,.002,.03]];return a,b
def test_eligible_baseline_survives_only_strengthened_exclusions():
    a,b=strengthened();r=compare(a,b);assert r['compatible'] and r['strengthened_exclusions']
def test_selected_baseline_grasp_cannot_be_excluded():
    a,b=strengthened();a['grasp_solutions']['banana_2'].update(yaw=0,waist_y_shift_m=.03);assert not compare(a,b)['compatible']
def test_candidate_reordering_is_not_a_compatible_policy():
    a,b=strengthened();b['grasp_search_policy']['candidates'].reverse();assert not compare(a,b)['compatible']
def test_exclusion_removal_fails_closed():
    a,b=strengthened();b['grasp_search_policy']['physical_exclusions']['banana_1']=[];assert not compare(a,b)['compatible']
def test_missing_second_selected_grasp_fails_closed():
    a,b=strengthened();del a['grasp_solutions']['banana_2'];assert not compare(a,b)['compatible']
def test_changed_physical_parameters_fail_closed():
    a,b=strengthened();b['grasp_search_policy']['lift_m']=.2;assert not compare(a,b)['compatible']
def test_malformed_exclusions_fail_closed():
    a,b=strengthened();b['grasp_search_policy']['physical_exclusions']['banana_2']=[[float('nan'),0,.002,.03]];assert not compare(a,b)['compatible']
