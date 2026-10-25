import json
import numpy as np
from kitchen_safety.extensions.lightwheel_pick_place.multi_evidence import audit
from kitchen_safety.extensions.lightwheel_pick_place.task_config import resolve_task,BANANA_TASK


def fixture(tmp_path):
    spec=resolve_task(BANANA_TASK);phases=(['close']*5+['lift']*5+['transfer','release'])*2;arrays={};native={};ranges={};initial={}
    for number,pick in enumerate(spec.pick_objects):
        start=number*12;end=start+12;ranges[pick]={'start':start,'end':end};states=np.zeros((24,13));states[:,6]=1
        states[start+5:end-1,2]=.12;states[end-1:,2]=.05;arrays['next_obs/'+pick]=states;arrays['obs/'+pick]=states.copy();arrays['obs/'+pick][start,2]=0;initial[pick]=states[0].tolist()
        native[pick]={'passed':True,'force_matrix_w_n':[[[[0,0,.2]]]],'final_state':states[-1].tolist()}
        records=[]
        for index in range(start,end-1):
            point=states[index,:3].tolist();contact={'count':1,'points_world_m':[point],'points_lime_local_m':[[0,0,0]],'normal_forces_n':[1.],'normals_world':[[0,0,1]],'separations_m':[0.]}
            records.append({'phase':phases[index],'action_index':index,'fingers':{f:contact for f in ('left_inner_finger','right_inner_finger')}})
        (tmp_path/('contacts_'+pick+'.json')).write_text(json.dumps(records))
    return spec,arrays,phases,{'pick_action_ranges':ranges,'final_native_destination_checks':native,'initial_object_states':initial}


def test_both_picks_have_independent_evidence(tmp_path):
    spec,arrays,phases,result=fixture(tmp_path)
    assert audit(tmp_path,result,spec,arrays,phases)['passed']


def test_first_success_does_not_hide_second_without_bowl_support(tmp_path):
    spec,arrays,phases,result=fixture(tmp_path)
    result['final_native_destination_checks']['banana_1']['force_matrix_w_n']=[[[[0,0,0]]]]
    report=audit(tmp_path,result,spec,arrays,phases)
    assert report['checks']['banana_2'] and not report['checks']['banana_1'] and not report['passed']


def test_contact_for_wrong_object_pose_is_rejected(tmp_path):
    spec,arrays,phases,result=fixture(tmp_path);p=tmp_path/'contacts_banana_1.json';rows=json.loads(p.read_text());rows[0]['fingers']['left_inner_finger']['points_world_m'][0][0]=.1;p.write_text(json.dumps(rows))
    assert not audit(tmp_path,result,spec,arrays,phases)['passed']
