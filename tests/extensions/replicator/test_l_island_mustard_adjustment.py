"""The original-layout authorization permits exactly two position fields."""
import copy
from scripts.extensions.replicator.adjust_l_island_mustard_layout import protected_base


def fixture():
    return dict(task_id='original',robot_position=[0,0,0],robot_yaw_deg=90.,stand_height=.8,
        kitchen_usd='original.usd',objects={name:dict(position=[0,0,1],yaw_deg=0.,usd_path=name+'.usd')
        for name in ('mustard_bottle','bowl')})


def test_only_two_object_positions_may_change():
    original=fixture();changed=copy.deepcopy(original)
    changed['objects']['mustard_bottle']['position']=[-.06,.05,1.001]
    changed['objects']['bowl']['position']=[.075,0,1.001]
    assert protected_base(original,changed)
    assert original==fixture()


def test_original_robot_background_assets_and_yaws_are_protected():
    original=fixture()
    for field,value in [('robot_position',[1,0,0]),('robot_yaw_deg',0.),('stand_height',1.),('kitchen_usd','changed.usd')]:
        changed=copy.deepcopy(original);changed[field]=value
        assert not protected_base(original,changed)
    for name in original['objects']:
        for field,value in [('yaw_deg',90.),('usd_path','substitute.usd')]:
            changed=copy.deepcopy(original);changed['objects'][name][field]=value
            assert not protected_base(original,changed)
