import pytest
from scripts.extensions.controls.render_first_violation import label_robot_hulls


def test_link_labels_preserve_shape_identity_and_order():
    shapes=[('arm',1,2,3),('robot_hull',4,5,6),('robot_hull',7,8,9),('gripper/finger',10,11,12)]
    result=label_robot_hulls(shapes,['/panda/panda_link5','/panda/panda_link6'])
    assert result==[('arm',1,2,3),('/panda/panda_link5',4,5,6),('/panda/panda_link6',7,8,9),('gripper/finger',10,11,12)]


def test_link_label_mismatch_fails_closed():
    with pytest.raises(ValueError,match='identity mismatch'):
        label_robot_hulls([],['unexpected'])
