"""A source IK goal never substitutes for the returned cuRobo path."""
from copy import deepcopy
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch
import numpy as np
import pytest
from kitchen_safety.extensions.replicator import physical_retry_round2 as planner
from kitchen_safety.extensions.replicator.joint_goal_repair import validate_round10_policy
from kitchen_safety.extensions.replicator.planner_refinement_round11 import validate_round11_policy
from tests.extensions.replicator.test_retry_segment_planning import FakePlanner


class Kin:
    limits=np.array([[-3.,3.]]*7)
    def fk(self,q):
        t=np.eye(4);t[:3,3]=q[:3];return t


def test_cartesian_goal_match_cannot_hide_wrong_joint_branch():
    target=np.eye(4);target[2,3]=.03
    goal=np.zeros(7);goal[2]=.03;goal[6]=.5
    recipe=dict(retry_round=10,joint_space_target_q_by_phase={'lower':[goal.tolist()]},hold_orientation_phases=[],
        planning_options_by_phase={'lower':{'enable_graph':False}})
    with TemporaryDirectory() as temp,patch.object(planner,'relative',lambda p:str(p)),patch.object(planner,'fk_batch',lambda k,q:np.array([k.fk(x) for x in q])):
        fake=FakePlanner(0)
        path,detail=planner.plan_phase(fake,Kin(),np.zeros(7),target,'lower',[],recipe,Path(temp),8)
        assert path is None and fake.calls==1
        assert detail['solver_status']=='JOINT_GOAL_ENDPOINT_LIMIT'
        assert detail['joint_space_endpoint_error_rad']==.5
        assert detail['endpoint_fk_position_error_m']==0
        assert (Path(temp)/'lower_segment_00.npz').is_file()


def mustard_recipe():
    approach=np.eye(4);lift=approach.copy();lift[2,3]=.15
    return dict(retry_round=10,retained_scene_revision=True,scene_settings_changed_this_round=False,payload_name='mustard_bottle',
        planning_path_policy=dict(kind='collision_checked_joint_goal_lift_corridor',previous_straight_line_gate_removed=True),
        planning_options_by_phase={'lift':{'enable_graph':False}},hold_orientation_phases=[],linear_motion_phases=[],
        joint_space_target_q_by_phase={'lift':[[0.,0.,.15,0.,0.,0.,0.]]},
        target_poses_world=dict(approach=approach.tolist(),lift=lift.tolist()),
        lift_path_validation=dict(reference_phase='approach',max_xy_deviation_m=.008,max_orientation_deviation_deg=4.,min_z_offset_m=-.002))


@pytest.mark.parametrize('change',['relax_gate','disable_gate','graph','soft_pose','wrong_phase','lateral_goal'])
def test_joint_goal_policy_cannot_relax_lift_or_mislabel_constraints(change):
    recipe=mustard_recipe();validate_round10_policy(recipe)
    if change=='relax_gate':recipe['lift_path_validation']['max_xy_deviation_m']=.01
    elif change=='disable_gate':recipe.pop('lift_path_validation')
    elif change=='graph':recipe['planning_options_by_phase']['lift']['enable_graph']=True
    elif change=='soft_pose':recipe['hold_orientation_phases']=['lift']
    elif change=='wrong_phase':recipe['joint_space_target_q_by_phase']['lower']=recipe['joint_space_target_q_by_phase'].pop('lift')
    elif change=='lateral_goal':recipe['target_poses_world']['lift'][0][3]=.01
    with pytest.raises(ValueError):validate_round10_policy(recipe)


def test_unit_weight_refinement_keeps_all_hard_lift_limits():
    recipe=mustard_recipe();recipe.update(retry_round=11,hold_orientation_phases=['lift'],linear_motion_phases=['lift'],
        planning_options_by_phase={'lift':{'constraint_weight':1.,'enable_graph':True}})
    recipe.pop('joint_space_target_q_by_phase')
    recipe['planning_path_policy']['kind']='collision_checked_lift_corridor'
    validate_round11_policy(recipe)
    for change in ('threshold','weight','remove_hold'):
        bad=deepcopy(recipe)
        if change=='threshold':bad['lift_path_validation']['max_orientation_deviation_deg']=22.
        elif change=='weight':bad['planning_options_by_phase']['lift']['constraint_weight']=20.
        else:bad['hold_orientation_phases']=[]
        with pytest.raises(ValueError):validate_round11_policy(bad)
