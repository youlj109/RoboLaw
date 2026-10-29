"""Exercise the real adapter with CPU stand-ins for the cuRobo public boundary.

Do not construct MotionGen, import torch, or execute a GPU/physical episode.
"""
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from kitchen_safety.extensions.replicator import physical_retry_round2 as planner


class Tensor:
    def __init__(self, value):
        self.value = np.asarray(value)

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.value

    def tolist(self):
        return self.value.tolist()


class Kin:
    def fk(self, q):
        pose = np.eye(4)
        pose[:3, 3] = q[:3]
        return pose


@pytest.fixture
def harness(monkeypatch):
    """Record actual config/metric arguments and run inherited seed reset."""
    names = (
        'curobo', 'curobo.types', 'curobo.types.math', 'curobo.types.robot',
        'curobo.wrap', 'curobo.wrap.reacher', 'curobo.wrap.reacher.motion_gen',
        'curobo.rollout', 'curobo.rollout.cost', 'curobo.rollout.cost.pose_cost',
    )
    modules = {name: ModuleType(name) for name in names}
    for name, module in modules.items():
        module.__path__ = []
        monkeypatch.setitem(sys.modules, name, module)
        parent, _, child = name.rpartition('.')
        if parent:
            setattr(modules[parent], child, module)
    metric_factory = Mock(side_effect=lambda **kw: SimpleNamespace(**kw))
    config_factory = Mock(side_effect=lambda **kw: SimpleNamespace(**kw))
    pose_factory = Mock(side_effect=lambda values, args: SimpleNamespace(values=values))
    state_factory = Mock(side_effect=lambda position, **kw: SimpleNamespace(position=position, **kw))
    modules['curobo.types.math'].Pose = SimpleNamespace(from_list=pose_factory)
    modules['curobo.types.robot'].JointState = SimpleNamespace(from_position=state_factory)
    modules['curobo.wrap.reacher.motion_gen'].MotionGenPlanConfig = config_factory
    modules['curobo.rollout.cost.pose_cost'].PoseCostMetric = metric_factory

    events = []
    p = planner.GraspRetryPlanner.__new__(planner.GraspRetryPlanner)
    p.args = SimpleNamespace(to_device=Tensor)
    p.T_B_W = p.T_ee_tcp = np.eye(4)
    p.obstacle_names = ['payload', 'hazard']
    p.world = SimpleNamespace(attachment=np.eye(4), _matches=lambda name, mask: name == mask)
    p._sync_gripper_and_attachment = Mock()
    p.torch = SimpleNamespace(cuda=SimpleNamespace(synchronize=Mock()))
    h = SimpleNamespace(planner=p, events=events, success=True, offset=0., outputs=[], joint_start_shift=0.,
                        config_factory=config_factory, metric_factory=metric_factory,
                        pose_factory=pose_factory, state_factory=state_factory)

    def solve(start, goal, config):
        events.append('plan')
        q = start.position.numpy()[0]
        end = q.copy()
        end[:3] = goal.values[:3]
        mid = (q + end) / 2
        mid[1] += h.offset
        output = SimpleNamespace(success=np.array([h.success]),
                                 status=None if h.success else 'TRAJOPT_FAIL',
                                 interpolation_dt=1/30,
                                 get_interpolated_plan=Mock(return_value=SimpleNamespace(
                                     position=Tensor(np.array([q, mid, end])))))
        h.outputs.append(output)
        return output

    def solve_js(start, goal, config):
        events.append('plan_js')
        q = start.position.numpy()[0].copy()
        end = goal.position.numpy()[0].copy()
        mid = (q + end) / 2
        mid[1] += h.offset
        q[0] += h.joint_start_shift
        output = SimpleNamespace(success=np.array([h.success]),
                                 status=None if h.success else 'FINETUNE_TRAJOPT_FAIL',
                                 interpolation_dt=1/30,
                                 get_interpolated_plan=Mock(return_value=SimpleNamespace(
                                     position=Tensor(np.array([q, mid, end])))))
        h.outputs.append(output)
        return output

    p.motion_gen = SimpleNamespace(
        reset=Mock(side_effect=lambda **kw: events.append(('reset', kw))),
        finetune_trajopt_solver=SimpleNamespace(reset_seed=Mock(side_effect=lambda: events.append('finetune'))),
        finetune_js_trajopt_solver=SimpleNamespace(reset_seed=Mock(side_effect=lambda: events.append('finetune_js'))),
        world_coll_checker=SimpleNamespace(enable_obstacle=Mock()),
        plan_single=Mock(side_effect=solve),
        plan_single_js=Mock(side_effect=solve_js),
    )
    return h


def call(h, **kwargs):
    target = np.eye(4)
    target[2, 3] = .02
    return h.planner.constrained_plan(np.zeros(7), target, 'lift', ['payload'],
                                     kwargs.pop('hold_orientation', True), timeout=3.25, **kwargs)


def assert_one_call_budget(h, *, graph):
    h.config_factory.assert_called_once()
    h.planner.motion_gen.plan_single.assert_called_once()
    actual = h.config_factory.call_args.kwargs
    assert actual['max_attempts'] == 1
    assert actual['finetune_attempts'] == 1
    assert actual['ik_fail_return'] == 1
    assert actual['timeout'] == 3.25
    assert actual['enable_finetune_trajopt'] is True
    assert actual['enable_graph'] is graph
    assert actual['enable_graph_attempt'] == (1 if graph else None)
    assert h.planner.motion_gen.plan_single.call_args.args[2].__dict__ == actual


@pytest.mark.parametrize('vertical,weights', [
    (False, [20., 20., 20., 0., 0., 0.]),
    (True, [20., 20., 20., 20., 20., 0.]),
])
def test_declared_cost_and_no_graph_use_single_call_and_full_seed_reset(harness, vertical, weights):
    h = harness
    options = dict(constraint_weight=20., enable_graph=False)
    path, detail = call(h, planning_options=options, linear_vertical=vertical)
    assert len(path) == 3
    assert_one_call_budget(h, graph=False)
    # This executes CuroboPlanner._reset_all_seeds, not a mocked replacement.
    assert h.events == [('reset', {'reset_seed': True}), 'finetune', 'finetune_js', 'plan']
    metric = h.config_factory.call_args.kwargs['pose_cost_metric']
    assert metric.hold_partial_pose is True
    assert metric.hold_vec_weight.tolist() == weights
    if vertical:
        assert metric.project_to_goal_frame is False
    else:
        assert not hasattr(metric, 'project_to_goal_frame')
    assert detail['pose_metric_hold_weights'] == weights
    assert detail['planner_seed_reset'] == 'all_including_finetune'
    assert detail['graph_enabled'] is detail['graph_fallback_enabled'] is False
    assert detail['planning_options'] == options
    options['constraint_weight'] = 50.
    assert detail['planning_options']['constraint_weight'] == 20.
    assert detail['physical_grasp_verified'] is False
    calls = h.planner.motion_gen.world_coll_checker.enable_obstacle.call_args_list
    assert [(c.args[0], c.kwargs['enable']) for c in calls] == [('payload', False), ('hazard', True)]


def test_solver_failure_does_not_retry_or_restore_graph(harness):
    h = harness
    h.success = False
    path, detail = call(h, planning_options=dict(constraint_weight=20., enable_graph=False), linear_vertical=True)
    assert path is None
    assert detail['status'] == 'failed'
    assert detail['solver_status'] == 'TRAJOPT_FAIL'
    assert detail['graph_fallback_enabled'] is False
    assert_one_call_budget(h, graph=False)
    h.outputs[0].get_interpolated_plan.assert_not_called()
    assert h.events == [('reset', {'reset_seed': True}), 'finetune', 'finetune_js', 'plan']


@pytest.mark.parametrize('options', [None, {}])
@pytest.mark.parametrize('vertical', [False, True])
def test_legacy_options_keep_weight_one_graph_and_original_reset(harness, options, vertical):
    h = harness
    path, detail = call(h, planning_options=options, linear_vertical=vertical)
    assert len(path) == 3
    assert_one_call_budget(h, graph=True)
    assert h.events == [('reset', {'reset_seed': True}), 'plan']
    h.planner.motion_gen.finetune_trajopt_solver.reset_seed.assert_not_called()
    h.planner.motion_gen.finetune_js_trajopt_solver.reset_seed.assert_not_called()
    weights = [1., 1., 1., 1., 1., 0.] if vertical else [1., 1., 1., 0., 0., 0.]
    assert h.config_factory.call_args.kwargs['pose_cost_metric'].hold_vec_weight.tolist() == weights
    for field in ('planning_options', 'planner_seed_reset', 'graph_enabled', 'graph_fallback_enabled', 'pose_metric_hold_weights'):
        assert field not in detail


def test_unconstrained_legacy_call_does_not_add_pose_metric(harness):
    h = harness
    path, detail = call(h, hold_orientation=False)
    assert len(path) == 3
    assert_one_call_budget(h, graph=True)
    h.metric_factory.assert_not_called()
    assert h.config_factory.call_args.kwargs['pose_cost_metric'] is None
    assert detail['orientation_hold_requested'] is False


@pytest.mark.parametrize('offset,expected_calls', [(0., 2), (.01, 1)])
def test_declared_options_reach_each_segment_once_and_gate_stops_later_calls(harness, monkeypatch, tmp_path, offset, expected_calls):
    h = harness
    h.offset = offset
    mid = np.eye(4)
    mid[2, 3] = .02
    target = mid.copy()
    target[2, 3] = .04
    recipe = dict(phase_waypoints_world={'lift': [mid.tolist()]},
                  segment_deviation_limits_m={'lift': .005},
                  linear_motion_phases=['lift'],
                  planning_options_by_phase={'lift': dict(constraint_weight=20., enable_graph=False)})
    monkeypatch.setattr(planner, 'relative', lambda path: str(path))
    monkeypatch.setattr(planner, 'fk_batch', lambda kin, q: np.array([kin.fk(x) for x in q]))
    path, detail = planner.plan_phase(h.planner, Kin(), np.zeros(7), target, 'lift', ['payload'], recipe, tmp_path, 3.25)
    assert h.planner.motion_gen.plan_single.call_count == expected_calls
    assert h.config_factory.call_count == expected_calls
    assert h.events == [('reset', {'reset_seed': True}), 'finetune', 'finetune_js', 'plan'] * expected_calls
    for config in h.config_factory.call_args_list:
        assert config.kwargs['max_attempts'] == config.kwargs['finetune_attempts'] == 1
        assert config.kwargs['enable_graph'] is False
        assert config.kwargs['enable_graph_attempt'] is None
        assert config.kwargs['pose_cost_metric'].hold_vec_weight.tolist() == [20., 20., 20., 20., 20., 0.]
    assert detail['segment_count_declared'] == 2
    assert detail['segment_count_attempted'] == expected_calls
    assert len(list(tmp_path.glob('*.npz'))) == expected_calls
    if offset:
        assert path is None
        assert detail['failed_segment_index'] == 0
        assert detail['solver_status'] == 'TCP_SEGMENT_DEVIATION_LIMIT'
        assert detail['segments'][0]['solver_success'] is True
        assert detail['segments'][0]['deviation_gate_passed'] is False
    else:
        assert path.shape == (6, 7)
        assert all(s['deviation_gate_passed'] for s in detail['segments'])


@pytest.mark.parametrize('success', [True, False])
def test_joint_goal_uses_only_public_js_api_once_without_cartesian_claim(harness, success):
    h = harness
    h.success = success
    goal = np.array([0., 0., .02, .1, -.2, .3, -.4])
    path, detail = call(h, hold_orientation=False, planning_options={'enable_graph':False}, joint_space_goal_q=goal)
    h.planner.motion_gen.plan_single.assert_not_called()
    h.planner.motion_gen.plan_single_js.assert_called_once()
    h.config_factory.assert_called_once()
    h.metric_factory.assert_not_called()
    actual = h.config_factory.call_args.kwargs
    assert actual['max_attempts'] == actual['finetune_attempts'] == actual['ik_fail_return'] == 1
    assert actual['enable_graph'] is False and actual['enable_graph_attempt'] is None
    assert actual['pose_cost_metric'] is None
    args = h.planner.motion_gen.plan_single_js.call_args.args
    np.testing.assert_array_equal(args[0].position.numpy(), np.zeros((1,7)))
    np.testing.assert_array_equal(args[1].position.numpy(), goal[None])
    assert args[1].joint_names == [f'panda_joint{i}' for i in range(1,8)]
    assert args[2].__dict__ == actual
    assert h.events == [('reset', {'reset_seed':True}), 'finetune', 'finetune_js', 'plan_js']
    assert detail['planner_api'] == 'MotionGen.plan_single_js'
    assert detail['joint_space_goal_q'] == goal.tolist()
    assert detail['orientation_hold_requested'] is detail['vertical_motion_cost_requested'] is False
    assert detail['pose_metric_hold_weights'] is None
    assert detail['graph_enabled'] is detail['graph_fallback_enabled'] is False
    assert 'No CPU path is executed' in detail['joint_space_seed_scope']
    if success:
        np.testing.assert_array_equal(path[-1], goal)
    else:
        assert path is None and detail['solver_status'] == 'FINETUNE_TRAJOPT_FAIL'
        h.outputs[0].get_interpolated_plan.assert_not_called()


class JointGoalKin(Kin):
    limits = np.tile([-3., 3.], (7,1))

    def fk(self, q):
        from scipy.spatial.transform import Rotation
        pose = super().fk(q)
        pose[:3,:3] = Rotation.from_euler('x', q[3]).as_matrix()
        return pose


def joint_recipe(goals):
    return dict(joint_space_target_q_by_phase={'lift':goals},
                planning_options_by_phase={'lift':{'enable_graph':False}},
                hold_orientation_phases=[], linear_motion_phases=[],
                segment_deviation_limits_m={'lift':.005})


@pytest.mark.parametrize('goal,error', [
    ([0.,0.,.025,0.,0.,0.,0.], 'does not reach'),
    ([0.,0.,.02,.02,0.,0.,0.], 'does not reach'),
    ([0.,0.,.02,0.,0.,0.,4.], 'outside source joint limits'),
])
def test_joint_goal_must_reach_frozen_tcp_within_source_limits_before_solver(harness, tmp_path, goal, error):
    target = np.eye(4)
    target[2,3] = .02
    with pytest.raises(ValueError, match=error):
        planner.plan_phase(harness.planner, JointGoalKin(), np.zeros(7), target, 'lift', [], joint_recipe([goal]), tmp_path, 3.25)
    harness.planner.motion_gen.plan_single.assert_not_called()
    harness.planner.motion_gen.plan_single_js.assert_not_called()
    harness.config_factory.assert_not_called()
    assert not list(tmp_path.iterdir())


def test_joint_solver_discontinuous_returned_path_is_rejected(harness):
    h = harness
    h.joint_start_shift = .002
    with pytest.raises(ValueError, match='discontinuous start'):
        call(h, hold_orientation=False, planning_options={'enable_graph':False},
             joint_space_goal_q=np.array([0.,0.,.02,0.,0.,0.,0.]))
    h.planner.motion_gen.plan_single.assert_not_called()
    h.planner.motion_gen.plan_single_js.assert_called_once()


def test_joint_segment_goals_are_forwarded_in_order_with_actual_fk_gate(harness, monkeypatch, tmp_path):
    h = harness
    middle = np.eye(4)
    middle[2,3] = .02
    target = middle.copy()
    target[2,3] = .04
    goals = [[0.,0.,.02,0.,0.,0.,0.], [0.,0.,.04,0.,0.,0.,0.]]
    recipe = joint_recipe(goals)
    recipe['phase_waypoints_world'] = {'lift':[middle.tolist()]}
    monkeypatch.setattr(planner, 'relative', lambda path:str(path))
    monkeypatch.setattr(planner, 'fk_batch', lambda kin, q:np.array([kin.fk(x) for x in q]))
    path, detail = planner.plan_phase(h.planner, JointGoalKin(), np.zeros(7), target, 'lift', [], recipe, tmp_path, 3.25)
    h.planner.motion_gen.plan_single.assert_not_called()
    assert h.planner.motion_gen.plan_single_js.call_count == 2
    assert path.shape == (6,7)
    for index, (call_args, segment) in enumerate(zip(h.planner.motion_gen.plan_single_js.call_args_list, detail['segments'])):
        np.testing.assert_array_equal(call_args.args[1].position.numpy()[0], goals[index])
        assert segment['planner_api'] == 'MotionGen.plan_single_js'
        assert segment['deviation_gate_passed'] is True
        assert segment['target_goal_fk_position_error_m'] == 0.
        assert segment['target_goal_fk_rotation_error_rad'] == 0.
    np.testing.assert_array_equal(h.planner.motion_gen.plan_single_js.call_args_list[1].args[0].position.numpy()[0], goals[0])
    assert detail['segment_count_attempted'] == detail['segment_count_declared'] == 2


def test_joint_branch_cannot_claim_cartesian_soft_constraint(harness):
    with pytest.raises(ValueError, match='must not imply Cartesian'):
        call(harness, hold_orientation=True, planning_options={'enable_graph':False},
             joint_space_goal_q=np.array([0.,0.,.02,0.,0.,0.,0.]))
    harness.planner.motion_gen.plan_single.assert_not_called()
    harness.planner.motion_gen.plan_single_js.assert_not_called()
