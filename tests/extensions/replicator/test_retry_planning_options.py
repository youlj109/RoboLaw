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
    h = SimpleNamespace(planner=p, events=events, success=True, offset=0., outputs=[],
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

    p.motion_gen = SimpleNamespace(
        reset=Mock(side_effect=lambda **kw: events.append(('reset', kw))),
        finetune_trajopt_solver=SimpleNamespace(reset_seed=Mock(side_effect=lambda: events.append('finetune'))),
        finetune_js_trajopt_solver=SimpleNamespace(reset_seed=Mock(side_effect=lambda: events.append('finetune_js'))),
        world_coll_checker=SimpleNamespace(enable_obstacle=Mock()),
        plan_single=Mock(side_effect=solve),
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
