"""CPU tests for truthful failed-plan execution labels, without a simulator."""
from scripts.extensions.replicator.mustard_family_collect import plan_execution_evidence


def test_planning_rejection_before_any_command_is_not_execution():
    row=plan_execution_evidence([],0,100,100)
    assert row['command_executed'] is False and row['action_counter_coverage_complete']


def test_safety_exception_after_first_substep_is_actual_execution():
    actions=[dict(before_physics_step=100,after_physics_step=101)]
    row=plan_execution_evidence(actions,0,100,101)
    assert row['command_executed'] is True and row['action_counter_coverage_complete']
    assert row['actions'][0]['observed_physics_substeps']==1


def test_previous_phase_does_not_make_unexecuted_plan_executed():
    actions=[dict(before_physics_step=70,after_physics_step=100)]
    row=plan_execution_evidence(actions,1,100,100)
    assert row['command_executed'] is False and row['actions']==[]


def test_missing_after_counter_must_not_claim_unexecuted():
    actions=[dict(before_physics_step=100,after_physics_step=None)]
    row=plan_execution_evidence(actions,0,100,101)
    assert row['command_executed'] is None and not row['action_counter_coverage_complete']
    assert row['execution_status']=='unknown_incomplete_counters'


def test_physics_advance_outside_recorded_commands_is_unknown():
    row=plan_execution_evidence([],0,100,101)
    assert row['command_executed'] is None and not row['action_counter_coverage_complete']


def test_partial_execution_remains_true_with_later_incomplete_counter():
    actions=[dict(before_physics_step=100,after_physics_step=102),
             dict(before_physics_step=102,after_physics_step=None)]
    row=plan_execution_evidence(actions,0,100,103)
    assert row['command_executed'] is True and not row['action_counter_coverage_complete']
