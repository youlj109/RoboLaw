"""Parallel execution order must preserve the original task and evidence primary."""
import pytest
from kitchen_safety.extensions.lightwheel_pick_place.task_config import BANANA_TASK,episode_spec,resolve_task,source_yaml_path

def test_reverse_order_preserves_native_task_and_selects_correct_primary():
 original=resolve_task(BANANA_TASK)
 spec=episode_spec({'task':BANANA_TASK,'pick_object':'banana_1','pick_objects':['banana_1','banana_2']})
 assert spec.pick_object=='banana_1'
 assert spec.pick_objects==('banana_1','banana_2')
 assert set(spec.pick_objects)==set(original.pick_objects)
 assert source_yaml_path(spec)==source_yaml_path(original)

@pytest.mark.parametrize('order,primary',[(['banana_1'],'banana_1'),(['banana_1','banana_1'],'banana_1'),(['banana_1','banana_2'],'banana_2')])
def test_missing_duplicate_or_misattributed_pick_cannot_change_native_scope(order,primary):
 with pytest.raises(ValueError):episode_spec({'task':BANANA_TASK,'pick_object':primary,'pick_objects':order})
