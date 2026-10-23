"""Explicit source bindings for the three assigned, unmodified Arena tasks."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
TASKS = {
    'left': ('press_left_toaster_cancel_button', 'toaster', 'sideL_button_cancel_joint',
             'toaster_main_group/Toaster015_sideL_button_cancel'),
    'right': ('press_right_toaster_cancel_button', 'toaster', 'sideR_button_cancel_joint',
              'toaster_main_group/Toaster015_sideR_button_cancel'),
    'temperature': ('turn_oven_temperature_knob', 'oven_temperature_knob', 'knob_temp_joint',
                    'oven_right_group/Oven038_knob_temp'),
}


def task_contract(key='left'):
    if key not in TASKS:
        raise ValueError(f'Unknown controls task: {key}')
    stem, asset, joint, body = TASKS[key]
    task_id = f'kitchen_bench_lightwheel_{stem}_g_shaped_large_scandinavian'
    source = ROOT / 'third_party/IsaacLab-Arena/isaaclab_arena_environments/kitchen_bench' / (task_id + '.yaml')
    return dict(key=key, task_id=task_id, source=source, asset=asset, joint=joint,
                contact_body=body)
