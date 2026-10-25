"""Per-task constants for Lightwheel pick-and-place B-group executors.

B01 lime, B02 lemon, B03 bananas, B04 ladle.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]

LIME_TASK = "kitchen_bench_lightwheel_place_lime_in_wooden_bowl_l_shaped_scandinavian"
LEMON_TASK = "kitchen_bench_lightwheel_pick_lemon_shelf_on_plate_u_shaped_with_island_farmhouse2"
BANANA_TASK = "kitchen_bench_lightwheel_place_bananas_in_wooden_bowl_u_shaped_with_island_farmhouse2"
LADLE_TASK = "kitchen_bench_lightwheel_place_ladle_in_plate_g_shaped_large_scandinavian"


@dataclass(frozen=True)
class TaskSpec:
    task_id: str
    kitchen_registry: str
    kitchen_cache_rel: str  # under ~/.cache/lightwheel_sdk/floorplan/
    layout_id: int
    style_id: int
    pick_object: str
    destination_object: str
    rigid_objects: tuple[str, ...]
    source_surface_suffix: str
    # Pregrasp TCP offset from pick geometry center (metres, world-aligned axes).
    pregrasp_offset_m: tuple[float, float, float]
    # Placement release offsets (metres) above destination root.
    transfer_z_m: float
    lower_z_m: float
    # Local body z-range (metres) for finger/object contact checks.
    grasp_body_z_range: tuple[float, float]
    stand_output_basename: str
    default_fixed_layout: Path
    default_variants: Path
    # Extra picks for parallel multi-object tasks (B03 dual bananas). Empty => (pick_object,).
    pick_objects: tuple[str, ...] = ()
    # Absolute env prim for PhysX source-support when not under kitchen (bananas on plate).
    source_surface_abs: str | None = None


TASKS: dict[str, TaskSpec] = {
    LIME_TASK: TaskSpec(
        task_id=LIME_TASK,
        kitchen_registry="lightwheel_kitchen_l_shaped_scandinavian",
        kitchen_cache_rel="robocasa-robocasakitchen-3-8/scene.usd",
        layout_id=3,
        style_id=8,
        pick_object="lime",
        destination_object="wooden_bowl",
        rigid_objects=("lime", "wooden_bowl", "broccoli", "sweet_potato", "red_bell_pepper"),
        source_surface_suffix="/counter_main_main_group/top_geometry_right",
        pregrasp_offset_m=(0.0, 0.0, 0.125),
        transfer_z_m=0.18,
        lower_z_m=0.135,
        grasp_body_z_range=(-0.001, 0.061),
        stand_output_basename="lime_droid_original_stand",
        default_fixed_layout=ROOT / "configs/extensions/lightwheel_pick_place/fixed_layout.json",
        default_variants=ROOT / "configs/extensions/lightwheel_pick_place/variants.json",
    ),
    LEMON_TASK: TaskSpec(
        task_id=LEMON_TASK,
        kitchen_registry="lightwheel_kitchen_u_shaped_with_island_farmhouse2",
        kitchen_cache_rel="robocasa-robocasakitchen-7-10/scene.usd",
        layout_id=7,
        style_id=10,
        pick_object="lemon",
        destination_object="plate",
        rigid_objects=("lemon", "plate", "bowl", "orange", "apple"),
        source_surface_suffix="/shelves_main_group/corpus/board_0",
        # Approach from robot side (-Y), not from above into board_1.
        pregrasp_offset_m=(0.0, -0.17, 0.0),
        transfer_z_m=0.08,
        lower_z_m=0.055,
        grasp_body_z_range=(-0.001, 0.061),
        stand_output_basename="lemon_droid_original_stand",
        default_fixed_layout=ROOT / "configs/extensions/lightwheel_pick_place/fixed_layout_lemon.json",
        default_variants=ROOT / "configs/extensions/lightwheel_pick_place/variants_lemon.json",
    ),
    BANANA_TASK: TaskSpec(
        task_id=BANANA_TASK,
        kitchen_registry="lightwheel_kitchen_u_shaped_with_island_farmhouse2",
        kitchen_cache_rel="robocasa-robocasakitchen-7-10/scene.usd",
        layout_id=7,
        style_id=10,
        pick_object="banana_1",
        destination_object="wooden_bowl",
        rigid_objects=("banana_1", "banana_2", "wooden_bowl", "plate", "apple", "orange"),
        source_surface_suffix="/plate",
        source_surface_abs="/World/envs/env_0/plate",
        pregrasp_offset_m=(-0.13, 0.07, 0.02),
        transfer_z_m=0.18,
        lower_z_m=0.135,
        grasp_body_z_range=(-0.001, 0.061),
        stand_output_basename="banana_droid_original_stand",
        default_fixed_layout=ROOT / "configs/extensions/lightwheel_pick_place/fixed_layout_bananas.json",
        default_variants=ROOT / "configs/extensions/lightwheel_pick_place/variants_bananas.json",
        pick_objects=("banana_1", "banana_2"),
    ),
    LADLE_TASK: TaskSpec(
        task_id=LADLE_TASK,
        kitchen_registry="lightwheel_kitchen_g_shaped_large_scandinavian",
        kitchen_cache_rel="robocasa-robocasakitchen-9-8/scene.usd",
        layout_id=9,
        style_id=8,
        pick_object="green_ladle",
        destination_object="plate",
        rigid_objects=("green_ladle", "plate", "spoon_1", "spoon_2", "spoon_3"),
        source_surface_suffix="/counter_main_main_group/top_geometry",
        pregrasp_offset_m=(0.0, 0.0, 0.125),
        transfer_z_m=0.14,
        lower_z_m=0.08,
        grasp_body_z_range=(-0.001, 0.061),
        stand_output_basename="ladle_droid_original_stand",
        default_fixed_layout=ROOT / "configs/extensions/lightwheel_pick_place/fixed_layout_ladle.json",
        default_variants=ROOT / "configs/extensions/lightwheel_pick_place/variants_ladle.json",
    ),
}


def resolve_task(task_id: str | None = None) -> TaskSpec:
    key = task_id or LIME_TASK
    if key not in TASKS:
        raise KeyError(f"Unsupported lightwheel pick-place task_id: {key}")
    return TASKS[key]


def picks(spec: TaskSpec) -> tuple[str, ...]:
    return spec.pick_objects if spec.pick_objects else (spec.pick_object,)


def short_name(task_id: str | None = None) -> str:
    spec = resolve_task(task_id)
    if spec.task_id == LEMON_TASK:
        return "lemon"
    if spec.task_id == LIME_TASK:
        return "lime"
    if spec.task_id == BANANA_TASK:
        return "bananas"
    if spec.task_id == LADLE_TASK:
        return "ladle"
    return spec.pick_object
