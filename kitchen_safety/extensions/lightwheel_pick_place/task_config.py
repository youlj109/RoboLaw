"""Per-task constants for Lightwheel pick-and-place B-group executors.

B01 lime, B02 lemon, B03 bananas, B04 ladle, B05 Pepsi.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]

LIME_TASK = "kitchen_bench_lightwheel_place_lime_in_wooden_bowl_l_shaped_scandinavian"
LEMON_TASK = "kitchen_bench_lightwheel_pick_lemon_shelf_on_plate_u_shaped_with_island_farmhouse2"
BANANA_TASK = "kitchen_bench_lightwheel_place_bananas_in_wooden_bowl_u_shaped_with_island_farmhouse2"
LADLE_TASK = "kitchen_bench_lightwheel_place_ladle_in_plate_g_shaped_large_scandinavian"

PEPSI_TASK = "kitchen_bench_lightwheel_place_pepsi_in_basket_u_shaped_with_island_farmhouse2"

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
    lift_height_m: float = .16


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
        pick_object="banana_2",
        destination_object="wooden_bowl",
        rigid_objects=("banana_1", "banana_2", "wooden_bowl", "plate", "apple", "orange"),
        source_surface_suffix="/plate",
        source_surface_abs="/World/envs/env_0/plate",
        pregrasp_offset_m=(-0.13, 0.07, 0.02),
        transfer_z_m=0.18,
        lower_z_m=0.135,
        grasp_body_z_range=(-0.018335, 0.018335),
        stand_output_basename="banana_droid_original_stand",
        default_fixed_layout=ROOT / "configs/extensions/lightwheel_pick_place/fixed_layout_bananas.json",
        default_variants=ROOT / "configs/extensions/lightwheel_pick_place/variants_bananas.json",
        pick_objects=("banana_2", "banana_1"),
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
        grasp_body_z_range=(0.003, 0.021),
        stand_output_basename="ladle_droid_original_stand",
        default_fixed_layout=ROOT / "configs/extensions/lightwheel_pick_place/fixed_layout_ladle.json",
        default_variants=ROOT / "configs/extensions/lightwheel_pick_place/variants_ladle.json",
    ),
    PEPSI_TASK: TaskSpec(
        task_id=PEPSI_TASK,
        kitchen_registry="lightwheel_kitchen_u_shaped_with_island_farmhouse2",
        kitchen_cache_rel="robocasa-robocasakitchen-7-10/scene.usd",
        layout_id=7, style_id=10, lift_height_m=.10,
        pick_object="pepsi_can", destination_object="mini_plastic_basket",
        rigid_objects=("pepsi_can","mini_plastic_basket","tuna_can","mug","corn_can"),
        source_surface_suffix="/island_island_group/top_geometry_left",
        pregrasp_offset_m=(0.,0.,.08), transfer_z_m=.22, lower_z_m=.14,
        grasp_body_z_range=(.02,.10),
        stand_output_basename="pepsi_droid_original_stand",
        default_fixed_layout=ROOT / "configs/extensions/lightwheel_pick_place/fixed_layout_pepsi.json",
        default_variants=ROOT / "configs/extensions/lightwheel_pick_place/variants_pepsi.json",
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


def source_yaml_path(spec: TaskSpec) -> Path:
    return ROOT / "third_party/IsaacLab-Arena/isaaclab_arena_environments/kitchen_bench" / f"{spec.task_id}.yaml"


def distractor_objects(spec: TaskSpec) -> tuple[str, ...]:
    return tuple(n for n in spec.rigid_objects if n not in (*picks(spec), spec.destination_object))


def episode_spec(metadata: dict) -> TaskSpec:
    from dataclasses import replace
    spec=resolve_task(metadata.get("task"));primary=metadata.get("pick_object") or spec.pick_object
    if spec.pick_objects:
        order=tuple(metadata.get("pick_objects") or spec.pick_objects)
        if len(order)!=len(spec.pick_objects) or set(order)!=set(spec.pick_objects) or order[0]!=primary:raise ValueError("Episode must retain every original parallel pick and identify its actual primary")
        return replace(spec,pick_object=primary,pick_objects=order)
    if primary!=spec.pick_object:raise ValueError("Episode primary differs from original task")
    return spec


def pick_object_name(metadata: dict, spec: TaskSpec | None = None) -> str:
    spec = spec or episode_spec(metadata)
    return metadata.get("pick_object") or spec.pick_object


def destination_object_name(metadata: dict, spec: TaskSpec | None = None) -> str:
    spec = spec or episode_spec(metadata)
    return metadata.get("destination_object") or spec.destination_object


def initial_pick_pose(metadata: dict) -> list:
    if "initial_lime_pose" in metadata:
        return metadata["initial_lime_pose"]
    pick = pick_object_name(metadata)
    key = f"initial_{pick}_pose"
    if key in metadata:
        return metadata[key]
    raise KeyError(f"Episode metadata missing initial pick pose (tried initial_lime_pose and {key})")


def h5_pose_dataset(group, object_name: str, metadata: dict) -> str:
    pick = metadata.get("pick_object")
    dest = metadata.get("destination_object")
    candidates: list[str] = []
    if object_name in (pick, "lime"):
        candidates.extend([c for c in (pick, "lime") if c])
    elif object_name in (dest, "wooden_bowl"):
        candidates.extend([c for c in (dest, "wooden_bowl") if c])
    else:
        candidates.append(object_name)
    seen: set[str] = set()
    for name in candidates:
        if not name or name in seen:
            continue
        seen.add(name)
        if name in group:
            return name
    raise KeyError(f"Missing trajectory dataset for {object_name}; tried {sorted(seen)}")
