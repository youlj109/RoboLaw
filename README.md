# RoboLaw: 11 mustard bottle scenes

This repository contains the **scene definitions, obstacle meshes, and collection code** for one DROID kitchen task: pick up the mustard bottle and place it in the bowl. It includes 6 property obstacles, 4 robot workspace obstacles, and 1 default MPFB human scene. The task and three original robot cameras are shared across scenes.

| Group | Scene IDs |
| --- | --- |
| Property (6) | `property_tall_glass`, `property_wine_glass`, `property_glass_bottle`, `property_ceramic_mug`, `property_vase`, `property_tablet_stand` |
| Robot workspace (4) | `robot_steel_clamp`, `robot_projecting_crossbar`, `robot_narrow_opening`, `robot_sharp_equipment_housing` |
| Default human (1) | `human_default_mpfb` (`scenes/human_mpfb.usda`, `assets/people/mpfb_corridor`) |

The 10 nonhuman layouts are defined in `assets/obstacle_library/{property,robot}/catalog.json` and their collection settings are in `configs/reproduction_presets.json`. The default human position and asset are in `configs/layout.json`. `configs/obstacle_library.json` is a readable ten-scene index. The human ID is a release label, not an obstacle-library catalog entry.

This release **does not include the risk grading/evaluation system**, its rules, HS/PT/RS/IR results, or historical trajectories and videos. It retains collision planning and trajectory clearance checks needed to execute the task. Some collector options inherited from the source project refer to the omitted observer; do not enable `--observer-config` or use prior observer output directories.

## Source and validation status

The 10 nonhuman layouts completed fixed-seed (42), zero-jitter physical grasp and place runs with data and clearance checks in the source project on 2026-10-17. That establishes fixed-layout feasibility, not randomized success rates. The default human scene has been collected in the source project, but the exact standalone command below has not been rerun from this GitHub checkout. GPU, driver, simulator, and asset versions can change trajectories; inspect each fresh `result.json`, `verification.json`, and `audit.json`.

## Install

Requires Linux x86-64, an RTX-capable NVIDIA GPU, Docker Engine, NVIDIA Container Toolkit, and access to NVIDIA's Isaac Sim container and asset servers. Expect substantial downloads and disk use. No simulator runtime, Python environment, credentials, or NVIDIA binary assets are committed.

```bash
git clone git@github.com:youlj109/RoboLaw.git
cd RoboLaw
python3 scripts/restore_modelscope_assets.py
bash scripts/fetch_arena_source.sh
# Use a separate container name if the source project already uses arena-kitchen-dev.
export ARENA_CONTAINER=robolaw-arena
# Read NVIDIA's Isaac Sim license first. This flag records your acceptance.
bash scripts/setup_arena.sh --accept-eula
bash scripts/arena_python.sh scripts/check_runtime.py
python3 scripts/check_scene_assets.py --require-restored
```

`fetch_arena_source.sh` checks out the source revision used by the original collection. `setup_arena.sh` creates a persistent Docker environment, restores required NVIDIA assets from the official host, and installs cuRobo. The Lightwheel kitchen and default human binary assets are restored from [ModelScope](https://www.modelscope.cn/datasets/YouLinjing/Isaac-Arena-Kitchen-Safety); the downloader checks SHA-256. Lightwheel content is governed by CC BY-NC 4.0; see `THIRD_PARTY_NOTICES.txt`. The GitHub repository's MIT `LICENSE` does **not** relicense third-party assets or dependencies.

## Collect

First print the intended commands without starting simulation:

```bash
python3 scripts/run_robolaw_scenes.py --dry-run
```

Then run all 11 sequentially, or one selected scene:

```bash
bash scripts/arena_python.sh scripts/run_robolaw_scenes.py
bash scripts/arena_python.sh scripts/run_robolaw_scenes.py --scene property_vase
bash scripts/arena_python.sh scripts/run_robolaw_scenes.py --scene human_default_mpfb
```

Each invocation creates a new directory under `datasets/robolaw/`. The nonhuman batch collector writes its own manifest, verification, and audit files. The human run writes `result.json`, a three-camera recording, and is followed by the same standalone validation and clearance audit. Run one Kit simulation at a time; do not start a second collection on the same GPU/container.

## Scope and licenses

The restored `assets/kitchen` directory is Lightwheel content under CC BY-NC 4.0 and may require other individual asset terms. The default human was generated using MPFB/MakeHuman assets; source attribution is in `assets/people/SOURCES.txt`. The ten obstacle meshes were authored for this project. NVIDIA robot, bottle, bowl, stand, and material assets are downloaded from NVIDIA by `scripts/restore_downloaded_assets.py` rather than mirrored. The Arena and cuRobo source trees are fetched at pinned revisions. Read `THIRD_PARTY_NOTICES.txt` before redistribution or commercial use.
