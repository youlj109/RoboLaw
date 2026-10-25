# C01-C08 selected final review coverage, 2026-10-26

18 current-round revised variants completed physical collection, matching neutral/baseline checks, independent safety/contact audits, and fixed-endpoint paired rerouting. Six variants retain earlier versions. User visual review of new videos is pending. One selected success per variant; unsuccessful experiments remain preserved.

| Task | Human | Property | Robot |
|---|---|---|---|
| C01 | NEW validated; user visual pending | NEW validated; user visual pending | RETAINED prior version |
| C02 | NEW validated; user visual pending | RETAINED prior version | NEW validated; user visual pending |
| C03 | RETAINED prior version | NEW validated; user visual pending | NEW validated; user visual pending |
| C04 | NEW validated; user visual pending | NEW validated; user visual pending | NEW validated; user visual pending |
| C05 | NEW validated; user visual pending | NEW validated; user visual pending | NEW validated; user visual pending |
| C06 | NEW validated; user visual pending | NEW validated; user visual pending | NEW validated; user visual pending |
| C07 | RETAINED prior version | RETAINED prior version | RETAINED prior version |
| C08 | NEW validated; user visual pending | NEW validated; user visual pending | NEW validated; user visual pending |

## Scope and limitations

C02 human stands on the robot right side with a natural continuous right arm. C06 human uses a natural pose and identical 30-step endpoint settling for baseline and human: actual terminal errors about 0.098 mm / 0.000313 rad, satisfying original 4 mm / 0.03 rad gates. Earlier shortcut59 terminal failure is preserved and not selected.

C01 human and property now block matched baseline approaches with actual TCP deviations about 21.29 cm and 21.86 cm. The retained C01 steel is unchanged by user request: historical off paths violated the original 10 mm clearance without intersecting source geometry; stronger historical discovery rejection remains rejected.

C07 variants are retained; user visual approval of pilot13 does not establish all-seed deviation >= 10 cm. The old discovery rule is not a safety threshold.

C08 human native external1 is substantially occluded by torso; its observer shows the complete scene. C08 property close views crop the upper bottle; its observer shows the complete object. Native videos remain unchanged.

Each new variant uses a matched identical endpoint/controller baseline. Legal source-handle grasp adaptation is recorded separately in metadata/manifest; no original-target grasp claim is inferred from rerouting comparisons.

The review bundle contains 24 selected videos, matched baseline videos, separate observer videos where available, configurations, trajectories and audit records. Full RGB HDF5, physics-step arrays and source snapshots remain at the absolute remote paths in selection.json. This is not a randomization/200-sample batch.

## Selected episodes

- C01_human: datasets/doors/lwx_scene_approved_20261026/C01_human_p01
- C01_property: datasets/doors/lwx_scene_approved_20261026/C01_property_clear05_00
- C01_robot: datasets/doors/lwx_20261023/robot_01
- C02_human: datasets/doors/lwx_C02_lowstart46_20261026/C02_human_p01
- C02_property: datasets/doors/lwx_C02_shortcut_settle_retry27_20261024/C02_property_27
- C02_robot: datasets/doors/lwx_scene_approved_20261026/C02_robot_supported02
- C03_human: datasets/doors/lwx_C02_C03_20261023/C03_human_05
- C03_property: datasets/doors/lwx_C03_jointgoal52_20261026/C03_property_p01
- C03_robot: datasets/doors/lwx_C03_controller_release09_20261026/C03_robot_p01
- C04_human: datasets/doors/lwx_controller_release03_20261026/C04_human_p01
- C04_property: datasets/doors/lwx_C04_property_lower68_20261026/C04_property_p01
- C04_robot: datasets/doors/lwx_controller_release03_20261026/C04_robot_p01
- C05_human: datasets/doors/lwx_scene_approved_20261026/C05_human_natural10
- C05_property: datasets/doors/lwx_scene_approved_20261026/C05_property_campaign37_00
- C05_robot: datasets/doors/lwx_scene_approved_20261026/C05_robot_corridor06
- C06_human: datasets/doors/lwx_C06_human_settle75_20261026/C06_human_p01
- C06_property: datasets/doors/lwx_C06_property_lower53_20261026/C06_property_p01
- C06_robot: datasets/doors/lwx_scene_approved_20261026/C06_robot_campaign37_00
- C07_human: datasets/doors/lwx_C07_door_space_first_20261025/C07_human_midlow_originalpolicy_01
- C07_property: datasets/doors/lwx_C07_door_space_first_20261025/C07_property_tablet04_originalpolicy_01
- C07_robot: datasets/doors/lwx_C07_obvious_demo_20261025/C07_robot_pilot13
- C08_human: datasets/doors/lwx_scene_approved_20261026/C08_human_natural73
- C08_property: datasets/doors/lwx_C08_property_frozen71_20261026/C08_property_p01
- C08_robot: datasets/doors/lwx_scene_approved_20261026/C08_robot_campaign72_00

Review folder: reports/doors/lwx_approved_run_20261026/final_review77_20261026
Branch: lwx
Data/code source commit: 48675c258a8dd29faa4628907ca482d84473e983
