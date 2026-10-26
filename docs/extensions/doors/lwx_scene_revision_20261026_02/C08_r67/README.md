# C08 lower-bar revision r67

The approved steel bar remains at its reviewed position. A new matching physical baseline and obstacle episode use the same source handle point, 5 cm pregrasp stand-off, roll 45 degrees, pitch -10 degrees, seed 42 and fixed redundant endpoint. The earlier r53 free path missed the steel bar; its rejection remains preserved.

Both new episodes physically open the oven. Matching neutral controls and the task, full physical validation, source-contact and self-collision gates pass. Seeds 42/43/44 have paired off/on plans: every off path intersects the source bar geometry; every on path passes the independent geometry checks. The measured baseline counterfactual has 22 action-before/after intersection records. The actual obstacle episode has a minimum source clearance of 51.94 mm against the unchanged 10 mm requirement. TCP maximum difference at normalized pregrasp progress is 25.41 cm; this is a path-difference metric, not the necessity criterion.

Complete decode records and an agent review of synchronized keyframes are provided. The arm visibly raises over the bar during approach and then reaches the same handle. The user separately approved the revised static scene and subsequently accepted this execution video with “C08robot没问题”. The exact feedback and its scope are recorded in user_visual_acceptance83.json.

## Outputs

- Physical baseline: `datasets/doors/lwx_scene_approved_20261026/C08_base_b67`
- Baseline neutral: `datasets/doors/lwx_scene_approved_20261026/C08_base_n67`
- Physical obstacle episode: `datasets/doors/lwx_scene_approved_20261026/C08_robot_r67`
- Obstacle neutral: `datasets/doors/lwx_scene_approved_20261026/C08_robot_r67_neutral`
- Raw reports and exact executed commands: `reports/doors/lwx_scene_revision_20261026_02/approved_run67/C08/state67.json`
- Finite control selection and failed candidates: `reports/doors/lwx_scene_revision_20261026_02/approved_run21/C08/controls65.json`
- Original-speed observer comparison: `reports/doors/lwx_scene_revision_20261026_02/C08_review76/C08_base_vs_robot_observer.mp4`
- Local review page: `RoboLaw_outputs/lwx_scene_revision_20261026_02/C08_review76/index.html`

Data and videos are external artifacts, not included in Git. `result78.json` identifies independent report hashes. The source USD collision hulls and exact supported primitives do not imply cooked PhysX face identity. This is one fixed-layout sample; three planning seeds are not three physical samples or a bulk-collection success rate. C04 human/property and C06 human remain unfinished and are not covered by this result.

## Validation and reproduction

Five door-specific scripts propagate `pregrasp_distance_m` through collection, matching paired plans, the full trajectory screen and avoidance target verification. The default remains 0.13 m; range is 0.03--0.20 m. No safety threshold changes. Python compilation, diff whitespace checks, actual b67/r67 collections and the independent gates validate the new 0.05 m path.

Use the exact commands stored in the run state, selecting fresh output names; existing outputs intentionally reject replacement. The saved manifest and observer configuration are included here for provenance. Restore external assets and physical baselines before running audits. Do not replay the unsafe no-obstacle counterfactual with the obstacle enabled.
