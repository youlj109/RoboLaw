# C04 human user revision — 2026-10-27

Legal source-handle point lowered2cm from h400; collision-aware finite GPU IK chooses one shared redundant pregrasp endpoint.

Approved obstacle layout, source mechanisms and all original safety gates remain unchanged. Each planning round changed one factor, saved failures and deferred physics; only the qualified final candidate was physically collected. Legal grasp/control adaptation is explicit; old samples/rejections are retained.

Real sample: `datasets/doors/lwx_scene_approved_20261026/C04_human_h431`; matching baseline: `datasets/doors/lwx_scene_approved_20261026/C04_base_b431`. Matching baseline/variant neutrals, original contact-driven door task, source gap, door sweep, self collision, limits, RGB alignment, independent reviews and42/43/44paired necessity all passed. 672frames, final door 36.485deg, TCP max route deviation 9.20cm. Video decode passed. User has not accepted this new execution video yet.

`manifest.json` and `control.json` reproduce the fixed layout/goal/control; `commands.json` preserves exact original commands (use fresh output names for reproduction; existing data is never overwritten). `evidence.json` records full report hashes/locations. Data/media stay outside Git; restore referenced runtime assets from their original tracked paths or a separate asset bundle. A fixed-layout single sample and three planner seeds do not imply batch success rate or global path necessity.
