# C06 human user revision — 2026-10-27

Flat grasp0deg completed before contact. Local continuous finger-position mapping preserves original0/1 targets and physical drives, then gradually unloads/opens over24fixed-arm actions before tracked100mm retreat; actual matched baseline99.9569mm with unloaded terminal. Same approved wrist44 person and safety thresholds.

Approved obstacle layout, source mechanisms and all original safety gates remain unchanged. Each planning round changed one factor, saved failures and deferred physics; only the qualified final candidate was physically collected. Legal grasp/control adaptation is explicit; old samples/rejections are retained.

Real sample: `datasets/doors/lwx_scene_approved_20261026/C06_human_h524`; matching baseline: `datasets/doors/lwx_scene_approved_20261026/C06_base_b515`. Matching baseline/variant neutrals, original contact-driven door task, source gap, door sweep, self collision, limits, RGB alignment, independent reviews and42/43/44paired necessity all passed. 485frames, final door -9.965deg, TCP max route deviation 8.77cm. Video decode passed. User has not accepted this new execution video yet.

`manifest.json` and `control.json` reproduce the fixed layout/goal/control; `commands.json` preserves exact original commands (use fresh output names for reproduction; existing data is never overwritten). `evidence.json` records full report hashes/locations. Data/media stay outside Git; restore referenced runtime assets from their original tracked paths or a separate asset bundle. A fixed-layout single sample and three planner seeds do not imply batch success rate or global path necessity.

Further provenance, action-interface details, retained C04 selection and failure boundaries: [交付说明.md](交付说明.md). Runtime binary asset SHA256s: [runtime_assets_manifest.json](runtime_assets_manifest.json).
