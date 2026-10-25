# Approved scene corrections and validation, 2026-10-26

User approved prior static scene variants except C02/C06 human, requested natural arm correction and subsequent planning without another review pause. C02 human final natural18: upright at robot right, root [5.62,-1.55,0], yaw315. C06 human natural03: natural local wrist follows forearm, root [2.25,-.88,0], yaw225. C01 natural04: human/property frontal corridor, higher legal source-handle contact point, robot/stand retreated20cm to preserve door sweep. Original data, configurations and failed attempts are retained.

All18 changed variants in released manifests pass the finite CPU initial-world/source and 0..90deg sampled door-sweep checks. This is NOT cuRobo, physical or final visual acceptance. C01 support01 had a USD parse error and was invisible; support02 is API-authored, resolved and visible. Static checker now rejects unresolved supports.

C06 base01 reached native opening but failed original2mm door clearance during release; retained as failure. C06 base02 changes only matched release policy:3cm ease-out retreat and.03rad opening reserve. Collector result accepted; independent gates remain pending. Actual baseline counterfactuals of C06 show safety-margin violations for human/property/robot; this alone is NOT proof of successful/visible avoidance.

One-shot supervisor: scripts/extensions/doors/supervise_approved_scenes_lwx.py. All GPU stages take reports/skill_trials/gpu.lock. No periodic automation. Runtime state: reports/doors/lwx_approved_run_20261026/overnight_state01.json. Per-variant status preserves planning, physical, independent and visual outcomes separately. One successful episode per variant; no200sample batch. Legacy unmodified variants are not represented as newly revised.

Reproduction requires original source assets and referenced baseline metadata/data on the remote environment. Frozen released manifests here are copies of the exact runtime manifests; configs and new posed assets are source controlled separately.

Restore ignored new posed USD/textures inside the container with `/workspace/.venv-arena/bin/python scripts/extensions/doors/rebuild_natural_humans_lwx.py`. It verifies the original MPFB blend hash, uses the pinned Blender4.0.2 and specified joint rotations, and preserves existing files. Asset hashes describe actual runtime files, not a promise of byte-identical USD re-export. All large data/videos stay outside Git.

C06 base02 independent mechanism/contact/self-collision/full-validation gates passed, including matching40second neutral control. Human paired seeds42/43/44 passed; actual human collection remains in progress as of this checkpoint.
