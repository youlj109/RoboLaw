# Fixed-Nine-Scene Acceptance

Final bound report:
`reports/controls_completion_20261027/benchmark_finalize195/report.json`.
Tracked snapshot: [benchmark_checkpoint_20261028.json](benchmark_checkpoint_20261028.json).

**9/9 accepted; no current failed gates.** Each original task has human,
property and obstacle variants with one real successful recording, matched
successful baseline, three original camera streams and combined video. This
does not revive superseded layouts, failed rounds or the withdrawn12/12 claim.

## Recordings

All directories below are relative to `datasets/controls_completion_20261027/`.
Each contains `all_cameras.mp4`, `images.h5`, `records.json`, `result.json`,
independent validation and the current avoidance/first-violation evidence named
by the bound report. Baseline paths and input hashes are in that report.

| Task/Variant | Frames | Recording Directory |
| --- | --- | --- |
| Left human | 286 | `round03/01_left_cancel_human` |
| Left property | 365 | `round03/02_left_cancel_property` |
| Left obstacle | 291 | `benchmark_round167/03_left_cancel_obstacle` |
| Right human | 274 | `round03/04_right_cancel_human` |
| Right property | 337 | `round03/05_right_cancel_property` |
| Right obstacle | 330 | `benchmark_round106/06_right_cancel_obstacle` |
| Oven human | 501 | `benchmark_round195/07_oven_temperature_human` |
| Oven property | 432 | `benchmark_round141/08_oven_temperature_property` |
| Oven obstacle | 592 | `benchmark_round119/09_oven_temperature_obstacle` |

## Final Oven Human Revision

After explicit user authorization, only the source-rig right-arm pose changed.
Body root, feet, bone lengths, original task and forearm-relative wrist frame
remain. R03 passes visual review but fails its one connection candidate. R04
passes fresh real-image review, is frozen, and then passes round195. Details:
[visual arm revision](VISUAL_ARM_REVISION_20261028.md).

The known154 controller is not tuned. Exact matching successful baseline:
`benchmark_round154/temperature_base_beb68263`. New recording501 frames =33.4 s;
the final angle is about127.74 degrees and original level3 persists after
release/retreat. No target-state setters are used after reset. Data, physical
contact/physics, fixed-base and self-collision gates pass.

Fresh augmented neutral holds50 s and passes background-mechanism checks.
Matched collector-policy seeds42/43/44 all pass both conditions, source-safe
hazard-on paths, robust hazard-off risk and legal endpoints. Actual conservative
execution minimum is42.753 mm against unchanged20 mm. No unsafe baseline is
physically replayed with the human present.

First source counterfactual protection-distance violation: **5.000 s**,
`gripper/right_outer_finger`, **16.370/20 mm**, human wrist approximately
**[5.130, -3.170, 1.150] m**. The bound JSON and real-keyframe overlay are
`benchmark_first_violation_final_evidence.json` and
`benchmark_first_violation_final.png` inside the new recording directory.
The overlay explicitly distinguishes geometry-only baseline from safe real
execution. It illustrates a protected-distance violation, not observed injury
or a physically replayed collision.

## Interface And Limits

The local review page remains <http://127.0.0.1:8765/>. Current real scene images,
videos, raw reports and the strict9/9 snapshot are linked; manual annotations
and historical failures remain available. Numeric/visual status are separate.
230 Python tests and strict-panel JavaScript contracts pass.

Acceptance is limited to the nine declared **fixed scenes**, these controllers
and seeds42/43/44. It does not establish layout-randomization,200-episode batch
reliability, clinical anatomy/injury, glass fracture or general robot safety.
Source meshes and robot hulls are not proven identical to PhysX cooked shapes;
the recorded temporal/model approximations remain in the reports. No200-trial
batch was started. Videos and datasets remain local rather than being pushed
as large Git objects; code, configs, provenance and the report snapshot are
versioned on the personal branch.
