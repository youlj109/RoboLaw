# Button Human Arm Revision

## Cause And Scope

Both historical button human episodes used `human_reach_elbow_up`.
The pose generator moved the forearm but restored the hand's original world
rotation. The resulting forearm/hand angle was approximately 65.77 degrees,
rather than the source rig's approximately 16.50 degrees. The shoulder's axial
orientation also did not explicitly preserve the source elbow bend plane.
This is a visual/structural defect even though the historical button predicates
and contact checks passed. Those videos remain available as historical data,
not as the corrected visual delivery.

The replacement asset is `human_reach_anatomical_down01`: align the source and
target elbow-plane frames, bend the elbow downward, and transport the source
hand-relative-to-forearm rotation with the forearm. Original skin weights and
preserve-volume deformation are retained. No reflection, negative scale,
robot/camera motion, or per-frame hand animation is used.

Independent export checks passed for six meshes, 2948 unchanged foot vertices,
unchanged non-arm bones, preserved shoulder position and arm lengths, and the
source wrist frame. Wrist bend is 16.502992 degrees before and 16.503004 degrees
after; target wrist error is below one micrometre. This is source-rig geometric
consistency, not clinical anatomical validation or an injury model.

## Layout And Rejected Candidates

The new arm occupies different geometry. Keeping the old root with the initial
wrist-only revision failed the manipulation endpoint screen for both buttons;
that candidate was not physically executed. The elbow-up bend-plane candidate
was also rejected visually. These assets and diagnostic reports are retained.

The selected left human root is `[2.90, -0.86, 0]`, yaw -30 degrees.
The selected right human root is `[2.94, -0.90, 0]`, yaw -30 degrees.
Relative to the old layout this moves the left human +4 cm in X and Y, and the
right human +4 cm in X. Feet and shoulder are unchanged within the asset, but
their world positions move with this explicitly declared root adjustment.
The source task, robot pose, TCP, controller, IK branch, safety margins and
property/obstacle variants are unchanged. Both full baseline manipulation
endpoint screens pass. Unsafe counterfactual paths are checked offline only.

The initial right candidate reused the left root and failed all three dense
planning checks: a proxy arm distance of 29.435 mm was below the unchanged
30 mm planning requirement. No physical playback was attempted. A right-task
search retained 15 candidates; rank 1 was the first ranked candidate with at
least 50 mm sampled endpoint clearance (73.887 mm; rank 0 had only 35.820 mm).
The selected candidate passed the full endpoint screen and all twelve paired
plans. Rejected planning reports remain in `right_paired01`; accepted plans
are in `right_paired02`. No threshold, cost, target or IK branch was changed.

## Reproduction

From the repository root, with Blender 4.0.2 and the original posed.blend:

```bash
blender --background assets/people/mpfb_corridor/posed.blend \
  --python scripts/extensions/controls/repose_human_arm.py -- \
  --wrist -.6 0 1.2 --elbow-down --arm-frame source-bend-plane \
  --hand-orientation follow-forearm --asset-dir FRESH_ASSET_DIR
```

Then run `wrap_reposed_human.py`, the independent
`audit_human_export_blender.py`, and `prepare_hand_revision.py` for a fresh
revision name. The asset manifest binds generated geometry, textures, pose and
export audit. Legacy generator defaults remain available for old recipes;
existing posed exports cannot be overwritten.

Selected configs:
`configs/extensions/controls/left_button_anatomical_candidate01.json` and
`configs/extensions/controls/right_button_anatomical_candidate02.json`.
The candidate selection records the search report hash and rank.

`complete_button_hand_revision.py` runs fail-closed paired seed gates and
collects one new human episode per task, with independent physical/contact,
self-collision, fixed-base, strict clearance and avoidance reviews. Unchanged
matching baseline/property/obstacle episodes are reused, not recollected or
counted as additional demonstrations. The oven task is unaffected.

Verification progress and per-step logs:
`reports/controls_hand_revision_20261024/verification01/` (left passed; first
right planning rejected) and `verification02/` (right-only retry).
Revised raw episodes:
`datasets/controls_hand_revision_20261024/left/human01/` and
`datasets/controls_hand_revision_20261024/right/human02/`.
Final independent summaries passed:
[left_results.json](hand_revision/left_results.json) and
[right_results.json](hand_revision/right_results.json).
The new complete delivery index is
[ZXW_ALL_HAND_REVISION_RESULTS.json](ZXW_ALL_HAND_REVISION_RESULTS.json).
All three tasks pass in this index; only the two button human episodes changed.
Old raw recordings/reports are never overwritten.

## Measured Results

| Revised human episode | Frames | First success (s) | Maximum button travel (mm) | Minimum human clearance (mm) |
| --- | ---: | ---: | ---: | ---: |
| Left | 271 | 17.700 | 2.034 | 37.962 |
| Right | 272 | 17.792 | 2.141 | 40.913 |

Both exceed the unchanged original 1.5083 mm button threshold through recorded
robot contact. The actual human clearance requirement remains 20 mm. All eight
initialization actions are included. Contact precedes success; full external
physics-rate clearance/background checks, independent action-rate self checks,
fixed-base checks and matching-baseline avoidance reviews pass. Three-camera
videos and original-speed baseline comparisons were fully decoded. Actual
hold-state images from both camera views were visually checked along with the
standalone arm preview. Current controls tests: 54 passed in the real container.

Videos are `all_cameras.mp4` in the two raw episode directories above; matching
`comparison.mp4` files show the original baseline and new physical execution.
The red unsafe-baseline distance curve is offline geometry, not unsafe replay.
Published actual pose images are [left_actual_hold.jpg](hand_revision/left_actual_hold.jpg)
and [right_actual_hold.jpg](hand_revision/right_actual_hold.jpg); the isolated
[arm_preview.png](hand_revision/arm_preview.png) is an asset preview, not a demo.

The first violation witnesses, resolved robot links and human wrist coordinates
are bound in the new delivery index and its nine published evidence sidecars.
No accepted revision gate remains unverified. Rejected first-right planning and
earlier unnatural/endpoint-blocking poses are not counted as successful samples.
Geometry, textures and raw videos remain ignored local assets; Git contains
recipes, manifests, independent summaries and preview evidence, not full data.
