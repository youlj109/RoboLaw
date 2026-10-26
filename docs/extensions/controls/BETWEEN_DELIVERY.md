# Human Hand In The Robot-Toaster Gap

Both revised human episodes and every independent gate passed. Current complete
index: [ZXW_ALL_BETWEEN_RESULTS.json](ZXW_ALL_BETWEEN_RESULTS.json). Individual
reports: [left](between_delivery/left_results.json) and
[right](between_delivery/right_results.json). Current controls tests: 65 passed
in the actual Arena container. No required revision gate is waived.

This revision addresses the request to keep the hand in the working gap while
collecting the original left/right cancel-button tasks. The initial draft03
layouts failed because their forearm sleeves occupied the fixed operation
poses; these rejected layouts and all old recordings are retained.

## Selected Layouts

Both final candidates use `human_hand_between_middle01`, rebuilt from the
original MPFB rig with asset-space wrist `[-0.72, 0.23, 1.23]` m, elbow down,
source elbow-plane mapping and source wrist-relative orientation preserved.
The independent export audit passes all seven gates, including unchanged feet,
non-arm bones and wrist frame. This is geometric rig consistency, not clinical
range-of-motion validation. The selected asset has no additional forearm roll.

| Task | Config | Human root (m) | Yaw |
| --- | --- | --- | ---: |
| Left cancel | `left_button_between_candidate05.json` | [2.865, -1.045, 0] | -29 degrees |
| Right cancel | `right_button_between_candidate02.json` | [2.845, -1.020, 0] | -30 degrees |

Configs are in `configs/extensions/controls/`. Hand placement is assessed from
the wrist, palm/fingers and actual camera views, not by forcing the wrist
marker onto the button centreline. Wrist positions are approximately
left `[2.347, -0.495, 1.230]`, right `[2.336, -0.461, 1.230]` m. The hand reaches
between the robot wrist and the toaster front/upper workspace. Both original
external views remain usable, unlike the earlier draft03 body placement.

No robot base, camera, button TCP, original predicate, speed, fixed IK branch,
planning clearance (human 30 mm) or execution clearance (human 20 mm) changed.
Posture/roll diagnostics were tried but no alternative robot policy was adopted.
The same original physical baselines remain valid matched controls. Unchanged
property/obstacle episodes and the oven task are reused, not newly collected.

## Verification And Data

Final selected paths were checked with planner seeds 42/43/44 and all four
roles (12 paired plans per button task). Unsafe hazard-off trajectories are
evaluated offline against the new source human meshes, never physically
replayed. Actual robot source hulls and articulated gripper geometry, not just
capsules, establish the baseline violations. Independent dense proxy checks
remain required for every new path, and no margin was relaxed.

Left pipeline: `reports/controls_between_20261026/verification_left05/`.
Right pipeline: `reports/controls_between_20261026/verification_right02/`.
The pipeline records its progress, every command and per-step log, binds config
fingerprints and fails before collection on any rejected screen/paired gate.
Its output roots are separate from all prior accepted reports and data.

Revised physical human directories:

- `datasets/controls_between_20261026/left/human05/`
- `datasets/controls_between_20261026/right/human02/`

Each directory contains actual camera videos, states/actions, physics-rate
contact/background/clearance audits, independent self-collision and fixed-base
checks, matching-baseline avoidance review, `comparison.mp4` and resolved first
violation evidence. A directory name alone is not acceptance; independent final
summaries must pass. Full data/video are local ignored files, not Git-uploaded.

### Measured Results

| Human episode | Frames | First success (s) | Maximum travel (mm) | Recorded-state minimum gap (mm) |
| --- | ---: | ---: | ---: | ---: |
| Left human05 | 286 | 18.700 | 2.035 | 34.228 |
| Right human02 | 274 | 17.925 | 2.140 | 33.480 |

These gap minima come from the strict before/after recorded-state audit at
action rate. Separate physics-update clearance checks all pass the unchanged
20 mm requirement; their exact minimum is not asserted by this table.

Both exceed the unchanged original strict 1.5083 mm button threshold through
real contact. Contact precedes success. Initial eight setup actions are recorded;
physics-rate external clearance/contact/background audits and independent
action-rate self/limit checks passed. Fixed-base/kitchen legality, all matching
control/source checks and all twelve paired plans per task passed. Each of the
nine hazard-off seed/role paths per task has a source-geometry violation and
every corresponding hazard-on source path is safe.

The first **offline baseline** source risks are left at 2.933 s, right at 3.000 s,
both `panda_link7`: 19.863 and 18.359 mm versus the 20 mm requirement. These are
safety-margin violations, not injury claims or physical collision playback.
The new physical episodes have no accepted clearance violation. Wrist coordinates
and source mesh witnesses are included in the index and published sidecars.

Original-time comparison videos, all three individual camera videos and combined
videos were fully decoded. Actual initial and hold-state views and both annotated
first-risk images were visually inspected. Actual hold images are included as
[left](between_delivery/left_actual_hold.jpg) and
[right](between_delivery/right_actual_hold.jpg); the full local videos are
`all_cameras.mp4` and `comparison.mp4` in each revised raw directory above.
The original button terminal semantics remain press-and-hold, not release/retreat.
This is one successful new human episode per task, not a batch-success estimate,
randomized environment validation or a moving-human injury certification.

## Rejected Attempts

Reports are retained under `reports/controls_layout_20261026/` and
`reports/controls_between_20261026/`:

- Draft03: source operation-pose intersections, invalid cuRobo goals.
- Local translations, raised/side elbow and opposite-side body searches:
  no eligible layout within their recorded search bounds.
- IK branch and 90/180-degree roll diagnostics: no screened lawful endpoint
  for the original draft03 human; no resulting motion was executed.
- Right candidate01, candidate03 probe and left candidate01: cuRobo returned
  paths, but independent dense gripper-sweep clearances rejected them.
- Left candidate02: source baseline remained safe (26.65 mm minimum), so this
  was not accepted as an avoidance augmentation.
- Left candidate03: initial source clearance fell below 20 mm.
- Left candidate04: capsule proxy risk existed but source meshes remained
  safe (34.88 mm minimum); rejected before planning/collection.

The source-only fine search found the chosen left candidate after millimetre
translation and one-degree yaw adjustments. The first returned eligible source
candidate was separately fully screened and paired-validated. A later optional
30-degree axial forearm-roll asset also passed its export audit and produced
screening candidates, but was not needed or used in the selected recordings.
Rejected candidates are not counted as successful data.

Generated USD wrappers retain their exporter's trailing blank line. Plain
`git diff --check` reports those seven generated EOF warnings; their bytes are
manifest-bound and are not reformatted after validation. No other whitespace
finding remains when that exporter-specific EOF rule is excluded.
