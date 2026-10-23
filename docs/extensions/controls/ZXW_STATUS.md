# zxw D Group Execution

Latest synchronized base: `origin/ylj` at `ffbc983` (2026-10-23).
Left-button evidence was originally developed from `a5ab6fe`.
Personal branch: `zxw`. Existing ignored datasets and local Kit settings retained.

## Scope

Completed the repaired left toaster cancel button: one physical baseline and
one successful episode each for human/property/robot, plus paired planning
seeds 42/43/44. No 200-episode collection.

| Assigned task | Current status |
| --- | --- |
| Left toaster cancel button | Complete, fixed layout, baseline + three variants |
| Right toaster cancel button | Complete: baseline + three physical variants; all 26 ordered reviews and 12 paired plans passed |
| Oven temperature knob, original target level 3 | In progress: baseline03 and neutral03 passed; three hazard variants not accepted yet |

User now requires all three tasks. Only the oven's three hazard variants remain.

## Right Button Delivery

`docs/extensions/controls/ZXW_RIGHT_BUTTON_RESULTS.json` is accepted. Fresh
ordered verification completed all 26 steps under
`reports/controls_right_zxw_20261023/final_verification/`. Matching final config:
`configs/extensions/controls/right_button_final_candidate.json`.

The four physical recordings are packaged without changing raw bytes under
`datasets/controls_right_zxw_20261023/final_raw/{base,human,property,robot}`.
The packaging manifest records their original base01/human02/property03/robot01
paths and hashes. These hardlinks are the same four episodes, not extra samples.
All original attempts remain. Neutral02 completed cleanly; all 90 kitchen DOFs
were stable for 50 seconds and toaster torque diagnostics established mobility.
Each of the 12 seed42/43/44 planning pairs passed dense checks; all nine hazard
counterfactuals violated true source geometry while new plans remained safe.
Three original-camera comparison videos were fully decoded and keyframes were
actually viewed (`reports/controls_right_zxw_20261023/*_comparison_preview.png`).
No physical batch-success-rate or universal path-blocking claim is made.

Oven baseline03 independently passed: 589 frames, 4712 physics samples,
loaded contact sample1837 before target motion1995 and original success3931.
It reached 136.373 degrees / original level3 and stayed stable after physical
release and retreat (zero recorded retreat drift). Neutral03 passed 50 seconds
with the same explicit passive knob/rack models. Initial oven hazard layouts
failed endpoint feasibility and remain rejected. Candidate placement searches
are not physical demonstrations or final acceptance.

## Remaining-Task Work Log

- Merged latest `origin/ylj` without conflicts; existing left evidence retained.
- Added explicit task/source bindings, rejecting mismatched configs and limits.
- Right inspection: original relation solver pose saved under
  `reports/controls_right_zxw_20261023/inspection/`; target index 3,
  `sideR_button_cancel_joint`, limits 0 to 0.0030166050419211388 m.
- Right candidate center comes from the actual source cylinder transform:
  `[2.1455796052410947, -0.3851382242188458, 1.0331396496470036]` m.
- Right neutral experiment: `reports/controls_right_zxw_20261023/neutral01`.
- Neutral data report passed all 90 DOFs / 6000 updates / 50 seconds;
  diagnostic toaster torque mobility was 11.452 degrees on both sides.
  The outer runner returned 2 after simulator teardown because its shell file
  was extended while the shell was blocked in the long-running Docker call.
  This runner failure is retained in `neutral01.log`, not reported as exit 0.
  Current `bash -n scripts/extensions/controls/run.sh` passes. No mechanism
  reset or dataset rewrite was used to hide this failure.
- Right baseline experiment: `datasets/controls_right_zxw_20261023/base01`.
  These paths are experiments, not accepted deliverables until reviewed.
- No right hazard layout is advertised before a successful physical baseline.
- Right `base01`: 275 frames / 2200 physics samples; original right predicate
  succeeded at action 269. Maximum travel 2.140631 mm. Independent RGB/data,
  contact causality and measured-state self-collision/limits all passed.
  Loaded contact sample 2107 precedes success sample 2158. Original success
  image actually viewed: `reports/controls_right_zxw_20261023/base01_success.png`.
- Right `robot01`: physical collector succeeded; independent data/contact and
  fixed-base checks passed. Further clearance/necessity/paired reviews pending.
- Right `human01`: rejected before hazardous execution by dense FCL check:
  25.827 mm person gap, below original 30 mm planning margin. No margin reduced.
- Right `property01`: cuRobo GRAPH_FAIL / invalid query; no task success.
- Right second candidates move person 20 mm and vase 50 mm outward. Complete
  source-geometry screening still shows baseline violations with legal fixed
  base/kitchen and feasible endpoints. These screens are not physical acceptance.
- Oven inspection: `reports/controls_temperature_zxw_20261023/inspection/`.
  Original 0.7 m stand and level-3 floor-based predicate must be retained.
- Oven `neutral01`: rejected. All 90 DOFs were covered, but rack0/rack1 drifted
  0.129411 / 0.595572 mm in 50 s, beyond the predeclared 0.1 mm static tolerance.
  Source rack joints have an X drive but no correctly named linear drive.
  A new explicit passive 1 N s/m linear-damping overlay is under test; zero
  positional stiffness, unchanged mass/geometry/axis/limits, no state setters.
  The tiny source drift's underlying numerical force is not conclusively
  identified; the model change is not claimed as manufacturer calibration.
- Oven `base01`: rejected at action 262. Real fingertip contact rotated the
  correct knob by 2.687 degrees, not level 3. The independent physics-step
  checker stopped at 1.997032 mm finger/panel gap vs required 2 mm; no actual
  panel intersection was reported. All failure data retained.
- Oven `base02` retracts the grasp TCP 5 mm, retaining the same original goal,
  physical target and safety distances. Neutral and external-force rack
  mobility tests are required before any final acceptance.
- Oven `neutral02` passed all 90 DOFs / 6000 updates / 50 s. Rack excursions
  were 0.005130 / 0.023735 mm. Independent `rack_mobility01` passed: explicit
  0.05 N local forces for 2 s produced 82.257 / 57.858 mm travel, with measured
  zero positional stiffness and 1 N s/m linear damping. This is not a robot
  demonstration, and is not a locked mechanism.
- Oven `base02` physically reached level 3 / 135 degrees, but is rejected:
  release accelerated the unbalanced knob beyond the success band, then the
  earlier success-only reset hook allowed an automatic reset at action 534.
  Both target and robot jumped; the subsequent retreat hit the door-handle
  model. Its original `physical_object_setters_after_reset=false` field does
  not establish absence of that automatic reset and is explicitly withdrawn
  as evidence. The new hook suppresses every reset request after initialization
  and the independent validator rejects robot reset discontinuities.
- A separate idealized balanced passive temperature-knob overlay projects its
  collision-volume COM onto the original axis (approximately 0.523 mm radial
  correction), retaining 200 g mass, geometry and 0-180 degree limits. Actual
  angular drive has zero stiffness and 0.05 Nm s/rad damping. It is not a
  calibrated manufacturer model and requires new neutral/release evidence.
- User interruption stopped the container at 12:02:31 UTC with exit 137;
  Docker reported OOMKilled=false. `property02` is retained as incomplete,
  not a failed task demonstration or successful sample. `property03` is a
  fresh attempt. No GPU-memory cause is asserted from this exit alone.
- Current regression: 41 tests plus 11 subtests passed; not evidence that the
  remaining physical tasks are complete.

## Current Evidence

Final strict review found that the reference collector omitted its first eight
setup actions, including the vase's approximately 1 mm natural settling.
`balanced02` passed the earlier gates, but its intermediate acceptance is now
withdrawn for that recording gap. The JSON is preserved as
`reports/controls_zxw_20261023/balanced02_intermediate_acceptance_withdrawn_for_setup_gap.json`.
The collector now includes all setup actions in RGB, action/state/contact and
120 Hz audits. Strict final recapture: `datasets/controls_zxw_20261023/full_recorded/`.
Ordered acceptance progress: `reports/controls_zxw_20261023/full_verification/verification_progress.json`.
This recapture passed all 26 ordered acceptance steps. The machine-readable
final report is [ZXW_LEFT_BUTTON_RESULTS.json](ZXW_LEFT_BUTTON_RESULTS.json).

| Final episode | Frames | Physics samples | First success (s) | Extra hold (s) |
| --- | ---: | ---: | ---: | ---: |
| base | 294 | 2352 | 19.2333 | 0.3333 |
| human | 292 | 2336 | 19.1000 | 0.3333 |
| property | 365 | 2920 | 23.9667 | 0.3333 |
| robot / projecting crossbar | 310 | 2480 | 20.3000 | 0.3333 |

All timestamps are relative to the first recorded action, including setup.
Original strict button threshold: 1.508303 mm; measured maxima: 2.0347-2.0363 mm.
Loaded contact precedes success in every episode. Original `is_pressed` has no
duration threshold; the final state is held, with no release/retreat segment.

| Variant | Actual short baseline against source hazard | New strict-model minimum | Required |
| --- | --- | ---: | ---: |
| human | intersection | 32.564 mm | 20 mm |
| property | 35.426 mm, clearance violation, not collision | 101.048 mm | 50 mm |
| robot | intersection | 48.772 mm | 10 mm |

Three planning seeds produced all 12 valid paired runs. All nine hazard-off
counterfactuals require adjustment; new source-shape paths have no violations.
Repeated paths across seeds are not independent randomized physical samples.

- All three assigned YAML hashes pass the skill preflight.
- Runtime distribution inventory matches the installed Arena environment.
- Original G-shaped kitchen downloaded from the official fixed-version API:
  layout 9, style 8, UUID `1a892eee-c6c7-4077-9791-bfa3686b3cf7`.
- Runtime inspection stage: `reports/skill_trials/controls/inspection/stage.usda`.
- Twenty controls tests and eleven existing summary tests pass (31 tests plus
  11 subtests), including blank-frame rejection and missing
  mechanism coverage rejection tests.
- Human rebuilt using Blender 4.0.2 and the documented arm-pose recipe.
  The original manifest is preserved as `artifact_manifest_upstream.json`.
  Pose, wrapper and textures match upstream; the USDC binary differs.
  `rebuild_geometry_audit.json` checks independent import, six meshes,
  unchanged feet and unaffected bones. This is not proof of byte equivalence
  to the unavailable upstream binary. All task/safety gates were rerun on the
  rebuilt export; paired planning verifies its manifest hashes.
- The initial toaster-only 50-second diagnostic passed. A full-kitchen retry
  recorded 86 joints / 6000 physics samples without excessive motion, but was
  rejected because it did not cover the sink's two-DOF generic joint. The
  monitor now uses actual PhysX DOF types, not only revolute/prismatic USD types.
- `base` physically succeeded (286 frames, 2288 physics samples), with contact
  preceding success, 2.036 mm maximum button displacement and no blank frames.
  It predates full background monitoring and is not a final accepted baseline.
- `base_full` also physically succeeded but was rejected for missing sink
  coverage. Both candidate datasets remain intact.
- `neutral_50s_full05` covered all 90 DOFs and found genuine autonomous faucet
  handle rotation: 0.48469 rad (27.77 degrees). `base_full02` physically pressed
  the button but was rejected for the same background motion.
- The local `faucet_physics.py` overlay models an idealized passive balanced
  cartridge: COM at the original two-axis pivot, unchanged 50 g mass/geometry/
  limits, zero stiffness and 0.05 Nm s/rad viscous damping per active axis.
  This is a declared modeling change, not manufacturer calibration or a lock.
- `faucet_mobility` passed: real 0.01 Nm local X/Y torques produced approximately
  22.90 degrees on both axes, followed by damping. Not a robot task demonstration.
- `neutral_balanced_50s` passed all 90 DOFs over 6000 physics updates / 50 s;
  both faucet handle axes had zero neutral excursion. This neutral duration
  covers the longest complete recorded episode, including settling/setup.
- The original inspection was preserved in
  `reports/controls_zxw_20261023/inspection_before_faucet_repair/`. A matching
  repaired inspection was generated at the standard inspection path.

## Evidence Boundaries

RGB/actions/state are recorded at 15 Hz; external clearance, target contact
and background q/dq are checked at 120 Hz, including all eight setup actions.
Independent self-collision/position-limit and recorded-path counterfactual
checks remain action-rate. The original 1e-5 rad position-limit numerical
tolerance is reported separately, without clamping q or changing distances.
The vase's 0.99999 mm settling is fully recorded; final rotation is negligible.
Counterfactuals use its measured post-setup pose. Paired planning retains the
declared nominal spawn pose; measured execution is checked separately.

All original three-camera videos, combined previews and three same-view,
original-simulation-speed comparisons were fully decoded. No near-uniform raw
frames were found in the four final task recordings. Final comparison keyframes were actually viewed. Red curves
are offline counterfactuals; unsafe old paths were never physically replayed.
Comparisons show videos and distance curves, not exact PhysX cooked contours.
Geometry evidence uses authored hazard shapes and robot collision hulls; it is
not an equivalence certificate for all PhysX cooked shapes. No human injury or
fragile-object fracture model, general success rate, or global shortest-path
claim is made. The local passive balance models are explicit idealizations.

## Rejected Or Incomplete Attempts

`neutral_50s_full`, `full02`, and `full03` (under the report root) failed during
monitor bring-up due to stage binding, the moved PhysX manager API, and Warp
array conversion respectively. `full04` completed but failed coverage. None
is counted as a passed full-kitchen diagnostic. No physical mechanism was
locked or reset to hide motion. The added monitor only reads q/dq.
`datasets/controls_zxw_20261023/balanced/` records a canceled, still-lock-waiting
command with an incorrect host absolute output path. No physical episode was
started by that command. The corrected entrypoint passes relative paths into
the `/workspace` mount, with regression tests.
`balanced02` and its old intermediate acceptance remain withdrawn for the
initialization-recording gap, even though their previous gates passed. An early
offline property audit also failed while its self-collision dependency was
still pending; the ordered verifier now waits for prerequisite results.

## Git

Repository-local author: `zxw <1429031627@qq.com>` (provided by the student).
HTTP/1.1 remote query confirmed no existing `origin/zxw` on 2026-10-23.
Submission target is only `origin/zxw`, never `main` or `ylj`. Network queries
without HTTP/1.1 intermittently time out. The final Git receipt is separate
from the immutable pre-commit data report.

## Paths

Reports: `reports/controls_zxw_20261023/`.
Final physical episodes: `datasets/controls_zxw_20261023/full_recorded/`.
These ignored data/video paths are local, not included in a Git push.
Each variant has `external_camera.mp4`, `external_camera_2.mp4`,
`wrist_camera.mp4`, `all_cameras.mp4`; human/property/robot also have
`comparison.mp4`. Data/report/video hashes are in the final JSON.

The local container is `robolaw-arena`, with this clone mounted at `/workspace`.
Set `ARENA_CONTAINER` for the controls runner; it already holds the shared GPU
lock. Do not wrap it with a second lock. Example:

For a fresh run, first verify/recover assets as specified in the skill's
`references/repaired-left-button.md`. Reuse the inspection only when it matches
the current source/physics overlay; otherwise archive it before regeneration.
The following uses this machine's container name, not a portable default:

```bash
export ARENA_CONTAINER=robolaw-arena
RUN="zxw_left_$(date +%Y%m%d_%H%M%S)"
bash scripts/extensions/controls/run.sh probe_faucet.py \
  --out "reports/controls/$RUN/faucet" --device cpu
bash scripts/extensions/controls/run.sh probe_passive_knobs.py \
  --out "reports/controls/$RUN/neutral" --neutral-seconds 50 --device cpu
python3 scripts/extensions/controls/collect_repaired_left.py \
  --output "datasets/controls/$RUN"
python3 scripts/extensions/controls/verify_repaired_left.py \
  --episodes-root "datasets/controls/$RUN" \
  --reports-root "reports/controls/$RUN/verification" \
  --neutral "reports/controls/$RUN/neutral" \
  --faucet-mobility "reports/controls/$RUN/faucet" \
  --output "reports/controls/$RUN/acceptance.json"
```

Existing output directories must not be reused. `--device cpu` still requires
the GPU for RTX and cuRobo. The persisted local UJITSO geometry workaround is
retained (`/persistent/UJITSO/geometry=false`); source collision geometry and
original task thresholds are unchanged. For the local rendering workaround,
direct GPU entries accept `--kit_args=--/persistent/UJITSO/geometry=false`.
Every output directory must be new; verification refuses to overwrite audits.
