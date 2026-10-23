# zxw D Group Execution

Base: `origin/ylj` at `a5ab6fe5d45d6eb75ca62c51754aba07bc437e64`.
Personal branch: `zxw`. Existing ignored datasets and local Kit settings retained.

## Scope

Completed the repaired left toaster cancel button: one physical baseline and
one successful episode each for human/property/robot, plus paired planning
seeds 42/43/44. No 200-episode collection.

| Assigned task | Current status |
| --- | --- |
| Left toaster cancel button | Complete, fixed layout, baseline + three variants |
| Right toaster cancel button | Not implemented or physically validated |
| Oven temperature knob, original target level 3 | Not implemented or physically validated |

Next task: right cancel button. This submission does not claim all D tasks done.

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
