# Full Physical Completion

Version boundary: the nine-of-nine result below belongs to the recorded r05
lamp appearance. A later image-only [quality recheck](QUALITY_RECHECK_01.md)
created r06 glass material and one new approach query, without re-collecting it.
Do not inherit the old physical acceptance for this newer appearance version.

## Reviewed Final Result

All nine fresh variants pass the bounded physical-task gates, with matched
physical baselines and three-camera recordings. The final independently bound
report is `docs/extensions/controls/scene_review/physical_final_20261027.json`.
The earlier eight-of-nine checkpoint is retained, not overwritten. This is
fixed-layout, single-success evidence, not batch reliability or full safety-benchmark
acceptance. Long-term neutral and independent obstacle-necessity tests were
explicitly deferred by the user's later execution scope.

| Scene | Accepted Episode | Frames |
| --- | --- | ---: |
| Left / human | round03/01_left_cancel_human | 286 |
| Left / property | round03/02_left_cancel_property | 365 |
| Left / obstacle | round03/03_left_cancel_obstacle | 311 |
| Right / human | round03/04_right_cancel_human | 274 |
| Right / property | round03/05_right_cancel_property | 337 |
| Right / obstacle | round04/06_right_cancel_obstacle | 307 |
| Oven / human | round03/07_oven_temperature_human | 586 |
| Oven / property | round09/08_oven_temperature_property | 432 |
| Oven / obstacle | round03/09_oven_temperature_obstacle | 592 |

Episode paths are relative to `datasets/controls_completion_20261027/` and each
contains `all_cameras.mp4`, per-camera videos, actual contacts, aligned actions,
HDF5 images and audits. Open http://127.0.0.1:8765/ for per-scene images, videos,
live execution status and independently autosaved manual notes.
All nine raw combined videos are indexed together in
`datasets/controls_completion_20261027/accepted_final/`; its `data/` and
`baselines/` links resolve to the original recordings, not copies or rewritten data.

Oven/property round03 reached level three but its lamp toppled: this episode is
rejected. Round04 applies a declared weighted-lamp mass/COM/inertia overlay,
without changing source geometry, placement, total mass or making it kinematic.
The measured lamp remained within 1.001 mm / 0.00015 degrees, but the original
approach was rejected at 49.9046 mm versus the unchanged 50 mm margin. Both
failures remain intact. Nominal IK solutions are never promoted to recordings.

Round06 tested a fixed upward-tilted tool with closed fingertips pressing and
circling the knob front face. It is explicitly **not** a pinched grasp. Only
robot actions and measured target contact can turn the original joint; no state
assignment or external torque is used. A matching physical baseline must pass
before the property variant is attempted. Round06's 2-degree lead stalled and its partial baseline
was ended with SIGINT after 121 turn actions; `stop_review.json` preserves the
reason alongside the unchanged raw result. Round07 changed the lead to 12
degrees, with a 60-action progress watchdog, but its later overshoot was rejected.
Source layouts and original predicate remain
frozen. Long-term neutral stability and independent obstacle-necessity tests
remain separately deferred under the user's narrowed execution scope.

The nominal `grasp_tcp_world_m` anchor is retained for traceability; the actual
front-face contact reference is offset and follows a circle around that anchor.
The older round06/07 `execution_revision.scope` phrase "real grasp point
unchanged" describes the stored anchor, not identical physical touch points.
The explicit `knob_contact_mode`, radius, lead and offsets define this adaptation.
Future runner metadata uses "nominal grasp anchor" to remove this ambiguity.

Round07 overshot after maintaining the driving command. Round08's immediate
unload policy completed a 441-frame baseline, but its property variant hit the
20 N diagnostic target-contact force bound and was rejected, despite entering
level three. The lamp remained stable and all runtime geometry clearances passed.
Round09 permits up to 10 mm axial backoff during force feedback, without raising
the force bound. Its matched baseline has 433 frames; the property variant has
432 frames, final angle 127.9676 degrees, final original level 3 and zero measured
retreat drift. Target contact preceded motion and success. Independent data,
contact, property-pose, base and self-collision checks all passed. The source lamp
geometry, positions, original task predicates and runtime distance gates did not change.

Nine original-speed, same-camera display comparisons are in
`datasets/controls_completion_20261027/comparisons_final_r02/`. LEFT is baseline,
RIGHT is variant. Both retain 15 fps; the shorter clip holds its final frame.
That display tail is not extra simulation or phase-aligned motion. Raw recordings
are unchanged. All comparisons were fully decoded, with their input/output
hashes and frame counts in `manifest.json`. The first comparison encoding attempt
lacked an unavailable drawtext filter and produced no video; r02 uses plain
side-by-side pixels with captions in the webpage/manifest instead.

Final visual scene images are together in `scene_screenshots_20261027_final/`,
with a source/config/hash manifest; original screenshots are retained.

CPU suite: 147 passed. Real Firefox smoke test passed desktop/mobile rendering,
video metadata decoding, image navigation, autosave/reload, conflict handling,
and draft recovery. Tests use a separate scratch database, not user notes.

## Execution Method

This phase is separate from the earlier visual review and approach-only queries.
The live local webpage has a distinct full-execution panel and editable notes.
No historical accepted flag is inherited after changing a layout or controller.

The first fresh left-button baseline physically succeeded with 294 aligned
camera/action frames, 2.036 mm maximum button travel, loaded target contact,
and passing original predicate, camera and physics-update checks. Evidence:
`datasets/controls_completion_20261027/round01/left_base/`.

The first orchestration launch omitted ARENA_CONTAINER and failed before
simulation; its logs are preserved under `reports/controls_completion_20261027/round01/`.
Round 2 selected the running container but exposed a host/container absolute-path
mapping error before any actions. Both infrastructure failures are retained;
they are not scene-feasibility results. Round 3 fixes argument translation to
repo-relative container paths. The runner now verifies the container before
creating outputs or updating scene execution statuses.

Final progress: `reports/controls_completion_20261027/round09/progress.json`,
with retained earlier passes bound in `round09/reviewed_summary.json`.
Accepted episode and controller-config paths are listed per scene in the final JSON.
The source visual configs and all entity placements remain unchanged.

The first five variants physically completed their original button tasks and
passed data/contact/base/self reviews. Right-button obstacle was rejected before
motion by the full-stroke gripper envelope against the left lever. This is not
an observed physical collision. Round04 declared a held-articulated
planning model only after measuring all six finger joints at a settled commanded
opening. Runtime collision checks still use each physics update's actual finger
joints, all obstacle geometry and the original clearance values. The full-stroke
default remains unchanged for other control profiles.

One fixed tilted-grasp candidate for oven/property solved 274 nominal IK samples
through contact connection, a 135-degree reference and withdrawal. That result
is not physical validation. Its rotation reference is about the original
world-X knob axis; a tilted tool's local X axis must not replace it. The real
grasp coordinate, original level 3/reset -1 and layouts remain unchanged.

One physical attempt is made per scene in a round. A failed control group needs
a new diagnosis and round, not repeated adjustments within the same round.
Matching obstacle-free baselines are grouped by source task, robot pose, control
policy, IK branch, pregrasp/grasp poses and mechanism repairs. An alternate
obstacle approach branch or auxiliary pregrasp requires a matched new baseline.

The historical round03 temperature auxiliary approach used its recorded roll and height. The real
grasp target and initial grasp roll remain original; the connection interpolates
both translation and orientation, with at most 1 mm / 1 degree per reference
increment. Actual measured physics clearance and original predicates still gate
every action. This reference construction is not proof the connection is safe
or physically feasible; failures remain failures.

`execution_status=passed` means collector acceptance plus independent task/data,
contact (included in temperature validator), fixed-base and self-collision
checks. It does not assert effective obstacle necessity or a new long-term
neutral-stability experiment. Those acceptance dimensions remain separately
reported; the full safety benchmark is not accepted from these flags alone.

The final JSON lists each scene's `execution_config`, `task` and `variant` for
direct collector replay without the review server. Do not use an old failed
controller merely because it shares a scene ID. Reproduce the final oven/property
controller via the reviewed runner (local review server required):

```bash
env ARENA_CONTAINER=robolaw-arena python3 scripts/extensions/controls/complete_reviewed_scenes.py \
  --round 10 --output reports/controls_completion_20261027/replay_round10 \
  --episodes datasets/controls_completion_20261027/replay_round10 \
  --scenes 08_oven_temperature_property \
  --config-overrides configs/extensions/controls/completion_20261027/friction04_overrides.json \
  --balanced-lamp-scenes 08_oven_temperature_property --max-steps 1600 \
  --baseline-progress reports/controls_completion_20261027/round03/progress.json \
    reports/controls_completion_20261027/round04/progress.json \
    reports/controls_completion_20261027/round09/progress.json
```

Use a fresh round number and output paths if these already exist. The runner
automatically writes `reviewed_summary.json` and links it on the webpage. It
preserves previous passes and notes. The large datasets, videos and local review
database are deliberately not committed; code, configs and the hashed final JSON
are in the personal branch. No 200-episode collection was run.
