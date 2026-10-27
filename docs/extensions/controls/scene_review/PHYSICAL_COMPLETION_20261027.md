# Full Physical Completion

## Reviewed Checkpoint

Eight of nine fresh variants pass the bounded physical-task gates. This is not
nine of nine, nor full safety-benchmark acceptance. The independently bound
checkpoint is `reports/controls_completion_20261027/round04_reviewed_summary.json`.

| Scene | Accepted Episode | Frames |
| --- | --- | ---: |
| Left / human | round03/01_left_cancel_human | 286 |
| Left / property | round03/02_left_cancel_property | 365 |
| Left / obstacle | round03/03_left_cancel_obstacle | 311 |
| Right / human | round03/04_right_cancel_human | 274 |
| Right / property | round03/05_right_cancel_property | 337 |
| Right / obstacle | round04/06_right_cancel_obstacle | 307 |
| Oven / human | round03/07_oven_temperature_human | 586 |
| Oven / property | Not accepted; round07 controller baseline in progress | - |
| Oven / obstacle | round03/09_oven_temperature_obstacle | 592 |

Episode paths are relative to `datasets/controls_completion_20261027/` and each
contains `all_cameras.mp4`, per-camera videos, actual contacts, aligned actions,
HDF5 images and audits. Open http://127.0.0.1:8765/ for per-scene images, videos,
live execution status and independently autosaved manual notes.

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
before the property variant is attempted. This experiment has no acceptance
claim while it runs. Round06's 2-degree lead stalled and its partial baseline
was ended with SIGINT after 121 turn actions; `stop_review.json` preserves the
reason alongside the unchanged raw result. Round07 changes the lead to 12
degrees, with a 60-action progress watchdog. This baseline has started turning
the knob but is not yet accepted. Source layouts and original predicate remain
frozen. Long-term neutral stability and independent obstacle-necessity tests
remain separately deferred under the user's narrowed execution scope.

The nominal `grasp_tcp_world_m` anchor is retained for traceability; the actual
front-face contact reference is offset and follows a circle around that anchor.
The older round06/07 `execution_revision.scope` phrase "real grasp point
unchanged" describes the stored anchor, not identical physical touch points.
The explicit `knob_contact_mode`, radius, lead and offsets define this adaptation.
Future runner metadata uses "nominal grasp anchor" to remove this ambiguity.

CPU suite: 141 passed. Real Firefox smoke test passed desktop/mobile rendering,
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

Live progress: `reports/controls_completion_20261027/round03/progress.json`.
Actual episodes: `datasets/controls_completion_20261027/round03/`.
Controller-only derived configs: `configs/extensions/controls/completion_20261027/round03/`.
The source visual configs and all entity placements remain unchanged.

The first five variants physically completed their original button tasks and
passed data/contact/base/self reviews. Right-button obstacle was rejected before
motion by the full-stroke gripper envelope against the left lever. This is not
an observed physical collision. The next round declares a held-articulated
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

The temperature auxiliary approach uses its recorded roll and height. The real
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

Reproduction uses a fresh round/output name; do not overwrite earlier evidence:

```bash
env ARENA_CONTAINER=YOUR_RUNNING_CONTAINER python3 scripts/extensions/controls/complete_reviewed_scenes.py --round N --output reports/controls_completion_20261027/roundNN --episodes datasets/controls_completion_20261027/roundNN
```
