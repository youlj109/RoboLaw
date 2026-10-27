# Visual Quality Cycle 02

## Acceptance Boundary

The current nine scenes pass the requested **visual-candidate and quick-approach
planning** tier. They are not independently validated safety-performance or
new physical-task successes. "High quality" here is bounded by what this cycle
is allowed to inspect, not a claim that untested safety effects passed.

Visual gates were: coherent anatomy/material identity; visually credible support;
clear relationship to the task's shared workspace; some visible access rather
than a sealed/ambiguous target; and an explicit natural/synthetic scene role.
Gate decisions used actual rendered images only. File/bone/hash checks protect
edit integrity, not physical safety or clinical anatomy.

## Visual Work

Seven unchanged candidates were retained from cycle01. Two weak task relationships
were revisited sequentially, before planning:

- Right cancel / obstacle: yaw90 looked directed outward. New yaw45 r03 was
  rejected because it did not improve that relationship. Yaw-45 r04 was rejected
  for ambiguous tube/robot silhouette overlap, not labeled proven penetration.
  Yaw20 r05 passed after reviewing close and side images: the front-workspace
  constraint, anchored support and visible separation are clearer. Its side
  image is the main review image; the original-camera images remain available.
- Oven / human: the old hand reached toward the door below the controls. A new
  source-rig right-arm pose raises the wrist to the control-panel neighborhood.
  Body root, feet and arm lengths are retained, and the source wrist frame follows
  the forearm instead of being independently bent. Close, wide and side renders
  were inspected; r02 passed with the complete side view selected for review.

Human poses must look anatomically continuous and share the manipulation region;
property must be supported, recognizable and visually vulnerable; obstacles must
have identifiable support and constrain relevant space without simply sealing
the control. Synthetic fixtures are legitimate geometry controls, not naturally
sampled kitchen objects. Fragile appearance does not imply a fracture model.

Both accepted revisions were frozen **before round93**. No entity placement,
fixture orientation or human mesh changed during rounds93-105. Config hashes
are bound in `configs/extensions/controls/quality_recheck02/freeze_round93.json`;
the human's USD/textures are bound by its asset manifest. Historical scenes,
failed candidates, images, recordings and manual notes were preserved.

## Planning Rounds

Each modification round submits at most one candidate per still-failed scene;
an annotation round does not issue a path query. No same-round manual retry or
independent path/neutral-stability audit was added.

| Round | Right Obstacle | Oven Human |
| --- | --- | --- |
| 93 | Original controller reference: GRAPH_FAIL | Original reference: GRAPH_FAIL |
| 94 | Endpoint annotation only; start valid, goal world constraint | Same diagnosis; annotation only |
| 95 | One IK initial-guess change: IK failed, no MotionGen | Auxiliary point +6 cm: GRAPH_FAIL |
| 96 | Annotation only | Annotation only |
| 97 | Original branch, axial roll +45: GRAPH_FAIL | Retain height, change tool matrix: GRAPH_FAIL |
| 98 | Annotation only; elbow/forearm versus tube proxy hints | Annotation only; mesh AABB hints not actual contact |
| 99 | Mirrored IK initial guess: GRAPH_FAIL | Recorded alternate IK initial guess: GRAPH_FAIL |
| 100 | Annotation: world constraint zero but model self constraint | Annotation only |
| 101 | Retain mirrored branch, axial roll +45: GRAPH_FAIL | Retain pose, lateral reference -5 cm: GRAPH_FAIL |
| 102 | Annotation only | Annotation only; cap contact-anchor distance at 15 cm |
| 103 | Original branch, axial roll -45: GRAPH_FAIL | Only +3 cm more height: generated, 116 points |
| 104 | Annotation only; replace unconstrained fixed-IK goal policy | Successful candidate retained |
| 105 | Same TCP/orientation, one constrained pose query: generated, 207 points | Not rerun |

Candidate families explicitly name their parent reference. A failed IK initial
guess is discarded, not silently accumulated into every later candidate.
Across this cycle there were 13 declared candidates and 12 MotionGen calls;
round95's right candidate failed before path planning.

Right r105 keeps the original prepress TCP/orientation. One `plan_single` call
with `max_attempts=1` lets the SDK's unchanged 32-seed constrained-IK batch choose
a redundant arm posture; this is not 32 manual retries or 32 collected episodes.
It returned 207 samples / 13.7333 s, with about 0.055 mm model endpoint position
error. The raw SDK status retains `GRAPH_FAIL` from the intermediate graph stage,
while the worker's final SDK-success guard passed and exported the trajectory.
That raw status was not hidden or rewritten. Future worker reports additionally
record the explicit final `sdk_success` flag.

Human r103 returned 116 samples / 7.6667 s. It reaches an adapted, tilted pregrasp
reference +9 cm vertically and -5 cm laterally from the original pregrasp. Its
distance from the original true contact anchor is about 13.7 cm, within the
declared 15 cm bound. **It does not reach the original fixed pregrasp or prove
the later contact/turn-to-level-three connection.** That distinction remains
visible in the webpage and raw result.

Endpoint diagnoses use cuRobo's own model. Proxy/AABB overlap hints, especially
the whole-human mesh bounding box, are not physical collision or penetration
depth measurements. They were used only to choose planning changes, never to
approve scenes or assert benchmark avoidance necessity.

## Evidence And Limits

- Live review and autosaved notes: http://127.0.0.1:8765/ .
- Current nine images: `scene_screenshots_quality_recheck02/` with source hashes.
- Structured report: `quality_recheck02.json` in this document directory.
- Local report/history backup: `reports/controls_completion_20261027/quality_recheck02/`.
- Full planning history/results: `reports/controls_planning_quick_20261027/quality_recheck02/`.
- Current right visual config: `configs/extensions/controls/visual_20261027/06_right_obstacle_r05.json`.
- Current human visual config: `configs/extensions/controls/visual_20261027/07_oven_human_r02.json`.
- Successful query configs: `configs/extensions/controls/quality_recheck02/round105/06_right_cancel_obstacle.json`
  and `round103/07_oven_temperature_human.json` under the same directory.

The batch entrypoint now supports an explicit scene subset and per-scene initial
state/controller references. Overrides that move frozen task/entities are
rejected, all old artifact links are retained, and failure hints prohibit moving
hazards. Query-only controller fields/configs are not certified collector entrypoints.

Human query loading now checks the bound asset manifest and rejects missing or
changed payloads; it also refuses a human variant with no loaded human collision
objects. These are provenance/loading guards, not independent safety validation.

CPU suite: 160 passed. No new physical recordings, long-term neutral tests,
independent geometry/path audits or avoidance-necessity/counterfactual evaluations
were performed, as requested. The old nine-of-nine physical report belongs to its
recorded versions and is not inherited by these new scenes. The webpage labels
their old videos as historical. No 200-episode collection was started.
Actual Firefox desktop/mobile checks passed the two new side views, exact
207/116-point preview selection, auxiliary-endpoint counts, historical-video
labels, autosave persistence and conflict recovery. Browser edits used a fresh
scratch database, never the production manual annotation fields.
