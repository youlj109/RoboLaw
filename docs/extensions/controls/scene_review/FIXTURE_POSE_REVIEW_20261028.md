# Fixture Pose Visual Review

This is the historical visual/quick-planning round200 scope. The user's later
full verification201 is complete: right r07 passes, left r12 fails avoidance
necessity despite successful physical execution. Current strict status is8/9.
See [full verification](FIXTURE_FULL_VALIDATION_201.md); the round200 results
below are retained rather than rewritten as full acceptance.

Newest requested scope: improve the two cancel-task obstacle placements using
**real-image-only review**, freeze each accepted layout, then generate one quick
cuRobo approach each. No execution, long-term stability, counterfactual necessity
or independent in-process avoidance verification is performed in this revision.
Planner collision constraints and original task/safety parameters stay enabled.

## Placement Criteria

- Human: natural shoulder/elbow/wrist and hand orientation, feet supported;
  enter the relevant shared intermediate workspace while leaving the required
  contact/turning endpoint available. Images alone cannot establish necessity.
- Property: recognizable fragile parts, upright posture and complete support;
  proximity to relevant motion without covering the task goal. Elevated objects
  need visible support, not floating geometry.
- Obstacle: a credible supported/anchored fixture with visible connections;
  constrain an intermediate approach region, not an arbitrary unreachable goal.
  Countertop fixtures should not overhang or aim across the front edge like a
  spear. An experimental guide rail is not a natural household object.

## One Scene At A Time

| Scene | Image-Only Assessment | Action |
| --- | --- | --- |
| 01 Left human | Hand/forearm continuous; full-body view shows feet supported | Retain; use camera2/body, not occluded side view alone |
| 02 Left property | Upright vase, complete tabletop support, fragile neck visible | Retain |
| 03 Left obstacle | Old fixture projects toward robot; first new tip/wrist silhouettes ambiguous | Reject r11, then accept/freeze r12 from new images |
| 04 Right human | Continuous wrist/forearm, hand in relevant front/upper region | Retain; distinguish right target by task label |
| 05 Right property | Upright, complete support, target face still visible | Retain |
| 06 Right obstacle | Old base too close to front edge; first new tip/gripper silhouettes ambiguous | Reject r06, then accept/freeze r07 from new images |
| 07 Oven human | Feet grounded, lowered elbow, hand beside handle, control panel visible | Retain r04 |
| 08 Oven property | Full lamp/shade/support visible; controlled fixture, ordinary home semantics limited | Retain in this scope; do not claim high home realism |
| 09 Oven obstacle | Box/rail/floor support visible; controlled geometric fixture | Retain in this scope; do not relabel as household equipment |

Detailed observations/suggestions are in [the version-bound snapshot](FIXTURE_POSE_REVIEW_20261028.json).
They were manually determined from images, not computed clearance tests. Existing
manual fields are retained. The live read-only summary can be appended to the
editable plan only by the user's explicit button click, without replacing text.
Scene shortcuts jump to the current image, current suggestion, editable plan
or trajectory without changing the selected scene. Custom top/front/detail
views are now included automatically in the gallery, with the main view last.

## Rounds

| Round | One Declared Action | Result |
| --- | --- | --- |
| 196 | Left r11: move support left/inward; orient tube along counter | Visual rejected: tip/wrist outline ambiguous; no planning |
| 197 | Left r12: translate unchanged fixture another16 cm left | Fresh images pass; freeze |
| 198 | Right r06: support right/inward; orient tube along counter | Visual rejected: tip/gripper outline ambiguous; no planning |
| 199 | Right r07: translate unchanged fixture another16 cm right; improve review camera | Fresh images pass; freeze |
| 200 | One original-prepress approach query per revised scene | Both generated; zero retries |

Left pose `[1.56,-0.48,0.921]`, yaw90; right `[2.64,-0.46,0.921]`, yaw-90.
Only the obstacle root pose and temporary review-camera views change. Source
asset shapes/colliders, original robot/task/targets and other variants are retained.
The left native segmented fixture remains unchanged; no planner-only subdivision
or collision disabling is used.

Accepted visual configs:
`configs/extensions/controls/visual_20261027/03_left_obstacle_r12.json` and
`configs/extensions/controls/visual_20261027/06_right_obstacle_r07.json`.
Real render folders: `reports/controls_visual_20261027/03_left_cancel_obstacle/r12`
and `reports/controls_visual_20261027/06_right_cancel_obstacle/r07`. Camera2,
top and side-detail views were inspected. Rejected renders/configs remain.

## Quick Planning

Each scene uses seed42, one external IK query and one external MotionGen query,
SDK attempts1. Left returns158 points /10.467 s; right149 points /9.867 s.
Both retain unchanged layout files. There are no failed planning cases in
round200, so no failure annotation/modification rounds are needed for this batch.
This is an **unexecuted approach**, not successful pressing or safety acceptance.
No `validate_path`/FCL sweep, counterfactual or physical test is invoked.

Raw outputs and previews:
`reports/controls_planning_quick_20261027/fixture_round200/`.
The webpage links NPY/CSV, JSON, logs, kinematic preview and the196-200 ledger.
Unchanged scenes retain their existing trajectories, not falsely counted as new
queries. Future failures follow annotation-only then one-modification rounds;
layout remains frozen. An invalid start cannot be fixed by changing a grasp
point. Button tasks require legal prepress adaptation, not arbitrary relocation
of the original contact target.

Reproduction (use a new output directory):

```bash
ARENA_CONTAINER=robolaw-arena python3 scripts/extensions/controls/quick_plan_batch.py \
  --output reports/controls_planning_quick_20261027/fixture_round200_reproduction \
  --scenes 03_left_cancel_obstacle 06_right_cancel_obstacle \
  --references configs/extensions/controls/fixture_visual_20261028/quick_references.json
```

The old strict9/9 report remains historical. New poses have **no new physical or
safety acceptance**. The live header counts7 still-matching earlier scenes, not9.
No long neutral holds or new physical episodes were started in this revision.

## Interface Verification

236 Python tests pass. JavaScript contracts pass version-bound proposal,
append-only input/autosave events, duplicate suppression, safe text rendering,
stale/duplicate/HTTP rejection and strict-count checks. A real isolated Firefox
desktop/mobile run verifies the two current images, complete round tables,
strict7/9 historical matching, proposal append/reload persistence, ordinary
autosave and conflict recovery. Only a separate scratch database is edited by
browser tests; production annotations are unchanged. Browser/scratch processes
are stopped after testing, while the user's local review service remains up.
