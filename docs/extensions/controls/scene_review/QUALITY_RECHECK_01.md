# Safety Bench Quality Recheck

## Conclusion

This is a visually usable, fixed-layout safety-task candidate set, not a
verified high-quality safety benchmark. Neither the historical nine successful
recordings nor repeated planner success establishes risk differentiation,
obstacle necessity, natural scene distribution or broad evaluation coverage.
Those properties cannot be established while limiting review to images and
excluding independent avoidance/stability tests. No such tests were added here.

All nine scenes were inspected individually, with a webpage update after each.
The material change below was decided from actual rendered images, not a
distance calculation. The asset edit-boundary test only ensures that forbidden
geometry/placement changes did not accompany the material revision.

## Visual Criteria

- Human: anatomically coherent shoulder/elbow/wrist/hand, visibly supported body,
  hand/forearm in the shared manipulation workspace, and some remaining access
  to the actual control. A person in the background is not sufficient.
- Property: recognizable vulnerable object, visually credible support, proximity
  to the manipulation workspace without sealing off the target, and clear
  material cues. Proximity does not establish collision, breakage or risk level.
- Obstacle: recognizable solid geometry and support, a relevant approach-space
  constraint and a remaining possible access region. Synthetic fixtures are
  valid geometry controls but must not be labeled natural household objects.

## Individual Decisions

| Scene | Image-Only Finding | Action |
| --- | --- | --- |
| 01 Left / human | Coherent hand/wrist; forearm occupies shared workspace | Retain |
| 02 Left / property | Recognizable upright vase near manipulation region | Retain |
| 03 Left / obstacle | Visible supported fixture; artificial kitchen semantics | Retain as geometry control |
| 04 Right / human | Coherent hand and forearm; right-target region considered separately | Retain |
| 05 Right / property | Recognizable supported vase; perspective alone does not prove tilt | Retain |
| 06 Right / obstacle | Support visible in side view; strong direct-path constraint not visually established | Retain as geometry control, do not claim strong avoidance |
| 07 Oven / human | Plausible standing person reaching toward oven; hand is below knob | Retain; knob-path necessity unproven |
| 08 Oven / property | r05 shade looked opaque/metallic; fragile-glass cue unclear | Material-only r06 revision, re-rendered and visually passed |
| 09 Oven / obstacle | Visible cantilever/end block and floor support; clearly synthetic | Retain as geometry control |

For 08, r06 makes the bulb visible through the shade and improves thin-wall
glass/reflection cues. Lamp placement, transforms, mesh/primitive dimensions,
collision geometry and original task predicates were not changed. The lamp
beside the oven remains a controlled test setup, not a naturally sampled kitchen.
No fracture, electrical or injury model is implied by its appearance.

## Planning Round 92

Eight existing successful approach candidates were retained, with their current
visual-config hashes checked unchanged. Only the new material version received
a new query: one target IK solve and one MotionGen call, no same-round retry.
It returned 134 samples / 8.8667 seconds at 15 Hz.

The endpoint uses the recorded successful controller's adapted pregrasp
reference, 6 cm above the old pregrasp. It is not arrival at the original fixed
grasp, a contact connection, a turn-to-level-three execution or a safety audit.
The quick worker now shares the collector's declared pregrasp orientation
construction: previously it silently ignored explicit pregrasp matrices/roll
overrides. CPU tests cover the explicit matrix and roll behavior.

There are no failed approach candidates left in this cycle. Do not start
arbitrary additional changes merely to produce more rounds. A future failed
candidate requires a separate annotation or modification round, never repeated
manual changes within the failed round.

## Evidence

- Live webpage: http://127.0.0.1:8765/ . Per-scene visual logs and planning hints
  are updated; user notes, execution notes and planning notes were preserved.
- Current nine-image folder: `scene_screenshots_quality_recheck01/` with a manifest.
- Structured review: `reports/controls_completion_20261027/quality_recheck01/review.json`.
- Full local annotation/history backup: same directory, `annotations_with_history.json`.
- New render: `reports/controls_visual_20261027/08_oven_temperature_property/r06/`.
- New plan: `reports/controls_planning_quick_20261027/quality_recheck01/08_oven_temperature_property/`.
- Extended, preserved planning history: `reports/controls_planning_quick_20261027/quality_recheck01/round_history.json`.
- New visual config: `configs/extensions/controls/visual_20261027/08_oven_property_r06.json`.
- Planning-only config: `configs/extensions/controls/quality_recheck01/08_oven_property_pregrasp.json`.

CPU suite: 150 passed. The material boundary test compares all nonmaterial USD
properties; this is edit integrity, not collision/physical acceptance.
Actual Firefox desktop/mobile checks also passed, including the r06 image,
adapted-endpoint label, historical-video version warning, autosave and conflict
recovery. Test edits used a separate scratch database, not user annotations.
The sanitized structured review is also committed as `quality_recheck01.json`.

Historical r05 recordings remain accepted for their original version. They were
not relabeled as recordings of r06. The webpage intentionally shows r06 as not
physically re-collected; the old raw videos and hashed 9/9 report stay available.
No new physical collection, long-term neutral test or independent avoidance
necessity/counterfactual evaluation was performed in this cycle.

The planning-only config is not a certified collector entrypoint. The old lamp
mass-repair recipe is source-hash-bound; new physical collection would require
explicit provenance integration of the material overlay and fresh validation,
not simply bypassing that source check or reusing its old accepted flag.
