# 2026-10-27 Visual-Only Layout Revision

All nine scenes were examined sequentially, one at a time. Each decision was
made by looking at the original and/or actual new rendered images. No numerical
collision/distance, planning, success-predicate, stability or damage result was
used as an acceptance gate. This is not a safety benchmark acceptance report.
The static renderer initializes/resets the original scene and then renders;
it sends no robot actions and performs no trajectory planning or data collection.

## Results

All nine are visually passed in the examined views, with the following scope:

| Scene | Change | Final Config In `configs/extensions/controls/visual_20261027/` | Final Render Round |
| --- | --- | --- | --- |
| Left / human | Retain body/hand; add whole-body view | `01_left_human_r01.json` | `01_left_cancel_human/r02` |
| Left / property | Retain vase; add side view | `02_left_property_r01.json` | `02_left_cancel_property/r01` |
| Left / obstacle | Move independent stand to reveal support | `03_left_obstacle_r01.json` | `03_left_cancel_obstacle/r01` |
| Right / human | Retain reach; add whole-body view | `04_right_human_r01.json` | `04_right_cancel_human/r01` |
| Right / property | Retain vase; add side view | `05_right_property_r01.json` | `05_right_cancel_property/r01` |
| Right / obstacle | Move stand; turn tube away from robot silhouette | `06_right_obstacle_r02.json` | `06_right_cancel_obstacle/r02` |
| Oven / human | Retain pose; add whole-scene/side views | `07_oven_human_r01.json` | `07_oven_temperature_human/r01` |
| Oven / property | Replace anonymous cylinder with recognizable reading lamp; reposition | `08_oven_property_r05.json` | `08_oven_temperature_property/r05` |
| Oven / obstacle | Retain supported geometric guard; add side/wide views | `09_oven_obstacle_r01.json` | `09_oven_temperature_obstacle/r01` |

Render roots are under `reports/controls_visual_20261027/`. Images are real
scene renders, not generated image edits or inpainted composites. Original
screenshots, assets, source configs, data and safety reports are untouched.
Extra review-camera transforms are temporary; source task cameras are unchanged.

Only three entities changed their actual layout/model:

- Left crossbar root: `[2.30, -0.40, 0.921]` m, yaw 0 degrees.
- Right crossbar root: `[2.33, -0.58, 0.921]` m, yaw 90 degrees.
- Oven reading lamp root: `[5.14, -3.73, 0.001]` m, yaw 0 degrees; fresh asset
  `assets/extensions/controls/visual_20261027/reading_lamp_r01.usda`.

These coordinates describe edits, not measured safety clearances. The lamp has
an open frosted shade, visible bulb/socket, arm, switch, post and weighted-looking
floor base. The material/geometry communicates fragile property visually only;
no fracture, electrical or thermal validation is claimed. The oven guard is
explicitly a synthetic geometric control, not a realistic kitchen item.

## Iteration Evidence

- Left human r01: feet obscured by stand/crop. r02 adds a body view; pose unchanged.
- Right obstacle r01: tube end overlaps/approaches the robot silhouette. r02
  rotates the complete stand; examined views separate the silhouettes.
- Oven property r01/r02: lamp identity improved, but bulb and arm relationship
  were inadequately shown. r03 movement worsened shade/gripper overlap and the
  low camera was blocked by the stand. r04 moves the lamp to the appliance side
  and fixes camera occlusion; r05 retains r04 layout and adds an upward view
  clearly showing the hollow opening and internal bulb.

Failed and insufficient images are retained in the web gallery and the complete
timestamped reasoning is in the database. No scene was passed from a script exit
code or metadata flag. Local web/storage tests concern the annotation tool only,
not scene acceptance.

## Viewing And Editing

Open <http://127.0.0.1:8765>. The original-image preliminary results are retained
as history, separately from the new visual-revision progress. Choose a scene,
switch between original/iteration/view thumbnails, and edit the annotation boxes
or personal notes. Autosave and conflict protection remain enabled. No user notes
were replaced by the assistant's separate visual-review log.

The editable record is `data/reviews.sqlite3`; `data/visual_only_20261027.json`
is a generated snapshot including edit history. Use the live export button for
later user edits. The UI/database remain local; nothing was committed or pushed
as part of this visual-only request.

## Reproducing A Static View

Annotation-tool verification: 13 storage/HTTP tests passed; the real Firefox
desktop/mobile smoke test passed autosave, refresh, draft recovery, conflict
handling and old/new image switching. All 66 registered review-image URLs were
readable. These checks validate the local webpage, not the scenes' safety.
New UI captures are `data/visual_20261027_page_desktop.png` and
`data/visual_20261027_page_mobile.png`; original UI captures were retained.

Use a fresh output directory; existing rounds are never overwritten:

```bash
env ARENA_CONTAINER=robolaw-arena bash scripts/extensions/controls/run.sh render_visual_scene.py --controls-task temperature --variant property --config configs/extensions/controls/visual_20261027/08_oven_property_r05.json --output reports/controls_visual_20261027/08_oven_temperature_property/recheck_new --device cpu
```

The new configs are explicitly marked `accepted_for_collection: false` inside
`visual_revision`. Historical successful recordings and safety acceptance do not
transfer to altered layouts/assets. Planning, effective avoidance, real contact,
original task success and physical stability were intentionally not tested here.
