# Hand Between Robot And Toaster: Layout Draft

Historical draft03, superseded by the independently accepted layouts/data in
[BETWEEN_DELIVERY.md](BETWEEN_DELIVERY.md). Draft03 itself remains rejected.

User-directed placement review, 2026-10-26. The initial revision was static-only.
Subsequently requested cuRobo planning tests rejected both draft03 goals; see
[HAND_BETWEEN_PLANNING_CHECK.md](HAND_BETWEEN_PLANNING_CHECK.md). No physical
collection was launched for these unsafe layouts. Previous accepted
configurations, demonstrations and reports remain unchanged. These drafts do
not inherit their acceptance status. Do not run collection until the layout is
confirmed and separately validated.

## Current Draft

Configs: `configs/extensions/controls/left_button_hand_between_draft03.json`
and `configs/extensions/controls/right_button_hand_between_draft03.json`.
Human asset: `assets/extensions/controls/human_hand_between01/collidable.usda`.
Original robot poses, task predicates, other hazards and controller are unchanged.

The body remains beside the workspace; the right arm reaches toward the middle
instead of moving the whole person into the stand/cabinet. Asset-space wrist
target is `[-0.72, 0.23, 1.16]` m, elbow down, source bend-plane frame and
forearm-relative wrist rotation preserved. The independent export audit passes
all seven checks, including unchanged feet and non-arm bones. Body yaw is zero.

| Task | Human root (m) | Wrist world (m, approximately) |
| --- | --- | --- |
| Left cancel | [2.820, -0.860, 0] | [2.100, -0.630, 1.160] |
| Right cancel | [2.885, -0.860, 0] | [2.165, -0.630, 1.160] |

The hand is aligned laterally with its corresponding button. Its Y position is
between the original robot base (-0.875 m) and the toaster buttons (-0.385 m).
This placement statement does not prove trajectory obstruction or necessary
avoidance. That determination is deliberately deferred.

## Static Previews

- [Left main view](/home/zxw/RoboLaw/reports/controls_layout_20261026/left_draft03/layout_external_camera.jpg)
- [Right main view](/home/zxw/RoboLaw/reports/controls_layout_20261026/right_draft03/layout_external_camera.jpg)

Both original reverse cameras are occluded by the new body placement (mostly
black images) and are not useful placement previews. Their raw images are kept;
no camera was moved to hide this limitation. Main-view images were actually
inspected and show the extended hand in front of the toaster beneath the gripper.

`layout_review.json` beside each preview records static FCL geometry distances.
The preview runner only resets and renders the scene, checks unchanged robot q,
and inspects initial geometry. It does not call MotionGen or send robot actions.
Source geometry checks are not a PhysX-cooked collision certification.

Left draft03 has no initial robot, stand or kitchen intersection. Its minimum
robot/person distance is 54.74 mm. The shoe/stand gap is only 5.02 mm and remains
an explicit layout concern, not a broad safety claim. Floor contact is expected.
Right draft03 also has no initial intersections: robot/person gap 38.49 mm,
shoe/stand gap 36.54 mm. Both report unchanged robot q during pure rendering.
All 58 controls tests passed in the container, including draft task/control
invariance, source-wrist checks and hand placement relative to robot/button.

## Rejected Placement Trials

Left draft01 used the previous bent-arm asset, yaw zero and root
`[2.70, -0.63, 0]`. Its shirt intersects a cabinet handle.
Left draft02 used yaw -30 and root `[2.63, -0.93, 0]`; its clothing intersects
the stand. Both were static-only trials, not executed trajectories. Their
configs and previews remain available. Corresponding right draft01/draft02
configs were created but not rendered or validated.

No commit/push or replacement of the accepted delivery is performed during this
collaborative placement review. Confirm the visual layout before further work.
