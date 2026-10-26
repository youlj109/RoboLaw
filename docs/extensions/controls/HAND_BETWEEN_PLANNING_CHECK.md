# Hand-Between Draft03 Planning Check

Historical rejection of the exact draft03 layout, not the later adjusted
layout. The later successful delivery is [BETWEEN_DELIVERY.md](BETWEEN_DELIVERY.md).

2026-10-26: user requested actual planning and a collection attempt if safe.
The exact left/right draft03 layouts were tested without moving the human,
robot base, cameras or target, and without lowering any safety distance.

## Outcome

| Task | Seed42 baseline | Seed42 human | cuRobo start/goal validity | Physical collection |
| --- | --- | --- | --- | --- |
| Left cancel | Passed | Failed: GRAPH_FAIL, valid_query=False | true / false | Not launched |
| Right cancel | Passed | Failed: GRAPH_FAIL, valid_query=False | true / false | Not launched |

The full recorded manipulation endpoint screen rejects both layouts, despite
their initial static state having no mesh intersection. The human occupies
the fixed-branch button operation pose, not only an intermediate approach.
The first left press state has only 4.648 mm source-geometry clearance versus
the unchanged 20 mm human requirement. Both hold states contain intersections.

Independent source-hull inspection of the historical baseline's hold pose
against the new human mesh resolves these intersections:

- Left: `panda_link5` and `panda_link6`, right forearm sleeve region.
- Right: `panda_link6`, right forearm sleeve region.

Representative offline intersection coordinates (world metres):
left link6 `[2.14535, -0.64994, 1.11288]`, right link6
`[2.21251, -0.63720, 1.11284]`. These are geometric witnesses, NOT contacts
recorded by physically executing a dangerous trajectory.

## Evidence

Under `reports/controls_layout_20261026/{left,right}_draft03/`:

- `full_layout_screen.json`: all recorded baseline manipulation states, source
  shape distances, illegal endpoint witnesses and fixed-base/kitchen checks.
- `planning_probe_seed42/paired_planning.json`: actual cuRobo results, identical
  task/control policy and seed, hazard-off success and hazard-on invalid goal.
- `hold_collision_links.json`: exact robot hull link and source human mesh,
  intersection coordinate and nearest source-rig bones.

These are rejected diagnostic attempts, not accepted avoidance demonstrations.
No new human data/video were collected and no old recordings were overwritten.
Seeds43/44 were not run because the same declared fixed goal already collides;
this is not a three-seed stability claim or proof that every IK branch fails.
The planner script exits zero after writing individual run failures, so the
per-run success/validity fields, rather than process exit status, were checked.

## Interpretation

With the current fixed IK branch and exact draft03 layout, planning/collection
cannot pass the safety gates. More free-space route samples cannot repair an
occupied goal. A different lawful robot posture for the same button TCP may
be feasible, or the human forearm may need a placement adjustment while keeping
the hand between robot and toaster. Either change requires a new matched
baseline/policy review or new layout validation; neither was silently applied
in this check. All original thresholds remain unchanged.
