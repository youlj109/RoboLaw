# Single-Change Rounds

## Current Status Correction

The 9/9 figure below is a historical count of planner-returned paths, not the
current completion count. Round 5 stops 24 cm short of the actual grasp point.
The stricter continuation retains eight approach paths and marks oven/property
failed until an appropriate original endpoint is reached. Rounds 7 onward and
annotation-only rounds are recorded in the newest `round_history.json` linked
on the local webpage. No contact action or full task execution is accepted.

The original account below is retained as historical evidence.

The previous round's eight generated approach paths were retained. Only oven
property remained pending. Subsequent rounds processed pending scenes in the
original order; after a failure the scene was not adjusted again within that
round. Each round used one explicitly declared modification, exactly one local
IK solve and at most one MotionGen query. The earlier multi-candidate endpoint
search was not used.

| Round | Only Modification | IK Solves | MotionGen Queries | Outcome |
| --- | --- | ---: | ---: | --- |
| 3 | Extra axial pregrasp stand-off 0.09 m | 1 | 1 | GRAPH_FAIL; leave to next round |
| 4 | Extra axial pregrasp stand-off 0.12 m | 1 | 1 | GRAPH_FAIL; leave to next round |
| 5 | Extra axial pregrasp stand-off 0.15 m | 1 | 1 | Generated approach trajectory |

The final path has 151 samples at 15 Hz and planned duration 10.0 s. Its selected
pregrasp TCP is `[5.02, -3.22609, 1.26588]` m, with the original orientation.
The original actual grasp TCP remains `[5.26, -3.22609, 1.26588]` m. Therefore
the remaining 0.24 m axial connection to actual grasp is **not generated or
validated**. There was no grasp, contact, turn-to-level-3, release, physical
execution, dataset collection or independent path/stability audit.

All scene configs/assets, obstacle positions, robot base, saved initial joints
and real contact targets stayed unchanged. Pre/post hashes match for all three
rounds. No collision margin was relaxed, and failed queries were not repeated
with another seed in the same round.

The approach-generation stage now has 9/9 planner-returned paths, using the new
farther pregrasp only for this scene. This is not 9/9 complete task executions.
Iteration stopped when no failed approach scene remained.

## Evidence

Root: `reports/controls_planning_quick_20261027/round_iterations_20261027/`.

- `round_history.json`: ordered round/action/result/query counts.
- `input_scenes.json`: the one pending scene at start.
- `round03/08_oven_temperature_property/`: first new failed attempt.
- `round04/08_oven_temperature_property/`: second new failed attempt.
- `round05/08_oven_temperature_property/`: generated path and kinematic preview.

Each attempt has `single_policy.json`, `result.json` and `worker.log`. Successful
output additionally has `positions.npy`, `positions.csv` and `preview.json`.
Earlier rounds and user annotation fields are retained. The local page displays
the round table, newest trajectory, downloadable evidence and editable notes.

The runner declares a finite sequence of single-parameter policies rather than
silently looping identical failures forever. Only the first three were needed;
remaining roll policies were not executed. A policy-exhausted state would be
reported as incomplete, not marked successful.

Command used:

```bash
env ARENA_CONTAINER=robolaw-arena python3 scripts/extensions/controls/repeat_plan_rounds.py --output reports/controls_planning_quick_20261027/round_iterations_20261027 --original-run reports/controls_planning_quick_20261027/run01
```

The output directory is immutable per run; do not reuse it for another attempt.
The web service stays running independently; the GPU jobs finished.

Tool verification: 26 scoped CPU tests passed, including the single-change,
single-IK contract. Real Firefox desktop/mobile tests passed the three-row round
table, newest preview, playback and automatic annotation saving/recovery. Tests
used a separate scratch database and did not alter user notes. Generated UI
captures use the `data/single_rounds_20261027_` prefix; current annotations/history
were exported as `data/single_change_rounds_20261027.json`. These are webpage
checks, not added physical or trajectory-clearance validation.
