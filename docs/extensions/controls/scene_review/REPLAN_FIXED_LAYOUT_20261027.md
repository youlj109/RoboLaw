# Fixed-Layout Second Attempt

Only the three failures from `run01` were processed, sequentially: left obstacle,
right obstacle, then oven property. No other successful scene was regenerated.

## Results

| Scene | Outcome | Endpoint Candidates | MotionGen Calls | Change Selected |
| --- | --- | ---: | ---: | --- |
| Left cancel / obstacle | Generated | 9 | 1 | Redundant IK branch only |
| Right cancel / obstacle | Generated | 6 | 1 | Redundant IK branch only |
| Temperature / property | Failed; skipped | 64 | 0 | No eligible endpoint in declared family |

The two generated paths have 195/186 samples and durations 12.93/12.33 seconds.
Both preserve the exact original requested TCP point and orientation. Their
different arm joint goals—not moved obstacles or changed contact points—resolved
the previously rejected endpoint branch. One MotionGen call per selected goal,
`max_attempts=1`; no failed path query was retried.

For oven property, all 64 candidate IK attempts were consumed: 40 produced
endpoints rejected by the configured cuRobo constraints, 14 failed the local IK
solve and 10 repeated prior solutions. No eligible goal existed within that
declared candidate family, so no MotionGen query was made and no fallback path
was fabricated. This is a failure at endpoint selection, not proof that the
whole fixed-layout task is impossible.

Latest aggregate page status: 8 scenes with generated approach paths, 1 failed.
This does not count physical success or safe trajectories.

## Bounded Modification Policy

The candidate policy was declared before processing the cases:

1. Same TCP point and orientation, up to 16 deterministic IK seeds.
2. Same TCP point, axial tool-roll offsets +/-15, +/-30, +/-60 degrees.
3. Pre-contact stand-off enlarged by 3 or 6 cm along the original approach
   axis, combined with 0 or +/-30 degree tool roll.

At most 64 IK candidates and 25 seconds for endpoint selection; select the first
endpoint eligible under the same cuRobo graph constraints, then request exactly
one path. Endpoint filtering is distinct from path-planning retries. Duplicate
solutions and IK failures are logged and count against the candidate limit.

The button press axis, knob axis, actual contact target, original task predicate,
robot base and initial joint state were not modified. No contact-face offsets
were sampled in this round. Existing clearances were not relaxed. Config and
hazard-asset hashes were checked before/after and all remained unchanged.

Only prepress/pregrasp approach paths were generated. Neither their contact
transition nor press/turn actions were generated or physically tested. No
independent dense path clearance, stability, damage or task-success audit was
performed, per the requested quick scope.

## Files And Page

New, separate files are under:
`reports/controls_planning_quick_20261027/run02_fixed_layout/<scene-id>/`.

- `result.json`: outcome, unchanged layout hashes, exact original/selected TCP,
  selected candidate policy and MotionGen call count.
- `endpoint_candidates.json`: every attempted endpoint and rejection/IK status.
- `positions.npy` / `positions.csv`: only for the two generated paths.
- `preview.json`: selected-path kinematics, or start-only failure display.
- `worker.log`: full per-case log.

The page at <http://127.0.0.1:8765> displays the newest previews first. Previous
round files and the read-only endpoint diagnosis remain linked, and the database
history retains earlier statuses. User notes and manual review categories were
not rewritten. Failed oven property remains explicitly failed; it was not marked
generated from an invalid endpoint or fake straight-line interpolation.

Invocation used:

Verification of the tool changes: 24 scoped CPU tests passed, as did the real
Firefox desktop/mobile annotation/playback smoke test. New preview URLs were
readable, prior-round links remained present, and protected config/asset hashes
still matched the saved sources. This is tool/file verification, not an added
physical or path-clearance acceptance test.

```bash
env ARENA_CONTAINER=robolaw-arena python3 scripts/extensions/controls/quick_plan_batch.py --output reports/controls_planning_quick_20261027/run02_fixed_layout --refine-failed-from reports/controls_planning_quick_20261027/run01
```

Do not rerun into this directory. Another attempt requires an explicit new round;
this user-requested second attempt is finished, including the skipped failure.
