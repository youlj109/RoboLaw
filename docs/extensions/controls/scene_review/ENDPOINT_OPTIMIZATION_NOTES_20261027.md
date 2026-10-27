# Fixed-Layout Endpoint Optimization Notes

## Diagnosis

The three failed one-shot runs all logged `GRAPH_FAIL`, `valid_query=False`
and `Start or End state in collision`. A read-only check using the exact saved
start/goal, unchanged configs/assets, measured gripper opening and the same
cuRobo world resolved the ambiguity:

| Scene | Saved Start | Saved Fixed-Branch Goal | IK Constraint Result |
| --- | --- | --- | --- |
| Left cancel / obstacle | Valid | Invalid | Same flags |
| Right cancel / obstacle | Valid | Invalid | Same flags |
| Temperature / property | Valid | Invalid | Same flags |

"Valid/invalid" here refers only to cuRobo's configured proxy constraints, not
physical collision proof. No individual colliding link/shape or separate
world/self/limit cause was localized by this diagnostic. Evidence:
`reports/controls_planning_quick_20261027/run01/endpoint_diagnosis_01.json`.
Source config and top-level asset hashes stayed unchanged. No new MotionGen
query, candidate IK search, trajectory, physics playback or layout edit occurred.

The earlier generic "try a detour waypoint" hint is superseded: first obtain
an admissible goal configuration. More search or a waypoint alone cannot make
this exact rejected goal admissible. This does not prove that a different
configuration, approach pose or legal contact choice cannot solve the task.

## What Can Be Optimized

The current quick generator fixes the desired TCP pose, solves it with one
predeclared IK seed, then calls `MotionGen.plan_single_js` on that specific joint
goal. cuRobo is not doing grasp selection. Optimizing a grasp requires a candidate
layer that proposes a TCP frame/contact choice, obtains alternative IK solutions,
checks endpoint feasibility and only then requests a path for an eligible goal.

Prefer this order while keeping every object, robot base and initial state fixed:

1. Preserve the exact TCP point and orientation; try alternative redundant
   joint configurations (different elbow/wrist branches).
2. If needed, vary only task-legal TCP orientation freedoms, such as tool roll
   around the operation axis. Small angles such as +/-15 or +/-30 degrees are
   untested candidate examples, not demonstrated repairs.
3. Adjust the pre-contact stand-off along the original approach axis, keeping
   a separate transition to the unchanged actual contact target. A feasible
   pre-contact endpoint does not demonstrate a feasible contact transition.
4. Only then sample physically legal contact points within the target's actual
   contact surface. This is a new contact policy, not fixed-endpoint detouring.
5. Add intermediate waypoints only after the start and selected goal are valid.

No object/human/light/fixture movement, base teleport, initial-joint teleport,
collision disabling, margin reduction or target-DOF assignment is an acceptable
substitute. If no admissible endpoint exists in the declared candidate family,
report that family exhausted; do not claim the entire fixed-layout problem is
mathematically impossible.

## Per-Scene Annotation Prompts

### Left Cancel / Obstacle

- It is a press task, not a grasp task. Start with alternative IK branches for
  the unchanged left prepress TCP, so the elbow/forearm need not use the original
  configuration near the fixture. Do not assume a specific link is at fault.
- Next consider tool roll about the press axis while preserving the final
  button-normal approach. Current `roll_deg=0`; this is not a mandatory task
  success condition, but changing it changes the contact policy.
- A different prepress stand-off may be considered without moving the actual
  button. Button-face contact offsets must stay on the real intended button,
  not the housing, gap or adjacent control. Do not blindly translate the TCP.
- After an eligible endpoint exists, consider an above-bar or free-end-side
  waypoint. No such candidate has been tested in this notes-only step.

### Right Cancel / Obstacle

- Optimize the right task independently; do not copy the left arm configuration
  or TCP. Preserve the right target and operation axis.
- Start with alternative redundant IK solutions to the exact right prepress
  TCP. Then consider press-axis tool roll and stand-off, followed by legal
  face-interior contact offsets only if necessary.
- An above-fixture or opposite-side elbow posture is an annotation hypothesis,
  not a diagnosed collision fix. Never use a detour that still terminates at
  the currently rejected fixed-branch joint goal.

### Temperature Knob / Property

- The rejected pose is `pregrasp`, not an established grasp. First retain the
  original knob axis/center and try alternative IK branches at that same TCP.
- Then vary approach/gripper roll around the knob axis; the existing initial
  roll is -90 degrees about world X. +/-15 or +/-30 degree changes are example
  candidate offsets only. Preserve jaw opposition and plausible knob contact.
- Adjust pregrasp stand-off along the existing axis if useful, but keep the
  original actual grasp center as a separately defined target. Do not relabel
  reaching a farther pregrasp as successful knob manipulation.
- If actual grasp points change, use compatible opposing contact points on the
  knob and update the grasp frame. An off-center grasp needs an appropriate
  axis-centered motion, not simply the old rotation commands with a translated
  TCP. The original level-3 target remains unchanged.

## Implementation Traps Before A Future Retry

- `quick_plan_scene.py` currently reads button roll/X/Z offsets from historical
  baseline `control_parameters`, not newly added config keys. Editing a JSON
  `roll_deg` alone would not affect this generator. A future optimizer must pass
  candidate values explicitly and record them.
- `button_prepress_pose` currently hardcodes prepress Y=-0.44 m. Stand-off
  optimization needs an explicit candidate prepress position; a new unconsumed
  config key will have no effect.
- The button execution loop currently increments only world Y after prepress.
  Contact-point/roll changes must be propagated consistently, retaining the
  button-normal push direction and original pressed predicate.
- Temperature approach/turn commands use `initial_roll_deg`, separate
  pregrasp/grasp positions, and a world-X rotation policy. A new grasp frame must
  be propagated through those phases rather than changing only the planner goal.
- All proposed candidate rules are still untested. Existing successful physical
  recordings do not validate a changed contact policy.

## Web Update

Only the three system-generated `planning_hint` fields and diagnosis-download
references were updated. Their failed statuses and original logs/trajectories
were not rewritten. User planning notes, review categories, visual history,
layout configs/assets and task targets were preserved. No new planning or
physical execution was triggered by the hints.
