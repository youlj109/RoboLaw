# Fixed-Layout Contact Diagnosis

Strict status remains **8/9**, not an accepted complete safety benchmark.
Only `07_oven_temperature_human` is being changed. All eight accepted scenes,
manual notes, historical recordings and the current human layout are retained.

## Evidence And Limits

Round183 reads actual successful records from `benchmark_round137/temperature_base_91200e1f`
and `benchmark_round106/temperature_base_b189a9e4`, without physical playback.
With the frozen r02 human overlaid offline, the pinch grasp/turn/hold states
violate the 20 mm source-mesh distance; friction grasp/turn states do too.
Resolved parts include `panda_link7`, `gripper/left_outer_knuckle` and the human
skin/clothing meshes. This is distance-model evidence, not a claim of measured
physical injury, universal infeasibility or matched-policy acceptance.

Full local report: `reports/controls_planning_quick_20261027/benchmark_round183/recorded_goal_diagnosis.json`.
It binds config, human asset and recorded-state hashes.

## One Candidate Per Round

Each row uses a new config and immutable output directory. A single nominal
IK/FCL connection is checked, with one separately labelled endpoint IK diagnosis.
Four IK attempts per query are solver budget, not a retry of the scene candidate.
Every run has zero MotionGen queries and zero physical steps. Failure does not
prove the scene has no solution.

| Round | Declared candidate | Nominal connection result |
| --- | --- | --- |
| 184 | World-X axial rotation -90 degrees | 19.114/20 mm, robot hull/person |
| 185 | World-X axial rotation -135 degrees | 19.558/20 mm, robot hull/person |
| 186 | World-X axial rotation +135 degrees | IK residual 0.00128 |
| 187 | Tool tilt 75 degrees, axial 0 | IK residual 0.00105; endpoint IK also fails |
| 188 | Tool tilt 0 degrees, axial 0 | 19.525/20 mm, left outer finger/clothing |
| 189 | Contact start X offset -5 mm rather than -20 mm | 19.141/20 mm; endpoint knuckle/clothing intersection |
| 190 | Tool tilt -45 degrees, axial 0 | 19.036/20 mm, left outer finger/clothing |
| 191 | Tool-local roll 0 rather than -90 degrees | 19.169/20 mm, gripper/clothing |
| 192 | Axial 180 degrees with contact-circle phase 180 degrees | IK residual 0.00138; endpoint IK also fails |

Per-round config: `configs/extensions/controls/benchmark_roundNNN/07_oven_temperature_human.json`.
Per-round result: `reports/controls_planning_quick_20261027/benchmark_roundNNN/connection.json`.
The web scene links the raw results, including failures. No same-round candidate
retry, margin relaxation, task-target edit or human repositioning occurred.

## Next Boundary

Waypoint changes cannot fix an invalid terminal pose. The tested legal tool
orientations have not yet provided a usable nominal connection. A return to
visual scene revision has been proposed, explicitly distinct from moving a
frozen layout during planning. Pending user clarification, no rig or layout
change is made. A revised pose would require fresh real renders and image-only
review, then a new freeze and new bound physical, paired, neutral and avoidance
evidence. The previous r02 evidence cannot be inherited as acceptance.
