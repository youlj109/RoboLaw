# lwx conservative endpoint discovery and actual C02 steel followup

This checkpoint preserves diagnosis, regression evidence, a fully checked C07
hold baseline, and the newly successful C02 steel episode. It does not complete
C01, C02, or C07. C03/C04/C05/C06/C08 remain the five previously delivered tasks.

## C01 status and bounded diagnosis

Original C01 physically opened successfully. The user's outstanding correction
is effective obstruction by human/property. Original successful data and the
approved steel presentation remain untouched. New placement experiments failing
IK or safety checks do not mean original C01 opening failed.

For each of two rejected new layouts, 48 deterministic endpoint IK attempts
produced 38 converged solutions at the unchanged seed-42 off target, with zero
solutions passing the original CPU external world check. Classification includes
all six original arm envelopes, including the separately named proximal mount.
The rigid distal (7,8,0.055 m) arm capsule intersected the human source in all 38
solutions. For property its distance was approximately 0.028 m, below the original
0.060 m planning margin. These are conservative proxy results, not proof of
physical source robot collision or global infeasibility.

The original source hand and distal sphere screens did not cover this capsule.
`search_task_hazards_v28_lwx.py --screen-invariant-envelopes` now optionally checks
the full initial conservative robot and rigid distal arm/gripper/finger proxies
through measured late states. It also checks these envelopes against proposed
supports. This only rejects offline placements. Redundant elbow postures remain
unconstrained, and final planning, physical safety and acceptance remain intact.

The rigid proxy relationship to the TCP was verified against the original
USD-derived kinematics at 24 different configurations. A negative control checks
that a proximal arm segment is not rigid to the TCP. Topology changes fail closed.
The related CPU suite passed 37 tests. Six old human and six old property layouts
were replayed: disabling the new option reproduces v27 candidate arrays and
statistics exactly; enabling it rejects all 12 layouts at the conservative
endpoint screen. These are bounded regressions, not scene acceptance.

`build_source_obstruction_grid_lwx.py` proposes placements around measured
physical source hull AABBs, with provenance hashes. Center alignment alone is not
intersection evidence. The subsequent exact source, support, initial/endpoint,
door sweep, paired and physical gates are all still required. The historical
builder source is retained because an earlier C01 grid records its hash; the
current builder also supports scene-only task configs using the same variant
template as the search.

## C02 steel actual success

Episode: `datasets/doors/lwx_C02_planning_padding_retry18_20261024/C02_robot_26`.
Matched baseline: the same root's `C02_base_01`.
Selected config: `configs/extensions/doors/c02_robot_candidate_26_10.json`.

The original unscaled projecting crossbar produced actual source intersections
in all three off seeds, safe on plans at the same Cartesian target, maximum joint
length ratio below 1.6, and maximum added TCP length 0.120497486308 m. A full
predicted approach screen preceded physical execution. The actual episode has
301 frames and passed collector, full validation, neutral/task review, source
contact, self-collision and avoidance checks. Original-speed comparison video
and the source risk illustration were produced remotely. Actual three-camera
keyframes and the risk illustration were viewed; signed visual sidecars and
image hashes are preserved here. Keyframes and overlays have their own stated
limitations and do not substitute for numerical audits.

Human08 from the previous C02 release root is retained. Property remains pending:
the previous shallow bottle candidates blocked all tested off routes but exceeded
the 0.15 m path-increase limit. A new tangent-placement search runs under the same
controller and margins. C02 is not a complete task delivery yet.

## C07 hold baseline and ongoing variants

`datasets/doors/lwx_C07_release_hold_retry58_20261024/C07_base_01` physically
succeeded with 1057 frames. Matching neutral control and all five independent
gates passed; its actual three-camera storyboard was viewed and the sidecar is
included. This uses the existing `release_settle_steps=15` controller option:
one fixed actuator target is held before the unchanged 0.03 m / -55 degree
release. Target joints are not assigned during execution and margins are unchanged.

Human60 and robot59 use this new matched baseline. New paired tests passed the
same-target source-intersection and small-detour gates and the full predicted
approach guard; actual collection was queued at this checkpoint. No variant
success is asserted here. Earlier root52 human/steel release failures remain
rejected and preserved. Property discovery continues and has not passed.

## Evidence and scope

`manifest.json` hashes the frozen copies under `evidence/`. Original trajectories,
datasets and videos remain remotely at the paths recorded in the reports; this
checkpoint is not a self-contained large-data archive. Subsequent runtime results
must be checked live. Source hulls/meshes, conservative capsules, planning spheres
and PhysX collision evidence remain distinct. Fixed-layout single episodes and
three planning seeds do not establish randomized success rates. No unsafe off
counterfactual was physically replayed. No duplicate accepted sample or 200-run
batch was requested or launched.
