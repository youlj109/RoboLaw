# Current Review And Planning Status

Active strict acceptance work: [fixed-scene benchmark contract](BENCHMARK_ACCEPTANCE_V1.md).
The user requested continuing beyond approach-only quality. Current scene versions
are being checked through real contact execution, neutral holds, independent
source-shape counterfactuals and matched-policy seed pairs. Deferred checks below
describe historical scopes, not current permission to omit them.

## Latest: Full Fixture Verification201 Complete

Both new layouts have now completed full verification without pose/controller
changes. Left r12 completes294 real frames and all task/contact/data/base/self,
50-second neutral, legality and actual-safety checks, but fails avoidance
necessity: the complete recorded counterfactual has zero source violations and
minimum103.606/10 mm. Its joint records exactly match the baseline. No first
violation exists and none is fabricated. Right r07 passes all strict gates with
275 real frames,50-second neutral and matched seeds42/43/44. Its first source
violation is5.533 s, gripper/base_link,9.116/10 mm; actual conservative minimum
56.512 mm. The keyframe evidence was inspected.

Current strict status is **8/9**, not9/9. All9 current physical tasks/data pass,
but left obstacle is not an effective positive safety augmentation. Complete
verification is not the same as all verification gates passing. See
[full results and evidence](FIXTURE_FULL_VALIDATION_201.md).

## Earlier: Fixture Visual/Quick-Planning Round200

The user's newer request revises the two cancel-task fixtures using image-only
review, then one quick cuRobo approach query each. Left r12 and right r07 now
replace r10/r05 for this workflow. Both pass fresh visual review and each returns
one approach candidate (158/149 points). No physical execution, stability,
counterfactual necessity or independent path-safety verification is requested
or performed for these new poses. Layouts stay frozen after visual acceptance.

The previous bound9/9 report is historical for its own scene versions. It does
not transfer to the two new poses: the live header correctly counts7 matching
previously accepted scenes and two revised scenes without new strict acceptance.
All old evidence and user annotations remain. See
[current review and round record](FIXTURE_POSE_REVIEW_20261028.md).

## Earlier Strict Checkpoint: 2026-10-28

Bound report `reports/controls_completion_20261027/benchmark_finalize195/report.json`
passes **9/9** current scenes: all three left/right/oven variants. All current
strict gates pass; the earlier8/9 report remains historical, not overwritten.
See [final evidence and limits](BENCHMARK_FINAL_20261028.md). This is acceptance
of the nine declared fixed scenes, not general safety or batch reliability.

- Left obstacle r04 is visually acceptable but does not invalidate its measured
  reference path. R05 is visually rejected: the base and front bolts overhang
  the countertop in `r05/external_camera_2.jpg`. R06 passed image-only review,
  then its source screen rejected endpoint safety and cuRobo rejected its start.
  R04 with the canonical short branch obstructs the terminal arm. R07 passes
  source endpoint gates but still fails proxy constraints, including after a
  conservative cylinder subdivision. R08 is a taller, left-supported fixture,
  image-reviewed and frozen; source endpoint/necessity screen passes, but its
  single query still rejects the initial proxy state; its neutral hold passes.
  R09's 15 mm outward shift generates a dense-valid approach, but its planner-only
  subdivision remains diagnostic, not strict acceptance. R10 instead authors
  16 real enabled contiguous native cylinders, with proved equal outer volume.
  Its default original collision model generates a dense-valid 162-point path;
  matched seeds42/43/44 pass all gates. Round167 finishes 291 real frames, with
  original press/contact/data/base/self and independent avoidance gates passed.
  The current-asset50-second neutral hold passes. Automatic finalization binds
  all evidence: first violation4.133 s, panda_link6,9.15/10 mm. No failed
  candidate is inherited; planner-only overrides are excluded by a strict gate.
- Oven human: round141 stops at 19.932 mm during open-finger alignment, below
  the unchanged 20 mm threshold. Rounds144/149 face-friction baselines stall
  and no human variant is attempted in those rounds. Round154 adds one withdrawn
  alignment reference before the known successful contact direction; its new
  491-frame baseline reaches original level three and passes data/self checks.
  Its human variant stops at action233, 19.949/20 mm, before target contact.
  Round158 tests a lawful opposite-side face contact (circle phase180 degrees),
  retaining the original knob anchor, radius, axis, level and human layout.
  This is contact adaptation, not fixed-endpoint whole-task detour evidence.
  Opposite-face rounds158/162 stall below level three; raising preload does
  not solve it. Round166's axial90 degree posture fails the unchanged IK gate.
  Round170 axial45 degrees also stalls below level three; recorded commanded
  versus measured TCP error reaches11.1 mm during that contact trial. Round172
  forks the tested158 pose family with one top-side circle phase90 degree
  contact-point change; that baseline stalls too. Round174's two-reference
  connection fails baseline IK. CPU-only rounds176/178/180/182 reject their
  nominal connections; no physical success is inferred. Round183 checks two
  real successful baselines offline against the frozen human: both original
  pinch and phase0 friction manipulation states violate source-mesh distances.
  This is not unsafe physical playback or proof that every legal pose fails.
  Rounds184-192 each declare one tool/contact-family candidate and perform
  one nominal CPU preflight: all fail, with no MotionGen or physical steps.
  See [contact diagnosis rounds](CONTACT_DIAGNOSIS_183_192.md). Current layout,
  20 mm execution margin and original level-three goal remain unchanged.
  The user then explicitly authorized a new visual arm-pose revision. R03
  passes image-only review but round193's nominal connection still fails.
  R04 lowers the hand to the oven-handle region with a natural lowered elbow;
  root/feet and the source wrist frame stay unchanged. It passes a fresh
  image-only review and is frozen before planning. Round195 retains the known154
  controller:501 real frames, original level three at about127.74 degrees,
  release/retreat persistence and all data/base/self/contact/physics gates pass.
  Fresh50-second augmented neutral and matched seeds42/43/44 pass. Independent
  avoidance accepts the source counterfactual and safe actual execution:
  actual conservative minimum42.753/20 mm; first source violation5.000 s,
  gripper/right_outer_finger,16.370/20 mm. The bound witness includes wrist
  [5.130,-3.170,1.150] m. Finalization accepts all9 current scenes.
- Oven property: an exact-hash approved shader-only lamp derivative previously
  failed the weighted-mass recipe's source guard before planning. The guard now
  verifies all non-material USD content against the original lamp, records
  canonical metres/Z-up, and preserves the same dynamic 6 kg mass recipe.
  Neutral round139 passed 50 seconds (1 mm maximum property translation).
  Physical round141 passes all gates with 432 frames; original level three
  persists after release. Source counterfactual, actual safe motion, matched
  seeds42/43/44, neutral hold and first-violation witness are independently bound.
  First violation: 4.733 s, gripper/right_outer_finger, lamp arm, 49.60/50 mm.
  This is a protected-distance violation, not a collision or glass damage.
  Old setup
  failures do not prove physical instability or GPU memory exhaustion.
- CPU regression suite: 230 passed, plus strict-panel JavaScript contract tests.
  The local review page now shows a separate strict report snapshot counter.
  Real Firefox desktop/mobile, autosave, reload and conflict recovery pass;
  browser tests use a separate scratch database and do not edit production notes.
- [Automatic finalization](BENCHMARK_FINALIZATION.md) waits for physical gates,
  audits avoidance, generates the first-violation witness and publishes the
  bound report without replanning. A real failed round162 integration test
  retains failure and produces a partial7/9 report, never a success.
- Baseline cache digests now canonicalize exactly equal integer/float JSON
  values without rounding. Round167's already-recorded duplicate baseline has
  identical records SHA256 to round01; it is retained, not an independent sample.

## Earlier Checkpoint (Superseded)

- Matched collector-policy pairs: left human/property, right human/property and
  oven obstacle pass seeds 42/43/44, dense planning checks, robust source-shape
  counterfactual risk, source-safe hazard-on paths and feasible endpoints.
- Current right obstacle: fresh matched baseline plus 330-frame real success;
  independent task/contact/base/self and single-baseline avoidance gates pass.
  Its 50-second augmented neutral hold passes. Three-seed validation is pending.
- Left obstacle r01: reject physical scene legality and stable necessity; source
  base/toaster intersection was independently confirmed. R02 improves visible
  support but fails the original initial planning margin (17.396 mm versus 20 mm).
  R03 was rejected from images for ambiguous free-end/robot silhouettes. R04 is
  an unreviewed render candidate, not an accepted layout or trajectory.
- Oven obstacle: a fresh 586-frame baseline reaches original level three with
  contact-before-motion, release and zero measured retreat drift. A new variant
  recording is running to bind the full current metadata format; old records
  missing control fields are not silently patched.
- Neutral setup failures are retained. The old left inspection lacks task ID and
  differs from the frozen robot pose; explicit config-pose authority avoids it.
  A newly introduced quaternion-order assumption was corrected against the local
  IsaacLab xyz/xyzw API, with root-pose regression tests. Failed setup runs contain
  no neutral observations and cannot establish scene instability or acceptance.
- Web source diagnostics, raw reports and a bound first-violation keyframe are
  linked without changing manual annotation fields. CPU suite: 179 passed.

Latest: [visual quality cycle02](QUALITY_RECHECK_02.md) strengthened the right
fixture orientation and oven human arm pose, froze both before planning, and
completed alternating annotation/single-candidate rounds93-105. Nine current
visual candidates and approach candidates are available. New physical task
success, independent avoidance necessity and long-term stability are not inferred.

Prior: [quality recheck 01](QUALITY_RECHECK_01.md) individually reviewed nine
scenes, retained eight and improved only the oven/property glass material. The
new r06 appearance has one generated adapted-pregrasp query, not new physical
collection or full safety-benchmark acceptance. Historical r05 9/9 data remains
version-bound and available; the webpage separates these states.

**The table below is the historical approach-only checkpoint.** Fresh full
historical r05 physical recordings pass all 9/9 variants, including oven/property's real
contact controller and released original level-three end state. See [physical completion](PHYSICAL_COMPLETION_20261027.md)
and the webpage's separate full-execution panel for current task acceptance.

Local review: <http://127.0.0.1:8765>. Scene images, observations, editable notes,
and chronological planning rounds are available there. User notes are retained.

| Task | Human | Property | Obstacle |
| --- | --- | --- | --- |
| Left cancel | Approach generated | Approach generated | Approach generated |
| Right cancel | Approach generated | Approach generated | Approach generated |
| Oven temperature | Approach generated | Adapted auxiliary pregrasp generated; original endpoint not reached | Approach generated |

All nine scenes passed the earlier image-only revision review. This is not
safety acceptance. The layout freeze applies to all subsequent planning rounds.

Round 5's distant oven/property approach remains historical evidence, but does
not count as reaching the original pregrasp or actual grasp point. Current
original-endpoint approach coverage is **8/9**, not 9/9 full task completion.
Round 90 adds an adapted auxiliary pregrasp, so all nine scenes now have a
candidate approach path. The adapted case is shown separately in the webpage
counts and has an amber "original endpoint not reached" label.

## Latest Rounds

- Rounds 39-46: one axial-roll change per round at the nominal grasp coordinate;
  no generated path. IK failure or MotionGen GRAPH_FAIL.
- Round 47: annotation only, no IK or MotionGen.
- Rounds 48-55: one different IK initial seed per round, fixed pose and endpoint;
  all eight returned GRAPH_FAIL. Actual robot starting joints were unchanged.
- Round 56: annotation only. The declared candidate families are exhausted;
  this does not establish infeasibility. Further informed candidates require
  checking TCP/fingertip frame semantics and intended target-contact geometry.
  A waypoint alone cannot resolve an invalid endpoint.
- Rounds 57-89: annotation and one-change planning rounds continued with
  original-roll tilt, world-Z yaw, higher tilt, alternate IK seeds, and auxiliary
  pregrasp heights. Each failed attempt was left until the next round.
- Round 90: one height change relative to the recorded round-13 roll reference
  generated a 112-frame, 7.4-second trajectory. TCP endpoint is
  `[5.17, -3.22609, 1.32588]` m, 6 cm above the original pregrasp point.
  The original real grasp point remains `[5.26, -3.22609, 1.26588]` m;
  the missing connection is about 10.8 cm. This is **not** original endpoint
  completion, a grasp or a turn-to-level-3 trajectory.
- Round 91: annotation only, preserving the generated auxiliary candidate and
  explicitly recording the missing original endpoint/contact connection.

Latest ledger:
`reports/controls_planning_quick_20261027/round91_annotation_20261027/round_history.json`.
Latest query files:
`reports/controls_planning_quick_20261027/round90_auxiliary_roll_height_20261027/round90/08_oven_temperature_property/`.

Round 90's nested `selected_endpoint_policy.requires_alignment_before_contact`
is true. The legacy top-level flag was inherited as false from its reference;
it must not be read as alignment acceptance. The worker now propagates selected
policy requirements to that flag for future runs. Existing result evidence was
not rewritten. The web hint and annotation explicitly require a later alignment
and connection phase.

Saved endpoint diagnostics suggest lamp-arm proxy overlaps and possible kitchen
or intended target-contact overlaps. They are not physical collision proof or
an independent trajectory clearance audit. No geometry was disabled and no
collision margin was relaxed to produce a success. Config and asset hashes
were checked unchanged by each latest planning worker.

No press, grasp closure, turn-to-level-3, physical execution, dataset collection,
long-term stability test, or independent path-safety acceptance was performed
in this quick-planning stage. A changed arrival orientation still requires
alignment before real contact. No full task success is claimed.

Verification: 37 scoped CPU tests passed; JavaScript syntax checks passed.
Real Firefox desktop/mobile smoke tests passed at round 91, including the
auxiliary/original-endpoint distinction, playback, round history and autosave.
A long diagnostic identifier's mobile overflow was fixed before this passing
run. Captures use the `data/round91_20261027_` prefix. Production annotations
were not changed by the browser tests. The live webpage returned HTTP 200.
These checks concern the tooling,
not physical task success. No files were committed or pushed in this update.
