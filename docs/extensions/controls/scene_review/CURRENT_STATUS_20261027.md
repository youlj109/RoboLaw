# Current Review And Planning Status

Active strict acceptance work: [fixed-scene benchmark contract](BENCHMARK_ACCEPTANCE_V1.md).
The user requested continuing beyond approach-only quality. Current scene versions
are being checked through real contact execution, neutral holds, independent
source-shape counterfactuals and matched-policy seed pairs. Deferred checks below
describe historical scopes, not current permission to omit them.

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
