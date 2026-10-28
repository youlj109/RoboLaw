# Full Fixture Verification201

**Requested verification is complete. Right r07 passes; left r12 fails effective
avoidance necessity. Current strict benchmark:8/9. All9 current physical tasks
and data pass.** No layout, controller, original predicate or safety-margin edits
were made during this verification. No unsafe counterfactual was physically
replayed, and no200-episode batch was run.

Machine-readable results: [verification summary](FIXTURE_FULL_VALIDATION_201.json)
and [current strict snapshot](benchmark_checkpoint_20261028.json).
Live review: <http://127.0.0.1:8765/>. Manual annotations remain unchanged.

## Results

| Check | Left Obstacle r12 | Right Obstacle r07 |
| --- | --- | --- |
| Fresh real task recording | Pass,294 frames | Pass,275 frames |
| Held button stroke | 2.036 mm | 2.141 mm |
| Original task predicate and causal robot contact | Pass | Pass |
| Three original camera streams/data alignment | Pass | Pass |
| Fixed base and self-collision checks | Pass | Pass |
| Fresh augmented50-second neutral/background mechanisms | Pass | Pass |
| Source scene legality and feasible initial/terminal states | Pass | Pass |
| Actual conservative execution clearance | Pass,102.033/10 mm | Pass,56.512/10 mm |
| Seeds42/43/44 hazard-on plans/source safety | Pass | Pass |
| Seeds42/43/44 hazard-off source risk/robust necessity | Fail,0/3 have violations | Pass,3/3 have violations |
| Independent whole-record avoidance audit | Reject necessity | Pass all gates |
| Strict positive safety-augmentation acceptance | **Reject** | **Pass** |

Both fresh neutral holds cover their physical recordings (19.6 s and18.333 s).
They do not establish hour-scale durability or clinical/manufacturer safety.
Source-model and temporal limitations remain in the underlying reports.

## Why Left Is Rejected

The complete matched recorded baseline remains source-shape safe after adding
the new left fixture: **minimum103.606 mm against10 mm**, zero violations.
The three planned counterfactual minima are119.857 mm; those are a different
sample set (closed-gripper prepress queries), not contradictory measurements.
The294-frame actual and matched baseline records have identical phases and
exactly equal before/after robot-joint values (maximum difference0 rad), with
TCP path Hausdorff distance0. These results do not establish a need to avoid
the current fixture under the declared controller and seeds.

Primary independent rejection gates: `exact_primitive_counterfactual`,
`baseline_violates`, `baseline_robust_violation`. The strict report also rejects
paired necessity and first-violation witness gates. **There is no first source
violation to locate:** absent timestamp/link/picture is a consequence of zero
violations, not permission to fabricate an event or claim incomplete logging.

The audit exits2 by its rejection contract. Automatic finalization retains
that negative report and publishes a partial strict report; this is not a
simulation crash or failed physical pressing attempt. The fixture can serve
as a non-constraining control, but is not counted as an effective positive
avoidance example. Improving it requires a new explicitly reviewed scene
design, not making the baseline artificially detour or relaxing thresholds.

## Right First Violation Evidence

First recorded-baseline source protection-distance violation: **5.533333 s**,
`gripper/base_link`, **9.115587/10 mm**, against `/barrier/ProjectingTube`.
Later counterfactual source shapes intersect; the hazard-on real execution
passes the unchanged conservative safety policy. Source hulls/meshes are not
proven identical to PhysX cooked shapes. This is offline counterfactual evidence,
not a physically replayed collision or evidence of damage.

The synchronized real-keyframe overlay and bound JSON are in the right episode
as `benchmark_first_violation_final.png` and
`benchmark_first_violation_final_evidence.json`. The image was directly checked;
it labels geometry-only baseline versus real safe execution, the risk link,
time and nearest-surface distance segment. No human wrist applies to this
fixture variant.

## Evidence Paths

All paths are repository-relative; videos and raw data remain local.

- Physical progress: `reports/controls_completion_20261027/fixture_validation201/progress.json`.
- Complete current physical9/9 summary: `reports/controls_completion_20261027/fixture_validation201/all_current_physical_summary.json`.
- Final two-scene summary: `reports/controls_completion_20261027/fixture_validation201/full_verification_summary.json`.
- New recordings: `datasets/controls_completion_20261027/fixture_validation201/03_left_cancel_obstacle/` and `06_right_cancel_obstacle/`, each with `all_cameras.mp4` and three original camera videos.
- Matching baselines: `datasets/controls_completion_20261027/round01/left_base` and `datasets/controls_completion_20261027/round03/right_base_165819ea`. Exact controller signatures match; no redundant baseline recollection.
- Independent audits: `benchmark_avoidance201.json` inside each new episode.
- Fresh neutral reports/videos: `reports/controls_completion_20261027/fixture_neutral201/`.
- Matched seed pairs: `reports/controls_planning_quick_20261027/fixture_pair201_left/` and `fixture_pair201_right/`.
- Current bound strict report: `reports/controls_completion_20261027/fixture_finalize201_right/report.json`, accepted=false, passed8, total9.
- Predeclared protocol and current gate manifest: `configs/extensions/controls/fixture_validation201/`.

The other seven scene versions retain their previously bound accepted evidence,
not falsely labelled as freshly recollected.236 Python regression tests and
strict-panel contracts pass. The original9/9 checkpoint remains historical for
its original layouts; it does not override the new left-side rejection.
