# lwx C01-C08 execution checkpoint, 2026-10-24

Scope: eight original assigned Lightwheel door tasks on branch lwx. Upstream origin/ylj was verified at eda19a68092a3adf3c8b3621c79a099fb7f44efb. Existing changes and rejected data remain intact. This implementation checkpoint does not claim that all eight tasks, or any new complete task, have passed.

The added entity must intersect the actual source geometry of each tested obstacle-off route, while obstacle-on planning makes a small safe change to the same target. Planning seeds42/43/44 are paired; a clearance-margin violation alone is insufficient. The predeclared additional selection limits are joint path ratio <=1.6 and added TCP length <=0.15m. Unsafe counterfactual paths are never physically executed. Human/property C01 placements remain rejected by the user; preserve the original steel-frame variant and redesign the other two.

Each selected operation requires a real matching baseline, an equal-or-longer neutral observation, all kitchen active DOF q/dq and native contact samples at120Hz, complete moving-door collision coverage, source-shape contact localization, self/external safety, three original camera streams, and actual-frame visual review. No200-trial batch is requested or run. No target joint writes after native reset, welded grasp, disabled door collision or relaxed safety thresholds are used.

Current evidence roots are datasets/doors/lwx_C02_C03_20261023, datasets/doors/lwx_C04_C08_20261024 and datasets/doors/lwx_C01_revision_20261024. Their report roots use the corresponding reports/doors prefix. Generated datasets/reports stay outside Git; a completed task gets a tracked gate summary and SHA256 manifest from finalize_task_delivery_lwx.py.

C02 baseline04/property01/robot01 pass independent gates. Human candidates must still establish actual source intersections in all seeds, not just the person clearance margin. C03 baseline05/robot02 pass; property02 and repeat03 are rejected for two unloaded pulling substeps. Human03 ended in native Kit crash139; the partial data are retained. Baseline06 contact approach failed; baseline07 tests a stronger lead with the original legal normal offset. C04 and C05 baseline01 pass. C04 property01 completed physically and is under independent review; other variants are in candidate/paired work. C06/C07 opening worked but release clearance failed, so neither is accepted. Settled-door and intermediate-finger release screening is queued. C08 baseline01 now passes after an actual longer neutral02 replaced the too-short40s control; the earlier rejected review remains archived.

The local IK command bounds must match in collection and paired planning. Earlier nonzero-margin paired reports need a current-policy revision; they are preserved, not silently promoted. New episodes capture immutable runtime source copies. Historical episodes retain hashes and do not retroactively claim such copies.

Validation at this checkpoint:27 CPU unit tests passed; all125 owned Python files compiled; skill preflight reported no issues. Static checks do not prove physical task completion. Every GPU/cuRobo/Kit job uses flock /workspace/reports/skill_trials/gpu.lock. An active thread continuation is scheduled every10minutes until completion, and work is requested through at least07:00 CST.

Reproduction entry points: restore_lightwheel_sources_lwx.py verifies fixed SDK versions/hashes; collect_tasks_lwx.py performs one native-reset operation or neutral observation; paired_tasks_lwx.py plans the three off/on controls; supervise_task_reviews_lwx.py runs independent gates; audit_task_avoidance_lwx.py checks actual and counterfactual source geometry; make_task_comparison_lwx.py creates original-speed comparisons. The physical collector refuses to overwrite existing trials.


## 2026-10-24 03:35 checkpoint (Asia/Shanghai)

No new complete C02-C08 task is claimed yet. C01 human/property remain rejected by the user and require replacement.

- C02: base04, property01 and robot01 passed all independent gates and received actual visual/source-risk illustration review. The both-arm same-source human pose17 candidate passed actual source intersection in all three planning seeds; human05 real collection is pending. A roll45 matching baseline05 is a fallback, not evidence of fixed-target success against base04.
- C03: robot02 now also passed the command-margin-correct paired revision05. Human05 opened the source door physically; matching neutral and independent reviews continue. New baseline07 has normal-load source contact (2956 loaded points checked), full validation and scene review; self-collision review is queued before property04 may start.
- C04: property01 passed real/neutral/full/source-contact/self/avoidance/comparison plus command-margin-correct paired revision02; actual three-camera frames and source-risk overlay were inspected. Human01 opened the door and matching neutral continues. Robot02 has eligible actual-obstruction candidates; bounded candidate comparison continues.
- C05: old bottle/robot candidates are rejected because seed42 has a common safe off-path. Vase/tablet searches did not find all-seed legal candidates. Refined search and a pitch-adjusted new matching baseline are discovery fallbacks; no new physical success is claimed.
- C06: baseline02 reached native success but release clearance fell to 1.9615mm, below the unchanged 2mm requirement; retained as rejected. Baseline03 tests an explicit 10mm outward TCP retreat during finger release. The option is recorded in controller parameters and propagated to matching baselines/variants. Geometry replay is only a static diagnostic and does not establish dynamic release safety.
- C07: baseline03 hit a Kit native exit139 before producing a final result. Exact screened policy is retried as baseline04; crash evidence retained.
- C08: baseline01 is accepted with a genuinely extended matching neutral. Robot01 and human01 passed all-seed actual obstruction and entered physical collection. Property candidate01 is eligible; bounded search compares the remaining candidates.

The new fixed-sample runner supports task listing, dry-run, explicit one-sample quota, maximum 10 attempts, three-identical-failure diagnosis and hash-bound resume without overwriting evidence. Only its list/dry-run and CPU negative-control tests are verified so far; do not present it as a separately completed physical reproduction. Current CPU suite: 34/34, reports/doors/lwx_C04_C08_20261024/unit_tests_all_05.log.


### 03:52 recovery update

C02 human05 passed the actual-intersection test for every predeclared seed with a maximum added TCP length of 0.0088714m and joint-path ratio 1.018009; physical collection remains pending. C03 baseline07 passed all five independent gates, enabling property04. C08 robot01 physically opened the door and continues matching-neutral validation. C04 human01 is retained after successful physical opening; neutral01 crashed natively, so only neutral02 and remaining audits are rerun.

C06 baseline03 with 10mm retreat still failed the unchanged release clearance. Recorded failed-door pose diagnostics show larger geometric reserve with 40mm retreat, but changed dynamics remain unverified. A single bounded baseline retry now runs under datasets/doors/lwx_C06_release_retry04_20261024. C07 roll90 baseline04 did not open the door; the earlier roll0/-0.06m physically opening policy is retried with 10mm release retreat under datasets/doors/lwx_C07_release_retry05_20261024. Neither is accepted yet.

C05 pitch screen04 rejected every candidate. Lower fixed handle points are screened separately; high/fine source-library robot and tablet searches found no all-seed candidates, so no physical collection was started from them. Broader two-arm human discovery continues. These are retained discovery/rejection results, not completed variants.
