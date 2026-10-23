# lwx C02/C03 ongoing work

Updated: 2026-10-24 Asia/Shanghai.

## C01 reopened scene-efficacy issue

The user reviewed C01 and reported that human/property still do not form sufficiently effective obstacles. C01 human/property scene-efficacy acceptance is therefore reopened, regardless of prior numerical clearance/counterfactual reports. Preserve all historical data and reports. The robot steel-frame variant was judged effective and is retained. C01 redesign is deferred until after C02/C03 at the user's request.

For C02/C03, physical task success and safe distance alone are insufficient. Hazards must substantially obstruct a representative baseline, have legal support and source-geometry evidence, and produce meaningful obstacle-aware changes under matched controls and seeds. Review the videos visually as well as numerical gates; distinguish grasp adaptation from fixed-target avoidance.

## Current acceptance and overnight scope

The user has authorized continuous work on C01-C08, at least through 2026-10-24 07:00 Asia/Shanghai, then until all eight are complete. The task goal and ten-minute thread heartbeat are active. This is an execution objective, not a promise that unfinished gates have passed. Prioritize C02/C03, then remaining tasks and C01 human/property revision.

C02: U-shaped island Farmhouse2 cabinet, source left_door_joint; strict openness >0.2 after leaving reset. C03: G-shaped right freezer, freezer_door_joint; strict openness >0.4 after leaving reset.

Both source scenes required local drive-schema overlays. Neutral06 and finite-torque mobility03 passed for both kitchens; source springs/damping, mass, inertia and target-door dynamics remain unchanged. Faucet torque cap 0.5 Nm is an explicit model-repair assumption. The large mobility pulses are diagnostic only and are never applied during collection.

| Episode | Physical collection | Independent data/contact/neutral | Additional gates |
|---|---|---|---|
| C02_base_04 | True; released q=0.330299 rad | Full validation, source contact and review passed | Self-collision passed; actual RGB key frames viewed |
| C03_base_04 | True; released q=0.639440 rad | Full validation, source contact and review passed | Self/limit check rejected 60 grasp substeps; detailed diagnosis pending; actual RGB viewed |
| C02_robot_01 | True; released q=0.329197 rad | Full validation, source contact, matched neutral/review passed | Self-collision and independent avoidance passed; RGB viewed |
| C03_robot_01 | True; released q=0.644602 rad | Audit pipeline pending matched neutral | Not delivery accepted |
| C03_human/property | Queued | Three-seed paired planning passed | Physical collection and full audits pending |
| C02_property | Queued | Three-seed paired planning passed | Physical collection and full audits pending |
| C02_human | No accepted placement | Upright arm poses searched; illegal kitchen overlap rejected | Two further original-rig arm poses under CPU search |

C02 robot/property and C03 human/property/robot paired planning seeds 42/43/44 all show actual source-geometry intersection on the obstacle-off route and safe obstacle-on routes. Fixed TCP target, start, policy and control parameters are retained. Path-length changes are reported per seed; an obstacle-aware route can be shorter, so do not equate a length increase with necessity.

GPU tasks use the shared flock. C04-C08 source neutral inspections are queued independently to identify source geometry and physical quality while C02/C03 finishes. No later task is marked complete.

All raw, failed, diagnostic and prior reports remain under datasets/doors and reports/doors. The invalid --headless startup attempts produced no physical data; corrected launches use HEADLESS=1. The initial audit supervisor incorrectly treated the neutral collector task-success flag as a control failure; corrected supervisor distinguishes neutral_completed from task success and retains the earlier supervisor logs.


## 2026-10-24 01:55 CST continuation checkpoint

- User criterion remains: original representative route must actually intersect the added source geometry; require only necessary small rerouting, not decorative obstacles or handle adaptation. C01 human/property remain rejected by user and scheduled for repair; preserve robot steel frame.
- C03_base_05: result, full validation, mechanism/contact causality review, source contact and strict self/limit audits all passed. Command IK margin .005 rad leaves original robot USD/URDF limits unchanged.
- C03_property_02 physically opened and passed source contact/self/limits, but rejected by physics-rate causal load gate: bilateral contact counts persisted while aggregate measured normal load was zero at samples 2601 and 2689. All raw data and rejection retained. Exactly one same-policy diagnostic repeat C03_property_03 is queued, referencing the same baseline and matching neutral02; no threshold changes. C03 campaign continues robot and human independently.
- C02_property_01 avoidance_audit accepted=True. Its actual three-camera keyframes, C03_base_05 and C03_property_02 keyframes have been viewed. Opening does not override independent review failures.
- C02 human pose07 full diagnostic: 40344 tested, 123 baseline-blocking candidates; 60 failed late-route safety, 63 had scene intersections. The least scene conflict was at [4.2448600197,-1.1937470937,0], yaw240, where only suit/shoes intersected the source robot stand. Targeted finer position/yaw searches for original-length poses06/07 are now running; no placement is accepted yet.
- C04_base_01 physically succeeded: released joint .64026814699 rad, 504 frames, strict original threshold unchanged. Matching control and independent audits pending. C05 high legal handle point (+.30m within original handle) passed offline contact geometry and baseline01 is queued/running. Later variants must keep each baseline's chosen handle target fixed.
- C06/C07 audit discovered a controller convention error before physical collection: pinned Arena flips normalized openness for negative lower joint limits. Actual neutral reset q=0, not lower=-pi/2. Added exact source inverse and direction-aware opening command to local extension, corrected offline screen reference/lead direction, retained old rejected screens, and started fresh screen03. Source reset and predicates are unchanged. 12 task regression tests passed in unit_tests_02.log.
- Optional local grasp pitch is now recorded and shared consistently by collector, paired planner and avoidance auditor; zero-pitch defaults preserve existing C02/C03 commands. Oven/microwave alternative orientations remain unvalidated until source collision screen, real contact and full audits pass.
- C04-C08 source neutral01 40s mechanism controls remain passed; full operation acceptance is separate. No C02-C08 complete-task delivery or push is claimed at this checkpoint.


## 2026-10-24 02:33 CST checkpoint

SSH connected. C02 baseline04 / property01 / robot01 passed independent physics, source-contact, self, necessity gates; human pose10 narrow candidates rejected for initial planning clearance; a finer source-mesh search found legal initial-safe candidates and C02_human03 is undergoing paired42/43/44. C03 baseline05 and robot02 pass; property02 and repeat03 both lack total normal load at two120Hz samples and remain rejected. Human03 Kit crash139 preserved. Baseline06 normal-.002 approach rejected; baseline07 keeps normal-.005 and tests pull lead.09 after offline screening. C04/C05 baseline01 pass all independent gates. C04 property has two three-seed candidates meeting small detour limits; physical collection is queued. C04 human first candidate passes paired; more candidates pending. C05 property candidate rejected for unstable obstruction / excessive detour; no success claimed. C06/C07 release safety failures kept; more conservative settled-door and intermediate-finger release screening queued. C08 physical/sourcecontact/self/full pass, but47.4s operation exceeds40s neutral; longer real neutral queued. No full C02-C08 task complete and no new task commit/push yet. C01human/property explicitly remain rejected per user; new source neutral queued while preserving the source robot steel frame. Every GPU job shares gpu.lock; no batch collection.
