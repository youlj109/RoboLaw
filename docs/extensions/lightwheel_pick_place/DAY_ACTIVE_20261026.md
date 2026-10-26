# Active daytime continuation, 2026-10-26

User is awake and explicitly asked to complete all remaining B03/B04/B05. No new deadline. Keep working; do not reactivate expired overnight automation. User can review JupyterLab images. No subagents requested.

## Approved frozen layouts
- Latest B03: configs/extensions/lightwheel_pick_place/variants_bananas_transportreview_20261026.json. Human [1.72,-3.28,0], rootyaw0, original human wrapped at childyaw255; property [2.3,-2.75,.922]; robot [2.3,-2.9,.922],yaw180. User approved revised human/property after images and kept approved robot unchanged.
- Approval/hashes: /root/autodl-tmp/B03-review/transport_review_20261026/scene_approval.json, committed B03_TRANSPORT_SCENE_APPROVAL_20261026.json.
- Previous daytime human [1.5,-3.3,0],yaw270 blocked bowl placement; property [2.3,-2.4,.922] blocked grasp endpoints. Failed trial data and old approvals remain. Latest replacements are true mid-transport blockers with baseline grasp/placement endpoints checked, not obstacles moved aside after an accepted collection.
- B04 user approved all three existing images B04-review/review_now/{human,property,robot}.jpg. Hashes /root/autodl-tmp/B04-review/user_scene_approval_20261026.json. Do not move these hazards.

## Critical actual-data fixes
- Failed robot B03 episode robot_lgd_b03_fixedscene_20261026T012516Z: close-derived carried model differed from actual at transfer start by11.58mm and91.88degrees. Actual TCP rotated only0.104degrees duringlift; object truly rotated within grasp. Preserve drift evidence /root/autodl-tmp/B03-review/day_review_20261026/robot_attachment_drift.json.
- Collector now updates planning attachment from measured root/arm FK at every carried segment, and computes lower TCP goal using actual current root/TCP offset. Collision model only, no PhysX weld or body setters.
- Explicit --attachment-feedback measured_segment (default for B03/B04) versus close_once. Auditor compares modes, inferred from recorded update history for already running episodes. Thus recapture a B03 matched baseline; do not pair measured-mode variants to old close-only baseline.
- B05 runner explicitly close_once to retain already verified B05 baseline/human/property policy; only timeout increases to7200seconds. Actual120Hz physics/collision evidence still required.
- SIGTERM/SIGINT now preserves partial data through collector finally and forces successfalse. Demonstrated on early B05 robot010442:73frames, signal15, trajectory/result saved. Never count interrupted data.
- Coarse STEP_WALL_PROFILE records camera,envstep,snapshot/contact costs without changing controls.
- Exact known failed B03 banana_2 candidate [tilt0,yaw0,pinch.002,waistY.03] is excluded after robot014542 physically failed secondlift. Other candidates/order unchanged. Source proof B03_SECOND_GRASP_FAILURE_20261026.json.
- Policy compatibility helper permits ONLY stronger physical exclusions if all other search inputs identical and every actual baseline/augmented selected grasp still eligible. Seven focused tests plus existing15 pass (22total). No changed candidates, removed exclusions or newly excluded baseline grasps can pass.

## Active pipeline IDs and logs
- Current native B03 fresh measured baseline: base_lgd_b03_fixedscene_20261026T020335Z; log /root/autodl-tmp/lgd-b03-logs/base_fixedscene_20261026T020335Z.log.
- Fresh series parent196642: /root/autodl-tmp/lgd-day-logs/b03_fresh_model_series.sh/log. Waited robot194977 (nowfailed), collects freshbase, then human/property in latest frozen config, CPU verifies and queues two paired panels. Its robot entry references old failed014542; do NOT use that as final robot result.
- Corrected robot retry198277: /root/autodl-tmp/lgd-day-logs/b03_robot_safe_grasp_retry_runner.log. Queued behind currentbase with same approved robot pose and excludes demonstrated bad secondgrasp. Must independently verify this newest result against successful freshbaseline.
- Saved B04 revised cuRobo probe queue198425: /root/autodl-tmp/lgd-day-logs/probe_b04_revised_curobo.sh/log, one GPU lock. Hypothetical endpoints only, no native success claim.
- B04/B05 parent196672: /root/autodl-tmp/lgd-day-logs/b04_revised_then_b05_series.sh/log, waits196642 then B04 base/human/property/robot, verifies all, then B05robot with old verified close-once policy.
- All GPU jobs share reports/skill_trials/gpu.lock. No simultaneous GPU jobs. Parent waiting is not proof of progress/success; inspect native result files.
- Fresh-series paired plan indices3/9 are provisional: inspect actual fresh baseline plans and recenter counts before final acceptance; rerun correct transfer legs if indices changed.

## B04 motion, pending actual validation
Same approved hazards, source/destination and original predicates. TaskSpec now commands90mm lift with4mm +X (within original5mm column gate), then same plate root offset-30mm X, release+85mm Z. CPU full-geometry endpoints found a common safe goal neighborhood for all3 variants; initial planning clearances human34.52mm vs30mm,property62.64mm vs60mm. Native baseline/variants not yet collected under these changed controls. No friction,mass,asset or safety-threshold changes.
Probe source: scripts/extensions/lightwheel_pick_place/probe_ladle_saved_motion.py. It derives hypothetical revised start from preserved actual data, checks cuRobo/self/full dense paths and explicitly does not claim a PhysX run.
B05human/property remain accepted with old baseline/close-once model; robot still pending successful complete capture.

## Ownership/authentication
Branch lgd, old user changes fixed_layout_ladle.json and restore_assets.py plus prior untracked assets/scripts preserved. Do not indiscriminately stage them. GitHub device login was offered while user awake; user chose unable to log in now, continue collection. Local login process canceled; do not repeat authentication question during this work. Push remains blocked, commit and bundle at delivery.


## 2026-10-26 05:07 UTC restart (supersedes old process list above)
- No native jobs remained at status check 05:04 UTC. Acknowledge missed continuation, not continuous progress.
- B05 robot023744 now native success, validation/full-world/source-support/fixed-hazard/background/self/avoidance all true. Human/property previously accepted; final package remains.
- B03 base023055 failed: banana_2 true; banana_1 false, final linear speed about0.0312m/s and oscillating bowl contact. Lower TCP compensation was rolled back before this episode; current lower offset uses lift-end measurement, contrary to earlier section.
- New v6 balanced-first policy prepends first-banana legal pinch candidates yaw180/0, world pinchZ+2.3mm, local waistY+11mm, based on approximate collision-hull volume centroid. Second banana explicitly excludes these new candidates to retain its successful policy. Not a certified cooked COM; physical proof still required. No physics/hazard changes.
- New native baseline base_lgd_b03_fixedscene_20261026T050735Z, runner208392, actual log lgd-b03-logs/base_fixedscene_20261026T050735Z.log.
- Continuation208876, /root/autodl-tmp/lgd-day-logs/b03_balanced_continuation.sh/log, waits this baseline. On success collects human/property/robot, verifies matched episodes, derives actual executed transfer indices for paired seeds42/43/44. On base failure stops instead of repeating known bad parameters. Frozen hazard hashes and disk reserve checked before each variant.
- B04 changed-motion base022533 failed at original wall; no new accepted variants. Do not repeat unchanged.


## 05:31 UTC centered placement continuation (current)
- Completed balanced-first baseline050735 failed only first final native predicate; banana_1 speed0.00097567m/s (<0.003), support angle47.00955deg (>45); banana_2 native true, speed0.00070673m/s. Thus balance grasp removed prior first-banana oscillation, but placement rests against bowl sidewall. Actual image saved in episode settle_final_external_camera_2.jpg.
- New baseline053117, runner210098, keeps v6 grasp and all physics/thresholds/frozen hazard layouts. First-banana commanded root X offset changes from-0.06 to-0.03m toward bowl center; second remains+0.06m. CLI --dual-first-place-offset-x, default-0.06 preserves existing callers. Dedicated b03_collect_centered.sh passes-0.03 for every baseline/variant.
- Dual-task avoidance audit now additionally matches full task_motion_parameters, preventing pairing changed placement to old baseline.
- New dispatcher /root/autodl-tmp/lgd-day-logs/b03_centered_continuation.sh/log waits210098, collects three variants only on native baseline success, records audits and actual-index paired panels. Previous balanced dispatcher exited2 on native failure.


## 05:52 UTC spaced placement (current)
- Centered baseline053117: banana_1 native checkpoint true, speed about0.0026m/s; second lower aborted at frame994 because carried banana vs first-banana clearance1.60806mm<2mm, no intersection. Keep original2mm gate.
- First centered state [2.11065,-2.56313,.98598], support [1.974,-1.100,3.144]N. First grip/placement improvement proven, whole task not passed.
- New baseline055157, runner211628. b03_collect_spaced.sh commands first offset-0.04m (2cm toward center from original), midpoint between wall-supported-0.06 and encroaching-0.03. Second offset+0.06 and all grasp/physics/hazards/thresholds unchanged. New matched baseline mandatory.
- Dispatcher /root/autodl-tmp/lgd-day-logs/b03_spaced_continuation.sh/log and finalizer b03_spaced_finalize.py/log supersede centered watchers (which exited on failure). Only passed manifest yields success package. Baseline native fail stops repeats.
- Preserved unused5.4GB CUDA installer under /root/lgd-preserved-cache/cuda_12.8.0_570.86.10_linux.run; original path remains symlink. No collection data removed. Data volume had12GB free after move.


## 06:05 UTC separated second placement (current)
- Spaced baseline055157: first checkpoint false, stable-looking root [2.0882,-2.568,1.008] but support [0.551,-0.727,0.648]N outside cone. Stopped owned native211635 with SIGINT after first checkpoint; partial674frames and result preserved successfalse. Stopped spaced dispatcher212120/finalizer212123. No need execute known failed first placement through second again.
- Restore first commanded offset-0.03, already proven native true in053117. Change only second from+0.06 to+0.075m, to avoid prior1.608mm inter-fruit near pass on second lower. Added CLI --dual-second-place-offset-x, default+0.06 preserves old callers; both offsets recorded/matched in task_motion_parameters.
- New native base060502, runner212908, b03_collect_separated.sh. Current dispatcher/finalizer use separated names in lgd-day-logs and baseline060502. They retain frozen hazard hash checks, one GPU lock, full validation, paired seeds and success-only packaging. No new passed full baseline yet.


## 06:32 UTC floor-centering (current)
- Separated baseline060502: original environment terminal success=true and final_live_success=true; both placement checkpoints true. Final simultaneous recheck first speed0.00309003m/s>0.003, so collector successfalse. Second passes. Do not conflate original terminal completion with current simultaneous final acceptance.
- Last120 recorded frames first speed min/median/max [0.00302141,0.00308175,0.00309965]m/s; no frame below3mm/s. Actual root displacement over8s only0.119micrometers and rotation0.000235deg; returned velocity can be solver/observation residual, but native predicate remains unpassed. No velocity override, no sleep/wake setters, no threshold changes. Waiting longer alone unsupported by current record.
- New baseline063247 runner214978, b03_collect_floor_center.sh keeps proven firstX-0.03 and secondX+0.075, adds firstY-0.03 to move settled fruit toward bowl floor center. CLI defaultfirstY0 preserves all existing callers; all three offset controls recorded/matched. No hazard/physics/grasp changes.
- CPU hypothetical spacing report pair_spacing_center_y_hypothesis_20261026.json: same measured failure shapes translated firstY-0.03 and secondX+0.015 retain10.75mm separation (>2mm), not a physical success claim.
- Current floor_center dispatcher/finalizer supersede exited separated watchers and retain success-only packaging. Native full baseline still pending.


## 06:45 UTC higher first release (current)
- Floor-center063247 failed450frames: first banana shifts while still clamped against bowl, gripper hull intersects bowl. Actual FK TCP at frames399/409/419/429/439: Z1.1294/1.1197/1.1100/1.1003/1.0942m; collision449 atZ1.0975 after contact-driven deflection. Original first lowerTCP1.095m.
- New first lowerZ0.16 (was0.135m above destination), desired TCP1.12015m, earlier than observed clamped-bowl deflection. FirstXY[-0.03,-0.03], secondX+0.075 unchanged; controlled physical open then gravity release, no object setters, no thresholds changed. Second release remains0.135. CLI defaultNone preserves prior behavior; recorded/matched.
- New native base064520 runner216367, b03_collect_floor_drop.sh, corresponding floor_drop dispatcher/finalizer in lgd-day-logs. User offered real failure image for manual input asynchronously; do not wait for optional review.
