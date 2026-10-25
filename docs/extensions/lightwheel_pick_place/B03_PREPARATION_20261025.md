CURRENT: B03 human episode171505 achieved full original dual-banana success; all independent contact/RGB/full-world/source/self checks passed. Effective avoidance still waits for a matched successful baseline. Read OVERNIGHT_ACTIVE_20261025.md for live jobs. Historical sections below are not the current status.

# B03 preparation

No new B03 physical success is claimed. GPU work starts with a neutral layout preview after B02 verification. Review the two bananas and plate before planning or collection.

The older failed run used a Cartesian-line reset-to-hover move that keeps the initial TCP orientation. A CPU endpoint screen found collision-clear IK endpoints with negative tilt and yaw 180 degrees; these are candidates only, not valid paths or grasps. New CLI options permit a MotionGen move to the oriented hover while retaining the original B02 defaults.

Banana collision-mesh local Z bounds are [-0.018335, 0.018335] m. Contact evidence must use banana geometry rather than the inherited lime range. Both bananas require separate raw contact records and independently checked lift/support evidence before acceptance; the existing single-object validator must be adapted before calling B03 complete.

The old banana variant coordinates appear copied from the shelf task and must be rebuilt near the actual banana-to-bowl corridor. Do not collect with those old layouts.


## Layout approval and bounded trials

The user approved /root/autodl-tmp/B03-review/base_layout on 2026-10-25. Both bananas remain on the plate after 4 seconds of neutral stepping. The first attempt stopped after one recorded frame because the exact plate support root did not include its nested collision mesh; source matching now includes only that root and descendants. The next attempt reached hover but cuRobo rejected contact due to banana_1. Offline two-segment planning passed at -20 mm world-X TCP bias, but physical closing had only one loaded finger (other side minimum separation 1.066 mm). No lift or success is claimed for those trials. A further -2 mm TCP centering correction passed the same offline checks before the next physical attempt.

Dual-pick execution now preserves each banana's raw contacts/action ranges, resets source support per pick, refreshes the moving collision world and checks both original final support predicates. The independent validator recomputes contact/lift/final evidence for both and a raw-mesh/source-contact audit. Unit tests cover a missing second placement and contact records for the wrong pose, immutable hull rebinding and exact source-root scope (5 passed). B03 is still in progress.


## User-approved fixed safety scenes and planning policy

The user approved /root/autodl-tmp/B03-review/review_now/ (human-front.jpg, property-side.jpg, robot-side.jpg) and explicitly said to fix these scenes before planning. Config: variants_bananas_review_layout_v2.json; human asset person_yaw30.usda. /root/autodl-tmp/B03-review/scene_approval.json binds the scene hashes. Hazard positions must not be moved to solve planning failures.

The approved task-rigid display poses come from the earlier physically settled base scene. The graph placer cannot represent their roll/pitch, so initial-state-layout sets those measured poses once during reset, before controls and observations, with zero velocity. There are no object setters during task execution. This source and its SHA256 are recorded. Static review images are visual placement only, not physical success evidence.

Fresh task meshes must be loaded into MotionGen after measured motion. The old backend update_dynamic_obstacle supports only glass/barrier boxes and returns False for task meshes. B03 now rebuilds the planning model from current task-rigid poses at each segment. The previously rejected held-banana lift passed complete cuRobo/FCL preflight with the fresh scene.

The first approved-scene side-grasp trial formed bilateral contacts but slipped during lift, and remains failed. The new b03-topdown-v1 policy searches upward hover/contact/lift candidates before executing, then direct/upper transfer routes, at fixed hazards. Both bananas use the same candidate family independently. Selected poses may differ between scenes; the policy/input controls must match for baseline comparison.

To reduce collection latency, later trials retain 120 Hz raw task-rigid/finger/fruit states and live 120 Hz added-hazard checks, with 15 Hz live full-world checks. Before acceptance, audit_deferred_world.py must complete every 120 Hz full-world check using unchanged geometry/margins and exactly reproduce every live checkpoint within 1e-7 m. Original pending evidence is preserved. No pending audit can pass validation or acceptance.

Read OVERNIGHT_ACTIVE_20261025.md for exact current overnight state and deadline.
