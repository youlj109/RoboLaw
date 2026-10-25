# B03 preparation

No new B03 physical success is claimed. GPU work starts with a neutral layout preview after B02 verification. Review the two bananas and plate before planning or collection.

The older failed run used a Cartesian-line reset-to-hover move that keeps the initial TCP orientation. A CPU endpoint screen found collision-clear IK endpoints with negative tilt and yaw 180 degrees; these are candidates only, not valid paths or grasps. New CLI options permit a MotionGen move to the oriented hover while retaining the original B02 defaults.

Banana collision-mesh local Z bounds are [-0.018335, 0.018335] m. Contact evidence must use banana geometry rather than the inherited lime range. Both bananas require separate raw contact records and independently checked lift/support evidence before acceptance; the existing single-object validator must be adapted before calling B03 complete.

The old banana variant coordinates appear copied from the shelf task and must be rebuilt near the actual banana-to-bowl corridor. Do not collect with those old layouts.


## Layout approval and bounded trials

The user approved /root/autodl-tmp/B03-review/base_layout on 2026-10-25. Both bananas remain on the plate after 4 seconds of neutral stepping. The first attempt stopped after one recorded frame because the exact plate support root did not include its nested collision mesh; source matching now includes only that root and descendants. The next attempt reached hover but cuRobo rejected contact due to banana_1. Offline two-segment planning passed at -20 mm world-X TCP bias, but physical closing had only one loaded finger (other side minimum separation 1.066 mm). No lift or success is claimed for those trials. A further -2 mm TCP centering correction passed the same offline checks before the next physical attempt.

Dual-pick execution now preserves each banana's raw contacts/action ranges, resets source support per pick, refreshes the moving collision world and checks both original final support predicates. The independent validator recomputes contact/lift/final evidence for both and a raw-mesh/source-contact audit. Unit tests cover a missing second placement and contact records for the wrong pose, immutable hull rebinding and exact source-root scope (5 passed). B03 is still in progress.
