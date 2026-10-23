# lwx follow-up discovery and supervision checks

The baseline supervisor accepts the existing bounded pull-step, release-distance and release-tilt parameters, preserving its original defaults. Search supervision can use v21 and waits for all baseline gates. Existing completed or active variant campaigns prevent repeated collection in the same root.

v22 additionally screens actual retained approach states and the rejected command with exact source geometry, using execution clearance plus 5mm for discovery. Source-asset mismatch is rejected explicitly. This is a conservative discovery filter, not physics acceptance. C02 search15 accidentally omitted the narrow_opening asset override and screened default projecting_crossbar; no candidate or physical trial resulted. It is retained and explicitly invalid for narrow_opening conclusions. Corrected search16 found 20 candidates, but its three paired candidates failed dense pregrasp FCL gates; none was physically collected. The mismatch integration check demonstrates the new guard; its exit code was not captured.

C07 tilt diagnostics preserve recorded failure hashes and explicitly distinguish recorded-pose geometry predictions and full bounded IK from dynamic acceptance. The new baseline attempt retains the grasp and tests 40mm/-60deg release only after both offline checks. The preceding 15deg baseline opened the door but failed the 2mm finger-door clearance during release; it remains rejected.

The C01 fingertip search builds an original-bone-length, fixed-feet source-rig pose and centers placements on a finite transient-space diagnostic sample. It is only CPU discovery. The first 2904 candidates yielded 441 actual baseline blockers but zero scene-legal accepted layouts; no physical trial was started. This does not prove the task impossible.

Seven source files passed syntax checks. Existing exact-source/FCL tests remain separately recorded (43 passing); these orchestration changes were checked with syntax and the actual retained workflows, not new unit tests. No execution or final delivery thresholds changed.
