# Authorized Oven Human Arm Revision

The user explicitly authorized returning to visual revision after rounds184-192.
Only the oven human right arm is revised. Body root, feet, source skeleton bone
lengths, original robot/task, the two cancel tasks and all other variants remain.
New layouts do not inherit earlier physical/safety acceptance.

## Image-Only Review

R03 asset: `assets/extensions/controls/human_oven_knob_quality03`.
Wrist recipe `-.74 .20 1.27`, source bend-plane, hand follows forearm.
The real `wide`, `side`, `external_camera` and `external_camera_2` views show
a continuous shoulder/elbow/wrist, feet on the floor and a hand extending
beside the oven controls. No obvious initial interpenetration or reversed wrist
is visible. Original camera1 has occlusion; other views are retained.
R03 passes visual review, then is frozen. Round193 retains the known154
controller, but its one nominal connection fails at19.707/20 mm. Round194
annotates an offline recorded-source check; no physical attempt is made.
These later geometry results are not the basis of its visual verdict.

R04 asset: `assets/extensions/controls/human_oven_knob_quality04`.
Wrist recipe `-.68 .20 1.15`, elbow down, source bend-plane, hand follows forearm.
The real `wide`, `side` and `external_camera_2` views show an elbow lowered
naturally below the shoulder and an ascending forearm reaching beside the oven
handle. The hand/forearm are continuous without an obvious reversed bend.
Feet remain visibly floor-supported; no obvious initial penetration is visible.
R04 passes visual review and is frozen before any planning checks.

Raw renders are in `reports/controls_visual_20261027/07_oven_temperature_human/r03`
and `r04`; each has config hashes and render metadata. The live webpage retains
both revisions, previous failure evidence and manual notes.

## Subsequent Checks, Not Visual Verdicts

Independent Blender imports confirm unchanged foot vertices and non-right-arm
bones, retained source-relative wrist frame and six exported meshes. The asset
manifests bind export, wrapper, pose, geometry-audit and texture hashes. These
checks support provenance only, not clinical anatomy or safety certification.

Round195 rebinds the unchanged154 contact controller to r04. Its single nominal
CPU connection passes159 IK queries; this is not physical acceptance. A fresh
real cuRobo/contact/data attempt uses the exact matching successful154 baseline.
The501-frame physical recording reaches original level three, then releases and
retreats while retaining the predicate. Independent data, contact/physics, fixed
base and self-collision checks pass. Fresh50-second neutral, matched seeds42/43/44,
source-shape avoidance and first-violation evidence are now independently bound.
Finalization `benchmark_finalize195/report.json` accepts all9 current scenes.
The r03 failure is retained; it does not inherit r04's acceptance.

An early independent avoidance audit was scheduled before the queued robot
self-collision audit had produced its required JSON. It terminated with
`FileNotFoundError: self_collision_detailed_audit.json`, without writing an
avoidance report. This is an orchestration precondition failure, not a physical
or trajectory failure. Finalization must wait for complete physical validation
and then run the independent audit; no recollection or replanning is justified.
