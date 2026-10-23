# Discovery checks, 2026-10-24

43 unit tests passed. The added sphere/source test compares against unscreened exact FCL across 64 randomly rotated boxes and preserves a cylinder-AABB corner regression.

v20 replaces the property/robot endpoint AABB rejection with exact source-shape sphere checks at the unchanged margin plus 3mm. AABB lower bounds only skip distant pairs. v21 additionally separates source door-sweep clearance (unchanged category margin) from support clearance (the existing independent audit requirement of 2mm). Final collection and audit thresholds are unchanged.

C01 property diagnostic25: all six candidates still fail the ORIGINAL SOURCE door clearance, even with support checks separated. It is a retained rejection, not a delivery. No unsafe baseline was executed.

C01 motion-space02 is finite sampling, not a feasibility or impossibility proof. It includes exact original robot endpoint geometry as well as planner endpoint spheres. The earlier motion-space01 omitted physical endpoint filtering and contained fixed-base points; it is retained remotely and must not be used as transient-volume evidence. The corrected maximum among samples is about 106.36mm, leaving little room for a finite human hand under the original 100mm planning margin. No threshold was reduced.
