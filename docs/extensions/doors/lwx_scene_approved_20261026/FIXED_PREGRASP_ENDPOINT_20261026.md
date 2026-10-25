# Matched pregrasp endpoint option

Optional task-local pregrasp_goal_q fixes one redundant joint endpoint for the existing Cartesian pregrasp target. It is used equally with the obstacle on and off. cuRobo plans the full route; no route waypoint is supplied. The existing default pose planner remains available when this option is absent.

The adapter rejects endpoints whose TCP differs by more than 1 mm or 0.01 rad from the requested pregrasp pose. Every trajectory retains dense cuRobo and independent source-geometry checks, unchanged safety clearances, speed and joint limits. The optional bounded_chords_v1 policy is also identical on/off and preserves endpoints, action count and peak joint step; each chord and final route require complete dense checking.

Collector metadata records all seven endpoint joints. Paired planning, source preflight, neutral collection and variant reproduction propagate them from the actual successful matching baseline. New frozen starts or policies require new real successful baseline and neutral episodes; offline discovery is not a matching physical baseline.

A legal contact-point change is recorded separately from fixed-endpoint avoidance. Source-geometry necessity, actual contact causality, scene stability, full door sweep, self-collision and visual review remain separate acceptance gates. A passing preflight alone does not establish physical acceptance.

Validation: 12 existing CPU checks for dense acceptance, rejected unsafe/incomplete shortcuts, preserved endpoints and speed, inherited scene hashes, and exact handle-contact scope passed. Per-episode physical and paired evidence is recorded in acceptance checkpoints rather than inferred from these CPU tests.
