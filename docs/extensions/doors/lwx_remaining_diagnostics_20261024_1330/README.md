# C01/C07 remaining work diagnostic checkpoint

Snapshot: 2026-10-24 13:30 Beijing. Overall delivery remains 6/8.

C01 original successful opening and user-approved steel presentation are retained. The user-requested original repair is human/property placement so the object genuinely blocks the original route. Later repair failures below do not establish an original opening failure. Steel is not modified or recaptured.

- C01 human76: repaired layout passed three-seed source obstruction and small detour, but actual pull action457 reached19.984984mm versus original20mm safety margin. Rejected. Human79 discovery now constrains measured failed motion; six conservative source-legal candidates, no acceptance claim.
- C01 property78: repaired wine layout passed three-seed source obstruction and small detour, but actual187-frame trial rejected a pull command. CPU reconstruction passes external geometry; new offline GPU isolation rejects with world and accepts without it, locating nominal source Wall03_10. This reconstructs nominal configured hazard and a proposed next command; it is not an exact observed failed-command or changed-hazard dynamics proof. Bottle81 and wine82 add conservative failed-motion/command constraints; at most one actual trial per campaign, wine82 proceeds only after bottle81 failure.
- C07 robot66: original door task achieved, but released_hold action985 finger-door clearance1.797012mm is below original2mm. Rejected. Secondary video frame mismatch986/985 is retained; repairing decoding would not make this physical trial pass.
- C07 human67: original door task achieved, but final release-target hold rejected by cuRobo at991 recorded frames. Rejected.
- C07 diagnostic93 compares bounded retreat directions at recorded states. -25deg has greater proposed clearance and passes the same original joint-increment/orientation checks in these finite CPU samples. It does not prove changed dynamics, GPU validity or actual success. Fresh matched baseline94 and new robot68/human69 campaigns are queued, with original4cm retreat and every final safety gate unchanged.
- C07 property: legal original handle offset-6cm passed screen92; fresh actual baseline92 and source-bottle discovery95 are queued. A legal handle change is grasp adaptation across baselines; each off/on pair must keep its baseline target fixed.

Evidence copies are byte-verified using evidence_copies.json. Original large trajectory/physics/video hashes are recorded in source_hashes.json; originals remain in datasets/doors. Copies of ongoing trial commands describe queued work only. No new successful variant, full delivery, physical success rate, or complete visual review is claimed in this checkpoint.
