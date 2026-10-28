# Asset Delivery Scope

Compared with remote `ylj` commit
`770c67b50f4fc2b20cb7b6afd237c852435e5957` using the authenticated GitHub tree
API. This is a packaging comparison, not a merge or a simulation revalidation.

The updated upstream still omits shared kitchen/robot binaries and the original
MPFB source assets, but it does include baked custom human USDs and textures
under `assets/extensions/replicator/hard_v1/` and
`assets/extensions/replicator/human_visual_low_reach01/`. Its older controls
example instead retains only a wrapper/recipe/manifest. Therefore upstream
does not have one blanket rule excluding every generated human payload.

This branch follows the complete custom-asset precedent for the human assets
required by its nine current D-group scene configurations:

- `assets/extensions/controls/human_hand_between_middle01/`: shared by the
  current left/right cancel human variants.
- `assets/extensions/controls/human_oven_knob_quality04/`: current oven human.
- `assets/extensions/controls/human_oven_reach_down/`: human asset still declared
  by the current oven property/obstacle configurations. The planning manifest
  verifier checks all declared variants, so this dependency is included too.

Each includes the existing baked `person.usdc`, six required texture PNGs,
collision wrapper, pose recipe, independent geometry audit and hash manifest.
The payload bytes are unchanged from those used in the recorded verification;
no regeneration, material/geometry edit, layout change or relaxed gate is made.
Their provenance remains in the asset manifests and `assets/people/SOURCES.txt`.

Shared kitchen/NVIDIA robot/original MPFB source assets, dependencies and
runtimes continue to use the repository's existing restore/install scripts.
Raw recordings, local inspection stages, datasets and user annotation databases
remain outside Git. Rejected and historical human variants are not all bundled
merely because they exist locally. Screenshot folders are also left untouched.

This closes the missing active-human-payload packaging gap. It does **not**
claim a clean-machine end-to-end reproduction: installation, shared-asset
restoration and local inspection/baseline generation are still prerequisites.
Current safety status remains 8/9; the left fixture necessity rejection is not
changed by committing asset files.
