"""Explicit contact pairs for the assigned G banana task, not object exclusions.

cuRobo's world switch is object-wide. It is used only to generate candidates;
the independent FCL checker restores every object and checks each robot part.
This policy never changes PhysX collision filtering or an environment margin.
"""
from dataclasses import dataclass


PHASES = ("reset", "warmup", "initial_settle", "pregrasp", "approach", "close",
          "lift", "transport", "lower", "release", "retreat", "final_settle")
FINGER_PHASES = frozenset(("approach", "close", "lift", "transport", "lower", "release", "retreat"))
DESTINATION_PHASES = frozenset(("lower", "release", "retreat", "final_settle"))


@dataclass(frozen=True)
class BananaContactPolicy:
    payload_name: str = "banana"
    destination_name: str = "bowl"
    source_support: str = "/World/envs/env_0/kitchen/CornerCounter_00_0"
    # Same source-mesh/contact band already used by the A baseline's support
    # exception. It is NOT an exemption for robot/counter or payload/other mesh.
    support_band_m: float = .006

    def validate_phase(self, phase):
        if phase not in PHASES:
            raise ValueError("No task contact policy for phase: " + str(phase))

    def cuda_candidate_masks(self, phase, attached):
        self.validate_phase(phase)
        masks = []
        if phase in FINGER_PHASES or attached:
            masks.append(self.payload_name)
        if phase in DESTINATION_PHASES:
            masks.append(self.destination_name)
        return tuple(masks)

    def finger_pair(self, label, phase):
        self.validate_phase(phase)
        return phase in FINGER_PHASES and label in (
            "gripper/left_inner_finger", "gripper/right_inner_finger")

    def redundant_finger_envelope(self, label, phase):
        # These envelopes span empty space between the two fingers. Their
        # source-derived per-part hulls remain in the very same pair query;
        # base/palm/knuckles/outer fingers are NOT exempted.
        return phase in FINGER_PHASES and label in ("finger", "robot_hull/whole_gripper")

    def report(self):
        return dict(payload=self.payload_name, destination=self.destination_name,
            source_support=self.source_support, source_support_contact_band_m=self.support_band_m,
            support_band_source="Existing A CollisionWorld._pairs source-contact band (6 mm); exact source support only",
            outside_contact_default_clearance_m=.002,
            outside_contact_source="Existing A CollisionWorld default_clearance; unchanged",
            cuda_limitation="Object-wide candidate masks only; FCL full world and part-wise contact policy required for every dense path and actual physics sample",
            allowed_fingers=["left_inner_finger", "right_inner_finger"],
            measured_midbody_contacts_required=True,
            physical_collision_filter_changes=False,
            destination_policy="Source-triangle intersection check replaces filled convex payload proxy only for intended bowl contact; no robot/bowl exemption")
