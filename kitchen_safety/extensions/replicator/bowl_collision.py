"""Independent G bowl/sink contact policy and source-payload world.

Reuses checked geometric methods without changing the frozen banana adapter.
The destination is the original sink basin-bottom reference, never a second
movable bowl or a whole-kitchen mask.
"""
from dataclasses import dataclass

from .collision_planner import CollisionWorld
from .task_collision import PayloadCollisionWorld, PayloadGeometry
from .task_contact_policy import BananaContactPolicy


@dataclass(frozen=True)
class BowlContactPolicy(BananaContactPolicy):
    payload_name: str = "bowl"
    destination_name: str = "/World/envs/env_0/kitchen/Sink/BasinBottom"
    source_support: str = "/World/envs/env_0/kitchen/CapSurface_02"

    def report(self):
        report = super().report()
        report.pop("measured_midbody_contacts_required")
        report.update(measured_upper_wall_rim_contacts_required=True,
            destination_policy="Only original Sink/BasinBottom payload contact uses source triangles; sink walls, other kitchen meshes and every robot part retain original checks",
            original_destination_kind="ObjectReference inside the source kitchen, not a scene rigid object")
        return report


class BowlCollisionWorld(PayloadCollisionWorld):
    def __init__(self, stage, kin, *, payload_usd, **kwargs):
        self.payload_name = "bowl"
        self.payload_geometry = PayloadGeometry(payload_usd)
        CollisionWorld.__init__(self, stage, kin, **kwargs)
        self.payload_model, self.payload_hull_vertices = self._convex(self.payload_geometry.vertices)
        self.contact_policy = BowlContactPolicy()
        parts = {name.rsplit("/", 1)[-1] for name, _, _ in self.gripper_models}
        if not {"base_link", "left_inner_finger", "right_inner_finger"} <= parts or len(parts) <= 3:
            raise ValueError("Bowl rim contact needs all original gripper part hulls")
        self.phase, self.payload_pose, self.gripper_joint_positions = "reset", None, None
        model = self.fcl.BVHModel()
        model.beginModel(len(self.payload_geometry.vertices), len(self.payload_geometry.triangles))
        model.addSubModel(self.payload_geometry.vertices, self.payload_geometry.triangles)
        model.endModel()
        self.payload_triangle_model = model

