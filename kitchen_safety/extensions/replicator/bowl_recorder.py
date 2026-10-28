"""Bowl/sink observations under the complete task-aware substep recorder.

Only the capture method differs from the frozen banana recorder: payload
identity is bowl, with the original sink ObjectReference and support sensor.
Every startup/physical-step callback, metadata and write/audit gate is reused.
"""
import numpy as np
from scipy.spatial.transform import Rotation

from .startup_recorder import array
from .task_recorder import TaskPhysicsRecorder


class BowlPhysicsRecorder(TaskPhysicsRecorder):
    def capture(self):
        from pxr import UsdGeom
        if self.object_names != ["bowl"] or self.contacts.object_name != "bowl":
            raise ValueError("Original bowl/sink recorder must observe exactly the source bowl payload")
        cache, robot = UsdGeom.XformCache(), self.robot
        body = np.concatenate((array(robot.data.body_pos_w)[0], array(robot.data.body_quat_w)[0]), axis=-1)
        palm = body[robot.body_names.index("base_link")]
        tcp = np.r_[palm[:3]+Rotation.from_quat(palm[3:]).apply([.131, 0., 0.]), palm[3:]]
        states = np.stack([array(self.scene[name].data.root_state_w)[0] for name in self.object_names])
        row = dict(physics_step=int(self.e.sim.get_physics_step_count()), phase=self.phase,
            action_index=self.action_index, q=array(robot.data.joint_pos)[0], dq=array(robot.data.joint_vel)[0],
            planning_attachment=np.eye(4), has_planning_attachment=False,
            robot_body_pose_w=body, tcp_pose_w=tcp, task_object_states=states,
            robot_contact_force=array(self.scene["robot_contacts"].data.net_forces_w)[0],
            destination_contact_force=array(self.scene[self.task.contact_sensor_cfg.name].data.force_matrix_w)[0],
            background_rigid_poses=array(self.background_view.get_transforms()),
            background_rigid_velocities=array(self.background_view.get_velocities()))
        for key, paths in (("static_anchor_transforms", self.static_paths), ("task_reference_transforms", self.reference_paths)):
            row[key] = np.asarray([np.asarray(cache.GetLocalToWorldTransform(self.e.sim.stage.GetPrimAtPath(path))).T
                                  for path in paths]).reshape(-1, 4, 4)
        for index, (_, view) in enumerate(self.articulation_views):
            row[f"background_articulation_{index}_q"] = array(view.get_dof_positions())
            row[f"background_articulation_{index}_dq"] = array(view.get_dof_velocities())
        self.rows.append(row)
        self.contacts.sample(self.phase, self.action_index, states[0, :7], physics_step=row["physics_step"])
        return row
