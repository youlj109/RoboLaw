"""Exact mustard/bowl identities under the complete task substep recorder."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from .startup_recorder import array
from .task_recorder import TaskPhysicsRecorder


class MustardFamilyRecorder(TaskPhysicsRecorder):
    def __init__(self,env,envdef,contacts):
        from kitchen_safety.grasp_geometry import bottle_body_geometry
        from .mustard_family_preparation import ROOT
        if set(envdef.resolved_task_layout['objects'])!={'mustard_bottle','bowl'} or contacts.object_name!='mustard_bottle':
            raise ValueError('Require exact original mustard/bowl identities')
        self.body_geometry=bottle_body_geometry(ROOT/envdef.resolved_task_layout['objects']['mustard_bottle']['usd_path'])
        super().__init__(env,envdef,contacts)
        self.metadata['source_bottle_body_geometry']=self.body_geometry

    def capture(self):
        from pxr import UsdGeom
        cache,robot=UsdGeom.XformCache(),self.robot
        body=np.concatenate((array(robot.data.body_pos_w)[0],array(robot.data.body_quat_w)[0]),axis=-1)
        palm=body[robot.body_names.index('base_link')]
        tcp=np.r_[palm[:3]+Rotation.from_quat(palm[3:]).apply([.131,0.,0.]),palm[3:]]
        states=np.stack([array(self.scene[name].data.root_state_w)[0] for name in self.object_names])
        row=dict(physics_step=int(self.e.sim.get_physics_step_count()),phase=self.phase,action_index=self.action_index,
            q=array(robot.data.joint_pos)[0],dq=array(robot.data.joint_vel)[0],
            planning_attachment=np.eye(4),has_planning_attachment=False,robot_body_pose_w=body,tcp_pose_w=tcp,
            task_object_states=states,robot_contact_force=array(self.scene['robot_contacts'].data.net_forces_w)[0],
            destination_contact_force=array(self.scene[self.task.contact_sensor_cfg.name].data.force_matrix_w)[0],
            background_rigid_poses=array(self.background_view.get_transforms()),
            background_rigid_velocities=array(self.background_view.get_velocities()))
        for key,paths in (('static_anchor_transforms',self.static_paths),('task_reference_transforms',self.reference_paths)):
            row[key]=np.asarray([np.asarray(cache.GetLocalToWorldTransform(self.e.sim.stage.GetPrimAtPath(path))).T
                                 for path in paths]).reshape(-1,4,4)
        for index,(_,view) in enumerate(self.articulation_views):
            row[f'background_articulation_{index}_q']=array(view.get_dof_positions())
            row[f'background_articulation_{index}_dq']=array(view.get_dof_velocities())
        self.rows.append(row)
        self.contacts.sample(self.phase,self.action_index,states[self.object_names.index('mustard_bottle'),:7],
                             physics_step=row['physics_step'])
        return row

    def write(self,directory,actions):
        from .mustard_family_collision import audit_bottle_body_contacts
        report=super().write(directory,actions)
        contacts=audit_bottle_body_contacts(self.contacts.records,self.body_geometry)
        report['bottle_body_contact_audit']=contacts
        report['checks']['every_bottle_contact_inside_source_body']=contacts['passed']
        report['passed']=all(report['checks'].values())
        (Path(directory)/'bottle_body_contact_audit.json').write_text(json.dumps(contacts,indent=2,allow_nan=False)+'\n')
        return report
