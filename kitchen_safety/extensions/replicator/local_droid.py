"""Upstream DROID configuration using local asset specs, without global patching."""
from dataclasses import replace
from pathlib import Path
from isaaclab_arena.embodiments.droid import droid
from isaaclab_arena.embodiments.embodiment_base import EmbodimentBase
from isaaclab_arena.embodiments.robot_on_stand_utils import compose_on_stand_usd
from isaaclab_arena.embodiments.gripper import RobotiqGripper
ROOT=Path(__file__).resolve().parents[3]

class LocalDroid(droid.DroidAbsoluteJointPositionEmbodiment):
    def __init__(self, *, enable_cameras, initial_pose, initial_joint_pose, stand_height_m):
        EmbodimentBase.__init__(self,enable_cameras=enable_cameras,initial_pose=initial_pose)
        self.gripper=RobotiqGripper()
        self.stand_height_m=stand_height_m
        self.stand_footprint_xy_m=droid._DROID_STAND_PRIM.stand_default_footprint_xy_m
        self.placement_bbox_stand_only=True
        self.scene_config=droid.DroidSceneCfg()
        prefix=ROOT/'assets/nvidia/Assets/Isaac/6.1/Isaac/IsaacLab/Arena/assets'
        robot_spec=replace(droid._DROID_ROBOT_PRIM,robot_usd_path=str(prefix/'robot_library/droid/franka_robotiq_2f_85_flattened.usd'))
        stand_spec=replace(droid._DROID_STAND_PRIM,stand_usd_path=str(prefix/'object_library/srl_robolab_assets/robots/franka_stand_grey.usda'))
        self.scene_config.robot.spawn.usd_path=compose_on_stand_usd(robot_spec,stand_spec,stand_height_m=stand_height_m,stand_footprint_xy_m=self.stand_footprint_xy_m,output_basename='replicator_local_droid')
        self.action_config=droid.DroidAbsoluteJointPositionActionsCfg()
        self.camera_config=droid.DroidCameraCfg()
        self.observation_config=droid.DroidObservationsCfg()
        self.event_config=droid.DroidEventCfg()
        self.set_initial_joint_pose(initial_joint_pose)
        self.reward_config=None;self.mimic_env=None
        self.add_camera_variations(self.camera_config)
