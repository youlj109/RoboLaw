"""CPU-only pose feasibility and original collision geometry diagnostic."""
import json
import numpy as np
from pathlib import Path
from pxr import Usd
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.controls.collision import ControlsCollisionWorld
state=json.loads(Path('reports/skill_trials/controls/inspection/state.json').read_text())
pose=state['robot_pose'];kin=DroidKinematics(pose[:3],Rotation.from_quat(pose[3:7]).as_euler('xyz',degrees=True)[2])
stage=Usd.Stage.Open('reports/skill_trials/controls/inspection/stage.usda')
world=ControlsCollisionWorld(stage,kin,env_root='/World/envs/env_0')
q=np.array([state['robot_joints'][f'panda_joint{i}'] for i in range(1,8)])
g=dict(zip(world.GRIPPER_JOINT_NAMES,np.array([1,1,-1,1,-1,-1])*.785))
print('initial',world.clearance(q,gripper_q=g),flush=True)
records=[]
for roll in [0,45,90,135,180,225,270,315]:
    for zoffset in [0,.01,-.01]:
        t=np.eye(4);t[:3,:3]=Rotation.from_euler('z',90,degrees=True).as_matrix()@Rotation.from_euler('x',roll,degrees=True).as_matrix()
        t[:3,3]=[2.08087763,-.5,1.03313965+zoffset]
        rec={'roll':roll,'zoffset':zoffset}
        try:
            v=kin.ik(t,q,attempts=4);rec['q']=v.tolist();rec['clearance']=world.clearance(v,gripper_q=g)
        except Exception as e:rec['error']=str(e)
        records.append(rec);print(json.dumps(rec),flush=True)
Path('reports/skill_trials/controls/pose_candidates.json').write_text(json.dumps(records,indent=2))
