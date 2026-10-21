import json,numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics
ROOT=Path(__file__).resolve().parents[3]
m=json.loads((ROOT/'datasets/skill_trials/doors/base_attempt_06/metadata.json').read_text());r=json.loads((ROOT/'datasets/skill_trials/doors/base_attempt_06/trajectory.json').read_text());kin=DroidKinematics(m['robot_base'],m['robot_yaw_deg']);report=[]
for offset in (0.,.05,.09,.12):
    q=np.array(next(x for x in r if x['phase']=='grasp_handle')['obs']['q'])[:7];handle=np.array(m['handle_initial'])+[0,0,offset];hinge=np.array(m['hinge']);R0=np.array([[0,1,0],[1,0,0],[0,0,-1.]])
    try:
        T=np.eye(4);T[:3,3]=handle;T[:3,:3]=R0;q=kin.ik(T,q,attempts=3)
        margin=99.
        for angle in np.linspace(0,.4,81):
            rr=Rotation.from_rotvec(np.array(m['axis'])*angle).as_matrix();T[:3,:3]=rr@R0;T[:3,3]=hinge+rr@(handle-hinge);q=kin.ik(T,q,attempts=1);margin=min(margin,float(np.min(np.minimum(q-kin.limits[:,0],kin.limits[:,1]-q))))
        report.append({'offset_z_m':offset,'all_ik_solved':True,'min_joint_limit_margin_rad':margin,'last_q':q.tolist()})
    except Exception as exc:report.append({'offset_z_m':offset,'all_ik_solved':False,'error':str(exc),'last_angle_rad':float(angle)})
print(json.dumps(report,indent=2));(ROOT/'reports/skill_trials/doors/grasp_height_kinematics.json').write_text(json.dumps(report,indent=2))
