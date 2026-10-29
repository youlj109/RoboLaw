#!/usr/bin/env python3
"""Freeze five evidence-guided banana adjustments without stepping physics."""
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from kitchen_safety.extensions.replicator.scene_build_runtime import sha
from kitchen_safety.extensions.replicator.quick_trajectory_retry import measured_banana_grasp
from kitchen_safety.extensions.replicator.task_collision import PayloadGeometry
from kitchen_safety.planning import DroidKinematics

def main():
    manifest=json.loads((ROOT/'exports/replicator_physical_retry_round1_20261028/manifest.json').read_text())
    evidence=ROOT/'reports/replicator_physical_retry_round1_20261028/visual_review_bananas.json'
    recipes={}
    for row in manifest['entries']:
        sid=row['scene_id']
        if row['execution_status']!='failed' or 'banana' not in sid:continue
        old=json.loads((ROOT/row['result_path']).read_text());layout=json.loads((ROOT/row['config_path']).read_text())
        targets={k:np.array(v) for k,v in old['targets'].items()};oldattach=np.array(old['nominal_attachment'])
        initial=targets['approach']@oldattach;grasp=targets['approach'].copy();info=dict(old['grasp_info'])
        if sid.startswith(('g_shape','l_island')):
            geometry=PayloadGeometry(ROOT/layout['objects']['banana']['usd_path'])
            yaw,offset=(-15.,0.) if sid.startswith('g_shape') else (45.,-.02)
            center,jaw,info=measured_banana_grasp(geometry,yaw,offset)
            jaw=initial[:3,:3]@jaw;down=np.array([0.,0.,-1.])
            grasp[:3,:3]=np.column_stack((down,jaw,np.cross(down,jaw)))
            grasp[:3,3]=initial[:3,:3]@center+initial[:3,3]
            grasp[2,3]=targets['approach'][2,3]  # Retain original support clearance.
            attachment=np.linalg.inv(grasp)@initial
            new={p:t@oldattach@np.linalg.inv(attachment) for p,t in targets.items()}
            new['approach']=grasp
            new['pregrasp']=grasp.copy();new['pregrasp'][:3,3]+=targets['pregrasp'][:3,3]-targets['approach'][:3,3]
            new['lift'][:2,3]=grasp[:2,3]
            targets=new
            description=('将抓取截面移向同任务多个成功变体采用的中段位置，保留原指端高度；闭合角0.50→0.55rad，携物时约束腕部朝向。' if sid.startswith('g_shape') else
                '原抓点靠近香蕉端部且未能稳定提起；改用源网格中段45°截面、沿截面法向偏移-20mm，保持指端高度，先竖直抬升，闭合角0.55rad并约束携物朝向。')
        else:
            attachment=oldattach.copy()
            targets['lower'][2,3]-=.008
            targets['retreat'][2,3]-=.008
            description='保留已能搬起香蕉的倾斜抓姿，闭合角0.50→0.55rad；携物段约束腕部朝向，下放终点降低8mm，张爪等待增至4秒，改善失持及释放支撑。'
        kin=DroidKinematics(np.array(layout['robot_position'])+[0,0,layout['stand_height']],layout['robot_yaw_deg'])
        with np.load(ROOT/row['source_trajectory_path']) as arr:
            seeds={p:arr['q'][arr['phase']==p][-1] for p in targets}
        checks={}
        for phase,t in targets.items():
            try:
                q=kin.ik(t,seeds[phase],attempts=2);actual=kin.fk(q)
                checks[phase]=dict(position_error_m=float(np.linalg.norm(actual[:3,3]-t[:3,3])),
                    angle_error_rad=float(Rotation.from_matrix(actual[:3,:3]@t[:3,:3].T).magnitude()),q=q.tolist())
            except Exception as error:checks[phase]=dict(error=str(error))
        recipes[sid]=dict(scene_id=sid,task_id=row['task_id'],payload_name='banana',layout_fixed=True,
            target_poses_world={k:v.tolist() for k,v in targets.items()},nominal_attachment=attachment.tolist(),
            nominal_gripper_joint_rad=.55,grasp_info=info,description=description,
            execution_overrides={'release_seconds':4.},hold_orientation_phases=['lift','transfer','lower'],
            evidence={'path':str(evidence.relative_to(ROOT)),'sha256':sha(evidence)},cpu_endpoint_ik=checks,
            source_config_sha256=row['config_sha256'],source_result_sha256=row['result_sha256'],
            limitations='CPU IK is not collision or force-closure proof; one frozen cuRobo candidate and one real test remain.')
    out=ROOT/'configs/extensions/replicator/physical_retry_round2_banana_recipes_20261028.json'
    with out.open('x') as f:json.dump(dict(schema_version=1,recipes=recipes),f,ensure_ascii=False,indent=2);f.write('\n')
    print(json.dumps({k:{p:'error' in v for p,v in r['cpu_endpoint_ik'].items()} for k,r in recipes.items()}))
if __name__=='__main__':main()
