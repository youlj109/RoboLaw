"""Exploratory CPU support-height and limited IK checks (not acceptance).
Use audit_source_relation.py for all upstream source-relation gates and
preflight_legal_layout.py for the current recorded-target IK/FCL probe.
"""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom
from kitchen_safety.extensions.replicator.layout import resolve_layout
from kitchen_safety.planning import DroidKinematics
ROOT=Path(__file__).resolve().parents[3]
j=resolve_layout();s=Usd.Stage.Open(str(ROOT/j['kitchen_usd']));b=UsdGeom.BBoxCache(0,['default','render','proxy'],False,True)
r=b.ComputeWorldBound(s.GetPrimAtPath('/World/CapSurface_03')).ComputeAlignedRange();low=np.array(r.GetMin());high=np.array(r.GetMax())
kin=DroidKinematics(np.array(j['robot_position'])+[0,0,j['stand_height']],j['robot_yaw_deg']);seed=np.deg2rad([0,-36,0,-144,0,108,0]);rows=[]
for name,extra in [('mustard_position',.125),('mustard_position',0),('bowl_position',.145)]:
 t=np.eye(4);t[:3,3]=np.array(j[name])+[0,0,extra];t[:3,:3]=Rotation.from_euler('z',180,degrees=True).as_matrix()@Rotation.from_euler('x',45,degrees=True).as_matrix()@np.array([[0,-1,0],[0,0,1],[-1,0,0]])
 try:q=kin.ik(t,seed);error=np.linalg.norm(kin.fk(q)[:3,3]-t[:3,3]);row=dict(target=name,extra=extra,ik=True,q=q.tolist(),position_error=float(error))
 except Exception as e:row=dict(target=name,extra=extra,ik=False,error=str(e))
 rows.append(row)
report=dict(task_id=j['task_id'],counter_bounds=[low.tolist(),high.tolist()],counter_height_agrees=bool(abs(high[2]-j['counter_height'])<1e-6),original_yaw_rad=float(np.deg2rad(j['robot_yaw_deg'])),ik=rows,collision_checked=False)
(ROOT/'reports/skill_trials/replicator/geometry.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
