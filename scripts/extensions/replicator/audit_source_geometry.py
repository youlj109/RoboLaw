"""Recheck all recorded physics substeps with source bottle collision geometry."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from scripts.extensions.replicator.audit import hazard_stage
from kitchen_safety.extensions.replicator.source_geometry import SourceBottleGeometry,source_clearance
from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld
from kitchen_safety.geometry_audit import static_collision_shapes
from kitchen_safety.planning import DroidKinematics
ROOT=Path(__file__).resolve().parents[3]
def main():
 p=argparse.ArgumentParser();p.add_argument('--episode',type=Path,required=True);p.add_argument('--layout',type=Path,required=True);p.add_argument('--report',type=Path,required=True);p.add_argument('--hull',action='store_true');a=p.parse_args()
 if a.report.exists():raise FileExistsError(a.report)
 layout=json.loads(a.layout.read_text());stage,_=hazard_stage(layout,'human',layout['obstacle']);hazard=static_collision_shapes(stage,'/World/Person');kin=DroidKinematics(np.asarray(layout['robot_position'])+[0,0,layout['stand_height']],layout['robot_yaw_deg']);world=CollisionWorld(stage,kin);bottle=SourceBottleGeometry(a.hull);data=np.load(a.episode/'physics_states.npz');names=data['joint_names'].tolist();ids=data['arm_ids'];rows=[];required=layout['planning']['execution_clearances_m']['person']
 for i,q in enumerate(data['q']):
  grip=dict(zip(names,q));m=source_clearance(world,bottle,q[ids],grip,data['mustard'][i,:7],hazard,required)
  rows.append(dict(physics_index=i,action_index=int(data['action_index'][i]),phase=str(data['phase'][i]),**m))
 report=dict(episode=str(a.episode),layout=str(a.layout),layout_sha256=hashlib.sha256(a.layout.read_bytes()).hexdigest(),physics_sha256=hashlib.sha256((a.episode/'physics_states.npz').read_bytes()).hexdigest(),source=bottle.provenance,required_m=required,samples=len(rows),minimum=min(rows,key=lambda x:x['distance_m']),violations=[x for x in rows if not x['valid']],phase_minima={s:min([r for r in rows if r['phase']==s],key=lambda x:x['distance_m']) for s in sorted(set(data['phase']))},passed=all(r['valid'] for r in rows),scope='All saved physics substeps; robot collision hulls and measured gripper configuration plus measured actual bottle pose, source collider geometry. Geometry-only counterfactual when layout differs; never physically replayed.')
 a.report.parent.mkdir(parents=True,exist_ok=True);a.report.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k not in ['violations','phase_minima','source']},indent=2))
if __name__=='__main__':main()
