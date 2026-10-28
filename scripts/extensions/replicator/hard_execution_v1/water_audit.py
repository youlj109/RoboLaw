"""CPU independent endpoint/approach geometry audit; no Kit/CUDA."""
import argparse
import json
from pathlib import Path
import numpy as np
from pxr import Gf, Usd, UsdGeom
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.replicator.hard_execution_v1.water_mechanism import ROOT, CONFIG, PIVOT, grasp_pose
from kitchen_safety.extensions.replicator.hard_execution_v1.water_control import WaterWorld
from kitchen_safety.extensions.replicator.task_motion import pose_matrix


def build_world():
    config=json.loads(CONFIG.read_text())
    layout=json.loads((ROOT/config["accepted_layout"]["path"]).read_text())
    stage=Usd.Stage.CreateInMemory()
    root="/World/envs/env_0"
    UsdGeom.Xform.Define(stage,root)
    def ref(name,path,position=None,yaw=0.):
        xf=UsdGeom.Xform.Define(stage,root+"/"+name)
        xf.GetPrim().GetReferences().AddReference(str(ROOT/path))
        if position is not None:
            xf.ClearXformOpOrder()
            xf.AddTranslateOp().Set(Gf.Vec3d(*position))
            xf.AddRotateZOp().Set(yaw)
    ref("kitchen",layout["kitchen_usd"])
    item=layout["objects"]["bowl"]
    ref("bowl",item["usd_path"],item["position"],item["yaw_deg"])
    obstacle=layout["obstacle"]
    ref("water_crossbar",obstacle["usd_path"],obstacle["position"],obstacle["yaw_deg"])
    ref("water_valve",config["valve_asset"],PIVOT,180.)
    kin=DroidKinematics(np.asarray(layout["robot_position"])+[0.,0.,layout["stand_height"]],layout["robot_yaw_deg"])
    world=WaterWorld(stage,kin,payload_usd=ROOT/item["usd_path"],env_root=root,
        clearances={},execution_clearances={},default_clearance=.002)
    pose=np.r_[item["position"],Rotation.from_euler("z",item["yaw_deg"],degrees=True).as_quat()]
    world.payload_pose=pose_matrix(pose)
    world.gripper_joint_positions,world.gripper_joint_position=np.zeros(6),0.
    return world,kin,stage


def audit():
    world,kin,stage=build_world()
    q=np.radians([0.,-36.,0.,-144.,0.,108.,0.])
    result=dict(scope="CPU full FCL endpoints and local approach; no CUDA self-collision, native physics or success claim",
        initial=dict(passed=world.valid(q),violation=world.last_clearance),targets=[],finger_centers={})
    for grip in (0.,.6):
        links=world._gripper_links(dict(zip(world.GRIPPER_JOINT_NAMES,np.array([1,1,-1,1,-1,-1])*grip)))
        points={}
        for name,model,verts in world.gripper_models:
            if name.endswith("inner_finger"):
                matrix=links[name]
                vertices=verts@matrix[:3,:3].T+matrix[:3,3]
                points[name.rsplit("/",1)[-1]]=vertices.mean(0).tolist()
        result["finger_centers"][str(grip)]=points
    for phase,height in (("valve_pregrasp",.14),("valve_approach",0.)):
        world.set_phase(phase)
        target=grasp_pose(55.,height)
        row=dict(phase=phase,target=target)
        try:
            q=kin.ik(target,q)
            row.update(ik=True,q=q,passed=world.valid(q),violation=world.last_clearance)
        except Exception as error:
            row.update(ik=False,passed=False,error=repr(error))
        result["targets"].append(row)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    result=audit()
    args.output.write_text(json.dumps(result,indent=2,default=lambda v:v.tolist())+"\n")
    print(json.dumps(dict(output=str(args.output),initial=result["initial"],targets=[
        {k:v for k,v in r.items() if k not in ("target","q")} for r in result["targets"]])))

if __name__=="__main__":
    main()
