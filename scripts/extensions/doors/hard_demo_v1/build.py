#!/usr/bin/env python3
"""Scripted original open-fridge scene demo; no policy, physics, or robot motion."""
from __future__ import annotations
import json,math,os,hashlib,sys
from pathlib import Path
import numpy as np
from pxr import Gf,Sdf,Usd,UsdGeom,UsdLux,Vt,UsdPhysics
ROOT=Path(__file__).resolve().parents[4]
OUT=ROOT/'assets/extensions/doors/hard_demo_v1'
REPORT=ROOT/'reports/doors_hard_demo_v1'
DOOR='/World/kitchen/Asset/fridge_main_group/Refrigerator032_fridge_door'
JOINT=DOOR+'/fridge_door_joint'
HUMAN='/World/human/Asset'
FPS=12;FRAMES=120
SCOPE='scripted scene demo; no robot policy/physics acceptance'

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,value):Path(p).parent.mkdir(parents=True,exist_ok=True);Path(p).write_text(json.dumps(value,indent=2,ensure_ascii=False)+'\n')
def dependency_hashes():
    paths=[ROOT/'assets/kitchen/scene.usd',ROOT/'assets/people/mpfb_corridor/posed.blend',ROOT/'configs/extensions/doors/fixed_source_layout.json',OUT/'robot_on_stand.usda']
    for pose in ('human_gap','human_clear'):
        paths.extend(OUT/pose/name for name in ('person.usdc','collidable.usda','pose.json'))
    return {str(p.relative_to(ROOT)):sha(p) for p in paths}
def T(position,quaternion):
    q=Gf.Quatd(float(quaternion[3]),Gf.Vec3d(*quaternion[:3]));m=Gf.Matrix4d(Gf.Rotation(q),Gf.Vec3d(*position));return np.asarray(m).T

def phase(frame):
    if frame<=18:return 'initial_opening',7.*frame/18.,0.
    if frame<=30:return 'human_enters',7.+5.*(frame-18)/12.,(frame-18)/12.
    if frame<=54:return 'stop_and_wait',12.,1.
    if frame<=66:return 'human_withdraws',12.,1.-(frame-54)/12.
    if frame<=108:return 'resume_opening',12.+13.*(frame-66)/42.,0.
    return 'opened_preview',25.,0.

def main():
    OUT.mkdir(parents=True,exist_ok=True);REPORT.mkdir(parents=True,exist_ok=True)
    scene=OUT/'scene.usda'
    if scene.exists():scene.unlink()
    s=Usd.Stage.CreateNew(str(scene));s.SetStartTimeCode(0);s.SetEndTimeCode(FRAMES-1);s.SetTimeCodesPerSecond(FPS);s.SetFramesPerSecond(FPS)
    s.SetDefaultPrim(UsdGeom.Xform.Define(s,'/World').GetPrim());UsdGeom.SetStageUpAxis(s,'Z');UsdGeom.SetStageMetersPerUnit(s,1.)
    def ref(name,path,position=(0,0,0),yaw=0):
        x=UsdGeom.Xform.Define(s,'/World/'+name);x.AddTranslateOp().Set(Gf.Vec3d(*position));x.AddRotateZOp().Set(yaw)
        s.DefinePrim('/World/'+name+'/Asset').GetReferences().AddReference(os.path.relpath(ROOT/path,OUT))
    ref('kitchen','assets/kitchen/scene.usd')
    fixed_path=ROOT/'configs/extensions/doors/fixed_source_layout.json';fixed=json.loads(fixed_path.read_text())
    ref('robot','assets/extensions/doors/hard_demo_v1/robot_on_stand.usda',fixed['robot_asset_pose']['position'],fixed['expected_robot_yaw_deg'])
    human_position=[3.50,-1.15,.0025];human_yaw=220.
    ref('human','assets/extensions/doors/hard_demo_v1/human_clear/collidable.usda',human_position,human_yaw)
    cache=UsdGeom.XformCache();door=s.GetPrimAtPath(DOOR);j=UsdPhysics.RevoluteJoint(s.GetPrimAtPath(JOINT));body0=s.GetPrimAtPath(j.GetBody0Rel().GetTargets()[0]);bodyT=np.asarray(cache.GetLocalToWorldTransform(body0)).T
    lq=j.GetLocalRot0Attr().Get();local=T(j.GetLocalPos0Attr().Get(),[*lq.GetImaginary(),lq.GetReal()]);hingeT=bodyT@local
    hinge=hingeT[:3,3];axis=hingeT[:3,'XYZ'.index(str(j.GetAxisAttr().Get()))];axis/=np.linalg.norm(axis)
    initial=np.asarray(cache.GetLocalToWorldTransform(door)).T;parent=np.asarray(cache.GetLocalToWorldTransform(door.GetParent())).T
    xform=UsdGeom.Xformable(door);xform.ClearXformOpOrder();op=xform.AddTransformOp(opSuffix='scriptedDoor')
    def matrix(angle):
        a=math.radians(angle);cross=np.array([[0,-axis[2],axis[1]],[axis[2],0,-axis[0]],[-axis[1],axis[0],0]])
        rot=np.eye(3)+math.sin(a)*cross+(1-math.cos(a))*(cross@cross)
        delta=np.eye(4);delta[:3,:3]=rot;delta[:3,3]=hinge-rot@hinge
        return Gf.Matrix4d((np.linalg.inv(parent)@delta@initial).T.tolist())
    op.Set(matrix(0))
    for frame in range(FRAMES):op.Set(matrix(phase(frame)[1]),frame)
    # Exact same evaluated body topology from two poses of the original MPFB rig.
    clear=Usd.Stage.Open(str(OUT/'human_clear/person.usdc'));gap=Usd.Stage.Open(str(OUT/'human_gap/person.usdc'))
    animated=[];static=[]
    for prim in clear.Traverse():
        if not prim.IsA(UsdGeom.Mesh):continue
        path=str(prim.GetPath());a=UsdGeom.Mesh(prim);b=UsdGeom.Mesh(gap.GetPrimAtPath(path))
        p0=np.asarray(a.GetPointsAttr().Get());p1=np.asarray(b.GetPointsAttr().Get())
        assert p0.shape==p1.shape and list(a.GetFaceVertexIndicesAttr().Get())==list(b.GetFaceVertexIndicesAttr().Get()),'Human pose topology changed'
        target=UsdGeom.Mesh(s.GetPrimAtPath(HUMAN+path.removeprefix('/Person')))
        if np.max(np.abs(p1-p0))<1e-6:static.append(path);continue
        # Remove authored normals locally so interpolated skin uses geometry normals.
        target.GetNormalsAttr().Block()
        for frame,weight in [(0,0),(18,0),(30,1),(54,1),(66,0),(119,0)]:
            target.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(((1-weight)*p0+weight*p1).astype(np.float32)),frame)
        extent=np.stack([np.minimum(p0.min(0),p1.min(0)),np.maximum(p0.max(0),p1.max(0))]);target.GetExtentAttr().Set(Vt.Vec3fArray.FromNumpy(extent.astype(np.float32)))
        animated.append(path)
    def camera(name,eye,target,focal=40.):
        cam=UsdGeom.Camera.Define(s,'/World/hard_demo/'+name);cam.CreateFocalLengthAttr(focal);cam.CreateClippingRangeAttr(Gf.Vec2f(.01,100.));cam.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse())
    camera('camera',(2.55,-3.25,2.15),(4.25,-.83,.99),37.)
    camera('detail_camera',(3.1,-2.2,1.9),(4.45,-1.0,1.15),45.)
    UsdLux.DomeLight.Define(s,'/World/sky').CreateIntensityAttr(700.)
    UsdLux.DistantLight.Define(s,'/World/sun').CreateIntensityAttr(1500.)
    s.GetRootLayer().customLayerData={'scope':SCOPE,'original_task_id':'kitchen_bench_lightwheel_open_fridge','robot_stationary':True,'not_a_robot_demonstration':True}
    s.GetRootLayer().Save();assert not s.GetCompositionErrors(),s.GetCompositionErrors()
    timeline={'fps':FPS,'frames':FRAMES,'duration_seconds':10.,'scope':SCOPE,'robot_stationary':True,'camera_prim':'/World/hard_demo/camera','phases':[
        {'start_seconds':0.,'end_seconds':1.5,'label_zh':'打开冰箱门（脚本演示）','phase':'initial_opening'},
        {'start_seconds':1.5,'end_seconds':2.5,'label_zh':'人手进入将来的开门区域','phase':'human_enters'},
        {'start_seconds':2.5,'end_seconds':4.5,'label_zh':'门暂停，等待人手离开','phase':'stop_and_wait'},
        {'start_seconds':4.5,'end_seconds':5.5,'label_zh':'人手撤回，门保持暂停','phase':'human_withdraws'},
        {'start_seconds':5.5,'end_seconds':9.,'label_zh':'区域清空，继续打开','phase':'resume_opening'},
        {'start_seconds':9.,'end_seconds':10.,'label_zh':'场景机制演示结束','phase':'opened_preview'}],
        'samples':[{'frame':f,'time_seconds':f/FPS,'phase':phase(f)[0],'door_angle_deg':phase(f)[1],'human_pose_weight':phase(f)[2]} for f in range(FRAMES)]}
    write(REPORT/'timeline.json',timeline)
    task=ROOT/'third_party/IsaacLab-Arena/isaaclab_arena_environments/kitchen_bench/kitchen_bench_lightwheel_open_fridge.yaml'
    original_robot=ROOT/'runtime/home/.cache/isaaclab_arena/usd/robot_on_stand/doors_droid_on_stand_0.8_1.08x0.91032.usd'
    source={'scope':SCOPE,'task_id':'kitchen_bench_lightwheel_open_fridge','original_task_kind':'OpenDoorTask','source_yaml':str(task.relative_to(ROOT)),'source_yaml_sha256':sha(task),'original_openable_joint_name':'fridge_door_joint','original_openness_threshold':.2,'original_reset_openness':0.,'native_predicate_evaluated':False,'task_success_claim':False,'human_animation':'Same-topology point interpolation of complete original MPFB skinned meshes between two natural bone poses; feet, root, left arm, torso unchanged. No detached hands.','animated_human_meshes':animated,'unchanged_human_meshes':static,'human_position_m':human_position,'human_yaw_deg':human_yaw,'door_prim':DOOR,'joint_prim':JOINT,'original_joint_axis_token':str(j.GetAxisAttr().Get()),'hinge_world_m':hinge.tolist(),'axis_world_unit':axis.tolist(),'joint_limit_degrees':[j.GetLowerLimitAttr().Get(),j.GetUpperLimitAttr().Get()],'robot_layout':fixed,'robot_source_wrapper':str(original_robot.relative_to(ROOT)),'robot_source_wrapper_sha256':sha(original_robot),'robot_portable_wrapper':str((OUT/'robot_on_stand.usda').relative_to(ROOT)),'robot_portable_change':'Original wrapper transform values retained, only absolute /workspace asset paths rewritten to relative paths.','scene':str(scene.relative_to(ROOT)),'scene_sha256':sha(scene),'timeline':str((REPORT/'timeline.json').relative_to(ROOT)),'no_simulation_started':True,'no_robot_trajectory':True,'direct_door_transform_animation':True,'notes':['Door transform animation is explicitly scripted visual mechanism illustration, not native task completion or a robot policy result.','Mesh interpolation demonstrates connected skin motion; this is not a measured real human trajectory.']}
    source['direct_source_dependency_sha256']=dependency_hashes()
    source['timeline_sha256']=sha(REPORT/'timeline.json')
    write(REPORT/'source.json',source);write(ROOT/'configs/extensions/doors/hard_demo_v1/demo.json',{'scene':source['scene'],'source':str((REPORT/'source.json').relative_to(ROOT)),'timeline':source['timeline'],'scope':SCOPE})
    print(json.dumps({'scene':source['scene'],'animated_meshes':animated,'hinge':hinge.tolist(),'axis':axis.tolist()},indent=2))
if __name__=='__main__':main()
