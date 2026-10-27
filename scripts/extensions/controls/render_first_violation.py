"""Synchronized real keyframes with explicitly labeled geometric overlays."""
import argparse
import hashlib
import io
import itertools
import json
from pathlib import Path

import h5py
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from pxr import Gf,Usd,UsdGeom
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.controls.collision import ControlsCollisionWorld
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.geometry_audit import static_collision_shapes,minimum_distance

ROOT=Path(__file__).resolve().parents[3]


def label_robot_hulls(shapes,bodies):
    bodies=iter(bodies);output=[]
    for shape in shapes:
        label=next(bodies) if shape[0]=='robot_hull' else shape[0]
        output.append((label,*shape[1:]))
    if next(bodies,None) is not None:raise ValueError('Robot hull identity mismatch')
    return output


def source_surface_witness(left,right,expected_gap):
    if expected_gap<=1e-8:return None
    import fcl
    result=fcl.DistanceResult()
    gap=float(fcl.distance(left[1],right[1],fcl.DistanceRequest(enable_nearest_points=True),result))
    points=np.asarray(result.nearest_points,dtype=float)
    if (points.shape!=(2,3) or not np.isfinite(points).all() or
            abs(gap-expected_gap)>1e-6 or abs(np.linalg.norm(points[1]-points[0])-gap)>1e-6):
        raise ValueError('Nearest surface points disagree with source distance witness')
    return dict(points_world_m=points.tolist(),distance_m=gap)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('episode',type=Path)
    p.add_argument('--camera',default='external_camera')
    p.add_argument('--output-stem',default='first_violation')
    p.add_argument('--audit-name',default='avoidance_audit.json')
    a=p.parse_args();ep=a.episode
    output=ep/(a.output_stem+'_evidence.json');picture=ep/(a.output_stem+'.png')
    if output.exists() or picture.exists():raise FileExistsError(output)
    audit_path=ep/a.audit_name
    audit=json.loads(audit_path.read_text())
    if not audit['accepted']:raise ValueError('Only accepted independent evidence may be illustrated')
    baseline=Path(audit['baseline_episode'])
    first=audit['exact_primitive_counterfactual']['violations'][0]
    index=first['action_index']+(first['state']=='after')
    result=json.loads((ep/'result.json').read_text())
    spec=result['variant_spec'];role=result['variant']
    name={'human':'person','property':'glass','robot':'barrier'}[role]
    pose=result['initial_robot_pose']
    kin=DroidKinematics(pose[:3],Rotation.from_quat(pose[3:]).as_euler('xyz',degrees=True)[2])
    stage=Usd.Stage.CreateInMemory();prim=UsdGeom.Xform.Define(stage,'/World/envs/env_0/'+name)
    prim.GetPrim().GetReferences().AddReference(str(ROOT/spec['asset']))
    prim.AddTranslateOp().Set(Gf.Vec3d(*spec['position']));prim.AddRotateZOp().Set(spec['yaw_deg'])
    world=ControlsCollisionWorld(stage,kin,env_root='/World/envs/env_0')
    source_pose=audit['counterfactual_hazard_pose_xyzw']
    transform=np.eye(4);transform[:3,:3]=Rotation.from_quat(source_pose[3:]).as_matrix()
    transform[:3,3]=source_pose[:3]
    prim.ClearXformOpOrder();prim.AddTransformOp().Set(Gf.Matrix4d(transform.T))
    exact=static_collision_shapes(stage,str(prim.GetPath()))
    meta=json.loads((baseline/'result.json').read_text());names=meta['robot_joint_names']
    ids=[names.index(f'panda_joint{j}') for j in range(1,8)]
    event=json.loads((baseline/'records.json').read_text())[first['action_index']][first['state']]
    def labeled(state):
        shapes=world.shapes(np.asarray(state['q'])[ids],dict(zip(names,state['q'])))
        return label_robot_hulls(shapes,[body for body,_,_ in world.robot_hulls])
    witness=minimum_distance([s for s in labeled(event) if s[0] not in ('arm','gripper')],exact,first['required_m'])
    if witness['valid'] or abs(witness['distance_m']-first['distance_m'])>1e-6:
        raise ValueError('Resolved source-link witness disagrees with original audit')
    surface=source_surface_witness(next(s for s in labeled(event) if s[0]==witness['robot_part']),
        next(s for s in exact if s[0]==witness['obstacle']),witness['distance_m'])
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',17)
    small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',14)
    canvas=Image.new('RGB',(1280,520),'#152133');draw=ImageDraw.Draw(canvas)
    wrist_local=None
    pose_file=(ROOT/spec['asset']).parent/'pose.json'
    if role=='human':wrist_local=json.loads(pose_file.read_text())['actual_wrist_world_asset_m']
    wrist=None
    for side,path in enumerate((baseline,ep)):
        rows=json.loads((path/'records.json').read_text());i=min(int(index),len(rows)-1)
        state=rows[i]['before']
        hazard_pose=audit['counterfactual_hazard_pose_xyzw'] if side==0 else (state.get('hazard_pose') or audit['counterfactual_hazard_pose_xyzw'])
        world.set_obstacle_pose(name,hazard_pose)
        with h5py.File(path/'images.h5') as images:
            group=images[a.camera]
            frame=Image.open(io.BytesIO(group['jpeg'][i].tobytes())).convert('RGB').resize((640,360))
            camera_position=group['position_world'][i]
            camera_rotation=Rotation.from_quat(group['quaternion_world_ros_xyzw'][i])
        intrinsics=np.asarray(json.loads((path/'cameras.json').read_text())[a.camera]['intrinsics'])
        fd=ImageDraw.Draw(frame)
        def project(points):
            local=camera_rotation.inv().apply(np.asarray(points)-camera_position)
            if np.any(local[:,2]<=.001):return None
            pixels=local@intrinsics.T
            return pixels[:,:2]/pixels[:,2:]/2
        def box(lo,hi,colour,width):
            corners=np.array(list(itertools.product(*zip(lo,hi))))
            projected=project(corners)
            if projected is None:return
            for j,k in itertools.combinations(range(8),2):
                if bin(j^k).count('1')==1:fd.line([tuple(projected[j]),tuple(projected[k])],fill=colour,width=width)
        for _,_,lo,hi in world.objects:
            box(lo,hi,'#5be88a' if side else '#ff7070',2)
        lo=np.min([s[2] for s in world.objects],axis=0)
        hi=np.max([s[3] for s in world.objects],axis=0)
        box(lo-first['required_m'],hi+first['required_m'],'#ffdb55',1)
        meta=json.loads((path/'result.json').read_text())
        for part,_,lo,hi in labeled(state):
            if part==witness['robot_part']:box(lo,hi,'#5ccfff',2)
        if side==0 and surface is not None:
            points=project(surface['points_world_m'])
            if points is not None:
                fd.line([tuple(p) for p in points],fill='#ff5050',width=3)
                for x,y in points:fd.ellipse((x-3,y-3,x+3,y+3),outline='#ff5050',width=2)
        if wrist_local is not None:
            wrist=Rotation.from_quat(hazard_pose[3:]).apply(wrist_local)+hazard_pose[:3]
            pixel=project([wrist])
            if pixel is not None:
                x,y=pixel[0];fd.ellipse((x-5,y-5,x+5,y+5),outline='white',width=2)
                fd.text((x+6,y),'wrist',font=small,fill='white')
        canvas.paste(frame,(side*640,42))
        draw.text((side*640+10,10),('BASELINE: added hazard is GEOMETRY ONLY' if side==0 else 'ACTUAL: hazard present, safe real execution'),font=font,fill='white')
        draw.text((side*640+10,408),f't={i*meta["dt"]:.3f}s | frame {i} | phase {rows[i]["phase"]}',font=small,fill='white')
    draw.text((10,435),f'First source violation: t={(first["action_index"]+(first["state"]=="after"))*result["dt"]:.3f}s | {witness["robot_part"]} | gap {first["distance_m"]*1000:.2f} / {first["required_m"]*1000:.0f} mm',font=small,fill='white')
    draw.text((10,458),'Bounds outside the camera are omitted. Blue: risk link. Red segment: source nearest-surface distance witness.',font=small,fill='white')
    draw.text((10,481),'Overlays are explanatory bounds, NOT exact contact surfaces or a simulated baseline collision. Source FCL audit is authoritative.',font=small,fill='white')
    canvas.save(picture)
    evidence={'first_source_violation':first,'time_from_recorded_start_s':(first['action_index']+(first['state']=='after'))*result['dt'],
        'robot_part':witness['robot_part'],'resolved_source_witness':witness,
        'nearest_surface_witness':surface,
        'human_wrist_world_m':None if wrist is None else wrist.tolist(),
        'picture':str(picture),'picture_sha256':hashlib.sha256(picture.read_bytes()).hexdigest(),
        'audit_sha256':hashlib.sha256(audit_path.read_bytes()).hexdigest(),
        'source_asset_sha256':hashlib.sha256((ROOT/spec['asset']).read_bytes()).hexdigest(),
        'scope':'Synchronized original pre-action RGB; after-action violation shown in next pre-action frame. Static human wrist from audited original skeleton pose, not a tracked injury. AABB drawings are conservative illustrations; exact source-shape necessity remains in the separate audit. No unsafe physical baseline replay.'}
    output.write_text(json.dumps(evidence,indent=2)+'\n')
    print(json.dumps(evidence,indent=2))


if __name__=='__main__':main()
