"""Frozen source meshes plus explicit kinematic PhysX limb proxies.

The native renderer keeps original materials. The proxies are NOT the source
triangles: source-mesh FCL checks are recorded separately at every physics step.
"""
import hashlib,json,math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,UsdPhysics,Gf,Vt
ROOT=Path(__file__).resolve().parents[4]
PARTS={'pelvis':.12,'spine_01':.14,'spine_03':.14,'head':.10,'upperarm_l':.045,'lowerarm_l':.04,'hand_l':.035,'upperarm_r':.045,'lowerarm_r':.04,'hand_r':.035,'thigh_l':.07,'calf_l':.055,'thigh_r':.07,'calf_r':.055}

def replace_cached_mesh(checker,name,mesh_id,pose):
    """Update an existing cuRobo 0.7.7 mesh without its broken Pose assignment.

    The installed update_mesh_from_warp assigns a Pose object to an eight-wide
    tensor row. Its public update_mesh_pose correctly writes the seven pose
    components and preserves the padding component. Only the mesh-ID slot has
    no working public replacement API in this version.
    """
    index=checker.get_mesh_idx(name,0)
    tensors=checker._mesh_tensor_list
    if (checker._env_mesh_names[0].count(name)!=1 or index<0 or
        index>=int(checker._env_n_mesh[0]) or
        tensors[1].shape[-1]!=8 or index>=tensors[0].shape[1]):
        raise RuntimeError('Unexpected existing cuRobo mesh cache slot: '+name)
    checker.update_mesh_pose(name=name,w_obj_pose=pose,env_idx=0)
    tensors[0][0,index]=int(mesh_id)
    checker.enable_mesh(True,name=name,env_idx=0)

def load_spec(path):
    spec=json.loads(Path(path).read_text());asset=ROOT/spec['human_asset']
    actual=hashlib.sha256(asset.read_bytes()).hexdigest()
    if actual!=spec['human_asset_sha256']:raise ValueError('Frozen human USD hash mismatch')
    return spec

class PersonAsset:
    def __init__(self,spec):
        self.spec=spec;self.stage=Usd.Stage.Open(str(ROOT/spec['human_asset']))
        self.anim=json.loads((ROOT/spec['human_animation']).read_text())
        self.rotation=Rotation.from_euler('z',spec['human_yaw_deg'],degrees=True).as_matrix()
        self.position=np.array(spec['human_position_m']);self.frame=None;self.shapes=[];self.meshes={}
        self.root_path='/World/envs/env_0/person'
        self.animated=[str(a.GetPath()) for p in self.stage.Traverse() for a in p.GetAttributes() if a.GetNumTimeSamples()]
    def bone(self,name,frame=0):
        a=np.asarray(self.anim['samples'][int(frame)]['bone_head_tail_asset_m'][name]);return a@self.rotation.T+self.position
    def proxy(self,name,frame=0):
        a,b=self.bone(name,frame);direction=b-a;length=float(np.linalg.norm(direction));z=direction/length
        x=np.cross([1,0,0] if abs(z[0])<.9 else [0,1,0],z);x/=np.linalg.norm(x);y=np.cross(z,x)
        quat=Rotation.from_matrix(np.column_stack((x,y,z))).as_quat()
        return np.r_[(a+b)/2,quat],length
    def configure_scene(self,cfg):
        import isaaclab.sim as sim
        from isaaclab.assets import AssetBaseCfg,RigidObjectCfg
        q=Rotation.from_matrix(self.rotation).as_quat()
        cfg.scene.hard_person=AssetBaseCfg(prim_path='{ENV_REGEX_NS}/person',spawn=sim.UsdFileCfg(usd_path=str(ROOT/self.spec['human_asset'])),init_state=AssetBaseCfg.InitialStateCfg(pos=tuple(self.position),rot=tuple(q)))
        for name,radius in PARTS.items():
            pose,length=self.proxy(name)
            setattr(cfg.scene,'hard_person_proxy_'+name,RigidObjectCfg(prim_path='{ENV_REGEX_NS}/hard_person_proxies/'+name,
                spawn=sim.CapsuleCfg(radius=radius,height=length,visible=False,rigid_props=sim.RigidBodyPropertiesCfg(kinematic_enabled=True,disable_gravity=True),collision_props=sim.CollisionPropertiesCfg(collision_enabled=True),mass_props=sim.MassPropertiesCfg(mass=.1)),
                init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(pose[:3]),rot=tuple(pose[3:]))))
    def apply(self,stage,scene,frame):
        frame=int(max(0,min(167,frame)))
        if frame==self.frame:return False
        for path in self.animated:
            src=self.stage.GetAttributeAtPath(path);dst=stage.GetAttributeAtPath(path.replace('/Person',self.root_path,1))
            if not dst:raise RuntimeError('Missing native human attribute '+path)
            dst.Set(src.Get(Usd.TimeCode(frame)))
        # Source meshes are visual only in PhysX. Dynamic collision response is
        # supplied by labelled kinematic proxies, never assumed from USD points.
        for p in Usd.PrimRange(stage.GetPrimAtPath(self.root_path)):
            if p.HasAPI(UsdPhysics.CollisionAPI):UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
        if scene is not None:
            import torch
            for name in PARTS:
                pose,_=self.proxy(name,frame);obj=scene['hard_person_proxy_'+name]
                obj.write_root_pose_to_sim_index(root_pose=torch.tensor(pose[None],dtype=torch.float32,device=obj.device))
        self.frame=frame;self._source_shapes(frame)
        return True
    def _source_shapes(self,frame):
        import fcl
        self.shapes=[];self.meshes={};cache=UsdGeom.XformCache(Usd.TimeCode(frame))
        for p in self.stage.Traverse():
            if not p.IsA(UsdGeom.Mesh):continue
            mesh=UsdGeom.Mesh(p);pts=np.asarray(mesh.GetPointsAttr().Get(frame),dtype=float);mat=np.asarray(cache.GetLocalToWorldTransform(p)).T
            pts=(pts@mat[:3,:3].T+mat[:3,3])@self.rotation.T+self.position
            indices=np.asarray(mesh.GetFaceVertexIndicesAttr().Get());faces=[];offset=0
            for count in mesh.GetFaceVertexCountsAttr().Get():
                poly=indices[offset:offset+count];offset+=count;faces.extend([[poly[0],poly[i],poly[i+1]] for i in range(1,count-1)])
            faces=np.asarray(faces,dtype=np.int32);model=fcl.BVHModel();model.beginModel(len(pts),len(faces));model.addSubModel(pts,faces);model.endModel()
            name=str(p.GetPath()).replace('/Person',self.root_path,1);self.meshes[name]=(pts,faces);self.shapes.append((name,fcl.CollisionObject(model),pts.min(0),pts.max(0)))
    def bind_world(self,world):
        retained=[(r,world._object_reference[i]) for i,r in enumerate(world.objects) if '/hard_person_proxies/' not in r[0] and not r[0].startswith(self.root_path+'/')]
        world.objects=[r for r,_ in retained]+self.shapes;world._object_reference=[r for _,r in retained]
        for name,obj,lo,hi in self.shapes:world._object_reference.append((np.eye(3),np.zeros(3),lo.copy(),hi.copy(),self.meshes[name]))
        for name in list(world.meshes):
            if name.startswith(self.root_path+'/'):del world.meshes[name]
        world.meshes.update(self.meshes);world.low=np.array([r[2] for r in world.objects]);world.high=np.array([r[3] for r in world.objects]);world._pair_selection_key=None;world._pair_selection_cache={}
    def sync_planner(self,planner):
        import warp as wp
        import torch
        from curobo.types.math import Pose
        previous=getattr(self,'_warp_meshes',[]);new_meshes=[];identity=Pose.from_list([0.,0.,0.,1.,0.,0.,0.],planner.args)
        # Finish any previous query before replacing the pointers it consumed.
        torch.cuda.synchronize(planner.args.device);wp.synchronize()
        for name,(vertices,faces) in self.meshes.items():
            points=vertices@planner.T_B_W[:3,:3].T+planner.T_B_W[:3,3]
            mesh=wp.Mesh(points=wp.array(points.astype(np.float32),dtype=wp.vec3,device='cuda:0'),indices=wp.array(faces.flatten(),dtype=wp.int32,device='cuda:0'))
            new_meshes.append((name,mesh))
        # Retain old AND new meshes if an update fails partway through; no live
        # collision-cache pointer may outlast its owning Warp mesh allocation.
        self._warp_meshes=previous+[mesh for _,mesh in new_meshes]
        for name,mesh in new_meshes:
            replace_cached_mesh(planner.motion_gen.world_coll_checker,name,int(mesh.id),identity)
        torch.cuda.synchronize(planner.args.device);wp.synchronize()
        self._warp_meshes=[mesh for _,mesh in new_meshes]
