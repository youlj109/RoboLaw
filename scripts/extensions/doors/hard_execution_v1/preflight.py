"""CPU-only frozen asset, event guard and native integration contract checks."""
import argparse,ast,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4];sys.path.insert(0,str(ROOT))
from kitchen_safety.extensions.doors.hard_execution_v1.person import PersonAsset,load_spec,PARTS,replace_cached_mesh
from kitchen_safety.extensions.doors.hard_execution_v1.events import IntrusionEvents
from kitchen_safety.extensions.doors.hard_execution_v1.completion import OpeningCompletion
from pxr import Usd,UsdGeom,Gf
import numpy as np
from scipy.spatial.transform import Rotation
p=argparse.ArgumentParser();p.add_argument('--config',default=str(ROOT/'configs/extensions/doors/hard_execution_v1/open_fridge.json'));p.add_argument('--output');args=p.parse_args()
spec=load_spec(args.config);actor=PersonAsset(spec);stage=Usd.Stage.CreateInMemory()
x=UsdGeom.Xform.Define(stage,actor.root_path);x.AddTranslateOp().Set(Gf.Vec3d(*spec['human_position_m']));x.AddRotateZOp().Set(spec['human_yaw_deg']);x.GetPrim().GetReferences().AddReference(str(ROOT/spec['human_asset']))
geometry=[]
for frame in (0,36,48,66,90,108,167):
 actor.apply(stage,None,frame);assert len(actor.shapes)==6
 for name in PARTS:
  pose,length=actor.proxy(name,frame);a,b=actor.bone(name,frame)
  assert np.allclose(Rotation.from_quat(pose[3:]).apply([0,0,1]),(b-a)/length,atol=1e-7),'xyzw proxy orientation mismatch'
 geometry.append({'frame':frame,'source_mesh_count':len(actor.shapes),'lowest_source_mesh_z_m':float(min(r[2][2] for r in actor.shapes))})
# A contact count alone must not trigger: caller passes true only for measured
# nonzero per-finger load exceeding the configured N threshold.
e=IntrusionEvents(spec)
for i in range(10):e.advance(1/15,door_q=.2,door_dq=0,arm_dq_max=0,both_fingers=False,at=i/15)
assert not e.started
for i in range(250):e.advance(1/15,door_q=.21,door_dq=0,arm_dq_max=0,both_fingers=True,at=i/15)
assert e.completed and e.stopped and not e.hold
names=[v['event'] for v in e.events]
assert names.index('robot_and_door_measured_stopped')<names.index('hand_fully_extended')<names.index('hand_withdrawn')<names.index('resume_after_person_clear')
bad=IntrusionEvents(spec);timed_out=False
try:
 for i in range(100):bad.advance(1/15,door_q=.21,door_dq=.3,arm_dq_max=1,both_fingers=True,at=i/15)
except RuntimeError:timed_out=True
assert timed_out and bad.frame<=48,'Unsettled robot must never receive a reaching human'
# A commanded angle is never used by this gate: only measured opening/speed,
# the unmodified native predicate and a fully withdrawn person are accepted.
completion=OpeningCompletion(spec)
sample={'door_q':completion.required_rad+.001,'door_dq':0.,'native_open':True,'events_complete':True}
for _ in range(completion.hold_actions-1):assert not completion.observe(**sample)
assert completion.observe(**sample)
assert not completion.observe(**(sample|{'door_q':completion.required_rad-.001}))
assert not completion.observe(**(sample|{'door_dq':completion.speed_limit+.001}))
assert not completion.observe(**(sample|{'native_open':False}))
assert not completion.observe(**(sample|{'events_complete':False}))
# Reproduce the installed eight-wide pose-cache contract without starting CUDA.
# The adapter must change only the requested ID, seven pose components and enable
# bit; the eighth component and another mesh's slot must remain untouched.
class CacheFixture:
 def __init__(self):
  self._env_mesh_names=[['other','person']];self._env_n_mesh=np.array([2])
  self._mesh_tensor_list=[np.array([[11,22]],dtype=np.int64),np.arange(16,dtype=float).reshape(1,2,8),np.zeros((1,2),dtype=np.uint8)]
 def get_mesh_idx(self,name,env_idx):return self._env_mesh_names[env_idx].index(name)
 def update_mesh_pose(self,*,name,w_obj_pose,env_idx):self._mesh_tensor_list[1][env_idx,self.get_mesh_idx(name,env_idx),:7]=w_obj_pose
 def enable_mesh(self,enabled,*,name,env_idx):self._mesh_tensor_list[2][env_idx,self.get_mesh_idx(name,env_idx)]=enabled
fixture=CacheFixture();padding=fixture._mesh_tensor_list[1][0,1,7];other=fixture._mesh_tensor_list[1][0,0].copy()
replace_cached_mesh(fixture,'person',123456,[0,0,0,1,0,0,0])
assert fixture._mesh_tensor_list[0].tolist()==[[11,123456]]
assert fixture._mesh_tensor_list[2].tolist()==[[0,1]]
assert fixture._mesh_tensor_list[1][0,1,7]==padding and np.array_equal(fixture._mesh_tensor_list[1][0,0],other)
entry=ROOT/'scripts/extensions/doors/hard_execution_v1/collect.py';text=entry.read_text();ast.parse(text)
assert 'scriptedDoor' not in text and 'write_joint_state' not in text and 'write_joint_position' not in text
assert "audit('physics_substep')" in text and "'accepted':False" in text
report={'scope':'CPU-only code/asset checks; no native execution or physics acceptance','passed':True,'gpu_started':False,'native_simulation_started':False,'asset_sha256':spec['human_asset_sha256'],'keyframes':geometry,'event_order':names,'loaded_contact_required':True,'unsettled_reach_blocked':timed_out,'native_pose_quaternions':'xyzw','door_execution_state_writes_in_new_entry':False,'mesh_cache_adapter_preserves_padding_and_other_slots':True,'measured_opening_hold_gate_fixture_passed':True,'configured_required_opening_rad':completion.required_rad,'known_limits':['Internal native-reset substeps are not sampled; reset return and every subsequent warmup substep are recorded.','Human PhysX capsules are coarse response proxies; separate source triangle audits remain authoritative.','Native planner collision/physics and success remain to be run and independently assessed.']}
if args.output:
 out=Path(args.output)
 if out.exists():raise SystemExit('Refusing to overwrite preflight report')
 out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
