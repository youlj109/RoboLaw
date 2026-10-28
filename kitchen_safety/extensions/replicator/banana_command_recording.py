"""Read-only pre-simulation command recording and original actual-q bounds."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from .startup_recorder import array,sha
ROOT=Path(__file__).resolve().parents[3]
URDF=ROOT/'assets/curobo/droid/urdf/franka_robotiq_2f_85_zero.urdf'
URDF_SHA256='0aad7ada239f91360d57376247dc894e9c32a818957ee3099cbb6027ed18c2af'

class JointBounds:
    def __init__(self,names):
        if sha(URDF)!=URDF_SHA256:raise ValueError('Original audited URDF changed')
        limits={}
        for node in ET.parse(URDF).getroot().findall('joint'):
            l=node.find('limit')
            if node.get('name') in names and l is not None:limits[node.get('name')]=[float(l.get('lower')),float(l.get('upper'))]
        if set(limits)!=set(names):raise ValueError('Actual joint inventory lacks original bounds')
        self.names=list(names);self.bounds=np.asarray([limits[x] for x in names]);self.rows=[]
    def sample(self,row):
        q=np.asarray(row['q']);dq=np.asarray(row['dq'])
        if q.shape!=self.bounds[:,0].shape or dq.shape!=q.shape or not np.isfinite(q).all() or not np.isfinite(dq).all():raise ValueError('Malformed actual joints')
        excess=np.maximum(np.maximum(self.bounds[:,0]-q,q-self.bounds[:,1]),0.)
        report=dict(physics_step=int(row['physics_step']),phase=str(row['phase']),action_index=int(row['action_index']),maximum_excess_rad=float(excess.max()),allowance_rad=1e-5,passed=bool(excess.max()<=1e-5),violations=[dict(joint=self.names[i],actual_q_rad=float(q[i]),bounds_rad=self.bounds[i].tolist(),excess_rad=float(excess[i])) for i in np.flatnonzero(excess>1e-5)],measured_q_modified=False)
        self.rows.append(report);return report

class CommandRecorder:
    """Instance-local read wrapper. Calls original sim.step once; writes no state/target."""
    def __init__(self,env,robot,physics_recorder,actions):
        self.env,self.robot,self.recorder,self.actions=env,robot,physics_recorder,actions
        self.rows=[];self.pending=None;self.initial=None
        self.names=list(robot.joint_names);self.ids=[self.names.index(f'panda_joint{i}') for i in range(1,8)]+[self.names.index('finger_joint')]
        self.native=robot.root_view
        if list(self.native.shared_metatype.dof_names)!=self.names:raise ValueError('Native DOF target order differs from actualq')
        self.term=env.action_manager.get_term('gripper_action')
        if type(self.term).__name__!='JointPositionAction' or self.term.action_dim!=1 or self.term.cfg.scale!=1. or self.term.cfg.offset!=0. or self.term.cfg.use_default_offset or self.term.cfg.clip is not None:
            raise ValueError('Explicit identity gripper target mapping required')
        self.original_step=env.sim.step
        def step(*args,**kwargs):
            if self.pending is not None:raise ValueError('Unmatched pre-step command evidence')
            if not self.actions:raise ValueError('Physics step has no recorded command')
            before=int(env.sim.get_physics_step_count());action=self.actions[-1]
            if self.recorder.action_index!=len(self.actions)-1:raise ValueError('Command/action index differs')
            raw=array(env.action_manager.action)[0];processed=array(self.term.processed_actions)[0]
            staged=array(robot.data.joint_pos_target)[0];native=array(self.native.get_dof_position_targets())[0]
            expected=np.asarray(action['action'])
            checks=dict(raw_action_matches=np.array_equal(raw,expected),gripper_processed_identity=processed.shape==(1,) and processed[0]==expected[-1],staged_targets_match=np.array_equal(staged[self.ids],expected),native_targets_match=np.array_equal(native[self.ids],expected),finite_targets=all(np.isfinite(x).all() for x in [raw,processed,staged,native]))
            item=dict(before_physics_step=before,after_physics_step=None,phase=self.recorder.phase,action_index=self.recorder.action_index,raw_action_rad=raw,processed_gripper_target_rad=processed,staged_joint_targets_rad=staged,native_joint_targets_rad=native,checks={k:bool(v) for k,v in checks.items()},passed=bool(all(checks.values())))
            self.rows.append(item);self.pending=item
            if not item['passed']:raise ValueError('Actual pre-step command mapping differs from declared radian action')
            result=self.original_step(*args,**kwargs)
            item['after_physics_step']=int(env.sim.get_physics_step_count())
            if item['after_physics_step']!=before+1:raise ValueError('Command recorder expected exactly one actual physics step')
            return result
        env.sim.step=step
    def observe(self,row):
        if self.initial is None:
            self.initial=dict(physics_step=int(row['physics_step']),q=np.asarray(row['q']).copy(),dq=np.asarray(row['dq']).copy(),applied_interval=False)
            return
        if self.pending is None or self.pending['after_physics_step']!=row['physics_step']:raise ValueError('Actual physics sample has no matching applied native target')
        self.pending.update(actual_q=np.asarray(row['q']).copy(),actual_dq=np.asarray(row['dq']).copy());self.pending=None
    def close(self):self.env.sim.step=self.original_step
    def report(self):
        return dict(accepted=False,physical_trial_passed=False,initial=self.initial,rows=self.rows,joint_names=self.names,controlled_joint_ids=self.ids,unmatched_pending=self.pending is not None,passed=self.initial is not None and self.pending is None and all(x['passed'] and 'actual_q' in x for x in self.rows),scope='Read-only local sim.step wrapper observes targets after source scene.write_data_to_sim and before original simulation. Each post-step actualq/dq binds the applied interval; no state or target setter is called by this recorder.')
