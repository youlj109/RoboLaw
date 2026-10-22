"""Capture causal original evidence without online geometry or risk evaluation.

Frames never receive a fabricated risk grade. Immutable copied snapshots go to
a bounded writer process; overflow blocks visibly and worker failures abort.
Frozen collision surfaces permit later complete-mesh evaluation.
"""
from pathlib import Path
import hashlib
import json
import multiprocessing as mp
import os
import queue
import time
import traceback
import numpy as np
from kitchen_safety.risk_binding import ArenaRiskRuntime, numpy, capture_rigid_actor_motion
from kitchen_safety.risk_profile_loader import bind_profile,digest
from kitchen_safety.risk_observer import write_json,finite_json


def _json_default(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    raise TypeError(type(value).__name__)


def _writer(directory,incoming):
    root=Path(directory);counts={};h=hashlib.sha256();started=time.perf_counter()
    try:
        with (root/'raw_evidence.jsonl').open('xb',buffering=1024*1024) as stream:
            while True:
                row=incoming.get()
                if row is None:break
                try:text=json.dumps(row,ensure_ascii=False,allow_nan=False,separators=(',',':'),default=_json_default)
                except ValueError:
                    row=finite_json(row);row['nonfinite_evidence_present']=True
                    text=json.dumps(row,ensure_ascii=False,allow_nan=False,separators=(',',':'))
                encoded=(text+'\n').encode();stream.write(encoded);h.update(encoded)
                counts[row['kind']]=counts.get(row['kind'],0)+1
            stream.flush();os.fsync(stream.fileno())
        result=dict(passed=True,counts=counts,bytes=(root/'raw_evidence.jsonl').stat().st_size,
                    sha256=h.hexdigest(),writer_wall_s=time.perf_counter()-started,fsync_completed=True)
    except BaseException as exc:
        result=dict(passed=False,error=repr(exc),traceback=traceback.format_exc(),counts=counts)
    write_json(root/'raw_writer_result.json',result)
    if not result['passed']:raise RuntimeError(result['error'])


def settings_policy(settings):
    return settings.get('capture_workflow',{'mode':'raw-only','processing':'offline'})


class RawSink:
    def __init__(self,directory,settings,binding,identity,dt,action_dt):
        self.directory=Path(directory);self.binding=binding;self.identity=identity
        self.dt=dt;self.action_dt=action_dt;self.stride=round(action_dt/dt)
        self.steps=0;self.closed=False;self.pending_commands=None;self.fault_events='fault'
        self.enqueue_seconds=0.;self.block_seconds=0.
        context=mp.get_context('spawn');self.queue=context.Queue(maxsize=64)
        self.worker=context.Process(target=_writer,args=(str(self.directory),self.queue),daemon=True)
        self.worker.start()
        profile=bind_profile(settings,binding)
        write_json(self.directory/'safety_observer_binding.json',dict(
            schema_version='arena-raw-capture-binding-1',identity=identity,binding=binding,
            physics_dt_s=dt,action_dt_s=action_dt,decision_stride=self.stride,
            resolved_config=settings['config'],resolved_profile=profile,profile_sha256=digest(profile),
            adapter_version='arena-raw-capture-4-gdist-frozen',evaluation_status='pending_offline',
            capture_workflow=settings_policy(settings),
            implementation_files_sha256=settings.get('implementation_files_sha256',{}),
            mesh_backend_runtime=settings.get('mesh_backend_runtime',{})))

    def submit(self,row):
        if self.closed:raise RuntimeError('Raw evidence arrived after writer close')
        start=time.perf_counter()
        while True:
            if not self.worker.is_alive():raise RuntimeError('Raw writer died; evidence was not silently dropped')
            try:self.queue.put(row,timeout=.25);break
            except queue.Full:self.block_seconds+=.25
        self.enqueue_seconds+=time.perf_counter()-start

    def _line(self,stream,value):
        self.submit(dict(kind='fault_event',payload=value,receive_wall_ns=time.time_ns()))

    def record_commands(self,phase,index,nominal,submitted,commands,errors=()):
        self.pending_commands=dict(physics_step=self.steps,action_index=index,phase=phase,
            nominal_action=nominal,submitted_action=submitted,targets=commands,errors=list(errors),
            receive_wall_ns=time.time_ns())

    def finish(self,result=None):
        if self.closed:return
        started=time.perf_counter();self.submit(None)
        self.worker.join(timeout=60)
        if self.worker.is_alive():raise TimeoutError('Raw writer did not drain within 60 seconds')
        self.closed=True
        report=json.loads((self.directory/'raw_writer_result.json').read_text())
        if self.worker.exitcode!=0 or not report['passed']:raise RuntimeError(str(report))
        finished_queue=self.queue;self.queue=None
        finished_queue.close();finished_queue.join_thread()
        del finished_queue
        self.worker.close()
        write_json(self.directory/'raw_capture_summary.json',dict(
            schema_version='arena-raw-capture-1',identity=self.identity,physics_steps=self.steps,
            evaluation_status='pending_offline',risk_levels=None,
            physics_hz=1/self.dt,observation_hz=1/self.action_dt,
            raw_writer=report,enqueue_s=self.enqueue_seconds,queue_full_wait_s=self.block_seconds,
            final_writer_drain_s=time.perf_counter()-started,
            raw_semantics='Original state, actual command submissions, unaggregated contact events and measured action observations; no online mesh queries or risk grades'))


class RawCaptureRuntime(ArenaRiskRuntime):
    observer_class=RawSink

    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        for i,sink in enumerate(self.observers):
            folder=sink.directory/'raw_meshes';folder.mkdir()
            models={}
            if self.rs_geometries[i].errors:raise ValueError(str(self.rs_geometries[i].errors))
            for geometry in (self.pt_geometries[i],self.hs_geometries[i],self.rs_geometries[i].geometry):
                if geometry:
                    if geometry.errors:raise ValueError(str(geometry.errors))
                    for e in geometry.entries:models[e['fingerprint']]=e['model']
            for fingerprint,model in models.items():
                actual=hashlib.sha256(model.vertices.tobytes()+model.faces.tobytes()).hexdigest()
                if actual!=fingerprint:raise ValueError('Frozen mesh fingerprint mismatch')
                np.savez_compressed(folder/(fingerprint+'.npz'),vertices=model.vertices,faces=model.faces)
            actor_paths=set(self.latest[i].get('rigid_actor_states_world',{}))
            kitchen_paths={self.geometries[i].root+'/'+name for name in self.kitchen_state.metadata['link_paths']}
            for geometry in (self.pt_geometries[i],self.hs_geometries[i]):
                if geometry is None:continue
                for entry in geometry.entries:
                    if entry['group']=='robot' or not entry.get('actor'):continue
                    actor=entry['actor']
                    if not actor.startswith('/'):
                        actor=self.geometries[i].root+'/'+actor
                    if geometry is self.pt_geometries[i] and not entry.get('dynamic'):continue
                    if actor not in actor_paths|kitchen_paths:
                        raise ValueError('Dynamic collider has no saved pose channel: '+actor)
            write_json(sink.directory/'raw_capture_schema.json',dict(
                schema_version='arena-raw-capture-1',evaluation_status='pending_offline',
                mesh_count=len(models),mesh_files='raw_meshes/<fingerprint>.npz',
                frame='world_m for physics packets; environment_local for action observations; origin in binding',
                quaternion_order='xyzw',physics_hz=1/self.dt,observation_hz=1/sink.action_dt,
                observation_policy='Copied measured obs[t], before action[t]; source_physics_step=4*t-1',
                contacts='Original actor/collider identities, point positions, normals, impulse vectors and separations; aggregate later',
                frozen_kitchen_metadata=self.kitchen_state.metadata,
                limits_policy='Runtime effort and position limits retained per physics packet',
                rigid_actor_paths_120hz=sorted(actor_paths),
                rigid_actor_state_semantics='PhysX get_transforms actor xyz/xyzw and get_velocities COM linear/angular velocity in world frame, all registered scene rigid_objects at 120Hz',
                exact_motion_channels=dict(robot=['body_com_position_w','body_com_linear_velocity_w','body_com_angular_velocity_w'],
                    rigid_actors='rigid_actor_motion_world: COM world position, COM linear velocity and world angular velocity',
                    semantics='Measured current PhysX COM telemetry at 120Hz; surface velocities are derived offline; missing channels remain unknown'),
                workflow_policy=settings_policy(self.settings),
                mesh_semantics='Authored collision surfaces with original scale baked into vertices; body poses provide measured rigid transforms'))
            write_json(sink.directory/'safety_scene_placement.json',dict(evaluation_status='pending_offline',
                policy='No online placement verdict; recompute using frozen meshes and initial measured poses'))

    def refresh(self,initial=False):
        # Snapshot each rigid-body view once per step, preserving its actual
        # row/path mapping across environments. Kitchen articulation links are
        # separately saved in each measured 30 Hz render observation.
        actors,actor_motion=capture_rigid_actor_motion(self.scene)
        self.latest=[];self.distances=[]
        for i,sink in enumerate(self.observers):
            if not initial:self.sample_steps[i]+=1
            row=self.state(i)
            root=self.geometries[i].root+'/'
            row['rigid_actor_states_world']={path:state for path,state in actors.items() if path.startswith(root)}
            row['rigid_actor_motion_world']={path:state for path,state in actor_motion.items() if path.startswith(root)}
            self.latest.append(row);self.distances.append({})
            if initial:sink.submit(dict(kind='initial_state',physics_step=-1,time_s=0.,state=self.latest[-1],
                                        receive_wall_ns=time.time_ns()))

    def record_action_observations(self,states,active):
        for i in active:
            # The normal snapshot already owns detached arrays; take ownership
            # of new copies so asynchronous pickle can never observe mutation.
            observation={k:v.copy() if isinstance(v,np.ndarray) else v for k,v in states[i].items()}
            sink=self.observers[i];ctx=self.context[i]
            sink.submit(dict(kind='observation',action_index=ctx['action_index'],phase=ctx['phase'],
                physics_step=sink.steps,source_physics_step=sink.steps-1,time_s=sink.steps*self.dt,
                observation=observation,nominal_action=ctx['nominal_action'],receive_wall_ns=time.time_ns()))

    def before_submission(self):pass

    def after_physics(self):
        self.refresh()
        for i in self.active:
            sink=self.observers[i];ctx=self.context[i]
            contacts=self.monitor.pending.pop(i,[])
            coverage=bool(self.monitor.status['subscribed'] and not self.monitor.status['errors'] and not self.monitor.errors)
            sink.submit(dict(kind='physics',physics_step=sink.steps,action_index=ctx['action_index'],
                phase=ctx['phase'],start_time_s=sink.steps*self.dt,end_time_s=(sink.steps+1)*self.dt,
                state=self.latest[i],contacts=contacts,contact_coverage=coverage,
                contact_errors=list(self.monitor.errors),commands=sink.pending_commands,
                errors=list(self.errors[i]),receive_wall_ns=time.time_ns()))
            sink.pending_commands=None;sink.steps+=1;self.errors[i]=[]

    def flush_capture(self):
        for name,original in self.original_setters.items():setattr(self.command_sink,name,original)
        self.monitor.close()
        backend=self.pt_geometries[0].fcl
        stats=backend.batch_stats()
        for sink in self.observers:
            sink.finish()
            p=sink.directory/'raw_capture_summary.json';r=json.loads(p.read_text())
            r['online_geometry_scalar_calls']=backend.STATS['calls']
            r['online_geometry_batch_calls']=stats['batches']
            r['online_risk_evaluations']=0
            write_json(p,r)

    def close(self,results):
        for name,original in self.original_setters.items():setattr(self.command_sink,name,original)
        self.monitor.close();self.flush_capture()
