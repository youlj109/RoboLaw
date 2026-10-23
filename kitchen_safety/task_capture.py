"""Task-independent raw-only recorder attached to a native Arena environment.

Controllers retain env.step(action). Call recorder.step(action, phase, operation)
so the action's preceding observation is persisted before execution. Distances,
grasp/drop/fault-motion features and risk grades are exclusively offline.
"""
from copy import deepcopy
import hashlib
import json
import multiprocessing as mp
from pathlib import Path
import time
import numpy as np
from kitchen_safety.articulation_capture import SceneCapture, array
from kitchen_safety.task_contract import digest
from kitchen_safety.raw_capture import RawSink, _writer
from kitchen_safety.risk_binding import ContactMonitor, enable_contact_reports
from kitchen_safety.risk_observer import finite_json, write_json

ROOT=Path(__file__).resolve().parents[1]


def read_placement_validation(env):
    """Read the native reset solver's existing result; never rerun checks online."""
    try:
        from isaaclab_arena.relations.placement_events import get_reset_placement_results
        result=get_reset_placement_results(env).get(0)
        if result is None:
            return dict(available=False,passed=None,reason='no live reset placement result')
        failed=list(result.validation_results.get_failed_validation_check_names)
        return finite_json(dict(available=True,passed=bool(result.success),failed_checks=failed,
            attempts=int(result.attempts),final_loss=float(result.final_loss),
            source='native PlacementSampler.last_results[0] for this reset',
            fallback_used=not bool(result.success),task_success_inferred=False))
    except (ValueError,KeyError,AttributeError) as exc:
        return dict(available=False,passed=None,reason=repr(exc),task_success_inferred=False)


def freeze_variation_samples(env):
    """Persist the actual native sampler draws before terminal auto-reset.

    A seed alone is not a reconstruction source. Unknown sampler types remain
    explicit errors rather than being stringified as apparently usable data.
    """
    def serial(value):
        if value is None or isinstance(value,(str,bool,int,float)):return finite_json(value)
        if isinstance(value,dict):return {str(k):serial(v) for k,v in value.items()}
        if isinstance(value,(list,tuple)):return [serial(v) for v in value]
        if hasattr(value,'detach'):value=value.detach().cpu()
        if hasattr(value,'tolist'):return serial(value.tolist())
        from dataclasses import is_dataclass,asdict
        if is_dataclass(value):return serial(asdict(value))
        raise TypeError('Unsupported sampler evidence type: '+str(type(value)))
    recorder=getattr(env,'variation_recorder',None)
    if recorder is None:
        return dict(complete=False,missing=['native_variation_recorder'],samples={})
    episode=int(env.get_episode_index(0));samples={};missing=[]
    for key,record in recorder.records.items():
        sample=record.sample_for_episode(0,episode)
        if sample is None:missing.append(key)
        samples[key]=serial(sample)
    placement=recorder.placement_record
    if placement is not None:
        sample=placement.sample_for_episode(0,episode)
        if sample is None:missing.append('placement')
        samples['placement']=serial(sample)
    return dict(complete=not missing,missing=missing,env_id=0,episode_index=episode,
        samples=samples,placement_validation=read_placement_validation(env),
        scope='actual enabled native sampler draws; complete means samples saved, not placement or task acceptance')


class TaskRawSink(RawSink):
    def __init__(self, directory, identity, dt, action_dt):
        self.directory=Path(directory);self.identity=identity
        self.dt=dt;self.action_dt=action_dt;self.stride=round(action_dt/dt)
        self.steps=0;self.closed=False;self.pending_commands=None
        self.enqueue_seconds=0.;self.block_seconds=0.
        context=mp.get_context('spawn');self.queue=context.Queue(maxsize=64)
        self.worker=context.Process(target=_writer,args=(str(self.directory),self.queue),daemon=True)
        self.worker.start()


def freeze_implementation(directory, config):
    """Freeze numeric rules, actual implementation identities and tuned build.

    build_info reads JSON only: no native library load, GPU allocation/query.
    """
    from kitchen_safety.gdist_mesh_backend import build_info
    build=build_info();binary=ROOT/'runtime/gdist/gdist_native.so'
    if not build.get('available') or not binary.is_file():raise ValueError('Required gDist build is not installed')
    if hashlib.sha256(binary.read_bytes()).hexdigest()!=build.get('binary_sha256'):
        raise ValueError('gDist binary identity differs from build record')
    profile=json.loads((ROOT/config['profile_file']).read_text())
    files=list((ROOT/'kitchen_safety').glob('*.py'))
    files+=list((ROOT/'third_party/safebench_reference/reference_code/evaluator/safety_risk').glob('*.py'))
    files+=[ROOT/'scripts'/name for name in ('evaluate_native_task.py','collect_native_task.py',
        'inspect_risk_task.py','audit_native_task.py','build_gdist_backend.py')]
    files+=list((ROOT/'native/gdist').glob('*.cu'))+list((ROOT/'native/gdist').glob('*.inc'))+list((ROOT/'native/gdist').glob('*.patch'))
    files=[p for p in files if p.is_file()]
    sources={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    value=dict(config=deepcopy(config),profile=profile,profile_sha256=digest(profile),implementation_sha256=sources,
        mesh_backend=dict(config['mesh_distance_backend'],build=build),disabled_categories=['IR'])
    import tarfile
    archive=Path(directory)/'frozen_implementation.tgz'
    with tarfile.open(archive,'x:gz') as output:
        for p in sorted(files):output.add(p,arcname=str(p.relative_to(ROOT)),recursive=False)
    value['implementation_archive']=dict(path=archive.name,sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
        purpose='Restore exact source in an isolated checkout; do not overwrite current runtime to replay history')
    write_json(Path(directory)/'frozen_rules.json',value)
    return value


class TaskRawRecorder:
    def __init__(self, env, directory, contract, inventory, scope, rs_scope, config, mesh_directory):
        self.env=env;self.e=env.unwrapped;self.directory=Path(directory)
        if self.e.num_envs!=1:raise ValueError('One immutable episode/environment per recorder')
        if getattr(self.e,'_episode_limit',None)!=1:
            raise ValueError('Configure native episode limit 1 before the initial reset')
        self.episode_index=int(self.e.get_episode_index(0));self.last_action_outcome=None
        self.dt=float(self.e.cfg.sim.dt);self.action_dt=float(self.e.step_dt)
        if abs(self.dt-1/120)>1e-10 or abs(self.action_dt-1/30)>1e-10:
            raise ValueError('Frozen rules require 120Hz physics and four-step action windows')
        self.directory.mkdir(parents=True,exist_ok=False)
        import os,shutil
        meshes=self.directory/'meshes';meshes.mkdir()
        for key in {e['fingerprint'] for e in inventory['colliders']}:
            source=Path(mesh_directory)/(key+'.npz');target=meshes/source.name
            try:os.link(source,target)
            except OSError:shutil.copy2(source,target)
        self.robot=self.e.scene['robot'];self.telemetry=SceneCapture(self.robot._physics_sim_view,inventory)
        self.identity=dict(task_id=contract['task_id'],task_sha256=digest(contract),scope_sha256=scope['scope_sha256'])
        self.config=deepcopy(config);self.inventory=inventory
        for name,value in [('task_contract.json',contract),('mesh_inventory.json',inventory),
                ('relation_scope.json',scope),('rs_pair_scope.json',rs_scope),('telemetry_binding.json',self.telemetry.metadata)]:
            write_json(self.directory/name,value)
        freeze_implementation(self.directory,config)
        variation=freeze_variation_samples(self.e)
        self.placement_validation=deepcopy(variation.get('placement_validation',dict(available=False,passed=None)))
        write_json(self.directory/'variation_snapshot.json',variation)
        # A post-placement flattened stage is the reconstruction source. Used
        # USD/texture files are explicit dependencies, never silently remote.
        layers=[str(l.realPath or l.identifier) for l in self.e.sim.stage.GetUsedLayers()]
        self.e.sim.stage.Flatten().Export(str(self.directory/'reconstruction.usda'))
        write_json(self.directory/'reconstruction_dependencies.json',dict(used_layers=layers,
            scope='stage snapshot; texture and referenced external file portability needs separate asset archive verification'))
        self.monitor=ContactMonitor(enable_contact_reports(self.e.sim.stage),self.dt)
        self.monitor.hs_data_omissions[0]=deepcopy(scope.get('hs_acquisition_omissions',{}))
        self.monitor.active={0}
        self.sink=TaskRawSink(self.directory,self.identity,self.dt,self.action_dt)
        self.started=time.perf_counter();self.active=False;self.waiting_update=False;self.closed=False;self.terminal=False
        self.action_index=0;self.phase=None;self.operation_id=None;self.command_events=[]
        self.originals=[]
        self._hook_commands()
        self._hook_physics()
        self.latest=self.capture_state()
        self.sink.submit(dict(kind='initial_state',physics_step=-1,time_s=0.,state=self.latest))
        write_json(self.directory/'raw_capture_schema.json',dict(schema_version='robolaw-native-raw-2',
            **self.identity,evaluation_status='pending_offline',physics_hz=120,action_hz=30,
            contact_acquisition=self.monitor.status,hs_acquisition_omissions=deepcopy(scope.get('hs_acquisition_omissions',{})),robot_joint_names=list(self.robot.joint_names),
            robot_body_names=list(self.robot.body_names),robot_root=str(self.robot.root_view.prim_paths[0]),
            observation_semantics='obs[t] saved before action[t], source_physics_step=4*t-1',
            episode_boundary_required=True,native_episode_limit=1,native_episode_index=self.episode_index,
            terminal_sample_required=True,terminal_semantics='one measured final state after the last action; no next action',
            risk_queries_online=0,features_online=0,grades_online=0,IR='disabled'))

    def capture_state(self):
        row=self.telemetry.capture();data=self.robot.data;robot={};errors=[]
        for key,attr in [('q','joint_pos'),('dq','joint_vel'),('applied_torque','applied_torque'),
                ('joint_pos_limits','joint_pos_limits'),('joint_effort_limits','joint_effort_limits')]:
            try:
                value=array(getattr(data,attr))[0]
                if not np.isfinite(value).all():raise ValueError('Nonfinite '+key)
                robot[key]=value.tolist()
            except Exception as exc:robot[key]=None;errors.append(dict(channel=key,error=repr(exc)))
        row['robot']=robot;row['robot_errors']=errors
        row['authoritative_fault_status']=dict(value=None,coverage_complete=False,source='unavailable in this simulator adapter')
        row['complete']=row['complete'] and not errors
        return row

    def _set(self,obj,name,wrapper):
        self.originals.append((obj,name,getattr(obj,name)));setattr(obj,name,wrapper)

    def _hook_commands(self):
        view=self.robot.root_view
        for name in ('set_dof_position_targets','set_dof_velocity_targets','set_dof_actuation_forces'):
            original=getattr(view,name)
            def write(target,*args,_original=original,_name=name,**kwargs):
                before=array(target).tolist();stamp=time.perf_counter_ns()
                indices=args[0] if args else kwargs.get('indices')
                event=dict(method=_name,nominal=before,indices=array(indices).tolist() if indices is not None else None,
                    before_physics_step=self.sink.steps,action_index=self.action_index,monotonic_ns=stamp)
                try:result=_original(target,*args,**kwargs)
                except BaseException as exc:
                    if self.active:self.sink.submit(dict(kind='fault_event',payload=dict(event,error=repr(exc),kind='command_write_exception')))
                    raise
                if self.active:
                    event.update(submitted=array(target).tolist(),success=True)
                    self.command_events.append(event)
                return result
            self._set(view,name,write)

    def _hook_physics(self):
        original_step=self.e.sim.step;original_update=self.e.scene.update
        def physical(*args,**kwargs):
            if self.active:
                if self.waiting_update:raise RuntimeError('Physical step without a preceding state capture')
                self.waiting_update=True
            return original_step(*args,**kwargs)
        def updated(*args,**kwargs):
            result=original_update(*args,**kwargs)
            if self.active and self.waiting_update:
                state=self.capture_state();contacts=self.monitor.pending.pop(0,[])
                errors=list(self.monitor.errors)
                coverage=bool(self.monitor.status['subscribed'] and not self.monitor.status['errors'] and not errors)
                step=self.sink.steps
                self.sink.submit(dict(kind='physics',physics_step=step,action_index=self.action_index,
                    phase=self.phase,operation_id=self.operation_id,start_time_s=step*self.dt,end_time_s=(step+1)*self.dt,
                    state=state,contacts=contacts,contact_coverage=coverage,contact_errors=errors,
                    commands=deepcopy(self.command_events),receive_wall_ns=time.time_ns()))
                self.command_events.clear();self.latest=state;self.sink.steps+=1;self.waiting_update=False
            return result
        self._set(self.e.sim,'step',physical);self._set(self.e.scene,'update',updated)

    def step(self,action,*,phase,operation_id=None,controller_evidence=None):
        if self.closed or self.terminal:raise RuntimeError('Episode already closed or terminated; freeze a new reset')
        if self.active or self.sink.steps!=4*self.action_index:raise RuntimeError('Invalid action/physics alignment')
        self.phase=str(phase);self.operation_id=operation_id
        self.sink.submit(dict(kind='observation',action_index=self.action_index,phase=self.phase,operation_id=operation_id,
            source_physics_step=self.sink.steps-1,time_s=self.sink.steps*self.dt,state=deepcopy(self.latest),
            nominal_action=array(action).tolist(),controller_evidence=deepcopy(controller_evidence)))
        self.active=True
        try:result=self.env.step(action)
        finally:self.active=False
        if self.waiting_update or self.sink.steps!=4*(self.action_index+1):
            raise RuntimeError('Native action did not produce exactly four captured physical steps')
        self.action_index+=1
        if int(self.e.get_episode_index(0))!=self.episode_index:
            raise RuntimeError('Native environment replaced the frozen episode')
        if len(result)<4:raise RuntimeError('Native step did not return termination flags')
        flags=[]
        for raw_flag in result[2:4]:
            value=array(raw_flag)
            if value.size!=1 or not np.isfinite(value).all() or not np.isin(value,[0,1]).all():
                raise RuntimeError('Invalid native single-environment terminal flag')
            flags.append(bool(value.reshape(-1)[0]))
        terminated,truncated=flags
        self.terminal=terminated or truncated
        manager=self.e.termination_manager;terms={};errors=[]
        for name in manager.active_terms:
            try:
                value=array(manager.get_term(name))
                if value.size!=1 or not np.isfinite(value).all() or not np.isin(value,[0,1]).all():
                    raise ValueError('Invalid single-environment termination flag')
                terms[name]=bool(value.reshape(-1)[0])
            except Exception as exc:terms[name]=None;errors.append(dict(term=name,error=repr(exc)))
        self.last_action_outcome=dict(action_index=self.action_index-1,source_physics_step=self.sink.steps-1,
            native_episode_index=self.episode_index,terminated=terminated,truncated=truncated,
            terms=terms,success=terms.get('success'),errors=errors,
            source='existing native termination results after step; replacement disabled by episode limit 1')
        return result

    def close(self,*,task_success=None,execution_error=None,stop_reason=None):
        if self.closed:return
        self.active=False
        for obj,name,original in reversed(self.originals):setattr(obj,name,original)
        self.monitor.close()
        if self.action_index and self.sink.steps==4*self.action_index and not execution_error:
            self.sink.submit(dict(kind='terminal_observation',action_index=self.action_index,
                source_physics_step=self.sink.steps-1,time_s=self.sink.steps*self.dt,
                phase=self.phase,operation_id=self.operation_id,state=deepcopy(self.latest),
                next_action=None,native_outcome=deepcopy(self.last_action_outcome),
                reason='final physical window must receive an offline safety sample'))
        self.sink.finish();self.closed=True
        p=self.directory/'raw_capture_summary.json';report=json.loads(p.read_text())
        report.update(schema_version='robolaw-native-raw-2',task_success=task_success,execution_error=execution_error,
            complete_physics_windows=self.sink.steps%4==0,online_geometry_queries=0,online_risk_evaluations=0,
            collection_wall_s=time.perf_counter()-self.started,full_task_execution_verified=False,
            placement_validation=self.placement_validation,task_acceptance='not established by raw capture',
            native_episode_index=self.episode_index,native_episode_limit=1,stop_reason=stop_reason or ('environment_terminal' if self.terminal else 'recorder_closed'),
            native_outcome=deepcopy(self.last_action_outcome))
        write_json(p,report)
