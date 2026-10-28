"""New smooth-opening diagnostic leaf; original schema03 and production stay unchanged."""
from copy import deepcopy
import json
from pathlib import Path
import numpy as np
from . import banana_destination_probe as parent
from .banana_release_schedule_candidate import INTERFACE
from .banana_destination_probe import (DestinationNativeContacts, release_boundary, bounded_transition_checks,
    SOURCE_ASSETS, ORIGINAL_COLLECTOR_SHA256, ROOT, digest, digest_text, DESTINATION_MESH)

PARENT_PATH='kitchen_safety/extensions/replicator/banana_destination_probe.py'
PARENT_SHA256='2b5b6ebea7e6c77aa89dc0d466d1fe39d057065a67bab83435ea37f4cd6b350b'
SCHEDULE_PATH='kitchen_safety/extensions/replicator/banana_release_schedule_candidate.py'
SCHEDULE_SHA256='e03c48b64195c7d0e649865ca8326e8cfffb7cfca67ab76bdd7ba3aa7a2dd485'
MODULE_PATH='kitchen_safety/extensions/replicator/banana_smooth_destination_probe.py'
DECLARATION=dict(parent.DECLARATION,schema_version=4,maximum_release_actions=30,maximum_release_physics_steps=240,
    release_seconds=2.,smooth_opening_ramp_actions=15,additional_zero_target_actions=15,
    action_interface=INTERFACE,source_gripper_open_rad=0.,source_gripper_close_rad=float(np.pi/4),
    original_joint_bound_allowance_rad=1e-5,
    terminal_predicate_rule=parent.DECLARATION['terminal_predicate_rule'].replace('56','240'),
    parent_probe_source=dict(path=PARENT_PATH,sha256=PARENT_SHA256),
    schedule_source=dict(path=SCHEDULE_PATH,sha256=SCHEDULE_SHA256),
    target_rule='Only local gripper action uses explicit joint-position radians. Nonrelease binary endpoints translate0/pi4; first15 release targets are right-endpoint samples of a1s quintic from actual boundary q to0, then15 targets remain0. Exactly2s/30actions/240substeps. No measured state setters or clamping.',
    target_recording_rule='Before every sim.step, after source apply_action/write_data_to_sim, read raw action, processed gripper target, articulation target buffer and native DOF target. Bind to the following actualq/dq sample and absolute physics step.')

def read_declaration(path):
    value=json.loads(Path(path).read_text())
    if value!=DECLARATION:raise ValueError('Unreviewed smooth diagnostic declaration')
    for relative,expected in [(PARENT_PATH,PARENT_SHA256),(SCHEDULE_PATH,SCHEDULE_SHA256),('scripts/extensions/replicator/task_collect.py',ORIGINAL_COLLECTOR_SHA256)]+list(SOURCE_ASSETS.values()):
        if digest(ROOT/relative)!=expected:raise ValueError('Smooth diagnostic source changed: '+relative)
    return value

def audit_declaration_archive(report,binding):
    entries=report['entries'];item=entries[binding['path']]
    if item['kind']!='diagnostic_declaration_input' or json.loads(item['source_text'])!=DECLARATION:
        raise ValueError('Missing exact schema04 declaration')
    required={binding['path']:binding['sha256'],MODULE_PATH:binding['module_sha256'],PARENT_PATH:PARENT_SHA256,SCHEDULE_PATH:SCHEDULE_SHA256}
    if binding['module_path']!=MODULE_PATH:raise ValueError('Wrong smooth diagnostic module')
    for path,expected in required.items():
        entry=entries[path]
        expected_kind='diagnostic_declaration_input' if path==binding['path'] else 'execution_source'
        if entry['kind']!=expected_kind:raise ValueError('Wrong archive source role: '+path)
        if not entry['unchanged'] or entry['initial_sha256']!=expected or entry['checked_sha256']!=expected or digest_text(entry['source_text'])!=expected:
            raise ValueError('Missing/changed archive: '+path)
    return dict(passed=True,binding=deepcopy(binding),required_source_sha256=required,accepted=False)

class DiagnosticDestinationWorld(parent.DiagnosticDestinationWorld):
    def _destination_measurement(self):
        obstacle=next(x for x in self.objects if x[0]==DESTINATION_MESH)
        pose=self._query_payload_pose
        shape=self.fcl.CollisionObject(self.payload_triangle_model,self.fcl.Transform(pose[:3,:3],pose[:3,3]))
        strict=self._exact_measurement((0.,'payload/banana',DESTINATION_MESH,shape,obstacle[1],.002 if self.phase=='lower' else 0.))
        strict.update(contact_pair='payload/destination',noncontact_required_m=.002,
            representation='Original banana/bowl source triangles; diagnostic lower retains 2 mm')
        if not self._probe_execution_active:return strict
        row=self.probe_row
        if row is None or self.probe_native is None or self.probe_native['physics_step']!=row['physics_step']:
            raise ValueError('Missing aligned native destination data')
        step=int(row['physics_step'])
        if step in self._probe_measurements:return deepcopy(self._probe_measurements[step])
        source=self.source_bowl_observation(pose)
        record=dict(physics_step=step,phase=self.phase,action_index=int(row['action_index']),source=source,
            original_strict_measurement=deepcopy(strict),diagnostic_continuation=False)
        answer=deepcopy(strict)
        if self.phase=='release':
            if self.release_start_step is None or not 1<=step-self.release_start_step<=240:
                raise ValueError('Release exceeds predeclared 240 actual substeps')
            previous_release=[x['physics_step'] for x in self.probe_records if x['phase']=='release']
            expected_step=(previous_release[-1]+1) if previous_release else self.release_start_step+1
            if step!=expected_step:raise ValueError('Bounded release observation has a gap/duplicate')
            force=np.asarray(row['destination_contact_force']).reshape(-1,3).sum(0)
            continuation=bounded_transition_checks(source,self.probe_native,force,self.probe_low,self.probe_high)
            continuation['scope']=continuation['scope'].replace('56 steps','240 steps')
            record['bounded_transition']=continuation
            if not continuation['passed']:
                answer.update(valid=False,diagnostic_observation_guard_failed=True)
            if not strict['valid']:
                record['support_continuation']=continuation
                if continuation['passed']:
                    answer.update(valid=True,diagnostic_continuation_only=True,original_strict_valid=False)
                    record['diagnostic_continuation']=True
                self.strict_failure_ledger.append(deepcopy(record))
        elif not strict['valid']:self.strict_failure_ledger.append(deepcopy(record))
        self.probe_records.append(record);self._probe_measurements[step]=deepcopy(answer)
        return answer

