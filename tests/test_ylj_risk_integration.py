"""Scope expansion must fail before a GPU query; source commands must survive."""
from copy import deepcopy
import json
from pathlib import Path
import unittest
from types import SimpleNamespace
import numpy as np
from kitchen_safety.task_contract import digest
from kitchen_safety.reviewed_scope_lock import apply_lock, load_lock, VERSION
from kitchen_safety.ylj_risk_capture import commands, input_preflight

ROOT=Path(__file__).resolve().parents[1]


class ScopeCeiling(unittest.TestCase):
    def setUp(self):
        self.lock=dict(version=VERSION, task_id='t', task_sha256='task', authorization='user',
            pt_targets={'keep':['keep/a']}, human_collision_paths=[],source_scope_sha256='before')
        relation=dict(category='PT',relation_id='r',source='/e/Robot/arm',source_colliders=['/e/Robot/arm/mesh'],
            target='/e/keep',target_colliders=['/e/keep/a','/e/keep/unreviewed'],
            contact_target_colliders=['/e/keep/a','/e/keep/unreviewed'],
            query_distance=True,persist_distance=True,grade_distance=True,grade_force=True,grade_impulse=True,
            applicability='active',evidence_only=False)
        self.scope=dict(task_id='t',task_sha256='task',relations=[relation],omitted=[],bindings={},
            human_inventory={'collision_paths':[]},rs={'independent':'unchanged'},scope_sha256='before')
        self.inventory={'env_root':'/e','colliders':[{'path':'/e/keep/a'},{'path':'/e/keep/unreviewed'}]}

    def test_exact_collider_intersection_and_source_immutable(self):
        original=deepcopy(self.scope);after=apply_lock(self.scope,self.inventory,self.lock)
        self.assertEqual(self.scope,original)
        self.assertEqual(after['relations'][0]['target_colliders'],['/e/keep/a'])
        self.assertEqual(after['relations'][0]['contact_target_colliders'],['/e/keep/a'])
        self.assertEqual(after['rs'],self.scope['rs'])
        omission=after['omitted'][-1]
        self.assertEqual(omission['reason'],'authorized_PT_target_subset')
        self.assertEqual(omission['source_colliders'],self.scope['relations'][0]['source_colliders'])
        for key in ('grade_distance','grade_force','grade_impulse','applicability','source_colliders'):
            self.assertEqual(after['relations'][0][key],self.scope['relations'][0][key])

    def test_same_count_different_asset_cannot_replace_reviewed_asset(self):
        self.scope['relations'][0].update(target='/e/new',target_colliders=['/e/new/a'],contact_target_colliders=['/e/new/a'])
        after=apply_lock(self.scope,self.inventory,self.lock)
        self.assertEqual(after['relations'],[])
        self.assertEqual(after['reviewed_scope_ceiling']['actual_PT_assets'],0)
        self.assertFalse(after['omitted'][0]['extract_risk_features'])

    def test_new_human_is_rejected_instead_of_reported_absent(self):
        self.scope['human_inventory']['collision_paths']=['/e/new_person/body']
        with self.assertRaisesRegex(ValueError,'human'):apply_lock(self.scope,self.inventory,self.lock)

    def test_wrong_task_or_contract_cannot_use_ceiling(self):
        for key in ('task_id','task_sha256'):
            lock=deepcopy(self.lock);lock[key]='wrong'
            with self.assertRaises(ValueError):apply_lock(self.scope,self.inventory,lock)

    def test_path_traversal_is_rejected(self):
        self.lock['pt_targets']={'../other':['keep/a']}
        with self.assertRaises(ValueError):apply_lock(self.scope,self.inventory,self.lock)

    def test_changed_scene_namespace_cannot_silently_become_empty_safe_scope(self):
        self.inventory['colliders']=[{'path':'/e/different_namespace/a'}]
        with self.assertRaisesRegex(ValueError,'identities do not bind'):
            apply_lock(self.scope,self.inventory,self.lock)

    def test_all_fourteen_locks_bind_actual_contracts(self):
        document=json.loads((ROOT/'configs/risk_integration/replicator_scope_lock.json').read_text())
        self.assertEqual(len(document['tasks']),14)
        for task_id,row in document['tasks'].items():
            contract=json.loads((ROOT/'configs/risk_task_suite'/(task_id+'.json')).read_text())
            self.assertEqual(load_lock(ROOT,contract)['task_sha256'],digest(contract))
            self.assertEqual(row['human_collision_paths'],[])
            self.assertTrue(row['pt_targets'])


class SourceCommands(unittest.TestCase):
    def data(self):
        return dict(q=np.arange(42).reshape(6,7)*.001, tcp_world=np.repeat(np.eye(4)[None],6,axis=0),
            phase=np.array(['pregrasp','approach','lift','transfer','lower','retreat']),
            gripper=np.array([0.,0.,.5,.5,.5,0.]),dt=np.array(1/30))

    def test_exact_npz_samples_and_ylj_hold_count(self):
        data=self.data();rows=list(commands(data,data['q'][0]))
        originals=[r for r in rows if r['source_index']>=0]
        self.assertEqual(len(rows),6+150+60+60+90)
        for i,r in enumerate(originals):
            np.testing.assert_array_equal(r['q'],data['q'][i])
            self.assertEqual(r['grip'],data['gripper'][i])
            self.assertEqual(r['source_phase'],data['phase'][i])

    def test_misaligned_arrays_do_not_truncate_commands(self):
        data=self.data();data['gripper']=data['gripper'][:-1]
        with self.assertRaises(ValueError):list(commands(data,data['q'][0]))

    def test_no_implicit_resampling(self):
        data=self.data();data['dt']=np.array(1/15)
        with self.assertRaises(ValueError):list(commands(data,data['q'][0]))

    def test_preflight_missing_inputs_and_new_human_are_explicit(self):
        index=json.loads((ROOT/'configs/extensions/replicator/final_154_scenes_20261028.json').read_text())
        for row in index['entries']:
            tid=row['task_id'].removeprefix('kitchen_bench_')
            contract=json.loads((ROOT/'configs/risk_task_suite'/(tid+'.json')).read_text())
            result=input_preflight(contract,index,row['scene_id'],load_lock(ROOT,contract))
            self.assertEqual(result['unreviewed_human_would_expand_scope'],row['variant_id']=='human_default_mpfb')
            self.assertEqual(result['online_risk_queries'],0)
            self.assertFalse(result['replanned'])


class EpisodeBoundary(unittest.TestCase):
    def recorder(self,continue_source):
        from kitchen_safety.task_capture import TaskRawRecorder
        r=object.__new__(TaskRawRecorder)
        r.closed=False;r.terminal=True;r.allow_terminal_continuation=continue_source
        r.active=False;r.waiting_update=False;r.action_index=0;r.diagnostic_actuation=None
        r.latest={'measured':'initial'};r.episode_index=0;r.dt=1/120
        rows=[];r.sink=SimpleNamespace(steps=0,submit=rows.append)
        manager=SimpleNamespace(active_terms=['success'],get_term=lambda name:np.array([True]))
        r.e=SimpleNamespace(get_episode_index=lambda env:0,termination_manager=manager)
        def step(action):
            self.assertTrue(r.active)
            r.sink.steps+=4;r.latest={'measured':'after four real substep callbacks'}
            return None,None,np.array([True]),np.array([False]),{}
        r.env=SimpleNamespace(step=step)
        return r,rows

    def test_original_native_entry_still_stops_at_terminal(self):
        recorder,rows=self.recorder(False)
        with self.assertRaises(RuntimeError):recorder.step(np.zeros((1,8)),phase='retreat')
        self.assertEqual(rows,[])

    def test_explicit_ylj_continuation_keeps_native_outcome_and_clock(self):
        recorder,rows=self.recorder(True)
        recorder.step(np.zeros((1,8)),phase='retreat')
        self.assertEqual(rows[0]['source_physics_step'],-1)
        self.assertEqual(recorder.last_action_outcome['source_physics_step'],3)
        self.assertTrue(recorder.last_action_outcome['success'])
        self.assertEqual(recorder.action_index,1)

if __name__=='__main__':unittest.main()
