"""CPU-only retry/lineage/no-render tests. Synthetic fixtures never alter real evidence."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from kitchen_safety.extensions.replicator import retry_physical_execution as run
from scripts.extensions.replicator import execute_failure_retry as cli


class RetryTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)
        for obj in (run,run.base,cli):
            p=patch.object(obj,'ROOT',self.root);p.start();self.addCleanup(p.stop)
        p=patch.object(run,'runtime_hashes',return_value={'executor.py':'synthetic-source'});p.start();self.addCleanup(p.stop)
        self.evidence=self.write('evidence.json',{'passed':True})
        first=[]; retakes=[]; entries=[]
        for i in range(154):
            sid='scene_'+str(i); task='task_'+str(i//11)
            layout=self.write(f'configs/{sid}.json',dict(task_id=task))
            planresult=self.write(f'planning/{sid}/result.json',dict(status='generated',trajectory_complete=True,
                scene_id=sid,composition={'source_paths_sha256':{}},start_q=[0.]*7))
            scene=self.write(f'planning/{sid}/source_scene.usda',{'synthetic':True})
            old=self.root/f'planning/{sid}/trajectory.npz'
            data=dict(q=np.zeros((6,7)),tcp_world=np.repeat(np.eye(4)[None],6,axis=0),
                phase=np.array(run.PHASES),gripper=np.array([0.,0.,.38,.38,.38,0.]),dt=np.array(1/30))
            np.savez(old,**data)
            row=dict(scene_id=sid,task_id=task,variant_id='variant',result_path=planresult['path'],result_sha256=planresult['sha256'],
                config_path=layout['path'],config_sha256=layout['sha256'],source_scene_path=scene['path'],source_scene_sha256=scene['sha256'],
                trajectory_path=str(old.relative_to(self.root)),trajectory_sha256=run.sha(old),
                execution_status='failed' if i<84 else 'success',task_success=i>=84,executed=True,video_sha256='not-rehashed-historical-video')
            actual=dict(row,kind='physical_execution',status='complete',physical_attempt_number=1,
                planning_result_path=row['result_path'],planning_result_sha256=row['result_sha256'])
            physical=self.write(f'first/executions/{sid}/result.json',actual)
            row.update(execution_result_path=physical['path'],execution_result_sha256=physical['sha256']);first.append(row)
            if i<4:
                retake=copy.deepcopy(row);actual['physical_attempt_number']=2
                extra=self.write(f'retake/executions/{sid}/result.json',actual)
                retake.update(physical_attempt_number=2,execution_result_path=extra['path'],execution_result_sha256=extra['sha256'],
                    first_pass={'result_sha256':physical['sha256']});retakes.append(retake)
            if i<84:
                new=self.root/f'adjusted/{sid}.npz';new.parent.mkdir(exist_ok=True)
                np.savez(new,**{k:np.repeat(v,2,axis=0) if k!='dt' else v for k,v in data.items()})
                entry={k:row[k] for k in run.SOURCE_FIELDS}
                entry.update(source_trajectory_path=row['trajectory_path'],source_trajectory_sha256=row['trajectory_sha256'],
                    trajectory_path=str(new.relative_to(self.root)),trajectory_sha256=run.sha(new),physical_attempt_number=3 if i<4 else 2,
                    adjustment=dict(kind='retime',description='Synthetic timing-only fixture',evidence_path=self.evidence['path'],evidence_sha256=self.evidence['sha256']))
                entries.append(entry)
        self.plan=dict(kind='physical_failure_retry_plan',source_first_pass_manifest=self.write('first/manifest.json',{'entries':first}),
            source_camera_retake_manifest=self.write('retake/manifest.json',{'entries':retakes}),entries=entries,
            execution_settings=dict(episode_seconds=200.,max_actions=6000))
        self.planpath=self.root/'retry_plan.json';self.write('retry_plan.json',self.plan)
        self.args=cli.parser().parse_args(['--plan',str(self.planpath),'--output',str(self.root/'retry')])
        cli.apply_settings(self.args,self.plan)

    def write(self,name,data):
        p=self.root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(data))
        return dict(path=str(p.relative_to(self.root)),sha256=run.sha(p))

    def test_complete_whitelist_and_physical_lineage(self):
        _,rows=run.validate_plan(self.planpath)
        self.assertEqual(len(rows),84);self.assertEqual(sum(r['physical_attempt_number']==3 for r in rows),4)
        self.assertEqual(len(rows[0]['prior_physical_attempts']),2)
        run.validate_entry_lineage(rows[0],self.args)

    def test_omitting_failure_or_duplicating_row_rejected(self):
        for entries in (self.plan['entries'][:-1],self.plan['entries']+[self.plan['entries'][0]]):
            self.plan['entries']=entries;self.write('retry_plan.json',self.plan)
            with self.assertRaises(ValueError):run.validate_plan(self.planpath)

    def test_adjusted_phase_endpoint_may_not_change(self):
        row=self.plan['entries'][0];p=self.root/row['trajectory_path'];d=dict(np.load(p));d['q'][0,0]=.1
        np.savez(p,**d);row['trajectory_sha256']=run.sha(p)
        with self.assertRaisesRegex(ValueError,'endpoints'):run.load_retry_inputs(row)

    def test_gripper_and_shortened_array_rejected(self):
        row=self.plan['entries'][0];p=self.root/row['trajectory_path'];d=dict(np.load(p));d['gripper'][4]=.73
        np.savez(p,**d);row['trajectory_sha256']=run.sha(p)
        with self.assertRaisesRegex(ValueError,'gripper'):run.load_retry_inputs(row)
        d['gripper']=d['gripper'][:-1];np.savez(p,**d);row['trajectory_sha256']=run.sha(p)
        with self.assertRaisesRegex(ValueError,'misaligned'):run.load_retry_inputs(row)

    def test_changed_original_config_fails_sha(self):
        row=self.plan['entries'][0];(self.root/row['config_path']).write_text('{}')
        with self.assertRaisesRegex(ValueError,'Frozen input changed'):run.load_retry_inputs(row)

    def test_attempt_number_must_include_camera_episode(self):
        self.plan['entries'][0]['physical_attempt_number']=2;self.write('retry_plan.json',self.plan)
        with self.assertRaisesRegex(ValueError,'attempt number'):run.validate_plan(self.planpath)

    def test_retry_budget_and_locked_physics(self):
        _,_,data=run.load_retry_inputs(self.plan['entries'][0]);self.assertEqual(run.validate_budget(data,self.args),372)
        self.args.max_actions=371
        with self.assertRaisesRegex(ValueError,'budget'):run.validate_budget(data,self.args)
        for key,value in [('device','cuda:0'),('close_seconds',1.),('physics_threads',32)]:
            with self.subTest(key=key),self.assertRaises(ValueError):cli.apply_settings(self.args,{'execution_settings':{key:value}})

    def test_no_camera_config_and_runtime_gate(self):
        cfg=SimpleNamespace(observations=SimpleNamespace(camera_obs=object()),sim=SimpleNamespace(),scene=SimpleNamespace(robot=object()))
        run.configure_no_render(cfg);self.assertIsNone(cfg.observations.camera_obs);self.assertEqual(cfg.sim.visualizer_cfgs,[])
        cfg.scene.camera=object()
        with self.assertRaisesRegex(ValueError,'Camera entity'):run.configure_no_render(cfg)
        sim=SimpleNamespace(is_rendering=False,has_gui=False,get_setting=lambda key:False,
            resolve_visualizer_types=lambda:[],visualizers=[],render_generation=0)
        env=SimpleNamespace(sim=sim,render_enabled=False,scene=SimpleNamespace(sensors={}))
        self.assertTrue(run.render_state(env)['passed'])
        sim.is_rendering=True;self.assertFalse(run.render_state(env)['passed'])

    def test_pending_selector_never_repeats_terminal_or_attempted(self):
        _,rows=run.validate_plan(self.planpath);manifest={'entries':rows}
        rows[0].update(execution_status='failed',attempts=1,executed=True)
        self.assertNotIn(rows[0],cli.select_pending(manifest,self.args))
        self.args.scene=rows[0]['scene_id'];self.assertEqual(cli.select_pending(manifest,self.args),[])
        rows[1]['attempts']=1
        with self.assertRaisesRegex(ValueError,'uncertain attempt'):cli.select_pending(manifest,self.args)

    def test_dry_run_creates_no_output_and_imports_no_kit(self):
        from contextlib import redirect_stdout
        from io import StringIO
        import sys
        with redirect_stdout(StringIO()) as output:
            cli.main(['--plan',str(self.planpath),'--output',str(self.root/'retry'),'--dry-run','--limit','1'])
        result=json.loads(output.getvalue());self.assertEqual(result['selected_pending'],1)
        self.assertFalse(result['physics_launched']);self.assertFalse(result['rendering_launched'])
        self.assertFalse((self.root/'retry').exists());self.assertNotIn('omni.kit.app',sys.modules)

    def test_manifest_source_mutation_is_rejected(self):
        _,rows=run.validate_plan(self.planpath)
        manifest=dict(kind='physical_failure_retry',retry_plan_path=str(self.planpath),retry_plan_sha256=run.sha(self.planpath),
            entries=copy.deepcopy(rows),runtime_revisions=[dict(id=0,source_sha256=run.runtime_hashes(),settings=run.runtime_settings(self.args))])
        cli.check_existing(manifest,rows,self.args)
        manifest['entries'][0]['config_sha256']='changed'
        with self.assertRaisesRegex(ValueError,'binding differs'):cli.check_existing(manifest,rows,self.args)


if __name__=='__main__':unittest.main()
