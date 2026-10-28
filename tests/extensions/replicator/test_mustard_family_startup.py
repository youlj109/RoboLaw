"""Fresh-process startup tests. Fake context failures never launch Kit or GPU."""
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[3]
PROFILE=ROOT/'reports/replicator_preparation/mustard_family_cpu02/l_island_controller01.json'


def run(code,*args):
    return subprocess.run([sys.executable,'-c',code,*map(str,args)],cwd=ROOT,check=True,
        text=True,capture_output=True)


def test_input_hash_capture_does_not_import_usd_or_omni():
    output=run('''
import sys,json
from pathlib import Path
from kitchen_safety.extensions.replicator.mustard_family_controller import read_controller_inputs
from kitchen_safety.extensions.replicator.mustard_family_evidence import MustardFamilyProvenance
path=Path(sys.argv[1]);controller,prepared=read_controller_inputs(path)
evidence=MustardFamilyProvenance(path,controller,prepared)
before=[name for name in sys.modules if name=='pxr' or name.startswith('pxr.') or name=='omni' or name.startswith('omni.')]
assert not before and evidence.check()['passed'] and not evidence.scene_bound
from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
get_isaaclab_arena_cli_parser().parse_args(['--enable_cameras','--device','cpu','--seed','42'])
after=[name for name in sys.modules if name=='pxr' or name.startswith('pxr.') or name=='omni.usd' or name.startswith('omni.usd.')]
assert not after
print(json.dumps(dict(before_arena_bootstrap=before,usd_after_official_arena_bootstrap=after,kit_launched=False)))
''',PROFILE)
    result=json.loads(output.stdout.strip().splitlines()[-1])
    assert result==dict(before_arena_bootstrap=[],usd_after_official_arena_bootstrap=[],kit_launched=False)


def test_fake_context_startup_failure_is_saved_without_physical_claim(tmp_path):
    destination=tmp_path/'failed'
    run('''
import sys,argparse,fcntl
from types import SimpleNamespace as N
class FakeContext:
    def __init__(self,args):pass
    def __enter__(self):raise RuntimeError('synthetic-context-startup-failure')
    def __exit__(self,*args):raise AssertionError('Failed enter must not exit')
sys.modules['isaaclab_arena.cli.isaaclab_arena_cli']=N(get_isaaclab_arena_cli_parser=lambda:N(parse_args=lambda _:argparse.Namespace()))
sys.modules['isaaclab_arena.utils.isaaclab_utils.simulation_app']=N(SimulationAppContext=FakeContext)
# This synthetic launch never acquires the actual GPU lock or creates Kit.
fcntl.flock=lambda *args:None
from scripts.extensions.replicator.mustard_family_collect import main
profile,destination=sys.argv[1:]
sys.argv=['mustard_family_collect.py','--controller-profile',profile,'--output',destination,
    '--neutral-seconds','45','--device','cpu','--physics-threads','2','--torch-threads','2']
try:main()
except RuntimeError as error:assert str(error)=='synthetic-context-startup-failure'
else:raise AssertionError('Synthetic startup failure was swallowed')
''',PROFILE,destination)
    state=json.loads((destination/'launch_state.json').read_text())
    failure=json.loads((destination/'launch_failure.json').read_text())
    result=json.loads((destination/'result.json').read_text())
    assert state['stage']=='launch_failed' and not state['kit_context_entered'] and not state['launch_complete']
    assert not state['collector_run_entered'] and state['before_arena_bootstrap_modules']==[]
    assert failure['stage']=='kit_initialization_requested' and not failure['physical_record_file_present']
    assert result['status']=='initialization_failed' and result['neutral_probe_passed'] is False
    assert result['physical_trial_passed'] is False and result['accepted'] is False
    assert (destination/'prelaunch_provenance.json').is_file()
    assert (destination/'launch_exit_provenance.json').is_file()
    assert not list(destination.glob('*.npz')) and not list(destination.glob('*.mp4'))


def test_inner_startup_failure_is_saved_before_context_forces_exit_zero(tmp_path):
    destination=tmp_path/'inner_failed'
    run('''
import sys,argparse,fcntl,os
from types import SimpleNamespace as N
class FakeContext:
    def __init__(self,args):pass
    def __enter__(self):return self
    def __exit__(self,*args):os._exit(0)
sys.modules['isaaclab_arena.cli.isaaclab_arena_cli']=N(get_isaaclab_arena_cli_parser=lambda:N(parse_args=lambda _:argparse.Namespace()))
sys.modules['isaaclab_arena.utils.isaaclab_utils.simulation_app']=N(SimulationAppContext=FakeContext)
fcntl.flock=lambda *args:None
from scripts.extensions.replicator import mustard_family_collect as module
def fake_run(args,provenance,on_entry=None):
    on_entry()
    raise ValueError('synthetic-source-validation-failure')
module.run=fake_run
profile,destination=sys.argv[1:]
sys.argv=['mustard_family_collect.py','--controller-profile',profile,'--output',destination,
    '--neutral-seconds','45','--device','cpu','--physics-threads','2','--torch-threads','2']
module.main()
raise AssertionError('Context should have terminated the subprocess')
''',PROFILE,destination)
    state=json.loads((destination/'launch_state.json').read_text())
    failure=json.loads((destination/'launch_failure.json').read_text())
    result=json.loads((destination/'result.json').read_text())
    assert state['stage']=='launch_failed' and state['kit_context_entered'] and state['collector_run_entered']
    assert not state['launch_complete'] and 'collector_return_code' not in state
    assert failure['error']=='synthetic-source-validation-failure' and failure['stage']=='collector_run_entered'
    assert result['neutral_probe_passed'] is False and result['physical_trial_passed'] is False
    assert result['accepted'] is False and not list(destination.glob('*.npz'))
