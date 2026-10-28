"""First-use provenance regression checked in a fresh CPU Python process."""
from pathlib import Path
import subprocess
import sys
import pytest

ROOT = Path(__file__).resolve().parents[3]
CHECK = r'''
from pathlib import Path
import argparse, importlib.abc, json, sys, types
from kitchen_safety.extensions.replicator.task_collection_evidence import TaskRunProvenance,input_key
root=Path.cwd();seen=set();captured={}
original=TaskRunProvenance.capture

def capture(self,path,kind,*args,**kwargs):
    answer=original(self,path,kind,*args,**kwargs)
    captured[input_key(path,root)]=kind
    return answer
TaskRunProvenance.capture=capture
modules={
 'kitchen_safety.extensions.replicator.'+name:'kitchen_safety/extensions/replicator/'+name+'.py'
 for name in ['banana_smooth_destination_probe','banana_destination_probe','banana_release_schedule_candidate','banana_command_recording']}
declaration='configs/extensions/replicator/g_banana_smooth_destination_contact_probe.json'
class Guard(importlib.abc.MetaPathFinder):
    def find_spec(self,fullname,path=None,target=None):
        if fullname in modules:
            assert all(captured.get(p)=='execution_source' for p in modules.values()),'NEW_LEAF_BEFORE_CAPTURE'
            assert captured.get(declaration)=='diagnostic_declaration_input','DECLARATION_BEFORE_CAPTURE'
            seen.add(fullname)
        return None
sys.meta_path.insert(0,Guard())
path=root/'scripts/extensions/replicator/banana_smooth_destination_contact_probe.py'
source=path.read_text()
if sys.argv[1]=='old_order':
    late='    # Capture every new leaf and the declaration before their first import/use.\n    from kitchen_safety.extensions.replicator.banana_smooth_destination_probe import read_declaration\n'
    source=source.replace(late,'')
    before='    provenance=TaskRunProvenance(LAYOUT,collector_path=__file__,input_paths=[args.controller_profile])'
    source=source.replace(before,'    from kitchen_safety.extensions.replicator.banana_smooth_destination_probe import read_declaration\n'+before)
module=types.ModuleType('fresh_smooth_probe');module.__file__=str(path)
exec(compile(source,str(path),'exec'),module.__dict__)
args=argparse.Namespace(controller_profile=root/'configs/extensions/replicator/g_shape_banana_bowl_controller.json',grasp_candidate_profile=root/'configs/extensions/replicator/g_shape_banana_bowl_halfcenter_candidate.json',transport_candidate_profile=root/'configs/extensions/replicator/g_shape_banana_bowl_transport_height_candidate.json',probe_declaration=root/declaration)
provenance=module.make_provenance(args);report=provenance.check()
assert report['passed']
assert set(modules)-{'kitchen_safety.extensions.replicator.banana_command_recording'}<=seen
assert len(report['entries'])==101
assert not any(x=='pxr' or x.startswith('pxr.') or x=='omni.usd' or x.startswith('omni.usd.') for x in sys.modules)
assert len([v for v in report['entries'].values() if v['kind']=='controller_input'])==1
print(json.dumps(dict(passed=True,first_imports_checked=sorted(seen),captures=len(report['entries']))))
'''

@pytest.mark.parametrize('mode',['current','old_order'])
def test_capture_precedes_first_new_leaf_import(mode):
    result=subprocess.run([sys.executable,'-c',CHECK,mode],cwd=ROOT,capture_output=True,text=True,timeout=30)
    if mode=='current':
        assert result.returncode==0,result.stderr
        assert '"passed": true' in result.stdout
    else:
        assert result.returncode!=0
        assert 'NEW_LEAF_BEFORE_CAPTURE' in result.stderr
