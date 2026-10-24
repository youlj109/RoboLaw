"""Build an isolated small-capacity backend and check exact frontier completion.

Run inside the configured CUDA/Python environment after build_gdist_backend.py.
Never replaces the installed binary or edits frozen episodes.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();output=args.output.resolve()
    info=json.loads((ROOT/'runtime/gdist/build_info.json').read_text())
    binary=ROOT/'runtime/gdist/gdist_native.so'
    if hashlib.sha256(binary.read_bytes()).hexdigest()!=info['binary_sha256']:
        raise ValueError('Installed backend identity mismatch')
    for name,expected in info['native_source_sha256'].items():
        if hashlib.sha256((ROOT/'native/gdist'/name).read_bytes()).hexdigest()!=expected:
            raise ValueError('Installed backend source differs: '+name)
    command=list(info['command']);old_flags=[x for x in command if x.startswith('-DGD_CAP=')]
    if len(old_flags)!=1 or command.count('-o')!=1:
        raise ValueError('Unrecognized compiler invocation; review before rebuilding')
    command[command.index(old_flags[0])]='-DGD_CAP=256'
    output.mkdir(parents=True,exist_ok=False)
    isolated=output/'gdist_native.so';command[command.index('-o')+1]=str(isolated)
    receipt=dict(complete=False,source_build=info,command=command,installed_binary_changed=False)
    report=output/'verification_build.json'
    report.write_text(json.dumps(receipt,indent=2)+'\n')
    subprocess.run(command,cwd=ROOT,check=True)
    receipt['candidate_sha256']=hashlib.sha256(isolated.read_bytes()).hexdigest()
    subprocess.run([sys.executable,str(ROOT/'scripts/check_gdist_capacity.py'),'--binary',str(isolated),
                    '--output',str(output/'capacity_check.json')],cwd=ROOT,check=True)
    if hashlib.sha256(binary.read_bytes()).hexdigest()!=info['binary_sha256']:
        raise ValueError('Installed executable changed during isolated verification')
    receipt['complete']=True;report.write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps(dict(passed=True,output=str(output),installed_binary_changed=False)))

if __name__=='__main__':main()
