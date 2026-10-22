#!/usr/bin/env python3
"""Build the pinned, headless gDist backend inside the Arena Python environment."""
import argparse,hashlib,json,os,platform,shutil,subprocess,sys,sysconfig,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
COMMIT='0300e214f0c9702e83292f35a5d07bdc27ea14ba'
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path);p.add_argument('--cuda-home',type=Path,default=Path(os.environ.get('CUDA_HOME','/usr/local/cuda')));p.add_argument('--arch',default='auto');a=p.parse_args()
 import pybind11
 if a.arch=='auto':
  capability=subprocess.check_output(['nvidia-smi','--query-gpu=compute_cap','--format=csv,noheader'],text=True).splitlines()[0].strip()
  a.arch='sm_'+capability.replace('.','')
 source=(a.source or ROOT/'third_party/gdist').resolve()
 if not source.exists():
  source.parent.mkdir(parents=True,exist_ok=True);subprocess.run(['git','clone','https://github.com/yingtix/DistanceQuery.git',str(source)],check=True)
 if subprocess.check_output(['git','-C',str(source),'rev-parse','HEAD'],text=True).strip()!=COMMIT:
  subprocess.run(['git','-C',str(source),'checkout','--detach',COMMIT],check=True)
 if subprocess.check_output(['git','-C',str(source),'status','--porcelain'],text=True).strip():raise RuntimeError('Pinned gDist source must be clean; use a separate source checkout')
 runtime=ROOT/'runtime/gdist';runtime.mkdir(parents=True,exist_ok=True)
 build=runtime/'build';build.mkdir(exist_ok=True);adapted=build/'gdist_adapted'
 if adapted.exists():shutil.rmtree(adapted)
 shutil.copytree(source/'distanceQuery',adapted)
 native=ROOT/'native/gdist'
 with (native/'gdist_linux.patch').open('rb') as patch:subprocess.run(['patch','-p1'],cwd=adapted,stdin=patch,check=True)
 nvcc=a.cuda_home/'bin/nvcc'
 if not nvcc.is_file():raise RuntimeError('CUDA compiler not found: '+str(nvcc))
 target=runtime/'gdist_native.so';temporary=runtime/'gdist_native.build.so'
 command=[str(nvcc),'-std=c++17','-O3','-DGD_CAP=4194304','--expt-relaxed-constexpr','-Xcompiler','-fPIC','-shared','-arch='+a.arch,
  '-I',str(adapted),'-I',pybind11.get_include(),'-I',sysconfig.get_path('include'),str(native/'gdist_bridge.cu'),'-o',str(temporary)]
 started=time.perf_counter();subprocess.run(command,cwd=ROOT,check=True);temporary.replace(target)
 info=dict(available=True,version='arena-gdist-runtime-1',source_repository='https://github.com/yingtix/DistanceQuery',source_commit=COMMIT,
  binary_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),native_source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in native.iterdir() if p.is_file()},
  compiler=subprocess.check_output([str(nvcc),'--version'],text=True).strip(),command=command,python=sys.version,platform=platform.platform(),architecture=a.arch,build_wall_s=time.perf_counter()-started,
  semantics='Author BVTT and triangle-distance kernels; persistent models/workspaces; explicit multi-stream batch orchestration; no approximate geometry or backend fallback',
  closest_point_coordinates='not exported; witness validity remains false')
 (runtime/'build_info.json').write_text(json.dumps(info,indent=2)+'\n');print(json.dumps(info,indent=2))
if __name__=='__main__':main()
