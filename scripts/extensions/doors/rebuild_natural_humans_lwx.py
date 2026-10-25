"""Restore ignored natural-pose assets from pinned local MPFB blend; preserve existing files."""
import argparse,hashlib,json,subprocess,tempfile,shutil
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--blender',default='/workspace/runtime/tools/blender-4.0.2-linux-x64/blender');a=p.parse_args();root=Path.cwd();source=root/'assets/people/mpfb_corridor/posed.blend';expected='7d022d459ebd584592cde705b7bd73cf112e6cfc719e99b8ea3096506ff650ea';assert hashlib.sha256(source.read_bytes()).hexdigest()==expected
jobs=[('human_natural_c02_upright_lwx_20261026','repose_human_natural_lean_lwx.py',['--wrist','-.65','.25','1.62','--torso-y-deg','0','--clavicle-y-deg','45','--elbow-down']),('human_natural_c06_lwx_20261026','repose_human_natural_wrist_lwx.py',['--wrist','-.58','.42','1.50','--elbow-down'])]
for name,script,args in jobs:
 dest=root/'assets/extensions/doors'/name
 if (dest/'person.usdc').exists() and (dest/'textures').is_dir():print('PRESERVED',name,hashlib.sha256((dest/'person.usdc').read_bytes()).hexdigest());continue
 with tempfile.TemporaryDirectory(prefix='robolaw-natural-') as temp:
  generated=Path(temp)/'asset';subprocess.run([a.blender,'-b',str(source),'--python',str(root/'scripts/extensions/doors'/script),'--',*args,'--asset-dir',str(generated)],check=True)
  assert (generated/'person.usdc').is_file() and (generated/'textures').is_dir()
  dest.mkdir(exist_ok=True,parents=True)
  for item in generated.rglob('*'):
   if not item.is_file():continue
   target=dest/item.relative_to(generated)
   if target.exists():continue
   target.parent.mkdir(exist_ok=True,parents=True);shutil.copyfile(item,target)
  print('RESTORED',name,hashlib.sha256((dest/'person.usdc').read_bytes()).hexdigest())
