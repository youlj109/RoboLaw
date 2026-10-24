"""CPU discovery using the original rig with a fingertip near a measured transient point."""
import argparse, hashlib, itertools, json, math, shutil, subprocess
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--baseline',type=Path,required=True)
p.add_argument('--probe',type=Path,required=True)
p.add_argument('--asset-dir',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
p.add_argument('--wrist-height-m',type=float,required=True)
p.add_argument('--texture-source',type=Path,default=Path('assets/extensions/doors/human_reach_lean_low_lwx_19/textures'))
p.add_argument('--resume-built-pose',action='store_true')
a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
if a.asset_dir.exists() and not a.resume_built_pose:raise FileExistsError(a.asset_dir)
if a.resume_built_pose:
 pose=json.loads((a.asset_dir/'pose.json').read_text())
 if pose['max_unchanged_bone_error_m']>1e-5 or pose['max_bone_length_change_m']>1e-5 or abs(pose['requested_wrist_world_asset_m'][2]-a.wrist_height_m)>1e-5:raise RuntimeError('Built pose does not match requested original-rig construction')
 if pose['source_sha256']!=hashlib.sha256(Path(pose['source']).read_bytes()).hexdigest():raise RuntimeError('Built pose source mismatch')
if not 1.1<=a.wrist_height_m<=1.7:p.error('Wrist height outside original-rig discovery range')
m=json.loads((a.baseline/'metadata.json').read_text())
probe=json.loads(a.probe.read_text())
if probe['baseline']!=str(a.baseline):raise RuntimeError('Probe baseline mismatch')
if not probe.get('best_samples'):raise RuntimeError('No measured transient sample to investigate')
target=probe['best_samples'][0]['point']
cmd=['runtime/tools/blender-4.0.2-linux-x64/blender','--background','--threads','2','assets/people/mpfb_corridor/posed.blend','--python','scripts/extensions/doors/repose_human_both_lwx.py','--','--asset-dir',str(a.asset_dir),'--wrist','-.90','.15',str(a.wrist_height_m),'--auto-reachable-target','--elbow-up','--clavicle-y-deg','45','--torso-y-deg','-18']
if not a.resume_built_pose:
 with a.output.with_suffix('.build.log').open('x') as f:subprocess.run(cmd,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,check=True)
shutil.copytree(a.texture_source,a.asset_dir/'textures',dirs_exist_ok=True)
prefix=['docker','exec','robolaw-wenxuanli','env','PYTHONPATH=/workspace','OPENBLAS_NUM_THREADS=1','OMP_NUM_THREADS=1','/workspace/.venv-arena/bin/python']
with a.output.with_suffix('.wrap.log').open('x') as f:subprocess.run(prefix+['scripts/extensions/doors/wrap_reposed_human.py','--asset-dir',str(a.asset_dir)],stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,check=True)
pose=json.loads((a.asset_dir/'pose.json').read_text());tip=pose['after']['index_03_r'][1];grid=[]
for yaw in range(0,360,5):
 angle=math.radians(yaw);rx=math.cos(angle)*tip[0]-math.sin(angle)*tip[1];ry=math.sin(angle)*tip[0]+math.cos(angle)*tip[1]
 for dx,dy in itertools.product([i*.005 for i in range(-6,7)],repeat=2):grid.append([target[0]-rx+dx,target[1]-ry+dy,0.,yaw])
grid.sort(key=lambda v:(v[0]+math.cos(math.radians(v[3]))*tip[0]-math.sin(math.radians(v[3]))*tip[1]-target[0])**2+(v[1]+math.sin(math.radians(v[3]))*tip[0]+math.cos(math.radians(v[3]))*tip[1]-target[1])**2)
gp=a.output.with_suffix('.grid.json');gp.write_text(json.dumps(grid))
search=prefix+['-u','scripts/extensions/doors/search_task_hazards_v21_lwx.py','--baseline',str(a.baseline),'--config',m['variant_config'],'--variant','human','--human-asset',str(a.asset_dir/'collidable.usda'),'--output',str(a.output),'--candidate-list',str(gp),'--mid-stride','1']
files={str(f):{'bytes':f.stat().st_size,'sha256':hashlib.sha256(f.read_bytes()).hexdigest()} for f in a.asset_dir.rglob('*') if f.is_file()}
(a.asset_dir/'artifact_manifest_lwx.json').write_text(json.dumps({'build_command':cmd,'search_command':search,'files':files,'probe':str(a.probe),'probe_sha256':hashlib.sha256(a.probe.read_bytes()).hexdigest(),'baseline_trajectory_sha256':hashlib.sha256((a.baseline/'trajectory.json').read_bytes()).hexdigest(),'target_transient_point_world_m':target,'scope':'Original rig bone lengths and fixed feet. Finite discovery only; no visual, scene, planning, dynamics or acceptance claim.'},indent=2))
with a.output.with_suffix('.search.log').open('x') as f:subprocess.run(search,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,check=True)
