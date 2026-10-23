"""CPU-only source-rig pose and fingertip-aligned C01 candidate discovery."""
import pathlib,json,subprocess,hashlib,shutil,itertools,math,random
r=pathlib.Path('reports/doors/lwx_C01_contact_retry02_20261024');asset=pathlib.Path('assets/extensions/doors/human_reach_tip_lwx_20')
if asset.exists():raise FileExistsError(asset)
cmd=['runtime/tools/blender-4.0.2-linux-x64/blender','--background','--threads','2','assets/people/mpfb_corridor/posed.blend','--python','scripts/extensions/doors/repose_human_both_lwx.py','--','--asset-dir',str(asset),'--wrist','-.90','.15','1.35','--auto-reachable-target','--elbow-up','--clavicle-y-deg','45','--torso-y-deg','-18']
with (r/'C01_human_pose20_build30.log').open('x') as f:subprocess.run(cmd,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,check=True)
shutil.copytree('assets/extensions/doors/human_reach_lean_low_lwx_19/textures',asset/'textures',dirs_exist_ok=True)
prefix=['docker','exec','robolaw-wenxuanli','env','PYTHONPATH=/workspace','OPENBLAS_NUM_THREADS=1','OMP_NUM_THREADS=1','/workspace/.venv-arena/bin/python']
with (r/'C01_human_pose20_wrap30.log').open('x') as f:subprocess.run(prefix+['scripts/extensions/doors/wrap_reposed_human.py','--asset-dir',str(asset)],stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,check=True)
pose=json.loads((asset/'pose.json').read_text());tip=pose['after']['index_03_r'][1];target=[4.501604667518113,-.9273952403157776,1.3119911849247927];grid=[]
for yaw in range(0,360,15):
 angle=math.radians(yaw);rx=math.cos(angle)*tip[0]-math.sin(angle)*tip[1];ry=math.sin(angle)*tip[0]+math.cos(angle)*tip[1]
 for dx,dy in itertools.product([i*.01 for i in range(-5,6)],repeat=2):grid.append([target[0]-rx+dx,target[1]-ry+dy,0.,yaw])
random.Random(42).shuffle(grid);gp=r/'C01_human_fingertip_grid30.json';gp.write_text(json.dumps(grid))
files={str(f):{'bytes':f.stat().st_size,'sha256':hashlib.sha256(f.read_bytes()).hexdigest()} for f in asset.rglob('*') if f.is_file()}
(asset/'artifact_manifest_lwx.json').write_text(json.dumps({'build_command':cmd,'files':files,'scope':'Source original bone lengths and fixed feet only. Numeric pose and offline discovery do not establish visual, scene, planning or physical acceptance.','target_transient_point_world_m':target,'target_scope':'Finite diagnostic sample; not a feasibility proof. Fingertip alignment is only a search center.'},indent=2)+'\n')
out=r/'C01_human_fingertip_search30.json'
with out.with_suffix('.log').open('x') as f:subprocess.run(prefix+['-u','scripts/extensions/doors/search_task_hazards_v21_lwx.py','--baseline','datasets/doors/lwx_C01_contact_retry02_20261024/C01_base_05','--config','configs/extensions/doors/c01_variants_lwx_01.json','--variant','human','--human-asset',str(asset/'collidable.usda'),'--output',str(out),'--candidate-list',str(gp),'--mid-stride','1'],stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,check=True)
print('CPU fingertip discovery complete',flush=True)
