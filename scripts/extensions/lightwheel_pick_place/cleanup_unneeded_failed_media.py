"""Delete only authorized unneeded failed-run media, after references and native-success checks."""
import datetime,hashlib,json,re,subprocess,sys
from pathlib import Path
root=Path('/root/autodl-tmp/RoboLaw-lgd');data=(root/'datasets/skill_trials/lightwheel_pick_place').resolve();review=Path('/root/autodl-tmp/B-group-review')
def read(p):
 try:return json.loads(p.read_text())
 except (OSError,ValueError):return None
states={p.name:(p,read(p/'result.json')) for p in data.iterdir() if p.is_dir() and not p.is_symlink()}
def native_ok(r):
 if not r:return False
 checks=r.get('final_native_destination_checks',{})
 return r.get('success') is True or (r.get('terminal_terms',{}).get('success') is True and checks and all(x.get('passed') is True for x in checks.values()))
active=subprocess.check_output(['ps','-eo','args'],text=True);active_names={n for n in states if n in active}
refs=set();direct=set();dependency=set()
name_pattern=re.compile(r'(?:base|human|property|robot)_lgd_[A-Za-z0-9_]+')
roots=[root/'docs',root/'configs',root/'scripts',root/'kitchen_safety']+[Path('/root/autodl-tmp')/(x+'-review') for x in ('B01','B02','B03','B04','B05','B-group')]
for base in roots:
 for p in base.rglob('*'):
  if p.is_symlink() or not p.is_file() or p.suffix not in ('.json','.md','.py','.sh') or p.stat().st_size>3*1024**2:continue
  if 'cleanup' in p.name.lower():continue
  try:text=p.read_text(errors='replace')
  except OSError:continue
  found=set(name_pattern.findall(text))&states.keys();refs.update(found)
  if p.name=='checkpoint.json':dependency.update(found)
  for n in found:
   for leaf in re.findall(re.escape(n)+r'/([^\s\"\'<>]+)',text):direct.add((n,leaf.rstrip('.,);:')))
# Successful episode prefix ancestry remains fully preserved even if its source failed overall.
for n,(ep,r) in states.items():
 if native_ok(r):
  for text in (json.dumps(r),):
   if r.get('prefix_replay') or r.get('development_release_checkpoint'):dependency.update(set(name_pattern.findall(text))&states.keys())
# Keep one latest representative media set per failure category, plus current important failures.
manual={'base_lgd_b03_matched_robot_20261028T023901Z','base_lgd_b04_matched_human_20261027T162825Z','base_lgd_b04_matched_historical_20261027T165151Z','base_lgd_b04_matched_property_20261028T021452Z'}
latest={};failed=[]
for n,(ep,r) in states.items():
 if n in active_names or native_ok(r) or r is None:continue
 if r.get('success') is not False:continue
 error=r.get('error','native unsuccessful');part=re.search(r"robot_part['\"]?: ['\"]([^'\"]+)",error);obstacle=re.search(r"obstacle['\"]?: ['\"]([^'\"]+)",error)
 group=('collision',part.group(1),obstacle.group(1)) if part and obstacle else (re.sub(r'[-+]?\d+(?:\.\d+)?','N',error[:180]),)
 failed.append((n,ep,r))
 if group not in latest or (ep/'result.json').stat().st_mtime>latest[group][1]:latest[group]=(n,(ep/'result.json').stat().st_mtime)
representatives=manual|{x[0] for x in latest.values()}
files=[];kept={}
for n,ep,r in failed:
 if n in dependency:kept[n]='checkpoint or successful-prefix dependency';continue
 for f in ep.iterdir():
  if not f.is_file() or f.is_symlink():continue
  media=f.name=='images.h5' or f.suffix.lower() in ('.mp4','.jpg','.jpeg','.png')
  scene=f.name=='scene.usda'
  if not(media or scene):continue
  if (n,f.name) in direct:continue
  if n in representatives and f.name!='images.h5':continue
  if scene and n in refs:continue
  if f.resolve().parent!=ep.resolve() or ep.resolve().parent!=data:raise RuntimeError('Path outside intended dataset root')
  stat=f.stat();files.append({'path':str(f.resolve()),'episode':n,'bytes':stat.st_size,'mtime_ns':stat.st_mtime_ns,'type':'scene' if scene else 'failed_media'})
manifest={'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'scope':'Only unsuccessful, inactive, non-dependent runs; keep every result/trajectory/contact/audit/NPZ and all successful raw data. Keep representative failure videos/scenes, delete their redundant images only. No whole directories removed.','active_preserved':sorted(active_names),'dependency_preserved':sorted(dependency),'representative_failure_media_preserved':sorted(representatives),'planned_files':len(files),'planned_bytes':sum(x['bytes'] for x in files),'files':files,'executed':False}
out=review/'failed_media_cleanup_20261028.json'
if out.exists() and read(out).get('executed'):raise FileExistsError('Preserve completed deletion manifest; choose a new manifest filename for future cleanup')
out.write_text(json.dumps(manifest,indent=2));print(json.dumps({k:v for k,v in manifest.items() if k in ('planned_files','planned_bytes','active_preserved')}),flush=True)
if '--execute' in sys.argv:
 deleted=[]
 for x in files:
  f=Path(x['path']);ep=f.parent;r=read(ep/'result.json')
  if native_ok(r) or ep.name in dependency or ep.name in active_names:raise RuntimeError('Protection changed')
  st=f.stat()
  if st.st_size!=x['bytes'] or st.st_mtime_ns!=x['mtime_ns']:raise RuntimeError('File changed since plan: '+str(f))
  assert f.resolve().parent.parent==data and not f.is_symlink()
  f.unlink();deleted.append(x['path'])
 manifest.update(executed=True,deleted_files=len(deleted),deleted_bytes=sum(x['bytes'] for x in files));out.write_text(json.dumps(manifest,indent=2));print('DELETED',len(deleted),'GiB',round(manifest['deleted_bytes']/2**30,2),flush=True)
