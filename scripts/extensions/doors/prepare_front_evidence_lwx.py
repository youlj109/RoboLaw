"""Prepare source/video evidence after a successful collection; never marks visual or physical gates passed."""
import argparse,os,json,time,subprocess,copy
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--property-attempt',required=True);p.add_argument('--config',type=Path,required=True);a=p.parse_args()
R=Path('reports/doors/lwx_front_20261023');D=Path('datasets/doors/lwx_front_20261023');episode=D/a.property_attempt;pipeline=R/(a.property_attempt+'.pipeline.json');deadline=time.monotonic()+2400
while time.monotonic()<deadline:
 if pipeline.exists():
  stages=json.loads(pipeline.read_text());collect=next((x for x in stages if x['stage']=='collect'),None)
  if any(x['state']=='failed' for x in stages):raise SystemExit('Physical pipeline failed; no success evidence prepared')
  if collect and collect['state']=='passed':break
 time.sleep(5)
else:raise SystemExit('Collection wait timed out')
assert json.loads((episode/'result.json').read_text())['accepted']
configs=json.loads(a.config.read_text());old=json.load(open('configs/extensions/doors/avoidance_variants.json'));canonical={}
for variant,path in [('human',D/'human_02'),('property',episode),('robot',Path('datasets/doors/lwx_20261023/robot_01'))]:
 canonical[variant]=copy.deepcopy(json.loads((path/'metadata.json').read_text())['variant_spec'])
 canonical[variant]['candidate_evidence']=str(path/'avoidance_audit.json')
canonical['property']['semantic_anchor']='Original bottle translated 0.18m in world x and -0.08m in world y toward the actual robot forward axis, with root 0.03m higher on a real floor column 0.03m taller; original source geometry and safety margins retained.'
canonical['_fixed_source_layout']=configs['_fixed_source_layout'];canonical['_revision']={'purpose':'C01 front revision; original robot group retained; human/property captured with dense_pregrasp_v4 and the same matching baseline policy','support_height_change_m':.03,'translations_from_original_m':{v:[round(x-y,8) for x,y in zip(canonical[v]['position'],old[v]['position'])] for v in ['human','property','robot']},'capture_configurations':{'base':'configs/extensions/doors/front_height_search_lwx_01.json','human':'configs/extensions/doors/front_variants_lwx_06.json','property':str(a.config),'robot':'configs/extensions/doors/avoidance_variants.json'},'scope':'Descriptive annotations corrected; original capture configs and whole-file hashes retained. Physical specs are compared to captured metadata.'}
cp=Path('configs/extensions/doors/front_variants_lwx_final.json');assert not cp.exists();cp.write_text(json.dumps(canonical,indent=2)+'\n')
env=os.environ.copy();env['PYTHONPATH']='/workspace'
for key in ['ISAACSIM_ROOT_PATH','ISAACLAB_PATH','OMNI_SERVER']:env.pop(key,None)
commands=[('asset_provenance_01',['scripts/extensions/doors/snapshot_assets.py','--output',R/'asset_provenance_01.json','--mdl-root','/workspace/.venv-arena/lib/python3.12/site-packages/isaacsim/kit/mdl/core/Base',D/'base_01',D/'human_02',episode,'datasets/doors/lwx_20261023/robot_01','datasets/doors/lwx_20261023/base_03']),('front_geometry_05',['scripts/extensions/doors/audit_front_anchors_lwx.py','--baseline',D/'base_01','--config',cp,'--output',R/'front_geometry_05.json']),('video_review_property_01',['scripts/extensions/doors/review_front_videos_lwx.py','--output',R/'video_review_property_01',episode])]
for label,args in commands:
 log=R/(label+'.log');assert not log.exists();print('PREPARE',label,flush=True)
 with log.open('x') as f:code=subprocess.run(['/workspace/.venv-arena/bin/python']+list(map(str,args)),env=env,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT).returncode
 (R/(label+'.exit')).write_text(str(code)+'\n')
 if code:raise SystemExit(code)
print('SOURCE_VIDEO_EVIDENCE_PREPARED; actual visual review and remaining pipeline gates still required',flush=True)
