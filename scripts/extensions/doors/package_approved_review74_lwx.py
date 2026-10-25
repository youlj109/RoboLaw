"""Build a completed review bundle, without relabelling retained evidence."""
import argparse, datetime, hashlib, html, json, shutil, subprocess
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
R=Path('reports/doors/lwx_approved_run_20261026');D=Path('docs/extensions/doors/lwx_scene_approved_20261026')
read=lambda p:json.loads(Path(p).read_text())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
new={}
for cp in sorted(D.glob('ACCEPTANCE_CHECKPOINT*.json')):
 d=read(cp)
 if 'variant' in d:new[d['variant'].replace(' ','_')]=dict(d,checkpoint=str(cp))
 for k,v in d.get('variants',{}).items():new[k]=dict(v,checkpoint=str(cp))
ret=read(R/'RETAINED_SELECTION_20261026.json')['variants']
keys=[f'C{i:02}_{v}' for i in range(1,9) for v in ['human','property','robot']]
assert len(new)==18 and len(ret)==6,(len(new),len(ret))
assert set(keys)==set(new)|{k.replace(' ','_') for k in ret}
assert not a.output.exists(),a.output
a.output.mkdir(parents=True);copies={};episodes={};selection={}
def copy(src,rel):
 src=Path(src);dest=a.output/rel
 if dest.exists():
  assert sha(src)==sha(dest);return rel.as_posix()
 dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(src,dest)
 digest=sha(dest);assert sha(src)==digest
 copies[rel.as_posix()]={'source':str(src),'sha256':digest,'bytes':dest.stat().st_size}
 return rel.as_posix()
def epcopy(ep,legacy=False):
 ep=Path(ep)
 if str(ep) in episodes:return episodes[str(ep)]
 prefix=Path('episodes')/(ep.name+'_'+hashlib.sha256(str(ep).encode()).hexdigest()[:8]);row={'source':str(ep),'files':{}}
 for name in ['metadata.json','result.json','trajectory.json','avoidance_audit.json','self_collision_audit.json','supplementary_observer_alignment.json','review_all_four_cameras.json','path_comparison.png','validation_full.json','review_audit.json','contact_geometry_audit.json','comparison_audit.json','task_validation_full.json','task_review_audit.json','source_contact_audit.json']:
  f=ep/name
  if f.exists():row['files'][name]=copy(f,prefix/name)
 video=ep/'review_all_four_cameras.mp4'
 if not video.exists():video=ep/'all_cameras.mp4'
 if video.exists():row['video']=copy(video,prefix/video.name)
 row['video_layout']='four synchronized cameras with real observer' if video.name.startswith('review') else 'three original synchronized cameras'
 review=ep/('review_audit.json' if legacy else 'task_review_audit.json')
 if review.exists():row['neutral_source']=read(review).get('matching_neutral',{}).get('path')
 row['raw_dataset_location']=str(ep.resolve());episodes[str(ep)]=row
 return row
gates=[('task_review_audit.json','passed'),('task_validation_full.json','accepted'),('source_contact_audit.json','passed'),('self_collision_audit.json','passed')]
for key in keys:
 t,v=key.split('_',1);legacy=key=='C01_robot'
 if key in new:
  d=new[key];ep=Path(d['episode']);av=read(ep/'avoidance_audit.json');baseline=d.get('baseline') or av['baseline_episode']
  assert read(ep/'result.json')['accepted'] and av['accepted'] and all(read(ep/n)[k] for n,k in gates)
  row={'status':'new_revision_validated_user_visual_review_pending','episode':str(ep),'baseline':baseline,'config':d['config'],'user_visual_approval':False,'fixed_only':True}
  row['checkpoint']=copy(d['checkpoint'],Path('checkpoints')/Path(d['checkpoint']).name)
  cp=Path(d['checkpoint']).stem.replace('ACCEPTANCE_CHECKPOINT','checkpoint');pair=D/cp/'paired_report.json'
  if not pair.exists() and d.get('paired_report'):pair=Path(d['paired_report'])
  if not pair.exists():pair=D/'supplemental_evidence70'/key/'paired_report.json'
  assert pair.exists() and read(pair)['fixed_endpoint_rerouting_supported'],(key,pair)
  row['paired_report']=copy(pair,Path('paired')/(key+'.json'))
 else:
  d=ret[t+' '+v];ep=Path(d['episode']);baseline=d['prior_matched_baseline']
  checks=[('review_audit.json','passed'),('validation_full.json','accepted'),('contact_geometry_audit.json','passed'),('self_collision_audit.json','passed')] if legacy else gates
  assert read(ep/'result.json')['accepted'] and read(ep/'avoidance_audit.json')['accepted'] and all(read(ep/n)[k] for n,k in checks)
  row={'status':'retained_prior_version_original_limits_preserved','episode':str(ep),'baseline':baseline,'config':d['variant_config'],'current_round_new_success':False,'fixed_only':True}
 row['evidence']=epcopy(ep,legacy);row['baseline_evidence']=epcopy(baseline,legacy)
 av=read(ep/'avoidance_audit.json')
 row['fixed_target_rerouting_evidence']={'same_handle_target':av.get('same_handle_target'),'same_controller_policy':av.get('same_controller_policy'),'handle_target_check':av.get('handle_target_check'),'path_comparison':av.get('path_comparison')}
 if key in new and d.get('neutral'):row['neutral_evidence']=epcopy(d['neutral'],legacy)
 if key=='C08_human':row['camera_limitation']='Native external1 is substantially occluded by torso; use synchronized observer for full person/robot/oven view.'
 if key=='C08_property':row['camera_limitation']='Native close views crop upper bottle; observer provides complete obstacle view.'
 for e in [row['evidence'],row['baseline_evidence']]:
  if e.get('neutral_source'):epcopy(e['neutral_source'],legacy)
 row['config_copy']=copy(row['config'],Path('configs')/Path(row['config']).name)
 if legacy:row['limitations']='Original steel off paths violate original10mm clearance without source intersection. Stricter historical discovery screening remains rejected; retained at user request.'
 if key=='C07_robot':row['limitations']='User approved pilot13 visually. Actual TCP deviation about9.85cm does not establish all-seed>=10cm; stronger historical discovery screening remains separate.'
 observer=ep/'supplementary_observer/visible_review_camera.mp4'
 if observer.exists():row['observer_video']=copy(observer,Path('observers')/(key+'.mp4'))
 selection[key]=row
for f in [R/'RETAINED_SELECTION_20261026.json',R/'RETAINED_EVIDENCE74.json',D/'FIXED_PREGRASP_ENDPOINT_20261026.md',D/'SUPPLEMENTAL_EVIDENCE70.json',D/'fixed_endpoint_cpu_tests70.log']:
 copy(f,Path('context')/f.name)
for source,name in [(R/'C06_human_settle75/settling_revision_provenance75.json','C06_settling_revision_provenance75.json'),(Path('datasets/doors/lwx_C06_human_shortcut59_20261026/C06_human_p01/avoidance_audit.json'),'C06_prior_endpoint_FAILED.json'),(R/'C08_property_frozen71/startup_recovery_provenance.json','C08_render_startup_recovery_provenance.json')]:
 if source.exists():copy(source,Path('context')/name)
out={'generated_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'git_head':subprocess.getoutput('git rev-parse HEAD'),'review_bundle_only':True,'new_revision_variants':18,'retained_prior_variants':6,'selected_variants':24,'episodes':selection,'dataset_paths':episodes}
(a.output/'selection.json').write_text(json.dumps(out,indent=2))
intro='本轮18个新修订完成真实采集与独立验收，另6个沿用用户要求保留的版本。新版待用户视觉审阅。每种仅选1条固定场景成功数据。完整 HDF5 和物理子步记录保留在 selection.json 所列远端路径，本包包含视频、轨迹、元数据与审计。'
parts=['<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>RoboLaw C01–C08 审阅</title><style>body{font:16px/1.7 system-ui;max-width:1200px;margin:30px auto;padding:20px;background:#f3f6fa;color:#173049}section{background:white;margin:20px 0;padding:20px;border-radius:12px}video{width:100%}table{border-collapse:collapse;width:100%}td,th{padding:8px;border:1px solid #cbd5df}</style><h1>RoboLaw C01–C08 审阅包</h1><p>'+intro+'</p>']
for key,row in selection.items():
 e=row['evidence'];b=row['baseline_evidence'];parts.append('<section><h2>'+key+'</h2><p>'+html.escape(row['status'])+'</p>')
 if row.get('limitations'):parts.append('<p>'+html.escape(row['limitations'])+'</p>')
 if row.get('camera_limitation'):parts.append('<p>'+html.escape(row['camera_limitation'])+'</p>')
 if e.get('video'):parts.append('<p>'+e['video_layout']+'</p><video controls preload="none" src="'+e['video']+'"></video>')
 if row.get('observer_video'):parts.append('<details><summary>单独查看原始观察相机（便于放大障碍）</summary><video controls preload="none" src="'+row['observer_video']+'"></video></details>')
 parts.append('<details><summary>匹配无障碍基线与证据</summary><p>'+html.escape(row['baseline'])+'</p>')
 if b.get('video'):parts.append('<video controls preload="none" src="'+b['video']+'"></video>')
 parts.append('<p><a href="'+row['config_copy']+'">配置</a> · <a href="'+e['files']['metadata.json']+'">元数据</a> · <a href="'+e['files']['avoidance_audit.json']+'">实际避障审计</a></p></details></section>')
parts.append('</html>');(a.output/'index.html').write_text('\n'.join(parts))
(a.output/'README.md').write_text('# RoboLaw C01–C08 review bundle\n\n'+intro+'\n\n解压后打开 index.html。每个变体下可展开匹配无障碍基线；有额外观察相机的版本可单独放大观看。selection.json 是准确选取清单，copies_sha256.json 用于核验复制文件。代码、配置及资产恢复方法见个人分支 lwx。\n\n远端 SSH: youlj@172.16.2.1；仓库 /home/youlj/projects/wenxuanli/RoboLaw。完整原始 RGB HDF5、物理子步数组、源快照与失败尝试保存在 selection.json 对应的远端 datasets 路径。\n\n本包不是随机化或200条批采，也不代表用户已批准本轮新视频。合法抓点适配与同一目标下的成对绕行证据分列。C01保留钢架的间隙不足不等于几何相交，C07的9.85cm参照也不等于所有seed均超过10cm。\n\ncontext 中保留 C06 先前因接近终点跟踪误差未通过的报告，以及 C08 首次渲染启动崩溃的恢复记录。失败不覆盖、不计为最终成功。\n')
(a.output/'copies_sha256.json').write_text(json.dumps(copies,indent=2));print('REVIEW_BUNDLE_CREATED',a.output,len(copies))
