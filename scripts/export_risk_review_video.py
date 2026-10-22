"""Export three camera views with recorded HS/PT/RS grades and evidence.

Reads a risk-enabled episode and its rendered/ RGB stream; never regrades it.
The compact layout omits the header, footer and exempt relationship annotations.
"""
from pathlib import Path
import argparse, importlib.util, json, bisect, hashlib, io, time
import av, h5py, numpy as np
from PIL import Image
from PIL import ImageDraw

import os
import risk_video_overlay as overlay

overlay.NAMES.update({'bowl':'碗','kitchen/counter_main_main_group':'主台面',
 'kitchen/counter_right_main_group':'右台面','kitchen/fridge_main_group':'冰箱',
 'kitchen/sink_main_group':'水槽','kitchen/stove_main_group':'灶台',
 'kitchen/microwave_main_group':'微波炉','kitchen/dishwasher_main_group':'洗碗机',
 'kitchen/coffee_machine_main_group':'咖啡机','kitchen/toaster_main_group':'烤面包机',
 'kitchen/knife_block_main_group':'刀架','kitchen/paper_towel_main_group':'纸巾架',
 'kitchen/cab_main_main_group':'主橱柜','kitchen/outlet_room':'插座1','kitchen/outlet_2_room':'插座2',
 'kitchen/light_switch_room':'开关1','kitchen/light_switch_2_room':'开关2',
 'kitchen/floor_room':'地板','kitchen/floor_backing_room':'地板背板'})
for direction,label in [('','正面墙'),('_left','左墙'),('_right','右墙')]:
 overlay.NAMES['kitchen/wall'+direction+'_room']=label
 overlay.NAMES['kitchen/wall'+direction+'_backing_room']=label+'背板'
for stack in range(1,5):
 for part in ['base','1','2','3','4']:
  overlay.NAMES[f'kitchen/stack_{stack}_main_group_{part}']=f'柜组{stack}'+('底座' if part=='base' else f'抽屉{part}')
for k in ('cab_1','cab_2','cab','cab_micro','cab_3','cab_4'):
 overlay.NAMES['kitchen/'+k+'_main_group']='橱柜 '+k

class ReviewPainter(overlay.Painter):
 H=784
 categories=('HS','PT','RS')
 def __init__(self,font):
  super().__init__(font)
  self.template=Image.new('RGB',(self.W,self.H),overlay.BG);d=ImageDraw.Draw(self.template)
  for i,(c,title) in enumerate(zip(self.categories,['人员安全','物体 / 财产安全','机器人自身安全'])):
   x=24+i*468
   d.rounded_rectangle((x,296,x+456,760),radius=12,fill=overlay.PANEL)
   d.text((x+16,309),c+'  '+title,font=self.fonts[22],fill=overlay.TEXT)
 def cards(self,record):
  cards=super().cards(record);features=record['input']['features'];report=record['report']
  hs=features.get('hs',{});card=cards['HS'];evidence=hs.get('distance_evidence',{});scopes=report.get('hs_scopes',{})
  lv=lambda scope:scopes.get(scope,{}).get('category',{}).get('level') or '未知'
  card['lines']=[f'判级最小间距：{overlay.num(report.get("derived",{}).get("D_m"),2,100)} cm',
   f'运动部件：{overlay.num(hs.get("d_moving_h_min_gt_m"),2,100)} cm / {lv("moving")}',
   f'末端→人：{overlay.num(hs.get("d_ee_h_min_gt_m"),2,100)} cm',
   f'目标物→人：{overlay.num(hs.get("d_obj_h_min_gt_m"),2,100)} cm',
   f'运动接近速度：{overlay.num(hs.get("moving_v_rel_h_gt_mps"),2,100)} cm/s']
  if not (hs.get('fixed_distance_exempt') or scopes.get('fixed',{}).get('distance_exempt')):
   card['lines'].insert(2,f'支座距离：{overlay.num(hs.get("d_fixed_h_min_gt_m"),2,100)} cm / {lv("fixed")}')
  card['nearest_geometry']=evidence.get('nearest',{});card['hs_scopes']=scopes;card['exempted_pairs']=evidence.get('exempted_pairs',[])
  distance_rule=next((r for r in report['rules'] if r['rule_id']=='HS-DIST-001'),{})
  de=distance_rule.get('evidence',{});used=de.get('used_distances_m',{})
  numeric={k:v for k,v in used.items() if isinstance(v,(int,float))};which=min(numeric,key=numeric.get) if numeric else 'robot'
  nn=evidence.get('nearest',{}).get(which,{})
  who='目标物' if which=='object' else '夹爪' if which=='ee' else '支架' if 'stand_instanceable' in nn.get('robot_or_object_path','') else '机械臂'
  humanpart='人体衣物' if 'casualsuit' in nn.get('human_path','') else '人体'
  protect=de.get('S_protective_m',de.get('S_rob_m'));warning=(protect+de['warning_buffer_m']) if protect is not None and de.get('warning_buffer_m') is not None else None
  if hs.get('human_contact_flag_gt') is not True and de.get('D_m') is not None:
   card['reason']=f'HS-DIST：{who}→{humanpart} {overlay.num(de["D_m"],2,100)} cm；保护/警戒边界 {overlay.num(protect,2,100)}/{overlay.num(warning,2,100)} cm。'
  card['reason']+=f' 有效人体接触：{overlay.yn(hs.get("human_contact_flag_gt"))}。'
  card['note']='运动等级含末端与目标物'
  if hs.get('dynamic_distance',{}).get('enabled'):
   dominant=de.get('dominant_pair') or {};actor=dominant.get('source_actor') or '未知'
   card['lines']=[f'风险关系：{actor.rsplit("/",1)[-1]}→人体',
    f'该对距离：{overlay.num(dominant.get("distance_m"),2,100)} cm',
    f'动态保护距离：{overlay.num(dominant.get("protective_distance_m"),2,100)} cm',
    f'安全余量 D−S：{overlay.num(dominant.get("margin_m"),2,100)} cm',
    f'机器人表面速度界：{overlay.num(dominant.get("robot_speed_bound_mps"),2,100)} cm/s',
    f'人体表面速度界：{overlay.num(dominant.get("human_speed_bound_mps"),2,100)} cm/s',
    f'候选停止：{overlay.num(dominant.get("stop_s"),3)} s / {overlay.num(dominant.get("brake_travel_m"),2,100)} cm']
   if hs.get('human_contact_flag_gt') is not True:
    card['reason']=(f'动态 HS-DIST：D−S={overlay.num(dominant.get("margin_m"),2,100)} cm；警戒带 {overlay.num(de.get("warning_buffer_m"),2,100)} cm。'
       if dominant else '动态距离证据不足；缺少速度、几何或停止模型的关系保持未知。')
    card['reason']+=f' 有效人体接触：{overlay.yn(hs.get("human_contact_flag_gt"))}。'
   card['reason']+=' 试算停止模型未标定。'
   card['note']='动态停止模型为实验参数，尚未标定'
  pt=cards['PT'];p=features.get('pt',{});pairs=p.get('protected_pairs',[]);summary=report.get('arena_pt_distance_summary',{});near=summary.get('nearest_pair')
  complete=sum(x.get('distance_coverage_complete') is True for x in pairs);count=summary.get('configured_object_count',0)
  rules=[r for r in report['rules'] if r['category']=='PT']
  forces=[r for r in rules if r['rule_id']=='PT-FORCE-002' and r.get('evaluation_status')!='not_applicable' and r['evidence'].get('measured') is not None]
  force=max(forces,key=lambda r:r['evidence']['measured'],default=None);f=force['evidence'] if force else {}
  near_name=(overlay.ACTORS.get(near['actor'],near['actor'])+'→'+overlay.NAMES.get(near['object_id'],near['object_id'])) if near else '无可确认的适用距离'
  g=p.get('grasp_evidence',{})
  pt['lines']=[f'{count}资产 / 几何完整 {complete}/{len(pairs)}对',f'最近有效：{overlay.num(near["distance_m"] if near else None,2,100)} cm',near_name,
   f'确认抓持：{overlay.yn(g.get("held"))}',f'掉落候选：{overlay.yn(p.get("drop_flag_gt"))}',
   f'最大力：{overlay.num(f.get("measured"))}/{overlay.num(f.get("qualified_limit"),0)} N',f'损伤证据：{overlay.yn(p.get("damage_flag_gt"))}']
  if pt['level'] is None:
   pt['reason']='几何覆盖完整；损伤、接触或抓持证据仍有未知，不能将整体财产安全标为L0。' if complete==len(pairs) else '部分几何缺测，且损伤证据未知，不能将整体财产安全标为L0。'
  pt['note']='损伤/支撑裕量未评价，不参与等级汇总'
  pt.update(distance_summary=summary,geometry_complete_pairs=complete,grasp_evidence=g,drop_candidate=p.get('drop_flag_gt'))
  rs=features.get('rs',{});r=cards['RS'];geo=rs.get('self_collision_evidence',{});fault=rs.get('fault_evidence',{})
  r['lines']=[f'最大力矩比：{overlay.num(rs.get("joint_torque_ratio_gt"),3)}',
   f'限位裕量：{overlay.num(rs.get("joint_limit_margin_gt_rad"),2,180/np.pi)}°',
   f'有效自碰撞：{overlay.yn(rs.get("self_collision_flag_gt"))}',
   f'检测{geo.get("evaluated_pair_count",0)}对 / 完整：{overlay.yn(geo.get("coverage_complete"))}',
   f'已观测故障事件：{len(fault.get("events",[]))}',
   f'故障后运动：{overlay.yn(rs.get("motion_after_fault_gt"))}',
   '权威故障状态：'+('完整' if fault.get('fault_state_coverage_complete') else '缺测')]
  if r['level'] is None:r['reason']='自碰撞有独立几何证据；缺少权威故障状态，不能仅因未记录到故障就认定整体L0。'
  r['note']='力矩/限位、几何自碰撞与故障证据分开记录';r.update(self_collision_evidence=geo,fault_evidence=fault)
  for cat, value in cards.items():
   category=report['categories'][cat]
   if category.get('status')=='not_applicable':
    value['lines']=['无人场景 / 不适用'];continue
   count=category.get('known_rule_count',0)
   inactive=sum(rule.get('evaluation_status')=='not_applicable' for rule in report.get('rules',[]) if rule.get('category')==cat)
   excluded=max(0,category.get('excluded_rule_count',0)-inactive)
   value['status_text']=f'已知 {count} / 未评价 {excluded}'
   value['missing']=category.get('missing',[])
   value['grading_policy']='known_results_only'
  if pt['level']=='L0':
   pt['reason']='参与判级的有效 PT 分项均为 L0。损伤、支撑裕量和其他缺测项不参与判级。'
  elif pt['level'] is None:pt['reason']='当前没有可参与判级的有效 PT 结果。'
  if r['level']=='L0':
   r['reason']='力矩、关节限位和自碰撞已知结果均为 L0。完整故障状态及故障后运动缺测，不参与判级。'
  elif r['level'] is None:r['reason']='当前没有可参与判级的有效 RS 结果。'
  # Display the measured trigger without annotations about excluded relations.
  if any(x in pt['reason'] for x in ('该碰撞对无抓持豁免','内指已按阶段豁免','指定抓持部件已按阶段豁免')):
   pt['reason']=pt['reason'].split('；',1)[0]+'。'
  return {cat: cards[cat] for cat in self.categories}
 def draw(self,cameras,record,scene,batch,frame,total,timestamp,phase,outcome,timeline):
  im=self.template.copy();d=ImageDraw.Draw(im)
  for x,camera in zip([24,492,960],cameras):im.paste(camera,(x,24))
  cards=self.cards(record)
  for i,cat in enumerate(self.categories):
   x=24+i*468;v=cards[cat];color=overlay.CYAN if v['status']=='not_applicable' else overlay.COLORS[v['level']]
   d.text((x+16,345),v['level'] or '未知',font=self.fonts[38],fill=color)
   for j,line in enumerate(v['lines']):
    self.text(d,(x+16,404+j*28),line.replace('cm cm','cm'),18,width=424,maxlines=1)
   d.line((x+16,604,x+440,604),fill='#34445b',width=1)
   self.text(d,(x+16,619),v['reason'],18,width=424,maxlines=5)
  return im,cards

def render_stream(job,out,painter,max_frames=None):
 start=time.time();p=Path(job['episode']);dest=out/job['filename']
 binding=overlay.load(p/'safety_observer_binding.json');report=overlay.load(p/'rendered/render_report.json');axes=overlay.load(p/'result_axes.json')
 profile_hash=hashlib.sha256(json.dumps(binding['resolved_profile'],sort_keys=True,ensure_ascii=False,allow_nan=False).encode()).hexdigest()
 times=[];levels=[]
 # Retain only the timeline index, not hundreds of MB of duplicated evidence.
 with (p/'safety_decisions.jsonl').open() as source:
  for line in source:
   r=json.loads(line)
   assert r['profile_sha256']==profile_hash
   assert all(r.get(k)==v for k,v in binding['identity'].items())
   times.append(r['decision_time_s']);levels.append(r['report']['risk_levels'].get('overall'))
 assert all(a<b for a,b in zip(times,times[1:]))
 stamps=[r['source_time_seconds'] for r in report['frames']];n=len(stamps)
 if max_frames is not None:stamps=stamps[:max_frames];n=len(stamps)
 selected=[bisect.bisect_right(times,t+1e-9)-1 for t in stamps];assert n and min(selected)>=0
 timeline=Image.new('RGB',(1392,14));draw=ImageDraw.Draw(timeline)
 for i,k in enumerate(selected):
  draw.rectangle((int(i*1392/n),0,int((i+1)*1392/n),14),fill=overlay.COLORS[levels[k]])
 fps=round(1/binding['action_dt_s']);tmp=dest.with_suffix('.part.mp4')
 checkpoints={0,n//4,n//2,3*n//4,n-1}
 current=-1;record=None
 with (p/'safety_decisions.jsonl').open() as source,h5py.File(p/'rendered/images.h5','r') as images,h5py.File(p/'trajectory.h5','r') as trajectory,dest.with_suffix('.frames.jsonl').open('w') as logs,av.open(str(tmp),'w',options={'movflags':'+faststart'}) as container:
  stream=container.add_stream('libx264',rate=fps);stream.width=painter.W;stream.height=painter.H;stream.pix_fmt='yuv420p';stream.options={'crf':'23','preset':'veryfast','threads':'2'}
  for i,k in enumerate(selected):
   while current<k:
    record=json.loads(next(source));current+=1
   source_frame=report['frames'][i]['source_obs_index'];t=stamps[i]
   assert record['decision_time_s']<=t+1e-9
   assert record['decision_physics_step']<=round(t/binding['physics_dt_s'])
   assert abs(float(trajectory['time'][source_frame])-t)<1e-7
   phase=trajectory['phase'][source_frame];phase=phase.decode() if isinstance(phase,bytes) else str(phase)
   cameras=[Image.open(io.BytesIO(np.asarray(images[c+'/jpeg'][i],dtype=np.uint8).tobytes())).convert('RGB').resize((456,256),Image.Resampling.LANCZOS) for c in ['external_camera','external_camera_2','wrist_camera']]
   image,cards=painter.draw(cameras,record,job['scene'],job['batch'],i,n,t,phase,axes['task']['legacy_success'],timeline)
   if i in checkpoints:image.save(out/(dest.stem+f'_frame_{i:04d}.jpg'),quality=92)
   logs.write(json.dumps(dict(frame=i,time_s=t,observation_id=record['observation_id'],source_physics_step=record['source_physics_step'],input_sha256=record['input_sha256'],profile_sha256=profile_hash,cards=cards),ensure_ascii=False)+'\n')
   frame=av.VideoFrame.from_image(image);frame.pts=i
   for packet in stream.encode(frame):container.mux(packet)
   if i%150==0:print(json.dumps(dict(frame=i,total=n,elapsed_s=round(time.time()-start,1))),flush=True)
  for packet in stream.encode():container.mux(packet)
 tmp.replace(dest)
 with av.open(str(dest)) as check:
  decoded=sum(1 for _ in check.decode(video=0));duration=float(check.duration or 0)/av.time_base
 assert decoded==n
 result=dict(job,displayed_categories=list(painter.categories),visual_layout='Three cameras; category labels, grades, metrics and reasons only',output=str(dest),frames=n,decoded_frames=decoded,fps=fps,duration_s=duration,passed=True,
  temporal_alignment='Latest risk decision at or before measured RGB source time; source timestamps are checked against recorded trajectory observations.',
  pt_mode=binding['resolved_profile']['rules']['pt_mode'],profile_sha256=profile_hash,
  task_success=axes['task']['legacy_success'],source_camera_mode='offline_measured_state_render',
  source_modified=False,timeline_scope='Latest known overall grade; timeline remains outside the compact video layout',bytes=dest.stat().st_size,elapsed_s=round(time.time()-start,2))
 dest.with_suffix('.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
 print(json.dumps(result,ensure_ascii=False),flush=True)
 return result

def find_font(requested=None):
 candidates = [requested, os.environ.get('ROBOLAW_CJK_FONT'),Path(__file__).resolve().parents[1]/'runtime/fonts/NotoSansCJK-Regular.ttc',
  '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
  '/usr/share/fonts/google-noto-cjk/NotoSansCJK-Regular.ttc',
  '/System/Library/Fonts/PingFang.ttc']
 for candidate in candidates:
  if candidate and Path(candidate).is_file():return Path(candidate)
 raise FileNotFoundError('Install fonts-noto-cjk or supply --font /path/to/CJK-font.ttc')

def main():
 parser=argparse.ArgumentParser(description=__doc__)
 parser.add_argument('--episode',type=Path,required=True)
 parser.add_argument('--output',type=Path,required=True,help='New directory for video, checkpoints and frame evidence')
 parser.add_argument('--font',type=Path,help='CJK TrueType/OpenType font; otherwise search standard Noto locations')
 parser.add_argument('--max-frames',type=int,help='Render only this many initial frames for a quick preview')
 args=parser.parse_args()
 if args.max_frames is not None and args.max_frames<1:parser.error('--max-frames must be positive')
 font=find_font(args.font)
 episode=args.episode.resolve()
 result_info=overlay.load(episode/'result.json')
 args.output.mkdir(parents=True,exist_ok=False)
 job={'batch':'记录风险回放','episode':str(episode),
      'scene':result_info.get('obstacle_id',episode.parent.name),'filename':'risk_review.mp4'}
 result=render_stream(job,args.output,ReviewPainter(font),args.max_frames)
 result['complete_trajectory']=args.max_frames is None
 (args.output/'manifest.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
 return 0
if __name__=='__main__':raise SystemExit(main())
