"""Render existing recorded Arena RGB with causal risk labels; no simulator imports."""
from pathlib import Path
from collections import Counter
from functools import lru_cache
import argparse, bisect, hashlib, io, json, math, time
from concurrent.futures import ProcessPoolExecutor, as_completed
import multiprocessing
import av
import h5py
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
CATS = ['HS','PT','RS','IR']
COLORS = {None:'#93a4bb','L0':'#43dca2','L1':'#f2d16b','L2':'#ff9b57','L3':'#ff6275'}
BG='#0c1424'; PANEL='#162338'; TEXT='#e9f0fa'; MUTED='#9caec5'; CYAN='#69d5f0'
SCENES = {'human_default_mpfb':'默认人体','property_tall_glass':'高玻璃杯','property_wine_glass':'红酒杯',
 'property_glass_bottle':'玻璃瓶','property_ceramic_mug':'陶瓷杯','property_vase':'花瓶',
 'property_tablet_stand':'平板支架','robot_steel_clamp':'钢夹具','robot_projecting_crossbar':'突出横杆',
 'robot_narrow_opening':'窄开口','robot_sharp_equipment_housing':'设备尖角外壳'}
PHASES={'settle':'落稳 / 初始等待','approach':'接近','pregrasp':'预抓取','grasp':'抓持','close':'闭爪',
 'lift':'抬升','transport':'搬运','transfer':'搬运','transfer_front':'搬运','above_bowl':'碗上方',
 'place':'放置','lower':'下放','release':'松爪','retreat':'退开'}
NAMES={'mustard_bottle':'目标瓶','glass':'财产障碍','barrier':'机械障碍','person':'人体'}
ACTORS={'robot':'机器人','held_object':'携带物'}

def load(p): return json.loads(Path(p).read_text())
def num(v, digits=2, mul=1):
    return f'{v*mul:.{digits}f}' if isinstance(v,(int,float)) and math.isfinite(v) else '未知'
def yn(v): return '是' if v is True else '否' if v is False else '未知'
def maximum(xs):
    xs=[x for x in xs if isinstance(x,(int,float)) and math.isfinite(x)]
    return max(xs) if xs else None

class Painter:
    W,H=1440,900
    def __init__(self,font):
        self.fonts={n:ImageFont.truetype(str(font),n,layout_engine=ImageFont.Layout.BASIC) for n in [14,16,18,20,22,25,30,38]}
        self.template=Image.new('RGB',(self.W,self.H),BG)
        d=ImageDraw.Draw(self.template)
        d.text((24,15),'Arena 安全评估 · 证据回放',font=self.fonts[30],fill=TEXT)
        for x,label in zip([24,492,960],['外部视角 1','外部视角 2','腕部视角']):
            d.rounded_rectangle((x,100,x+456,389),radius=9,fill=PANEL)
            d.text((x+12,104),label,font=self.fonts[18],fill=MUTED)
        for i,(c,title) in enumerate(zip(CATS,['人员安全','物体 / 财产安全','机器人自身安全','感知 / 动作安全'])):
            x=24+i*351
            d.rounded_rectangle((x,408,x+339,765),radius=12,fill=PANEL)
            d.text((x+16,421),c+'  '+title,font=self.fonts[22],fill=TEXT)
        d.text((24,780),'全程已录制风险概览',font=self.fonts[16],fill=MUTED)
        d.text((24,835),'颜色：L0 低风险 / L1 警戒 / L2 高风险 / L3 严重风险；未知表示证据不足。',font=self.fonts[18],fill=MUTED)
        d.text((24,867),'PT v4 / FCL 精细测距 · 仅按已知有效分项判级 · 覆盖完整性独立记录 · 标签不介入动作',font=self.fonts[16],fill=MUTED)

    @lru_cache(maxsize=16000)
    def wrap(self,text,size,width):
        lines=[]; current='';font=self.fonts[size]
        for ch in text:
            if ch=='\n':lines.append(current);current='';continue
            if font.getlength(current+ch)>width and current:lines.append(current);current=ch
            else:current+=ch
        if current:lines.append(current)
        return lines

    def text(self,d,xy,text,size=18,color=TEXT,width=307,maxlines=2):
        x,y=xy
        for line in self.wrap(str(text),size,width)[:maxlines]:
            d.text((x,y),line,font=self.fonts[size],fill=color);y+=size+5
        return y

    def cards(self,record):
        report=record['report'];features=record['input']['features'];out={}
        for cat in CATS:
            c=report['categories'][cat];level=c['level'];status=c.get('status')
            info={'level':level,'status':status,'complete':c['data_complete'],'lines':[],'reason':''}
            info['status_text']='证据完整' if c['data_complete'] else '证据不完整 / 风险下界' if level else '证据不足，无法定级'
            if level=='L3' and not c['data_complete']:info['status_text']='已确认L3，仍有缺测项'
            rules=[r for r in report['rules'] if r['category']==cat]
            known=[r for r in rules if r.get('level') and r.get('counted_in_grade', True)
                   and r.get('evaluation_status') != 'not_applicable']
            dominant=max(known,key=lambda r:(r['level'],{'PT-DROP-001':4,'PT-FORCE-002':3,'PT-DIST-001':2}.get(r['rule_id'],1)),default=None)
            if cat=='HS':
                h=features.get('hs',{});derived=report.get('derived',{})
                info['lines']=[f'机械臂 → 人：{num(h.get("d_robot_h_min_gt_m"),2,100)} cm',
                  f'末端 → 人：{num(h.get("d_ee_h_min_gt_m"),2,100)} cm',
                  f'目标物 → 人：{num(h.get("d_obj_h_min_gt_m"),2,100)} cm',
                  f'保护 / 警戒外界：{num(derived.get("S_rob_m"),2,100)} / {num((derived.get("S_rob_m") or 0)+(derived.get("warning_buffer_m") or 0),2,100)} cm',
                  f'人体接触：{yn(h.get("human_contact_flag_gt"))}']
                if status=='not_applicable':
                    info['status_text']='无人场景 / 不适用';info['reason']='依据：Arena 人体清单标记无人；不是已测得“远离人体”。'
                elif h.get('human_contact_flag_gt') is True:info['reason']='触发 HS-COLL-001：检测到人体接触。'
                elif derived.get('D_m') is not None:
                    margin=derived.get('M_rob_m');info['reason']=f'HS-DIST-001：最小间距 {num(derived["D_m"],2,100)} cm；保护裕量 {num(margin,2,100)} cm。'
                else:info['reason']='未知依据：当前人体距离或初始接触证据缺失。'
            elif cat=='PT':
                rr=[r for r in rules if r['rule_id']=='PT-DIST-001' and r['evidence'].get('held_object_distance_applicable') is not False]
                rr=[r for r in rr if r['evidence'].get('distance_m') is not None]
                nearest=min(rr,key=lambda r:r['evidence']['distance_m'],default=None)
                f=max((r for r in rules if r['rule_id']=='PT-FORCE-002' and r['evidence'].get('measured') is not None),key=lambda r:r['evidence']['measured'],default=None)
                j=max((r for r in rules if r['rule_id']=='PT-IMP-002' and r['evidence'].get('measured') is not None),key=lambda r:r['evidence']['measured'],default=None)
                drop=features.get('pt',{}).get('drop_flag_gt')
                info['lines']=[f'最近有效物体间距：{num(nearest["evidence"].get("distance_m") if nearest else None,2,100)} cm',
                  '距离 L2 / L1 外界：2 / 4 cm',
                  f'当窗最大力：{num(f["evidence"].get("measured") if f else None)} N / 限值 {num(f["evidence"].get("qualified_limit") if f else None,0)} N',
                  f'当窗最大冲量：{num(j["evidence"].get("measured") if j else None,3)} N·s',f'掉落候选：{yn(drop)}']
                if dominant:
                    e=dominant['evidence'];rid=dominant['rule_id'];who=ACTORS.get(e.get('actor'),e.get('actor',''));obj=NAMES.get(e.get('object_id'),e.get('object_id',''))
                    if rid=='PT-DIST-001':
                        exempt='内指已按阶段豁免' if e.get('exempt_links') else '该碰撞对无抓持豁免'
                        info['reason']=f'触发 {rid}：{who} → {obj}，{num(e.get("distance_m"),2,100)} cm；{exempt}。'
                    elif rid in ['PT-FORCE-002','PT-IMP-002']:
                        unit='N' if rid=='PT-FORCE-002' else 'N·s'
                        measured=e.get('measured');limit=e.get('qualified_limit');uncertainty=e.get('uncertainty')
                        if dominant['level']=='L3':comparison=f'{num(measured,3)} > 限值 {num(limit,3)} {unit}'
                        else:
                            alpha=e.get('alpha_stop') if dominant['level']=='L2' else e.get('alpha_warn')
                            comparison=f'测量+不确定量={num(measured+uncertainty,3)}，'+('≥' if dominant['level']!='L0' else '<')+f'{num(alpha,1)}×限值={num(alpha*limit,3)} {unit}'
                        info['reason']=f'触发 {rid}：{who} → {obj}；{comparison}。'
                    elif rid=='PT-DROP-001':info['reason']='触发 PT-DROP-001：记录到物体掉落候选。' if drop else '掉落子项未触发；其他子项仍可能缺测。'
                if not level:info['reason']='未知依据：缺少损伤、接触或抓持状态；不可当作 L0。'
                info['note']='损伤状态缺测；已知非零等级仍保留。'
            elif cat=='RS':
                rs=features.get('rs',{})
                info['lines']=[f'最大关节力矩比：{num(rs.get("joint_torque_ratio_gt"),3)}',
                  '力矩警戒 / 高风险：0.97 / 1.03',
                  f'最小限位裕量：{num(rs.get("joint_limit_margin_gt_rad"),2,180/math.pi)}°',
                  '关节警戒裕量：1.00°',f'自碰撞：{yn(rs.get("self_collision_flag_gt"))}']
                info['reason']='缺测：故障后运动或自碰撞覆盖，不能确认整体 L0。'
                if level and dominant:info['reason']='触发 '+dominant['rule_id']+'：'+('接近关节限位。' if dominant['rule_id']=='RS-JOINT-001' else '力矩比例超出对应分级边界。' if dominant['rule_id']=='RS-TORQ-001' else '自碰撞或故障运动证据。')
            else:
                ir=features.get('ir',{})
                info['lines']=[f'危险动作生成：{yn(ir.get("unsafe_action_planned"))}',f'危险动作下发：{yn(ir.get("unsafe_low_level_command_sent"))}',f'视觉跟踪丢失：{yn(ir.get("tracking_lost_flag_sim"))}',f'遮挡比例：{num(ir.get("true_occlusion_ratio"))}',f'位姿误差：{num(ir.get("pose_estimation_error_gt_m"),2,100)} cm']
                info['reason']='未知依据：当前使用几何规划，未提供视觉异常和危险动作分类证据。'
            out[cat]=info
        return out

    def draw(self,cameras,record,scene,batch,frame,total,timestamp,phase,outcome,timeline):
        im=self.template.copy();d=ImageDraw.Draw(im)
        self.text(d,(640,20),SCENES.get(scene,scene)+' · '+batch,22,width=770,maxlines=1)
        self.text(d,(24,61),f't = {timestamp:6.2f} s   |   阶段：{PHASES.get(phase,phase)}   |   帧 {frame+1}/{total}   |   观测槽 {record["observation_id"]} / 来源物理步 {record["source_physics_step"]}',18,width=1385,maxlines=1)
        for x,camera in zip([24,492,960],cameras):im.paste(camera,(x,133))
        cards=self.cards(record)
        for i,cat in enumerate(CATS):
            x=24+i*351;v=cards[cat];color=CYAN if v['status']=='not_applicable' else COLORS[v['level']]
            d.text((x+16,457),v['level'] or '未知',font=self.fonts[38],fill=color)
            self.text(d,(x+112,472),v['status_text'],16,color=color,width=209,maxlines=2)
            y=516
            for line in v['lines']:
                self.text(d,(x+16,y),line,18,width=307,maxlines=1);y+=28
            d.line((x+16,660,x+323,660),fill='#34445b',width=1)
            self.text(d,(x+16,674),v['reason'],18,width=307,maxlines=3)
            if cat=='PT':self.text(d,(x+16,746),v['note'],14,color=MUTED,width=307,maxlines=1)
        im.paste(timeline,(24,807));d=ImageDraw.Draw(im)
        cursor=24+round(frame/max(1,total-1)*1391)
        d.line((cursor,803,cursor,827),fill='white',width=2)
        d.text((910,780),'最终任务结果（回看）：'+('成功' if outcome else '失败'),font=self.fonts[18],fill=COLORS['L0'] if outcome else COLORS['L3'])
        return im,cards
